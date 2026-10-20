// ============================================================
// xy_gcode_plotter.c
//
// Bare-metal AVR-C port of xy_gcode_plotter.ino for ATmega328P
// @ 16MHz external crystal. No Arduino core, no Servo.h.
//
// Pen actuator: a hobby servo on PB3 (D11), driven by a
// software PWM generator running in the Timer1 compare-match
// ISR (same technique Arduino's Servo library uses internally).
// Timer1 is used in CTC mode with its hardware output-compare
// function left DISCONNECTED (TCCR1A = 0), so PB1 (D9, OC1A)
// keeps working as a plain GPIO pin -- it's still DIR_PIN_X
// here and is never touched by the timer hardware, only by our
// own PORTB writes. Only the ISR toggles PB3 in software.
//
// Homes X then Y on boot, then listens on UART (TTL) for
// G-code and executes it. Sends "ok" after EVERY line so a
// streaming host can flow-control.
//
// X motor:  STEP PB0(D8), DIR PB1(D9), EN PB2(D10), LIMIT PD2(D2)
// Y motor:  DIR PD5(D5), STEP PD6(D6), EN PD7(D7),  LIMIT PD3(D3)
// Pen:      servo signal PB3(D11)
// ============================================================

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <stdbool.h>

// ---------------- X axis (PORTB) ----------------
#define STEP_PIN_X         PB0
#define DIR_PIN_X          PB1
#define EN_PIN_X           PB2
#define STEPS_PER_MM_X     80.0
#define MAX_MM_X          275.0
#define HOMING_DELAY_US_X  500
#define BACKOFF_STEPS_X    100

// ---------------- Y axis (PORTD) ----------------
#define DIR_PIN_Y          PD5
#define STEP_PIN_Y         PD6
#define EN_PIN_Y           PD7
#define STEPS_PER_MM_Y     80.0
#define MAX_MM_Y          190.0
#define HOMING_DELAY_US_Y  500
#define BACKOFF_STEPS_Y    100

// ---------------- limit switches (PORTD) ----------------
#define LIMIT_PIN_X        PD2
#define LIMIT_PIN_Y        PD3

// ---------------- pen (servo on PB3 / D11) ----------------
#define PEN_SERVO_PIN      PB3
#define PEN_UP_ANGLE       40      // <-- tune to your linkage
#define PEN_DOWN_ANGLE     90      // <-- tune to your linkage
#define PEN_Z_THRESHOLD    0.0     // Z at or below this = pen down
#define PEN_MOVE_MS        180     // let the servo settle before moving

// Standard hobby-servo pulse range for 0-180 deg (matches Arduino Servo lib default)
#define SERVO_MIN_US       544
#define SERVO_MAX_US       2400
#define SERVO_PERIOD_US    20000   // 50Hz frame

// Timer1 @ prescaler 8, 16MHz -> 2MHz tick rate -> 0.5us/tick -> 2 ticks/us
#define US_TO_TICKS(us)    ((uint16_t)((us) * 2UL))

// ---------------- speeds (half-pulse delay, us) ----------------
#define DRAW_DELAY_US      500     // G1 pen-down feed
#define TRAVEL_DELAY_US    400     // G0 rapid
#define MIN_DELAY_US       350     // fastest a step is ever allowed

long   curXsteps    = 0;
long   curYsteps    = 0;
bool   absoluteMode = true;
float  unitScale    = 1.0;   // 1.0 = mm, 25.4 = inches
bool   penIsDown    = false;

char    lineBuf[96];
uint8_t lineIdx = 0;

// ---- software servo state (driven from Timer1 COMPA ISR) ----
volatile uint16_t servo_pulse_ticks;   // current HIGH width, in timer ticks
volatile uint8_t  servo_phase;         // 0 = about to rise, 1 = about to fall

// ---- runtime microsecond delay (util/delay.h needs compile-time
//      constants, so loop 1us delays for runtime-variable timing) ----
static inline void delay_us_var(uint16_t us) {
    while (us--) _delay_us(1);
}

// angle (0-180) -> pulse width in microseconds, same mapping Arduino's
// Servo.write() uses by default.
static uint16_t angleToUs(uint8_t angle) {
    return (uint16_t)(SERVO_MIN_US + ((long)angle * (SERVO_MAX_US - SERVO_MIN_US)) / 180);
}

// Update the servo target angle. Written from the main loop, read from
// the ISR -- guard the 16-bit write so the ISR never sees a torn value.
static void servo_write_angle(uint8_t angle) {
    uint16_t ticks = US_TO_TICKS(angleToUs(angle));
    uint8_t sreg = SREG;
    cli();
    servo_pulse_ticks = ticks;
    SREG = sreg;
}

// CTC mode auto-reloads TCNT1 to 0 on every compare match, so writing
// OCR1A here sets the *interval until the next interrupt*, not an
// absolute compare point -- that's what makes the two-phase toggle work.
ISR(TIMER1_COMPA_vect) {
    if (servo_phase == 0) {
        PORTB |= (1 << PEN_SERVO_PIN);
        OCR1A = servo_pulse_ticks;
        servo_phase = 1;
    } else {
        PORTB &= ~(1 << PEN_SERVO_PIN);
        OCR1A = US_TO_TICKS(SERVO_PERIOD_US) - servo_pulse_ticks;
        servo_phase = 0;
    }
}

static void servo_init(void) {
    DDRB |= (1 << PEN_SERVO_PIN);
    servo_pulse_ticks = US_TO_TICKS(angleToUs(PEN_UP_ANGLE));
    servo_phase = 0;

    TCCR1A = 0;                            // OC1A/OC1B left disconnected -- PB1 stays plain GPIO
    TCCR1B = (1 << WGM12) | (1 << CS11);   // CTC mode, prescaler = 8
    OCR1A  = servo_pulse_ticks;
    TIMSK1 |= (1 << OCIE1A);
    sei();
}

// ---- UART, 9600 baud @ 16MHz ----
void uart_init(void) {
    uint16_t ubrr = (F_CPU / 16 / 9600UL) - 1;
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_putc(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

void uart_puts(const char *s) {
    while (*s) uart_putc(*s++);
}

static inline bool uart_available(void) {
    return (UCSR0A & (1 << RXC0)) != 0;
}

static inline char uart_getc(void) {
    while (!(UCSR0A & (1 << RXC0)));
    return UDR0;
}

// ---- step pulse helpers ----
static inline void stepPulse_X(uint16_t delayUs) {
    PORTB |= (1 << STEP_PIN_X);
    delay_us_var(delayUs);
    PORTB &= ~(1 << STEP_PIN_X);
    delay_us_var(delayUs);
}

static inline void stepPulse_Y(uint16_t delayUs) {
    PORTD |= (1 << STEP_PIN_Y);
    delay_us_var(delayUs);
    PORTD &= ~(1 << STEP_PIN_Y);
    delay_us_var(delayUs);
}

// ---- pen control (real servo) ----
void penDown(void) {
    if (penIsDown) return;
    servo_write_angle(PEN_DOWN_ANGLE);
    _delay_ms(PEN_MOVE_MS);
    penIsDown = true;
}

void penUp(void) {
    if (!penIsDown) return;
    servo_write_angle(PEN_UP_ANGLE);
    _delay_ms(PEN_MOVE_MS);
    penIsDown = false;
}

// ---- homing ----
void homeAxisX(void) {
    PORTB |= (1 << DIR_PIN_X);
    while (PIND & (1 << LIMIT_PIN_X)) stepPulse_X(HOMING_DELAY_US_X);

    PORTB &= ~(1 << DIR_PIN_X);
    for (uint16_t i = 0; i < BACKOFF_STEPS_X; i++) stepPulse_X(HOMING_DELAY_US_X);

    PORTB |= (1 << DIR_PIN_X);
    while (PIND & (1 << LIMIT_PIN_X)) stepPulse_X(HOMING_DELAY_US_X * 2);
}

void homeAxisY(void) {
    PORTD |= (1 << DIR_PIN_Y);
    while (PIND & (1 << LIMIT_PIN_Y)) stepPulse_Y(HOMING_DELAY_US_Y);

    PORTD &= ~(1 << DIR_PIN_Y);
    for (uint16_t i = 0; i < BACKOFF_STEPS_Y; i++) stepPulse_Y(HOMING_DELAY_US_Y);

    PORTD |= (1 << DIR_PIN_Y);
    while (PIND & (1 << LIMIT_PIN_Y)) stepPulse_Y(HOMING_DELAY_US_Y * 2);
}

void homeAll(void) {
    penUp();
    homeAxisX();
    homeAxisY();
    curXsteps = 0;
    curYsteps = 0;
}

long clampSteps(long s, float maxMm, float stepsPerMm) {
    long maxSteps = (long)(maxMm * stepsPerMm);
    if (s < 0)        return 0;
    if (s > maxSteps) return maxSteps;
    return s;
}

// Coordinated linear move to absolute (tx, ty) in steps, Bresenham-timed
// so the shorter axis' pulses are spread evenly -> straight diagonals.
void moveLinear(long tx, long ty, uint16_t delayUs) {
    tx = clampSteps(tx, MAX_MM_X, STEPS_PER_MM_X);
    ty = clampSteps(ty, MAX_MM_Y, STEPS_PER_MM_Y);

    long dx = tx - curXsteps;
    long dy = ty - curYsteps;

    if (dx >= 0) PORTB &= ~(1 << DIR_PIN_X); else PORTB |= (1 << DIR_PIN_X);
    if (dy >= 0) PORTD &= ~(1 << DIR_PIN_Y); else PORTD |= (1 << DIR_PIN_Y);

    long ax = labs(dx);
    long ay = labs(dy);
    long n  = (ax > ay) ? ax : ay;
    if (n == 0) return;

    long errX = n / 2;
    long errY = n / 2;

    for (long i = 0; i < n; i++) {
        uint8_t sx = 0, sy = 0;

        errX -= ax; if (errX < 0) { errX += n; sx = 1; }
        errY -= ay; if (errY < 0) { errY += n; sy = 1; }

        if (sx) PORTB |= (1 << STEP_PIN_X);
        if (sy) PORTD |= (1 << STEP_PIN_Y);
        delay_us_var(delayUs);
        if (sx) PORTB &= ~(1 << STEP_PIN_X);
        if (sy) PORTD &= ~(1 << STEP_PIN_Y);
        delay_us_var(delayUs);
    }

    curXsteps = tx;
    curYsteps = ty;
}

// Pull a numeric value that follows `key` (e.g. 'X'). Returns false if absent.
// C has no reference params, so this takes a pointer instead of float&.
bool getVal(const char *s, char key, float *out) {
    const char *p = strchr(s, key);
    if (!p) return false;
    *out = atof(p + 1);
    return true;
}

// Approx feed (mm/min) -> per-step half-delay. Uses X scale; clamped.
uint16_t feedToDelay(float feed_mm_min) {
    if (feed_mm_min <= 0) return DRAW_DELAY_US;
    float sps = (feed_mm_min / 60.0) * STEPS_PER_MM_X;
    if (sps <= 0) return DRAW_DELAY_US;
    long d = (long)(1000000.0 / (2.0 * sps));
    if (d < MIN_DELAY_US) d = MIN_DELAY_US;
    if (d > 4000)         d = 4000;
    return (uint16_t)d;
}

void processLine(char *line) {
    // strip comments: everything after ';' and anything from '('
    char *c;
    if ((c = strchr(line, ';')) != NULL) *c = 0;
    if ((c = strchr(line, '(')) != NULL) *c = 0;

    for (char *p = line; *p; p++) *p = toupper((unsigned char)*p);

    bool hasContent = false;
    for (char *p = line; *p; p++) if (!isspace((unsigned char)*p)) { hasContent = true; break; }
    if (!hasContent) { uart_puts("ok\r\n"); return; }

    float g, m, z, f, xmm, ymm;

    // ---- modal / non-motion codes ----
    if (getVal(line, 'G', &g)) {
        int gi = (int)g;
        if      (gi == 20) unitScale = 25.4;
        else if (gi == 21) unitScale = 1.0;
        else if (gi == 90) absoluteMode = true;
        else if (gi == 91) absoluteMode = false;
        else if (gi == 28) homeAll();
    }

    if (getVal(line, 'M', &m)) {
        int mi = (int)m;
        if      (mi == 3 || mi == 4) penDown();
        else if (mi == 5)            penUp();
        // M2 / M30 -> program end, nothing to do
    }

    if (getVal(line, 'Z', &z)) {
        if (z * unitScale <= PEN_Z_THRESHOLD) penDown();
        else                                  penUp();
    }

    // ---- motion ----
    bool isG0 = false, isG1 = false;
    if (getVal(line, 'G', &g)) {
        int gi = (int)g;
        if      (gi == 0) isG0 = true;
        else if (gi == 1) isG1 = true;
    }

    if (isG0 || isG1) {
        long tx = curXsteps;
        long ty = curYsteps;

        if (getVal(line, 'X', &xmm)) {
            long xs = lround(xmm * unitScale * STEPS_PER_MM_X);
            tx = absoluteMode ? xs : curXsteps + xs;
        }
        if (getVal(line, 'Y', &ymm)) {
            long ys = lround(ymm * unitScale * STEPS_PER_MM_Y);
            ty = absoluteMode ? ys : curYsteps + ys;
        }

        uint16_t delayUs = isG0 ? TRAVEL_DELAY_US : DRAW_DELAY_US;
        if (getVal(line, 'F', &f)) delayUs = feedToDelay(f * unitScale);

        moveLinear(tx, ty, delayUs);
    }

    uart_puts("ok\r\n");
}

// ------------------------------------------------------------
int main(void) {
    // ---- X axis pins ----
    DDRB |= (1 << STEP_PIN_X) | (1 << DIR_PIN_X) | (1 << EN_PIN_X);

    // ---- Y axis pins ----
    DDRD |= (1 << STEP_PIN_Y) | (1 << DIR_PIN_Y) | (1 << EN_PIN_Y);

    // ---- limit switches, input with pull-up ----
    DDRD  &= ~((1 << LIMIT_PIN_X) | (1 << LIMIT_PIN_Y));
    PORTD |=  (1 << LIMIT_PIN_X) | (1 << LIMIT_PIN_Y);

    // ---- enable drivers (A4988/DRV8825 EN is active-low) ----
    PORTB &= ~(1 << EN_PIN_X);
    PORTD &= ~(1 << EN_PIN_Y);

    uart_init();

    servo_init();           // starts the background pulse train at PEN_UP_ANGLE
    penIsDown = false;
    _delay_ms(PEN_MOVE_MS); // let it settle before homing moves start

    uart_puts("Homing...\r\n");
    homeAll();
    uart_puts("Ready\r\n");
    uart_puts("ok\r\n");     // primes hosts that wait for a first ok

    while (1) {
        if (uart_available()) {
            char ch = uart_getc();
            if (ch == '\n' || ch == '\r') {
                if (lineIdx > 0) {
                    lineBuf[lineIdx] = 0;
                    processLine(lineBuf);
                    lineIdx = 0;
                }
            } else if (lineIdx < sizeof(lineBuf) - 1) {
                lineBuf[lineIdx++] = ch;
            }
        }
    }
}
