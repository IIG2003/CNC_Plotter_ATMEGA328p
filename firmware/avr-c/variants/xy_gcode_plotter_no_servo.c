// ============================================================
// xy_gcode_plotter_no_servo.c
//
// Bare-metal AVR-C port of xy_gcode_plotter_no_servo.ino for
// ATmega328P @ 16MHz external crystal. No Arduino core.
//
// Pen servo not mounted yet -- M3/M4/M5 and Z-triggered pen
// moves are parsed but just print a message instead of moving
// a servo. See the commented block at the bottom for the
// register-level servo (Timer2 PWM) version once it's wired.
//
// Homes X then Y on boot, then listens on UART (TTL) for
// G-code and executes it. Sends "ok" after EVERY line so a
// streaming host can flow-control.
//
// X motor:  STEP PB0(D8), DIR PB1(D9), EN PB2(D10), LIMIT PD2(D2)
// Y motor:  DIR PD5(D5), STEP PD6(D6), EN PD7(D7),  LIMIT PD3(D3)
// ============================================================

#define F_CPU 16000000UL
#include <avr/io.h>
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

// ---------------- pen (servo not mounted yet) ----------------
#define PEN_Z_THRESHOLD    0.0     // Z at or below this = pen down

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

// ---- runtime microsecond delay (util/delay.h needs compile-time
//      constants, so loop 1us delays for runtime-variable timing) ----
static inline void delay_us_var(uint16_t us) {
    while (us--) _delay_us(1);
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

// ---- pen control: stubbed until the servo is mounted ----
void penDown(void) {
    if (penIsDown) return;
    uart_puts("(pen down - servo not mounted, no-op)\r\n");
    penIsDown = true;
}

void penUp(void) {
    if (!penIsDown) return;
    uart_puts("(pen up - servo not mounted, no-op)\r\n");
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

    uart_puts("Homing...\r\n");
    homeAll();
    uart_puts("Ready (pen servo not mounted yet - pen commands are no-ops)\r\n");
    uart_puts("ok\r\n");   // primes hosts that wait for a first ok

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

/* ============================================================
 * ONCE THE SERVO IS MOUNTED, restore real pen control using
 * Timer2 fast-PWM (no Servo.h in bare-metal -- generate the
 * 50Hz/1-2ms pulse yourself). Example using PB3 (D11, OC2A):
 *
 * 1. Add near the top:
 *      #define PEN_UP_TICKS    ...   // tune via oscilloscope/servo tester
 *      #define PEN_DOWN_TICKS  ...
 *      #define PEN_MOVE_MS     180
 *
 * 2. In main(), before homeAll(), configure Timer2 for
 *    Phase-Correct PWM at ~50Hz on OC2A (PB3), set DDRB |= (1<<PB3).
 *    ATmega328P's 8-bit Timer2 can't natively hit exact 50Hz/1-2ms
 *    servo timing well -- Timer1 (16-bit, OC1A/PB1) is the better
 *    choice for this. Since PB1 is your DIR_PIN_X here, you'd need
 *    to move the servo to another Timer1-capable pin or free up
 *    PB1 by remapping X's DIR line elsewhere on PORTB/PORTC.
 *
 * 3. Replace penDown()/penUp() with direct OCR1A writes setting
 *    the compare value to PEN_DOWN_TICKS / PEN_UP_TICKS, followed
 *    by a delay_us_var(PEN_MOVE_MS * 1000UL) settle time.
 * ============================================================ */
