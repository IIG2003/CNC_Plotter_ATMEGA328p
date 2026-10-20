// ============================================================
// xy_gcode_plotter.c   —  bare-metal AVR (ATmega328P) port.
//
// Pin map (ATmega328P):
//   X: STEP PB0(D8)  DIR PB1(D9)  EN PB2(D10)  LIMIT PD2(D2)
//   Y: DIR  PD5(D5)  STEP PD6(D6) EN PD7(D7)   LIMIT PD3(D3)
//   PEN servo: OC2A = PB3 (D11)
//

// ============================================================

#define F_CPU 16000000UL
#define BAUD  9600

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/delay_basic.h>
#include <util/setbaud.h>   // computes UBRR/U2X from F_CPU + BAUD
#include <stdlib.h>         // strtod, labs
#include <string.h>         // strchr
#include <ctype.h>          // toupper, isspace
#include <stdint.h>

// ---------------- geometry / motion config ----------------
#define STEPS_PER_MM_X    80.0f
#define MAX_MM_X          275.0f
#define HOMING_DELAY_US_X 500
#define BACKOFF_STEPS_X   100

#define STEPS_PER_MM_Y    80.0f
#define MAX_MM_Y          190.0f
#define HOMING_DELAY_US_Y 500
#define BACKOFF_STEPS_Y   100

// direction convention (matches the original HIGH/LOW):
//   toward the limit switch = HIGH(1), away = LOW(0)
#define TOWARD_HOME_X     1
#define AWAY_FROM_HOME_X  0
#define TOWARD_HOME_Y     1
#define AWAY_FROM_HOME_Y  0

// ---------------- pen (servo) ----------------
#define PEN_UP_ANGLE      40      // <-- tune to your linkage
#define PEN_DOWN_ANGLE    90      // <-- tune to your linkage
#define PEN_Z_THRESHOLD   0.0f    // Z at or below this = pen down
#define PEN_MOVE_MS       180     // let the servo settle

// ---------------- speeds (half-pulse delay, us) ----------------
#define DRAW_DELAY_US     500
#define TRAVEL_DELAY_US   400
#define MIN_DELAY_US      350

// ---------------- pin macros ----------------
#define X_STEP_HI()  (PORTB |=  _BV(PB0))
#define X_STEP_LO()  (PORTB &= ~_BV(PB0))
#define Y_STEP_HI()  (PORTD |=  _BV(PD6))
#define Y_STEP_LO()  (PORTD &= ~_BV(PD6))

#define X_LIMIT_OPEN()  (PIND & _BV(PD2))   // pull-up: HIGH = not triggered
#define Y_LIMIT_OPEN()  (PIND & _BV(PD3))

static inline void x_dir(uint8_t high) { if (high) PORTB |=  _BV(PB1);
                                         else      PORTB &= ~_BV(PB1); }
static inline void y_dir(uint8_t high) { if (high) PORTD |=  _BV(PD5);
                                         else      PORTD &= ~_BV(PD5); }

// ---------------- state ----------------
static int32_t curXsteps    = 0;
static int32_t curYsteps    = 0;
static uint8_t absoluteMode = 1;
static float   unitScale    = 1.0f;   // 1.0 = mm, 25.4 = inches
static uint8_t penIsDown    = 0;

static char    lineBuf[96];
static uint8_t lineIdx = 0;

// ============================================================
// timing: variable-length microsecond busy-wait
//   _delay_loop_2 burns 4 CPU cycles/iter -> 0.25 us @ 16 MHz,
//   so `us` microseconds == us*4 iterations. Ignores a few cycles
//   of call overhead (fine — feed timing is approximate anyway).
//   Valid up to ~16 ms; our largest call is ~4 ms.
// ============================================================
static inline void delay_us_var(uint16_t us) {
    if (us == 0) return;
    _delay_loop_2((uint16_t)(us * 4));
}

// ============================================================
// UART (USART0) with interrupt-driven RX ring buffer
// ============================================================
#define RX_BUF_SIZE 128                 // must be a power of two
static volatile uint8_t rxbuf[RX_BUF_SIZE];
static volatile uint8_t rx_head = 0, rx_tail = 0;

ISR(USART_RX_vect) {
    uint8_t c    = UDR0;
    uint8_t next = (uint8_t)((rx_head + 1) & (RX_BUF_SIZE - 1));
    if (next != rx_tail) {              // drop byte if buffer full
        rxbuf[rx_head] = c;
        rx_head = next;
    }
}

static uint8_t uart_available(void) { return rx_head != rx_tail; }

static char uart_getc(void) {
    while (rx_head == rx_tail) { }       // caller guarantees availability
    char c  = (char)rxbuf[rx_tail];
    rx_tail = (uint8_t)((rx_tail + 1) & (RX_BUF_SIZE - 1));
    return c;
}

static void uart_putc(char c) {
    while (!(UCSR0A & _BV(UDRE0))) { }
    UDR0 = (uint8_t)c;
}

static void uart_puts(const char *s) { while (*s) uart_putc(*s++); }

static void uart_println(const char *s) {   // mimics Serial.println (\r\n)
    uart_puts(s);
    uart_putc('\r');
    uart_putc('\n');
}

static void uart_init(void) {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= (uint8_t)~_BV(U2X0);
#endif
    UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);  // RX+TX, RX interrupt
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);              // 8N1
}

// ============================================================
// pen servo — Timer2 fast PWM on OC2A (PB3)
// ============================================================
static uint8_t angle_to_ocr(uint8_t angle) {
    // 0 deg -> 1000 us, 180 deg -> 2000 us, at 64 us/tick
    uint16_t pulse = (uint16_t)(1000u + ((uint16_t)angle * 1000u) / 180u);
    uint16_t ticks = pulse / 64u;
    if (ticks > 255u) ticks = 255u;
    return (uint8_t)ticks;
}

static void servo_init(void) {
    // OC2A non-inverting, Fast PWM (TOP=0xFF), prescaler /1024 -> ~61 Hz
    TCCR2A = _BV(COM2A1) | _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS22)   | _BV(CS21)  | _BV(CS20);
    OCR2A  = angle_to_ocr(PEN_UP_ANGLE);
}

static void pen_down(void) {
    if (penIsDown) return;
    OCR2A = angle_to_ocr(PEN_DOWN_ANGLE);
    _delay_ms(PEN_MOVE_MS);
    penIsDown = 1;
}

static void pen_up(void) {
    if (!penIsDown) return;
    OCR2A = angle_to_ocr(PEN_UP_ANGLE);
    _delay_ms(PEN_MOVE_MS);
    penIsDown = 0;
}

// ============================================================
// homing
// ============================================================
static void step_pulse_reg(volatile uint8_t *port, uint8_t mask, uint16_t d) {
    *port |= mask;  delay_us_var(d);
    *port &= (uint8_t)~mask; delay_us_var(d);
}

// Drive toward the switch until triggered, back off, re-approach slowly.
static void home_axis(volatile uint8_t *step_port, uint8_t step_mask,
                      void (*set_dir)(uint8_t),
                      volatile uint8_t *limit_pin, uint8_t limit_mask,
                      uint8_t toward_high, uint16_t d, uint16_t backoff) {
    set_dir(toward_high);
    while (*limit_pin & limit_mask) step_pulse_reg(step_port, step_mask, d);

    set_dir(!toward_high);
    for (uint16_t i = 0; i < backoff; i++) step_pulse_reg(step_port, step_mask, d);

    set_dir(toward_high);
    while (*limit_pin & limit_mask) step_pulse_reg(step_port, step_mask, (uint16_t)(d * 2));
}

static void home_all(void) {
    pen_up();
    home_axis(&PORTB, _BV(PB0), x_dir, &PIND, _BV(PD2),
              TOWARD_HOME_X, HOMING_DELAY_US_X, BACKOFF_STEPS_X);
    home_axis(&PORTD, _BV(PD6), y_dir, &PIND, _BV(PD3),
              TOWARD_HOME_Y, HOMING_DELAY_US_Y, BACKOFF_STEPS_Y);
    curXsteps = 0;
    curYsteps = 0;
}

// ============================================================
// coordinated linear move (Bresenham-timed -> straight diagonals)
// ============================================================
static int32_t clamp_steps(int32_t s, float maxMm, float stepsPerMm) {
    int32_t maxSteps = (int32_t)(maxMm * stepsPerMm);
    if (s < 0)        return 0;
    if (s > maxSteps) return maxSteps;
    return s;
}

static void move_linear(int32_t tx, int32_t ty, uint16_t delayUs) {
    tx = clamp_steps(tx, MAX_MM_X, STEPS_PER_MM_X);   // soft limits
    ty = clamp_steps(ty, MAX_MM_Y, STEPS_PER_MM_Y);

    int32_t dx = tx - curXsteps;
    int32_t dy = ty - curYsteps;

    x_dir((dx >= 0) ? AWAY_FROM_HOME_X : TOWARD_HOME_X);
    y_dir((dy >= 0) ? AWAY_FROM_HOME_Y : TOWARD_HOME_Y);

    int32_t ax = labs(dx);
    int32_t ay = labs(dy);
    int32_t n  = (ax > ay) ? ax : ay;
    if (n == 0) return;

    int32_t errX = n / 2;
    int32_t errY = n / 2;

    for (int32_t i = 0; i < n; i++) {
        uint8_t sx = 0, sy = 0;

        errX -= ax; if (errX < 0) { errX += n; sx = 1; }
        errY -= ay; if (errY < 0) { errY += n; sy = 1; }

        if (sx) X_STEP_HI();
        if (sy) Y_STEP_HI();
        delay_us_var(delayUs);
        if (sx) X_STEP_LO();
        if (sy) Y_STEP_LO();
        delay_us_var(delayUs);
    }

    curXsteps = tx;
    curYsteps = ty;
}

// ============================================================
// G-code parsing
// ============================================================
static int32_t round_l(float v) {           // replaces lround(), no libm
    return (int32_t)(v + (v >= 0.0f ? 0.5f : -0.5f));
}

// Pull the numeric value that follows `key` (e.g. 'X'). 0 if absent.
static uint8_t get_val(const char *s, char key, float *out) {
    const char *p = strchr(s, key);
    if (!p) return 0;
    *out = (float)strtod(p + 1, NULL);
    return 1;
}

// Approx feed (mm/min) -> per-step half-delay (us). Uses X scale; clamped.
static uint16_t feed_to_delay(float feed_mm_min) {
    if (feed_mm_min <= 0) return DRAW_DELAY_US;
    float sps = (feed_mm_min / 60.0f) * STEPS_PER_MM_X;
    if (sps <= 0) return DRAW_DELAY_US;
    int32_t d = (int32_t)(1000000.0f / (2.0f * sps));
    if (d < MIN_DELAY_US) d = MIN_DELAY_US;
    if (d > 4000)         d = 4000;
    return (uint16_t)d;
}

static void process_line(char *line) {
    char *c;
    if ((c = strchr(line, ';')) != NULL) *c = 0;   // strip line comment
    if ((c = strchr(line, '(')) != NULL) *c = 0;   // strip '(' comment

    for (char *p = line; *p; p++) *p = (char)toupper((unsigned char)*p);

    uint8_t hasContent = 0;
    for (char *p = line; *p; p++)
        if (!isspace((unsigned char)*p)) { hasContent = 1; break; }
    if (!hasContent) { uart_println("ok"); return; }

    float g, m, z, f, xmm, ymm;

    // ---- modal / non-motion codes ----
    if (get_val(line, 'G', &g)) {
        int gi = (int)g;
        if      (gi == 20) unitScale = 25.4f;
        else if (gi == 21) unitScale = 1.0f;
        else if (gi == 90) absoluteMode = 1;
        else if (gi == 91) absoluteMode = 0;
        else if (gi == 28) home_all();
    }

    if (get_val(line, 'M', &m)) {
        int mi = (int)m;
        if      (mi == 3 || mi == 4) pen_down();
        else if (mi == 5)            pen_up();
        // M2 / M30 -> program end, no-op
    }

    if (get_val(line, 'Z', &z)) {
        if (z * unitScale <= PEN_Z_THRESHOLD) pen_down();
        else                                  pen_up();
    }

    // ---- motion ----
    uint8_t isG0 = 0, isG1 = 0;
    if (get_val(line, 'G', &g)) {
        int gi = (int)g;
        if      (gi == 0) isG0 = 1;
        else if (gi == 1) isG1 = 1;
    }

    if (isG0 || isG1) {
        int32_t tx = curXsteps;
        int32_t ty = curYsteps;

        if (get_val(line, 'X', &xmm)) {
            int32_t xs = round_l(xmm * unitScale * STEPS_PER_MM_X);
            tx = absoluteMode ? xs : curXsteps + xs;
        }
        if (get_val(line, 'Y', &ymm)) {
            int32_t ys = round_l(ymm * unitScale * STEPS_PER_MM_Y);
            ty = absoluteMode ? ys : curYsteps + ys;
        }

        uint16_t delayUs = isG0 ? TRAVEL_DELAY_US : DRAW_DELAY_US;
        if (get_val(line, 'F', &f)) delayUs = feed_to_delay(f * unitScale);

        move_linear(tx, ty, delayUs);
    }

    uart_println("ok");
}

// ============================================================
// setup + main loop
// ============================================================
int main(void) {
    // outputs: X step/dir/en (PB0..2) + servo OC2A (PB3); Y dir/step/en (PD5..7)
    DDRB |= _BV(PB0) | _BV(PB1) | _BV(PB2) | _BV(PB3);
    DDRD |= _BV(PD5) | _BV(PD6) | _BV(PD7);

    // limit switches: inputs with pull-ups (PD2, PD3)
    DDRD  &= (uint8_t)~(_BV(PD2) | _BV(PD3));
    PORTD |=  _BV(PD2) | _BV(PD3);

    // stepper drivers: active-low enable -> LOW = enabled
    PORTB &= (uint8_t)~_BV(PB2);   // EN_X
    PORTD &= (uint8_t)~_BV(PD7);   // EN_Y

    uart_init();
    servo_init();          // parks pen at UP angle
    sei();                 // enable interrupts (UART RX)

    penIsDown = 0;
    _delay_ms(PEN_MOVE_MS);

    uart_println("Homing...");
    home_all();
    uart_println("Ready");
    uart_println("ok");    // primes hosts that wait for a first ok

    for (;;) {
        while (uart_available()) {
            char ch = uart_getc();
            if (ch == '\n' || ch == '\r') {
                if (lineIdx > 0) {
                    lineBuf[lineIdx] = 0;
                    process_line(lineBuf);
                    lineIdx = 0;
                }
            } else if (lineIdx < sizeof(lineBuf) - 1) {
                lineBuf[lineIdx++] = ch;
            }
        }
    }
}