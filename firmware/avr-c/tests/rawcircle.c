// ============================================================
// raw_circle_draw.c
//
// Bare-metal AVR-C port of raw_circle_draw.ino for ATmega328P
// @ 16MHz external crystal. No Arduino core -- direct register
// access only. Homes X and Y, then traces a circle.
//
// X motor:  STEP PB0(D8), DIR PB1(D9), EN PB2(D10), LIMIT PD2(D2)
// Y motor:  DIR PD5(D5), STEP PD6(D6), EN PD7(D7),  LIMIT PD3(D3)
// ============================================================

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------------- X axis (PORTB) ----------------
#define STEP_PIN_X        PB0
#define DIR_PIN_X          PB1
#define EN_PIN_X            PB2
#define STEPS_PER_MM_X     80.0
#define MAX_MM_X          275.0
#define HOMING_DELAY_US_X  500
#define BACKOFF_STEPS_X    100

// ---------------- Y axis (PORTD) ----------------
#define DIR_PIN_Y           PD5
#define STEP_PIN_Y          PD6
#define EN_PIN_Y             PD7
#define STEPS_PER_MM_Y     80.0
#define MAX_MM_Y          190.0
#define HOMING_DELAY_US_Y  500
#define BACKOFF_STEPS_Y    100

// ---------------- limit switches (PORTD) ----------------
#define LIMIT_PIN_X          PD2
#define LIMIT_PIN_Y          PD3

// ---------------- circle parameters ----------------
#define CENTER_X_MM   137.5   // middle of 275mm X travel
#define CENTER_Y_MM    95.0   // middle of 190mm Y travel
#define RADIUS_MM      60.0
#define SEGMENTS       144    // 2.5 degree steps -> smooth circle
#define DRAW_DELAY_US  500    // step half-pulse width while tracing

long curXsteps = 0;
long curYsteps = 0;

// ---- runtime microsecond delay (util/delay.h needs compile-time
//      constants, so loop 1us delays for runtime-variable timing) ----
static inline void delay_us_var(uint16_t us) {
    while (us--) _delay_us(1);
}

// ---- UART, 9600 baud @ 16MHz (for status prints only) ----
void uart_init(void) {
    uint16_t ubrr = (F_CPU / 16 / 9600UL) - 1;
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_putc(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

void uart_puts(const char *s) {
    while (*s) uart_putc(*s++);
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

// ---- homing ----
void homeAxisX(void) {
    PORTB |= (1 << DIR_PIN_X);              // toward home
    while (PIND & (1 << LIMIT_PIN_X)) stepPulse_X(HOMING_DELAY_US_X);

    PORTB &= ~(1 << DIR_PIN_X);             // back off
    for (uint16_t i = 0; i < BACKOFF_STEPS_X; i++) stepPulse_X(HOMING_DELAY_US_X);

    PORTB |= (1 << DIR_PIN_X);              // slow final approach
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

// Coordinated linear move to absolute (tx, ty) in steps -- Bresenham
// timed so the shorter axis' pulses spread evenly for a straight line.
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

void moveToMm(float xmm, float ymm, uint16_t delayUs) {
    long tx = lround(xmm * STEPS_PER_MM_X);
    long ty = lround(ymm * STEPS_PER_MM_Y);
    moveLinear(tx, ty, delayUs);
}

void drawCircle(void) {
    // travel to the start point of the circle first (angle 0)
    moveToMm(CENTER_X_MM + RADIUS_MM, CENTER_Y_MM, DRAW_DELAY_US);

    for (int i = 1; i <= SEGMENTS; i++) {
        float ang = 2.0 * M_PI * i / SEGMENTS;
        float x = CENTER_X_MM + RADIUS_MM * cos(ang);
        float y = CENTER_Y_MM + RADIUS_MM * sin(ang);
        moveToMm(x, y, DRAW_DELAY_US);
    }

    // return to center so it's obvious the circle closed correctly
    moveToMm(CENTER_X_MM, CENTER_Y_MM, DRAW_DELAY_US);
}

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
    uart_puts("Homed. Drawing circle...\r\n");

    drawCircle();

    uart_puts("Circle done.\r\n");

    while (1) { }   // done -- reset board to run again
}
