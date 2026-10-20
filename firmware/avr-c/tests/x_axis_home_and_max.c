// ============================================================
// x_axis_home_and_max.c
//
// Bare-metal AVR-C port of single_axis_home_and_max.ino, X axis
// only. Homes using the limit switch, then drives to the far
// end of travel (MAX_MM), prints progress over UART, then stops.
//
// Motor:  STEP PB0(D8), DIR PB1(D9), EN PB2(D10)
// Switch: PD2(D2), COM -> PD2, NO -> GND, INPUT_PULLUP
//
// If it drives INTO the switch during homing instead of toward
// it, flip TOWARD_HOME_X. If it drives back toward the switch
// during the max-travel move instead of away from it, flip
// AWAY_FROM_HOME_X.
//
// Built for an 8MHz clock -- change F_CPU to 16000000UL if this
// chip is running off a 16MHz crystal instead. Everything else
// (UART divisor, delay_us_var) derives from F_CPU automatically.
// ============================================================

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>

// ---------------- X axis (PORTB / PORTD) ----------------
#define STEP_PIN_X         PB0
#define DIR_PIN_X          PB1
#define EN_PIN_X           PB2
#define LIMIT_PIN_X        PD2

#define STEPS_PER_MM       80.0   // 1/16 microstepping (MS1-3 all HIGH), 20T GT2 pulley
#define MAX_MM            275.0   // change to 290.0 if this is wired as the Y axis instead

#define HOMING_DELAY_US   1500    // slow speed while homing -- safer near hard-stops
#define RUN_DELAY_US       400    // normal speed for the max-travel move
#define BACKOFF_STEPS      100    // steps to back off after first contact

// Direction that drives the axis TOWARD the switch (used while homing)
#define TOWARD_HOME_X      true

// Direction that drives the axis AWAY from the switch (used to reach MAX_MM)
#define AWAY_FROM_HOME_X   (!TOWARD_HOME_X)

// ---- UART, 9600 baud (divisor derives from F_CPU) ----
static void uart_init(void) {
    uint16_t ubrr = (F_CPU / 16 / 9600UL) - 1;
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static void uart_putc(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

static void uart_puts(const char *s) {
    while (*s) uart_putc(*s++);
}

// ---- runtime microsecond delay (util/delay.h needs compile-time
//      constants, so loop 1us delays for runtime-variable timing) ----
static inline void delay_us_var(uint16_t us) {
    while (us--) _delay_us(1);
}

static inline void setDirX(bool towardHome) {
    if (towardHome) PORTB |= (1 << DIR_PIN_X);
    else            PORTB &= ~(1 << DIR_PIN_X);
}

static inline bool limitTriggered(void) {
    return (PIND & (1 << LIMIT_PIN_X)) == 0;   // pulled up, switch pulls to GND
}

static void stepPulse(uint16_t delayUs) {
    PORTB |= (1 << STEP_PIN_X);
    delay_us_var(delayUs);
    PORTB &= ~(1 << STEP_PIN_X);
    delay_us_var(delayUs);
}

// Drive toward the switch until triggered, back off, then
// re-approach slowly for a repeatable zero point.
static void homeAxis(void) {
    setDirX(TOWARD_HOME_X);
    while (!limitTriggered()) stepPulse(HOMING_DELAY_US);

    setDirX(AWAY_FROM_HOME_X);
    for (uint16_t i = 0; i < BACKOFF_STEPS; i++) stepPulse(HOMING_DELAY_US);

    setDirX(TOWARD_HOME_X);
    while (!limitTriggered()) stepPulse(HOMING_DELAY_US * 2);

    // this is now position 0
}

// Drive from 0 to MAX_MM
static void moveToMax(void) {
    long steps = (long)(MAX_MM * STEPS_PER_MM);

    setDirX(AWAY_FROM_HOME_X);
    for (long i = 0; i < steps; i++) stepPulse(RUN_DELAY_US);
}

int main(void) {
    DDRB |= (1 << STEP_PIN_X) | (1 << DIR_PIN_X) | (1 << EN_PIN_X);
    DDRD &= ~(1 << LIMIT_PIN_X);
    PORTD |= (1 << LIMIT_PIN_X);   // INPUT_PULLUP

    PORTB &= ~(1 << EN_PIN_X);     // active-low: LOW = driver enabled

    uart_init();

    uart_puts("Homing...\r\n");
    homeAxis();
    uart_puts("Homed (0). Moving to max...\r\n");

    _delay_ms(500);
    moveToMax();
    uart_puts("Reached max travel. Done.\r\n");

    while (1) {
        // done -- nothing to repeat. Reset the board to run again.
    }
}
