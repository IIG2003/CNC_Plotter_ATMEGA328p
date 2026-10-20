// ============================================================
// osccal_cal_8mhz.c  —  tune the internal 8 MHz RC for clean UART
//
// Sweeps OSCCAL around its factory value and prints a test line at
// each setting. Open picocom at 9600 8N1 and watch: most settings
// print garbage, but there will be a BAND of settings where the
// line is perfectly readable. Read the OSCCAL number in the middle
// of that clean band -- that's your value.
//
// Then put this as the FIRST line of main() in the real firmware,
// before uart_init():
//        OSCCAL = <that number>;
//
// Notes:
//  * Run the chip at a stable 5 V; RC frequency drifts with voltage
//    and temperature, so calibrate at the voltage you'll actually use.
//  * Changing OSCCAL nudges F_CPU slightly off 8 MHz on purpose --
//    that's what makes the *baud* line up. Steppers/servo don't care.
// ============================================================

#define F_CPU 8000000UL
#define BAUD  9600

#include <avr/io.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <stdint.h>

static void uart_init(void) {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= (uint8_t)~_BV(U2X0);
#endif
    UCSR0B = _BV(TXEN0);
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);      // 8N1
}

static void uart_putc(char c) {
    while (!(UCSR0A & _BV(UDRE0))) { }
    UDR0 = (uint8_t)c;
}

static void uart_puts(const char *s) { while (*s) uart_putc(*s++); }

static void uart_putu8(uint8_t v) {          // print 0..255 as decimal
    char b[3];
    int8_t i = 0;
    if (v == 0) { uart_putc('0'); return; }
    while (v) { b[i++] = (char)('0' + (v % 10)); v /= 10; }
    while (i--) uart_putc(b[i]);
}

int main(void) {
    uart_init();
    int16_t base = OSCCAL;                    // factory value loaded at reset

    for (;;) {
        for (int16_t v = base - 15; v <= base + 15; v++) {
            if (v < 0 || v > 255) continue;
            OSCCAL = (uint8_t)v;
            _delay_ms(50);                    // let the RC settle
            uart_puts("OSCCAL=");
            uart_putu8((uint8_t)v);
            uart_puts("  the quick brown fox 0123456789\r\n");
            _delay_ms(400);
        }
    }
}
