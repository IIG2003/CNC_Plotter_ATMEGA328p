// ============================================================
// limit_test_8mhz.c  —  limit-switch checker, ATmega328P @ 8 MHz internal RC
//
// Prints the state of both limit inputs ~5x/sec over serial.
// Open (not pressed) = pull-up holds HIGH -> "open".
// Pressed            = switch pulls to GND -> "HIT".
//
// Wire each switch: COMMON -> GND, NORMALLY-OPEN -> the MCU pin.
//   X limit = PD2 (Arduino D2)
//   Y limit = PD3 (Arduino D3)
//
// Open a serial monitor at 9600 baud, press each switch by hand,
// and watch it flip open <-> HIT.
// ============================================================

#define F_CPU 8000000UL
#define BAUD  9600

#include <avr/io.h>
#include <util/delay.h>
#include <util/setbaud.h>

static void uart_init(void) {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= (uint8_t)~_BV(U2X0);
#endif
    UCSR0B = _BV(TXEN0);                     // TX only, no interrupts needed
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);      // 8N1
}

static void uart_putc(char c) {
    while (!(UCSR0A & _BV(UDRE0))) { }
    UDR0 = (uint8_t)c;
}

static void uart_puts(const char *s) { while (*s) uart_putc(*s++); }

int main(void) {
    uart_init();

    // limit inputs with internal pull-ups
    DDRD  &= (uint8_t)~(_BV(PD2) | _BV(PD3));
    PORTD |=  _BV(PD2) | _BV(PD3);

    uart_puts("limit test ready\r\n");

    for (;;) {
        uart_puts((PIND & _BV(PD2)) ? "X:open " : "X:HIT  ");
        uart_puts((PIND & _BV(PD3)) ? "Y:open\r\n" : "Y:HIT\r\n");
        _delay_ms(200);
    }
}
