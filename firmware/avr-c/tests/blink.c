#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

int main(void) {
    DDRB |= (1 << PB1);   // PB1 (physical pin 15) as output

    while (1) {
        PORTB ^= (1 << PB1);  // toggle LED
        _delay_ms(500);
    }
}
