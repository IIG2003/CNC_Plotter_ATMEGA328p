#define F_CPU 16000000UL // 16 MHz Clock Speed
#include <avr/io.h>
#include <util/delay.h>

void pwm_init_pb3(void) {
    // Set PB3 (OC2A) as an output pin
    DDRB |= (1 << PB3);

    // Configure Timer2 for Fast PWM Mode 3 (Fixed TOP = 255)
    // TCCR2A: Clear OC2A on Compare Match, set OC2A at BOTTOM (Non-inverting)
    TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);
    
    // Prescaler = 1024
    // F_PWM = 16,000,000 / (1024 * 256) = 61.03 Hz (~16.38ms period)
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); 
}

void set_servo_angle_pb3(uint8_t angle) {
    // Total timer period is 16.38ms represented by 256 ticks (0.064ms per tick)
    // 1ms pulse ≈ 16 ticks (0 degrees)
    // 1.5ms pulse ≈ 23 ticks (90 degrees)
    // 2ms pulse ≈ 31 ticks (180 degrees)
    
    if (angle > 180) angle = 180; // Limit check
    
    // Formula maps 0-180 degrees across the 16 to 31 tick span
    OCR2A = 16 + (((uint16_t)angle * (31 - 16)) / 180);
}

int main(void) {
    pwm_init_pb3(); // Initialize Timer2 for PB3

    while (1) {
        set_servo_angle_pb3(0);   // Move to 0 degrees
        _delay_ms(1000);

        set_servo_angle_pb3(90);  // Move to 90 degrees
        _delay_ms(1000);

        set_servo_angle_pb3(180); // Move to 180 degrees
        _delay_ms(1000);
    }

    return 0;
}
