/* Blinking LED */
/* blinking_led.c */

#include <avr/io.h>
#include <util/delay.h>

void init_port(){
	DDRD |= (1 << PD3);   // Set PD3 (D2) as output

	DDRD &= ~(1 << PD2);  // Set PD2 (D3) as input
	PORTD |= (1 << PD2);  // Enable pull-up resistor
}

void toggle_led(){
	PORTD ^= (1 << PD3);  // Toggle PD3
}

uint8_t switch_pressed(){ // Returns true if button is pressed (active low)
	if (!(PIND & (1 << PD2))) {
		_delay_ms(100); // debounce
		if (!(PIND & (1 << PD2))) {
			return 1;
		}
	}
	return 0;
}

void main(void) {
	init_port();
	while (1) {
		if (switch_pressed()) {
			toggle_led();
		}
	}
}
