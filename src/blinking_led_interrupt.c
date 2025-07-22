/* Blinking LED using Interrupt */
/* blinking_led_interrupt.c */

#include "blinking_led_interrupt.h"
#include "global_variables.h"

void init_port(){
	DDRD |= (1 << PD3);		// Set PD3 (D3) as output
	PORTD |= (1 << PD2);	// Enable pull-up resistor on PD2 (INT0)
}

void init_interrupt(){
	EIMSK |= (1 << INT0);	//Enable INT0
	EICRA |= (1 << ISC01);	// Falling edge of INT0 generates interrupt
	sei();					// Enable global interrupts
}

ISR(INT0_vect){
	_delay_ms(50);			// Debounce delay
	flag = 1;				// ISR: set flag when button pressed, here flag is a global variable
}

void toggle_led(){
	PORTD ^= (1 << PD3);  // Toggle PD3
	flag = 0;
}

void blinking_led_isr_main(void) {
	init_port();
	init_interrupt();
	while (1) {
		if (flag) {
			toggle_led();
		}
	}
}
