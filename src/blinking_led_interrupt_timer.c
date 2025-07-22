/* Blinking LED using Timer Interrupt */
/* blinking_led_interrupt_timer.c */
/* Toggle LED at 500 ms interval */

#include "blinking_led_interrupt_timer.h"
#include "global_variables.h"


//#define _250ms 61630;
#define _500ms 57723;
//#define _1s 49911;

void init_port(){
	DDRD |= (1 << PD3);		// Set PD3 (D3) as output
}

void timer1_init(){
	TCCR1A = 0x00;							// Set timer 1 to normal mode
	TCCR1B |= (1 << CS12) | (1 << CS10);	// Set prescaler to 1024
	TIMSK1 |= (1 << TOIE1);					// Enable Timer1 overflow interrupt
	TCNT1 = _500ms;							// Initialize counter; Overflow occur at every 500 ms
	sei();									// Enable global interrupts
}

ISR(TIMER1_OVF_vect){
	toggle_led();							// Toggle LED
	TCNT1 = _500ms; 						// Reload for next 500 ms
}

void toggle_led(){
	PORTD ^= (1 << PD3);  					// Toggle PD3
}

void blinking_led_timer_main(void) {
	init_port();
	timer1_init();
	while (1) {
		// Main loop does nothing, CPU is free
	}
}
