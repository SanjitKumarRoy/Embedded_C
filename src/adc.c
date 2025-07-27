/* Function to access ADC */
/* adc.c */

#include "adc.h"

void adc_init(){
	ADMUX = (1 << REFS0);	// Use AVcc (5V) as voltage reference, select ADC0 (pin PC0/A0)
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Enable ADC, prescaler = 64
}

uint16_t adc_read(uint8_t pin){
	ADMUX = (ADMUX & 0xF0) | (pin & 0x0F);	// Select pin 
	ADCSRA |= (1 << ADSC);	// Start conversion
	while(ADCSRA & (1 << ADSC));	// Wait for completion
	return ADC;	// Return ADC register value
}
