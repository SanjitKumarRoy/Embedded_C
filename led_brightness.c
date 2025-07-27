/* Controll LED Brightness */
/* led_brightness.c */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "./src/uart.h"
#include "./src/adc.h"
#include "./src/sys_spec.h"


ISR(ADC_vect) {
    char buffer[16];
	uint16_t prev_adc_value, curr_adc_value;
    float volt;
   
    curr_adc_value = ADC;
//	if(curr_adc_value > (prev_adc_value + 0.5 * prev_adc_value) || curr_adc_value < (prev_adc_value - 0.5 * prev_adc_value)){
	if(curr_adc_value != prev_adc_value){
		volt = ADC_TO_VOLT(curr_adc_value);
	
	    dtostrf(volt, 6, 2, buffer);
	
	    if (volt > 2.5) {
	        PORTD |= (1 << PD3); // LED ON
	        uart_print("LED ON");
	    } else {
	        PORTD &= ~(1 << PD3); // LED OFF
	        uart_print("LED OFF");
	    }
	
	    uart_print("\t Volt = ");
	    uart_print(buffer);
	    uart_print("\r\n");
	}
	prev_adc_value = curr_adc_value;

    ADCSRA |= (1 << ADSC);  // Start next conversion
}

int main(void) {
    DDRD |= (1 << PD3);     // Set PD3 as output
    adc_init();
    uart_init(MYUBRR);

    ADCSRA |= (1 << ADIE);  // Enable ADC interrupt
    sei();                  // Enable global interrupts
    ADCSRA |= (1 << ADSC);  // Start first conversion

    while (1) {
        // Main loop intentionally left empty
    }
}

