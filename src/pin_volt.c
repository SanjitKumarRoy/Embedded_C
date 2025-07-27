/* Read pin voltage (sensor output) with ADC
 * and print it using UART */

#include "pin_volt.h"

void init_port(){
	DDRD |= (1 << PD3); // Set PD3 (D3) as output
}

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

float get_voltage(uint8_t pin){
	uint16_t value = adc_read(pin); // Get ADC value
	return (value * VREF) / 1023.0; // Convert ADC value to voltage
}

void pin_volt_main(){
	float volt;
	char buffer[10];

	init_port();
	uart_init(MYUBRR);
	adc_init();

	while(1){
		volt = get_voltage(PC0); // Pin A0
		// Convert float to string with width = 6 and precision = 2
		dtostrf(volt, 6, 2, buffer);
		if(volt > 2.5){
			PORTD |= (1 << PD3); // LED at pin D3 is ON
			uart_print("LED ON");
		}
		else{
			PORTD &= ~(1 << PD3); // LED at pin D3 is ON
			uart_print("LED OFF");
		}
		uart_print("\t Volt = ");
		uart_print(buffer);
		uart_print("\r\n");

		_delay_ms(3000);
	}
}
