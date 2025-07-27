/* Get ADC value and Pin voltage */
#include <stdlib.h>
#include <util/delay.h>
#include "./src/adc.h"
#include "./src/uart.h"

void main(){
	float volt;
	char buffer[10];

	DDRD |= (1 << PD3);	// Set pin PD3 as output
						//
	uart_init(MYUBRR);
	adc_init();

	while(1){
		volt = ADC_TO_VOLT(adc_read(PC0)); // PC) is for pin A0
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

		_delay_ms(2000);
	}
}
