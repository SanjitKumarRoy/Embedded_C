#ifndef SYS_SPEC_H 
#define SYS_SPEC_H 

#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1	// Here MYUBRR value is 103
#define VREF 5.0 // Reference voltage
#define ADC_TO_VOLT(adc) ((adc * VREF) / 1023.0) // ADC to Voltage conversion

#endif
