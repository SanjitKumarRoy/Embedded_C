#ifndef GLOBAL_VARIABLE_H
#define GLOBAL_VARIABLE_H

#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1	// Here MYUBRR value is 103

volatile uint8_t flag = 0;

#endif
