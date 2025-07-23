/* uart_basic.h */

#ifndef UART_BASIC_H 
#define UART_BASIC_H

#include <avr/io.h>

void uart_init(unsigned int ubrr);
void uart_transmit(char data);
char uart_receive(void);
int uart_main(void);

#endif
