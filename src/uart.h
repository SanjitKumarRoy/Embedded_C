/* uart.h */

#ifndef UART_H
#define UART_H

#include <avr/io.h>
#include "sys_spec.h"

void uart_init(unsigned int ubrr);
void uart_send(char data);
char uart_receive(void);
void uart_print(const char*);

#endif
