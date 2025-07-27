/* uart.c */

#include "uart.h"

void uart_init(unsigned int ubrr) {
	UBRR0H = (ubrr >> 8);   // Set baud rate high byte
	UBRR0L = ubrr;          // Set baud rate low byte
	UCSR0B = (1 << TXEN0) | (1 << RXEN0);  // Enable TX and RX
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8 data bits, 1 stop bit
}

void uart_send(char c) {
	while (!(UCSR0A & (1 << UDRE0)));  // Wait for empty buffer
	UDR0 = c;  // Send data
}

char uart_receive(void) {
	while (!(UCSR0A & (1 << RXC0)));  // Wait for data to be received
	return UDR0; // Send received data
}

void uart_print(const char* s){
	while(*s){
		uart_send(*s);
		s++;
	}
}
