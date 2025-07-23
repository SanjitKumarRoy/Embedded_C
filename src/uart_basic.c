/* uart_basic.c */

#include "uart_basic.h"

#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1	// Here MYUBRR value is 103

void uart_init(unsigned int ubrr) {
	UBRR0H = (ubrr >> 8);   // Set baud rate high byte
	UBRR0L = ubrr;          // Set baud rate low byte
	UCSR0B = (1 << TXEN0) | (1 << RXEN0);  // Enable TX and RX
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8 data bits, 1 stop bit
}

void uart_transmit(char data) {
	while (!(UCSR0A & (1 << UDRE0)));  // Wait for empty buffer
	UDR0 = data;  // Send data
}

char uart_receive(void) {
	while (!(UCSR0A & (1 << RXC0)));  // Wait for data to be received
	return UDR0;
}

int uart_main(void) {
	uart_init(MYUBRR);
	while (1) {
		char received = uart_receive();
		uart_transmit(received);  // Echo back
		uart_transmit(received);  // Echo back
	}
}

