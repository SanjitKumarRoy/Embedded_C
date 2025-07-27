#include <avr/io.h>
#include <util/delay.h>

void init_port(){
	DDRD |= (1 << PD3);
}

void toggle_led(){
	PORTD ^= (1 << PD3);  // Toggle PD2
}

void uart_init(void) {
    UBRR0L = 103; // 9600 baud @ 16MHz
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_send(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
        UDR0 = c;
}

void uart_print(const char* s) {
    while (*s) {
        uart_send(*s++);
    }
}

int main() {
	init_port();
    uart_init();
    while (1) {
		toggle_led();
		if(PORTD &= (1 << PD3)){
			uart_print("LED ON\n");
		}
		else{
			uart_print("LED OFF\n");
		}
        _delay_ms(2000);
    }
}
