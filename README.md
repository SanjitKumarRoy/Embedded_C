# Embedded C on ATmega328P

## Table of Contents
- [Part I: Simulate ATmega328P Code on Ubuntu](#part-i-simulate-atmega328p-code-on-ubuntu-using-simavr)
- [Part II: Flash to ATmega328P (Arduino Uno)](#part-ii-flash-to-atmega328p-arduino-uno)
- [Notes](#notes)

---

## Part I: Simulate ATmega328P Code on Ubuntu (Using `simavr`)

### Step 1: Install Required Tools

#### 1.1 Update Package List
```bash
sudo apt update
```

#### 1.2 Install AVR Toolchain
```bash
sudo apt install gcc-avr avr-libc binutils-avr avrdude
```

#### 1.3 (Optional) Install Emulator: `simavr`
```bash
sudo apt install simavr
```

#### 1.4 Confirm Installation
```bash
avr-gcc --version
```

---

### Step 2: Write and Simulate an Embedded C Program

#### 2.1 Create `blink.c`
```c
/* Blinking LED */

#include <avr/io.h>
#include <util/delay.h>

int main(void) {
    DDRD |= (1 << PD2); // Set PD2 (pin 2 on Arduino Uno) as output, Data Direction Register D (DDRD)

    while (1) {
        PORTD ^= (1 << PD2); // Toggle PD2
        _delay_ms(500);      // 500 ms delay
    }
}
```

#### 2.2 Compile for ATmega328P
```bash
avr-gcc -mmcu=atmega328p -DF_CPU=16000000UL -Os -o blink.elf blink.c
```

#### 2.3 Run with `simavr`
```bash
simavr -m atmega328p -f 16000000 blink.elf
```

---

### Step 3: Optional - Print via UART in Simulation

```c
#include <avr/io.h>
#include <util/delay.h>

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

int main(void) {
    uart_init();
    while (1) {
        uart_print("LED Toggle\n");
        _delay_ms(1000);
    }
}
```

---

## Part II: Flash to ATmega328P (Arduino Uno)

### Step 1: Convert ELF to HEX
```bash
avr-objcopy -O ihex blink.elf blink.hex
```

### Step 2: Upload Using `avrdude`

#### 2.1 Find the Serial Port
```bash
ls /dev/ttyUSB*
# or
ls /dev/ttyACM*
```

#### 2.2 Upload to Arduino Uno
```bash
avrdude -c arduino -p m328p -P /dev/ttyUSB0 -b 115200 -U flash:w:blink.hex
```

> Replace `/dev/ttyUSB0` with your actual serial port if different.

---

## Notes

<!--
- The Arduino Uno uses the ATmega328P running at 16 MHz.
- Add your user to the `dialout` group to avoid permission issues:
  ```bash
  sudo usermod -aG dialout $USER
  ```
-->
- Use `-Os` to optimize for code size, which is important in embedded systems.
- `simavr` is great for logic-level testing but may not perfectly emulate real-world peripherals.

---

## Example: Make your own library
### Blinking LED with Button – AVR (ATmega328P / Arduino Uno):
This project demonstrates a basic embedded C application on an AVR microcontroller using `avr-gcc`.
It toggles an LED connected to digital pin `D2` when a button connected to digital pin `D3` is pressed.
### Project Structure
```bash
├── main.c                   # Entry point of the program
└── src/
    ├── blinking_led.c       # LED control and switch logic implementation
    └── blinking_led.h       # Header file with function declarations
```
### Code:
```c
/* main.c */
/* Blinking LED */

#include "./src/blinking_led.h"

int main(){
	blinking_led_main();
}
```

```c
/* blinking_led.h */
/* Blinking LED */

#ifndef BLINKING_LED_H
#define BLINKING_LED_H

#include <avr/io.h>
#include <util/delay.h>

void init_port();
void toggle_led();
uint8_t switch_pressed();
void turn_led_off();
void blinking_led_main();

#endif
```

```c
/* blinking_led.c */
/* Blinking LED */

#include "blinking_led.h"

void init_port(){
	DDRD |= (1 << PD2);   // Set PD2 (D2) as output

	DDRD &= ~(1 << PD3);  // Set PD3 (D3) as input
	PORTD |= (1 << PD3);  // Enable pull-up resistor
}

void toggle_led(){
	PORTD ^= (1 << PD2);  // Toggle PD2
	_delay_ms(500);       // Wait 500 ms
}

uint8_t switch_pressed(){ // Returns true if button is pressed (active low)
	if (!(PIND & (1 << PD3))) {
		_delay_ms(50); // debounce
		if (!(PIND & (1 << PD3))) {
			return 1;
		}
	}
	return 0;
}

void turn_led_off(){
	PORTD &= ~(1 << PD2);  // Ensure LED is OFF
}

void blinking_led_main(void) {
	init_port();
	while (1) {
		if (switch_pressed()) {
			toggle_led();
		} else {
			turn_led_off();
		}
	}
}
```

```bash
## Makefile ##

# Target microcontroller
MCU = atmega328p

# Clock frequency
F_CPU = 16000000UL

# Source and output names
SRC = main.c src/blinking_led.c
TARGET = program

# Compiler and flags
CC = avr-gcc
OBJCOPY = avr-objcopy
CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os

# Upload tool and port (update as needed)
PROGRAMMER = arduino
#PORT = /dev/ttyUSB0
PORT = /dev/ttyACM0
BAUD = 115200

all: $(TARGET).hex

$(TARGET).elf: $(SRC)
	$(CC) $(CFLAGS) -o $@ $^

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

flash: $(TARGET).hex
	avrdude -c $(PROGRAMMER) -p $(MCU) -P $(PORT) -b $(BAUD) -U flash:w:$(TARGET).hex

simulate: $(TARGET).elf
	simavr -m $(MCU) -f 16000000 $<

clean:
	rm -f *.elf *.hex
```

### Command to compile and runt the `Blinking LED` program on `Arduino Uno`
```bash
make #Compile the Blinking LED program
make flash # Flash to Arduino
```
Note: Update the PORT in the `Makefile` if necessary (e.g., `/dev/ttyUSB0`, `/dev/ttyACM0`, etc.)


