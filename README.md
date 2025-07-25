# Embedded C on ATmega328P (using AVR)

## Table of Contents
- [1 Install AVR and Run Embedded C Program](#1-install-avr-and-run-embedded-c-program)
    - [1.1 Install AVR](#11-install-avr)
    - [1.2 Write an Embedded C Program and run it](#12-write-an-embedded-c-program-and-run-it)
<!--
        - [1.2.1 Embedded C Program `blink.c`](#121-embedded-c-program-blinkc)
        - [1.2.2 Compile `blink.c` to run on ATmega328P](#122-compile-blinkc-to-run-on-atmega328p)
        - [1.2.3 Run with `simavr` simulator](#123-run-with-simavr-simulator)
        - [1.2.4 Run on (Flash to) ATmega328P (Arduino Uno)](#124-run-on-flash-to-atmega328p-arduino-uno)
-->
- [2 Create your own library](#2-create-your-own-library)
    - [2.1 Project Description: Blinking LED with Button - AVR (ATmega328P / Arduino Uno)](#21-project-description-blinking-led-with-button--avr-atmega328p--arduino-uno)
    - [2.2 Project Structure](#22-project-structure)
    - [2.3 Code:](#23-code)
        <!--
        - [2.3.1 `main.c`](#231-mainc)
        - [2.3.2 `./src/blinking_led.h`](#232-srcblinking_ledh)
        - [2.3.3 `./src/blinking_led.c`](#233-srcblinking_ledc)
        -->
    - [2.4 `Makefile`](#24-makefile)
        <!--
        - [2.4.1 Command to compile and run using `make`](#241-command-to-compile-and-run-using-make)
        -->
- [3 AVR Interrupts Cheat Sheet](#3-avr-interrupts-cheat-sheet)
    - [3.1 Enable Specific Interrupt](#31-enable-specific-interrupt)
    - [3.2 Configure Trigger Type](#32-configure-trigger-type)
    - [3.3 Enable Global Interrupts](#33-enable-global-interrupts)
    - [3.4 Write ISR](#34-write-isr)
    - [3.5 Common Interrupt Vectors](#35-common-interrupt-vectors)
    - [3.6 Important Registers](#36-important-registers)
    - [3.7 Timer Overflow](#37-timer-overflow)
        <!--
        - [3.7.1 Example (Timer1 - 16-bit)](#371-example-timer1---16-bit)
        - [3.7.2 Best Practices](#372-best-practices)
        -->
    - [3.8 Timer Overflow Calculation for 500ms Using Timer1](#38-timer-overflow-calculation-for-500ms-using-timer1)
        <!--
        - [3.8.1 Understand Timer Tick Time](#381-understand-timer-tick-time)
        - [3.8.2 Calculate Ticks Needed for 500 ms](#382-calculate-ticks-needed-for-500-ms)
        - [3.8.3 Preload `TCNT1` to Reach Overflow at 7813 Ticks](#383-preload-tcnt1-to-reach-overflow-at-7813-ticks)
        - [3.8.4 We can re-write it in another way](#384-we-can-re-write-it-in-another-way)
        -->
- [4 UART Communication Tutorial (AVR / ATmega328P)](#4-uart-communication-tutorial-avr--atmega328p)
    - [4.1 What is UART?](#41-what-is-uart)
    - [4.2 Registers Involved in UART (ATmega328P)](#42-registers-involved-in-uart-atmega328p)
        <!--
        - [4.2.1 `UBRR0H` and `UBRR0L` - Baud Rate Registers](#421-ubrr0h-and-ubrr0l---baud-rate-registers)
        - [4.2.2 `UCSR0A` - Control and Status Register A](#422-ucsr0a---control-and-status-register-a)
        - [4.2.3 `UCSR0B` - Control and Status Register B](#423-ucsr0b---control-and-status-register-b)
        - [4.2.4 `UCSR0C` - Control and Status Register C](#424-ucsr0c---control-and-status-register-c)
        - [4.2.5 UDR0 - UART Data Register](#425-udr0---uart-data-register)
        -->
    - [4.3 Full Example: Echo Program](#43-full-example-echo-program)
    - [4.4 Test UART with PC using `screen`](#44-test-uart-with-pc-using-screen)
        <!--
        - [4.4.1 Install `screen`](#441-install-screen)
        - [4.4.2 Create a Named Screen Session for `/dev/ttyACM0`](#442-create-a-named-screen-session-for-devttyacm0)
        - [4.4.3 List Screen Sessions](#443-list-screen-sessions)
        - [4.4.4 Close a Screen Session](#444-close-a-screen-session)
        -->

- [5 Analog to Digital Converter (ADC) on AVR](#5-analog-to-digital-converter-adc-on-avr)
    - [5.1 ADC Registers and Their Fields](#51-adc-registers-and-their-fields)
        <!--
        - [5.1.1 ADMUX (ADC Multiplexer Selection Register)](#511-admux-adc-multiplexer-selection-register)
        - [5.1.2 ADCSRA (ADC Control and Status Register A)](#512-adcsra-adc-control-and-status-register-a)
        -->
    - [5.2 ADC Basic Program](#52-adc-basic-program)
        <!--
        - [5.2.1 Initialization](#521-initialization)
        - [5.2.2 Reading a Pin](#522-reading-a-pin)
        - [5.2.3 Voltage Calculation](#523-voltage-calculation)
        - [5.2.4 Example Use Case](#524-example-use-case)
        -->
    - [5.3 ADC Clock and Prescaler Notes](#53-adc-clock-and-prescaler-notes)
    - [5.4 Important Notes](#54-important-notes)
    - [5.5 Working Principle Recap](#55-working-principle-recap)

---

## 1 Install AVR and Run Embedded C Program
<!-- ## Part 1: Simulate ATmega328P Code on Ubuntu (Using `simavr`) -->

### 1.1 Install AVR
```bash
sudo apt update
sudo apt install gcc-avr avr-libc binutils-avr avrdude #Install avr toolchains
sudo apt install simavr #(Optional) Install Emulator: simavr
```
**Confirm Installation**
```bash
avr-gcc --version
```

---

### 1.2 Write an Embedded C Program and run it
#### 1.2.1 Embedded C Program `blink.c`
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

#### 1.2.2 Compile `blink.c` to run on ATmega328P
```bash
avr-gcc -mmcu=atmega328p -DF_CPU=16000000UL -Os -o blink.elf blink.c
```

#### 1.2.3 Run with `simavr` simulator
```bash
simavr -m atmega328p -f 16000000 blink.elf
```

---

#### 1.2.4 Run on (Flash to) ATmega328P (Arduino Uno)
```bash
avr-objcopy -O ihex blink.elf blink.hex #Convert ELF to HEX
avrdude -c arduino -p m328p -P /dev/ttyUSB0 -b 115200 -U flash:w:blink.hex #Upload to Arduino Uno
```
Replace `/dev/ttyUSB0` with your actual serial port, if different.

**Command to find the Serial Port**
```bash
ls /dev/ttyUSB*
# or
ls /dev/ttyACM*
```
---

## 2 Create your own library
### 2.1 Project Description: Blinking LED with Button - AVR (ATmega328P / Arduino Uno)
This project demonstrates a basic embedded C application on an AVR microcontroller using `avr-gcc`.
It toggles an LED connected to digital pin `D2` when a button connected to digital pin `D3` is pressed.
### 2.2 Project Structure
```bash
├── main.c                   # Entry point of the program
└── src/
    ├── blinking_led.c       # LED control and switch logic implementation
    └── blinking_led.h       # Header file with function declarations
```

### 2.3 Code:
#### 2.3.1 `main.c`
```c
/* main.c */
/* Blinking LED */

#include "./src/blinking_led.h"

int main(){
	blinking_led_main();
}
```
#### 2.3.2 `./src/blinking_led.h`
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
#### 2.3.3 `./src/blinking_led.c`
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
### 2.4 `Makefile`
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

#### 2.4.1 Command to compile and run using `make`
```bash
make #Compile the Embedded C Program 
make flash #Run on Arduino
make simulate #Run on simavr simulator
```
**Note:** Update the PORT in the `Makefile` if necessary (e.g., `/dev/ttyUSB0`, `/dev/ttyACM0`, etc.)

---


## 3 AVR Interrupts Cheat Sheet
### 3.1 Enable Specific Interrupt
Set the appropriate bit in a control register:
- External: `EIMSK |= (1 << INT0);`
- Timer: `TIMSK1 |= (1 << TOIE1);` // Timer1 overflow

### 3.2 Configure Trigger Type
For external interrupts (`INT0`, `INT1`):
```c
EICRA |= (1 << ISC01);  // Falling edge on INT0
```
|ISC01  |ISC00  |Trigger Type   |
|---    |---    |---            |
|0      |0      |Low Level      |
|0      |1      |Any change     |
|1      |0      |Falling edge   |
|1      |1      |Rising edge    |

---

### 3.3 Enable Global Interrupts
```c
sei();   // Set Global Interrupt Enable bit
cli();   // Clear (disable) if needed
```

### 3.4 Write ISR
```c
ISR(INT0_vect) {
    // Your interrupt code
}
```
- `ISR` names must match vector name (e.g., `INT0_vect`, `TIMER1_OVF_vect`).
- Keep `ISR` short and fast.

### 3.5 Common Interrupt Vectors
|Vector Name        |Trigger Event              |
|---                |---                        |
|`INT0_vect`        |External `INT0` (PD2)      |
|`INT1_vect`        |External `INT1` (PD3)      |
|`TIMER0_OVF_vect`  |`Timer0` overflow          |
|`TIMER1_OVF_vect`  |`Timer1` overflow          |
|`TIMER2_COMPA_vect`|`Timer2` compare match A   |
|`ADC_vect`         |ADC conversion complete    |
|`USART_RX_vect`    |USART receive complete     |

---

### 3.6 Important Registers
|Register   |Description                                    |
|---        |---                                            |
|`EIMSK`    |Enable `INT0`, `INT1`                          |
|`EICRA`    |Set edge type (ISC bits)                       |
|`EIFR`     |Flag bits for external interrupts              |
|`TIMSKx`   |Enable Timer interrupts                        |
|`TIFRx`    |Flag bits for Timer interrupts                 |
|`GICR/SREG`|Global interrupt control (`SREG` holds `I` bit)|

---

### 3.7 Timer Overflow
#### 3.7.1 Example (Timer1 - 16-bit)
```c
/* Blinking LED using Timer Interrupt */
/* blinking_led_timer_interrupt.c */

#include <avr/io.h>
#include <avr/interrupt.h>

#define _500ms 57723;

void init_port(){
	DDRD |= (1 << PD3);		// Set PD3 (D3) as output
}

void timer1_init(){
	TCCR1A = 0x00;							// Set Timer1 to normal mode
	TCCR1B |= (1 << CS12) | (1 << CS10);	// Set prescaler to 1024
	TIMSK1 |= (1 << TOIE1);					// Enable Timer1 overflow interrupt
	TCNT1 = _500ms;							// Initialize counter; Overflow occur at every 500 ms
	sei();									// Enable global interrupts
}

ISR(TIMER1_OVF_vect){
	PORTD ^= (1 << PD3);  					// Toggle LED
	TCNT1 = _500ms; 						// Reload for next 500 ms
}

void main(void){
	init_port();
	timer1_init();
	while(1){
		// Main loop does nothing, CPU is free
	}
}
```
#### 3.7.2 Best Practices
- Use `volatile` keyword for variables shared between ISR and main code:
```c
volatile uint8_t flag = 0;
```
- Avoid `delay()` inside ISRs.
- Avoid long loops or heavy computation inside ISRs.

---

### 3.8 Timer Overflow Calculation for 500ms Using Timer1
**Hardware Specifications Assumed:** <br>
Microcontroller: **ATmega328P** <br>
System Clock: **16 MHz** <br>
Timer: **Timer1 (16-bit)** <br>
Timer Mode: **Normal mode** <br>
Goal: Overflow every **500 ms**

---

#### 3.8.1 Understand Timer Tick Time
When you use a **prescaler**, each timer tick takes more time.
- **Prescaler** = `1024`  
- **CPU Frequency (F_CPU)** = `16,000,000 Hz`
```ini
Tick Time = Prescaler / F_CPU = 1024 / 16,000,000 = 64 μs
```

#### 3.8.2 Calculate Ticks Needed for 500 ms
```ini
500 ms = 500,000μs
Ticks needed = 500,000/64 = 7812.5 ≈ 7813
```

#### 3.8.3 Preload `TCNT1` to Reach Overflow at 7813 Ticks
Since `Timer1` overflows at 65536 ticks, you must preload it to:
```ini
65536 - 7813 = 57723
```
So, preload:
```c
TCNT1 = 57723;
```
This means the timer will count from 57723 to 65535, which is exactly 7813 ticks = **500 ms**.

#### 3.8.4 We can re-write it in another way
```ini
500 ms = {(2^16 - TCNT1) * 1024} / 16,000,000
=> 0.5 = {(65536 - TCNT1) * 1024} / 16,000,000
=> (0.5 * 16,000,000)/1024 = 65536 - TCNT1
=> TCNT1 = 65536 - 7813 = 57723
```

## 4 UART Communication Tutorial (AVR / ATmega328P)
### 4.1 What is UART?
**UART (Universal Asynchronous Receiver/Transmitter)** is a hardware-based serial communication protocol that allows two devices to exchange data one bit at a time over a single line (TX for transmit, RX for receive), without requiring a clock signal.

---

### 4.2 Registers Involved in UART (ATmega328P)
Here’s a breakdown of the most relevant **UART registers** and their key **bit fields**.
#### 4.2.1 `UBRR0H` and `UBRR0L` - Baud Rate Registers
These two registers form a 16-bit value used to set the **baud rate.**
- Formula:
	```ini
	UBRR = (F_CPU / (16 * BAUD)) - 1
	```
- Example (F_CPU = 16 MHz, BAUD = 9600):
	```ini
	UBRR = (16000000 / (16 * 9600)) - 1 = 103
	```
- High byte: `UBRR0H = (UBRR >> 8)`
- Low byte: `UBRR0L = UBRR`

#### 4.2.2 `UCSR0A` - Control and Status Register A
|Bit    |Name   |Description                                    |
|---    |---    |---                                            |
|7      |`RXC0` |Receive Complete (1 when data is ready to read)|
|6      |`TXC0` |Transmit Complete                              |
|5      |`UDRE0`|Data Register Empty (1 if ready to transmit)   |
|4 - 0  |Others |Advanced options (not needed for basic UART)   |

---

**Example:**
```c
while (!(UCSR0A & (1 << UDRE0))); // Wait until buffer is empty
```

#### 4.2.3 `UCSR0B` - Control and Status Register B
|Bit|Name       |Description                            |
|---|---        |---                                    |
|7  |`RXCIE0`   |RX Complete Interrupt Enable           |
|6  |`TXCIE0`   |TX Complete Interrupt Enable           |
|5  |`UDRIE0`   |Data Register Empty Interrupt Enable   |
|4  |`RXEN0 `   |Receiver Enable                        |
|3  |`TXEN0 `   |Transmitter Enable                     |
|2  |`UCSZ02`   |Character Size bit 2                   |
|1  |`RXB80 `   |Receive Data Bit 8 (for 9-bit mode)    |
|0  |`TXB80 `   |Transmit Data Bit 8 (for 9-bit mode)   |

---
**Example:**
```c
UCSR0B = (1 << RXEN0) | (1 << TXEN0); // Enable RX and TX
```

#### 4.2.4 `UCSR0C` - Control and Status Register C
|Bits   |Name           |Description                                |
|---    |---            |---                                        |
|7 - 6  |`UMSEL0[1:0]`  |Mode Select (00 for async)                 | 
|5 - 4  |`UPM0[1:0]  `  |Parity Mode                                |
|3      |`USBS0      `  |Stop Bit Select (0 = 1-bit, 1 = 2-bit)     |
|2 - 1  |`UCSZ0[1:0] `  |Character Size (e.g., 8-bit = 11)          |
|0      |`UCPOL0     `  |Clock Polarity (only in synchronous mode)  |

---
**Example:**
```c
UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8-bit data, 1 stop bit
```

#### 4.2.5 UDR0 - UART Data Register
- Used to **transmit** and **receive** one byte of data.
- When writing: sends data.
- When reading: receives data.

### 4.3 Full Example: Echo Program
```c
#include <avr/io.h>

#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR (F_CPU / 16 / BAUD - 1)

void uart_init(unsigned int ubrr) {
    UBRR0H = (ubrr >> 8);         // Set baud rate high byte
    UBRR0L = ubrr;                // Set baud rate low byte
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);     // Enable receiver and transmitter
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);   // 8 data bits, 1 stop bit
}

void uart_transmit(char data) {
    while (!(UCSR0A & (1 << UDRE0))); // Wait until buffer is empty
    UDR0 = data;                      // Send data
}

char uart_receive(void) {
    while (!(UCSR0A & (1 << RXC0))); // Wait until data is received
    return UDR0;                     // Return received byte
}

int main(void) {
    uart_init(MYUBRR);               // Initialize UART

    while (1) {
        char c = uart_receive();     // Wait and receive character
        uart_transmit(c);           // Echo back
    }
}
```

### 4.4 Test UART with PC using `screen`
#### 4.4.1 Install `screen`
```bash
sudo apt install screen 
```
<!--
Use a serial to create a screen:
```bash
screen /dev/ttyACM0 9600
# or
minicom -b 9600 -D /dev/ttyACM0
```
Use Arduino Serial Monitor with 9600 baud to see echo behavior.
-->

#### 4.4.2 Create a Named Screen Session for `/dev/ttyACM0`
```bash
screen -S arduino /dev/ttyACM0 9600
```
- `-S arduino` - gives the screen session a name (arduino in this case).
- `/dev/ttyACM0` - your Arduino's serial port.
- `9600` - the baud rate (must match the one in your Arduino sketch).

#### 4.4.3 List Screen Sessions
```bash
screen -ls
```
**Example output:**
```bash
There is a screen on:
    12345.arduino    (Attached)
```
<!--
### Reconnect to a Named Session
```bash
screen -r arduino
```
-->
#### 4.4.4 Close a Screen Session
```bash
screen -X -S 12345.arduino quit
```
---


<!--
### Summary Table
|Register	|Function           | 
|---|---|
|`UBRR0H/L`	|Baud rate          | 
|`UCSR0A  ` |Buffer status flags| 
|`UCSR0B  ` |Enable TX/RX       | 
|`UCSR0C  ` |Frame settings     | 
|`UDR0	  ` |Send/receive byte  | 
---
-->


## 5 Analog to Digital Converter (ADC) on AVR
An ADC (Analog-to-Digital Converter) is an electronic component (or part of a microcontroller) that converts analog voltage signals (continuous) into digital values (discrete numbers) that a computer or microcontroller can understand and process.
It is used to read analog sensors (e.g., temperature, light, potentiometers, etc.).

In the Context of AVR (e.g., ATmega328P):
- The ADC takes an input voltage (typically between 0V and a reference voltage like 5V) and converts it into a 10-bit digital value.
- This value ranges from 0 (for 0V) to 1023 (for V<sub>REF</sub>), because 2¹⁰ = 1024 possible values.

**Example:**
If V<sub>REF</sub> is 5V:
- 0 V input → ADC value = 0
- 2.5 V input → ADC value ≈ 512
- 5.0 V input → ADC value = 1023

### 5.1 ADC Registers and Their Fields
|Register   |Fields	        |Description                         | 
|---        |---            |---                                 |  
|ADMUX	    |REFS1, REFS0	|Reference voltage selection         | 
|	        |ADLAR          |Left Adjust Result (for 8-bit reads)| 
|	        |MUX3..0	    |Select ADC input pin (ADC0 - ADC7)  | 
|ADCSRA	    |ADEN	        |ADC Enable                          | 
|	        |ADSC	        |Start Conversion                    | 
|	        |ADATE	        |Auto Trigger Enable                 | 
|	        |ADIF	        |Interrupt Flag                      | 
|	        |ADIE	        |Interrupt Enable                    | 
|	        |ADPS2..0	    |Prescaler Select bits               | 
|ADCH/ADCL	|10-bit result	|ADC data (high/low bytes)           | 

---
#### 5.1.1 ADMUX (ADC Multiplexer Selection Register)
**Purpose:**
Configures input pin, reference voltage, and result format (left/right adjustment).

**Bit-wise Breakdown:**
|Bit(s) |Name	|Function                          | 
|---    |---    |---                               |
|7      |REFS1	|Reference Selection Bit 1         | 
|6      |REFS0	|Reference Selection Bit 0         | 
|5      |ADLAR	|ADC Left Adjust Result            | 
|3:0	|MUX3:0	|Analog Channel and Gain Selection | 

---

**Reference Voltage Selection (REFS1:0):**<br>
These bits select the voltage against which the input voltage is measured (V<sub>REF</sub>):

|REFS1	|REFS0	|Reference              |
|---    |---    |---                    |
|0	    |0	    |AREF pin (external)    |
|0	    |1	    |AVcc (typically 5V)    |
|1	    |0	    |Reserved               |
|1	    |1	    |Internal 1.1V reference|

---
**Most common:**<br>
`(1 << REFS0)` → use AVcc (5V)

**Channel Selection (MUX3:0)**<br>
The MUX3:0 bits in the `ADMUX` register select which analog input pin (ADC0 - ADC7) to read from.

**MUX3..0 → Input Pin**

|MUX3..0|Input Pin	        |Pin  |    
|---    |---                |---  |
|`0000` |ADC0	            |PC0  |
|`0001` |ADC1	            |PC1  |
|`0010` |ADC2	            |PC2  |
|`0011` |ADC3	            |PC3  |
|`0100` |ADC4	            |PC4  |
|`0101` |ADC5	            |PC5  |
|`0110` |ADC6 (if available)|     | 
|`0111` |ADC7 (if available)|     | 

---
On ATmega328P, ADC6 and ADC7 are only available in TQFP or QFN packages (not on DIP packages like Arduino Uno).

**To set the pin:**
```c
ADMUX = (ADMUX & 0xF0) | (pin & 0x0F);
```
**This:**
- Preserves the upper bits (REFS and ADLAR)
- Sets the lower 4 bits to select the ADC input



#### 5.1.2 ADCSRA (ADC Control and Status Register A)
**Purpose:** <br>
Starts conversions, sets prescaler, enables ADC, and handles interrupt behavior.

**Bit-wise Breakdown:**

|Bit(s)	|Name	    |Function                       |
|---    |---        |---                            |
|7	    |ADEN	    |ADC Enable                     | 
|6	    |ADSC	    |Start Conversion               |
|5	    |ADATE	    |Auto Trigger Enable            |
|4	    |ADIF	    |Interrupt Flag                 |
|3	    |ADIE	    |Interrupt Enable               |
|2 - 0	|ADPS2:0    |ADC Prescaler (clock divider)  |

---

**Enable ADC (ADEN):**
```c
ADCSRA |= (1 << ADEN);  // Turn on ADC hardware
```

**Start Conversion (ADSC):**
```c
ADCSRA |= (1 << ADSC);  // Starts a new conversion
```
`ADSC` remains 1 while conversion is running.
It becomes 0 when done.

### 5.2 ADC Basic Program 
#### 5.2.1 Initialization
```c
void adc_init(){
	ADMUX = (1 << REFS0);	// Use AVcc (5V) as voltage reference
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Enable ADC with prescaler 64
}
```

**Explanation:**
```ini
REFS0 = 1 → VREF = AVcc
ADEN = 1 → Enable ADC
Prescaler = 64 → ADC clock = F_CPU/64 = 250 kHz (within recommended range)
```

#### 5.2.2 Reading a Pin 
```c
uint16_t adc_read(uint8_t pin) {
    ADMUX = (ADMUX & 0xF0) | (pin & 0x0F);  // Select ADC pin
    ADCSRA |= (1 << ADSC);                  // Start conversion
    while (ADCSRA & (1 << ADSC));           // Wait for conversion to complete
    return ADC;                             // Read 10-bit result from ADCL/ADCH
}
```

#### 5.2.3 Voltage Calculation
```c
#define VREF 5.0  // Reference voltage (AVcc)

float get_voltage(uint8_t pin) {
    uint16_t result = adc_read(pin);    // Get ADC value
    return (result * VREF) / 1023.0;    // Convert ADC value to voltage
}
```

#### 5.2.4 Example Use Case
 Read Pin `A0` and Control LED
```c
int main(void) {
    DDRD |= (1 << PD3);  // Set PD3 as output (LED)
    adc_init();

    while (1) {
        float voltage = get_voltage(PC0);  // Pin A0
        if (voltage > 2.5) {
            PORTD |= (1 << PD3);  // LED ON
        } else {
            PORTD &= ~(1 << PD3); // LED OFF
        }
    }
}
```

### 5.3 ADC Clock and Prescaler Notes
To ensure accurate results, the ADC clock should be between 50 kHz and 200 kHz:

|Prescaler	|ADC Clock @ 16MHz|Accepted?|
|---        |---              |---      |
|2	        |8 MHz (too fast) |&#10008; |    
|4	        |4 MHz            |&#10008; |  
|8	        |2 MHz            |&#10008; |  
|16	        |1 MHz            |&#10008; |  
|32	        |500 kHz          |&#10008; |  
|64	        |250 kHz          |&#10004; |
|128        |125 kHz          |&#10004; | 

---
Use `ADPS2:0` in `ADCSRA` to set prescaler.

### 5.4 Important Notes
- ADC result is in ADCL and ADCH (read ADCL first!).
- ADC input voltage must not exceed V<sub>REF</sub>.
- Floating ADC pins can cause unstable readings.
- Use AVcc (5V) or internal 1.1V reference if available.


### 5.5 Working Principle Recap:
- Configure `ADMUX` to select reference voltage and input channel.
- Enable the **ADC** using **ADEN** in `ADCSRA`.
- Start a conversion using **ADSC**.
- Wait until **ADSC** becomes 0.
- Read 10-bit result from **ADC** (combined **ADCL** & **ADCH**).
- Optionally use interrupts or auto-triggering for continuous conversions.

---




<!--
## 6 Project: LED Brightness Control Using Sensor Input
#### 6.1 Objective:
Control the brightness of an LED connected to a PWM pin based on an analog sensor reading (e.g., from a potentiometer or photoresistor). The sensor input is read by the ADC and mapped to PWM duty cycle.
Components Needed

- AVR microcontroller (ATmega328P or Arduino Uno)
- LED with current limiting resistor (~220Ω)
- Analog sensor (potentiometer or photoresistor)
- Breadboard and jumper wires
- Power supply (5V)

#### 6.2 Concept
- Read analog sensor voltage (0-5V) via ADC (10-bit resolution: 0–1023).
- Map ADC value to PWM duty cycle (0–255 for 8-bit PWM).
- Output PWM signal to LED pin to control brightness.

#### 6.2 Code Overview (in Embedded C)
```c
#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 16000000UL

void adc_init() {
    ADMUX = (1 << REFS0); // AVcc reference, ADC0 channel
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC, prescaler=128
}

uint16_t adc_read(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); // select ADC channel
    ADCSRA |= (1 << ADSC); // start conversion
    while (ADCSRA & (1 << ADSC)); // wait until done
    return ADC;
}

void pwm_init() {
    // Set Fast PWM mode, non-inverting, prescaler=64, OC0A (Pin PD6) as output
    DDRD |= (1 << PD6);  // PD6 as output (OC0A)
    TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00); // Fast PWM, clear OC0A on compare match
    TCCR0B = (1 << CS01) | (1 << CS00); // prescaler = 64
}

void set_pwm_duty(uint8_t duty) {
    OCR0A = duty; // Set duty cycle (0-255)
}

int main(void) {
    adc_init();
    pwm_init();

    while (1) {
        uint16_t sensor_val = adc_read(0); // read ADC0 (analog pin 0)
        uint8_t duty = sensor_val / 4; // map 0-1023 to 0-255
        set_pwm_duty(duty);
        _delay_ms(10);
    }
}
```

Explanation

    ADC Setup:

        Uses AVcc as voltage reference.

        ADC clock prescaler 128 for proper conversion speed.

        Reads analog input on channel 0.

    PWM Setup:

        Uses Timer0 Fast PWM on OC0A (Pin PD6 / Arduino pin 6).

        Non-inverting mode for LED brightness control.

        8-bit PWM, duty cycle set by OCR0A register.

    Control Logic:

        Sensor ADC value (0-1023) converted to PWM duty (0-255).

        PWM duty changes LED brightness proportionally.

Wiring
Microcontroller Pin	Component
ADC0 (PC0 / A0)	Analog sensor (potentiometer middle pin)
PD6 (OC0A / Arduino D6)	LED + resistor to GND
Extensions / Improvements

    Use interrupts for ADC conversion completion.

    Implement smoothing/filtering on ADC input.

    Use multiple sensors for multi-LED brightness control.

    Use UART to send sensor and PWM data for monitoring.
-->
