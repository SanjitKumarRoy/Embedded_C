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
#include <avr/io.h>
#include <util/delay.h>

int main(void) {
    DDRB |= (1 << PB0); // Set PB0 as output

    while (1) {
        PORTB ^= (1 << PB0); // Toggle PB0
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