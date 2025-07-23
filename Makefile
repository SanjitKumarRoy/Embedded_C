# Target microcontroller
MCU = atmega328p

# Clock frequency
F_CPU = 16000000UL

# Source and output names
SRC = main.c src/uart_basic.c
#SRC = blinking_led.c
#SRC = blink_uart_simulation.c
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

