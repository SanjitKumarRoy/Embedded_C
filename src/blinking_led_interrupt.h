/* blinking_led_interrupt_timer.h */

#ifndef BLINKING_LED_INTERRUPT_H
#define BLINKING_LED_INTERRUPT_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>

void init_port();
void init_interrupt();
void toggle_led();
void blinking_led_isr_main();

#endif
