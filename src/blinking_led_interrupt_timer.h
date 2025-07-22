/* blinking_led_interrupt_timer.h */

#ifndef BLINKING_LED_INTERRUPT_TIMER_H
#define BLINKING_LED_INTERRUPT_TIMER_H

#include <avr/io.h>
#include <avr/interrupt.h>

void init_port();
void timer1_init();
void toggle_led();
void blinking_led_timer_main();

#endif
