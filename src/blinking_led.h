/* blinking_led.h */

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
