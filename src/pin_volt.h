/* pin_volt.h */

#ifndef PIN_VOLT_H 
#define PIN_VOLT_H

#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include "uart.h"
#include "sys_spec.h"

void init_port();
void adc_init();
uint16_t adc_read(uint8_t);
float get_voltage(uint8_t);
void pin_volt_main();

#endif
