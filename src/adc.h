/* adc.h */

#ifndef ADC_H
#define ADC_H

#include <avr/io.h>
#include "sys_spec.h"

void adc_init();
uint16_t adc_read(uint8_t);

#endif
