#ifndef _BSP_ADC_H__
#define	_BSP_ADC_H__

#include "gd32f4xx.h"

void adc_config(void);
unsigned int Get_ADC_Value(uint8_t  ADC_CHANNEL_x);

#endif