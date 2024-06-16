#ifndef __SYSTEM_H
#define __SYSTEM_H

//FreeRTOS相关文件
#include "FreeRTOS.h"
#include "task.h"

//项目头文件
#include "gd32f4xx.h"
#include "systick.h"
#include <stdio.h>
#include "stdlib.h"
#include "string.h"
#include "main.h"
#include "sys.h"
#include "delay.h"
//底层硬件驱动
#include "bsp_led.h"
#include "bsp_usart.h"
#include "bsp_key.h"
#include "bsp_pwm.h"
#include "bsp_basic_timer.h"
#include "bsp_dma.h"
#include "motor.h"
#include "encoder.h"
#include "attitude.h"
#include "sensors.h"

void systeminit(void);


#endif /* __SYSTEM_H */