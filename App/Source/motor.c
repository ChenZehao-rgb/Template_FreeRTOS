#include "motor.h"
#include "bsp_pwm.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"



void motorTask(void *parameter)
{
    while (1)
    {
        MOTOR1_DIR=0;
	    motor1_out(5000);
        vTaskDelay(50);
    }
    
}