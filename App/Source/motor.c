#include "motor.h"
#include "bsp_pwm.h"
#include "bsp_usart.h"
#include "encoder.h"
/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"

uint32_t Motor1_Encoder_Data, Motor2_Encoder_Data;

void motorTask(void *parameter)
{
    motor_config();
    Motor_Encoder_Init();
    printf("motor_init\r\n");
    while (1)
    {
        MOTOR1_DIR=1;

	    motor1_out(5000);

        Motor1_Encoder_Data=Motor1_Encoder_Value();
        printf("motor1_encoder_value:%d\r\n",Motor1_Encoder_Data);
        vTaskDelay(1000); 
    }
    
}