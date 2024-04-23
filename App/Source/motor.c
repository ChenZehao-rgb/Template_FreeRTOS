#include "motor.h"
#include "bsp_pwm.h"
#include "bsp_usart.h"
#include "encoder.h"
/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"

uint32_t Motor1_Encoder_Data, Motor2_Encoder_Data;
extern int circle_count, motor_dir_flag;
int current_circle, last_circle=0;
float per_circle, velocity;
int dt=2000; //用于计算速度
uint32_t last_cnt=0, current_cnt=0;
int32_t cnt_diff=0;
void motorTask(void *parameter)
{
    motor_config();
    Motor_Encoder_Init();
    printf("motor_init\r\n");
    while (1)
    {
        MOTOR1_DIR=1;

        current_cnt=Motor1_Encoder_Value();
        cnt_diff = current_cnt - last_cnt;
        current_circle = circle_count;
        
        cnt_diff+=(current_circle-last_circle)*396;
        
        velocity = (float)cnt_diff/dt/396.0*1000; // circle/s
        last_circle = current_circle;
        last_cnt = current_cnt;
        // per_circle=(float)Motor1_Encoder_Data/396.0;
	    motor1_out(7500); // 速度5000，1s 25圈
        printf("velocity:%.2f, circle_count:%d\r\n", velocity, current_circle);
        // printf("motor1_encoder_value:%d circle_count:%d\r\n",Motor1_Encoder_Data, circle_count);
        vTaskDelay(dt); 
    }
    
}