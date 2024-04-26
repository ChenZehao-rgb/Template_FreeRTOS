#include "motor.h"
#include "bsp_pwm.h"
#include "bsp_usart.h"
#include "encoder.h"
#include "pid.h"
/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"

PidObject PosionPID;
int16_t speed;

extern int circle_count; //定时器中断计电机圈数
int current_circle, last_circle=0;
float per_circle, velocity;
int dt=20; //用于计算速度
uint32_t last_cnt=0, current_cnt=0;
int32_t cnt_diff=0;

static float get_motor_velocity(int32_t cnt_diff, int dt, double line)
{
    return cnt_diff/dt/line*1000;   // circle/s
}
void motorTask(void *parameter)
{
    int target = 180;
    PID_Param_Init(&PosionPID, target, 19, 1, 0.5, 5000, 5000);

    float pid_temp[3] = {PosionPID.Kp, PosionPID.Ki, PosionPID.Kd};
    

    
    printf("motor_init\r\n");
    while (1)
    {
        current_cnt=Motor1_Encoder_Value();
        cnt_diff = current_cnt - last_cnt;
        current_circle = circle_count;
        
        cnt_diff+=(current_circle-last_circle)*396;
        
        // velocity = get_motor_velocity(cnt_diff, dt, 396.0); // circle/s
        speed = PosionPID_Realize(&PosionPID, cnt_diff);
        motor1_out(speed);

        last_circle = current_circle;
        last_cnt = current_cnt;
        // per_circle=(float)Motor1_Encoder_Data/396.0;
        
        // printf("velocity:%d, speed:%d\r\n", cnt_diff, speed);
        // printf("motor1_encoder_value:%d circle_count:%d\r\n",Motor1_Encoder_Data, circle_count);
        printf("real:%d, %.2f, %d\n", cnt_diff, PosionPID.target_val, speed);
        vTaskDelay(dt);
    }
    
}

//设置目标值
void set_pid_target(float temp_val)
{
	PosionPID.target_val = temp_val;
}

//获取目标值
float get_pid_target(void)
{
	return PosionPID.target_val;
}

//设置pid参数
void set_pid(float kp, float ki, float kd)
{
	PosionPID.Kp = kp;
	PosionPID.Ki = ki;
	PosionPID.Kd = kd;
}

void set_kp(float kp)
{
    PosionPID.Kp = kp;
}

void set_ki(float ki)
{
    PosionPID.Ki = ki;
}

void set_kd(float kd)
{
    PosionPID.Kd = kd;
}