#include "motor.h"
#include "bsp_pwm.h"
#include "bsp_usart.h"
#include "encoder.h"
#include "pid.h"
/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"

#define en_motor_speed 1

PidObject PosionPID;
PidObject SpeedPID;
int16_t cnt_speed;      //计算编码器差值

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

static void motor_speed(void)
{
    current_cnt = Motor1_Encoder_Value();   //获取当前编码器值
    cnt_diff = current_cnt - last_cnt;      //计算编码器差值
    current_circle = circle_count;          //获取当前圈数

    cnt_diff += (current_circle - last_circle) * 396;   //根据圈数补充编码器差值
    
    cnt_speed = PosionPID_Realize(&PosionPID, cnt_diff);    //编码器差值作为实际值，计算PID输出

    motor1_out(cnt_speed);
    // velocity = get_motor_velocity(cnt_diff, dt, 396.0); // circle/s
    
    last_circle = current_circle;
    last_cnt = current_cnt;

    printf("real:%d, %.2f, %d\n", cnt_diff, PosionPID.target_val, cnt_speed);
}

static void motor_angle(void)
{
    current_cnt = Motor1_Encoder_Value();   //获取当前编码器值
    
    int16_t cnt_angle = PosionPID_Realize(&PosionPID, current_cnt);    //编码器差值作为实际值，计算PID输出

    motor1_out(cnt_angle);

    // printf("real:%d, %.2f, %d\n", current_cnt, PosionPID.target_val, cnt_angle);
}

void motorTask(void *parameter)
{
    int target = 180;
    
    #ifndef en_motor_speed
        PID_Param_Init(&SpeedPID, target, 5, 1, 0.5, 500, 4500);
    #else
        PID_Param_Init(&PosionPID, target, 5, 1, 0.5, 500, 4500);
    #endif

    printf("motor_init\r\n");
    while (1)
    {
        #ifndef en_motor_speed
            motor_speed();
        #else
            motor_angle();
        #endif
        // per_circle=(float)Motor1_Encoder_Data/396.0;
        
        // printf("velocity:%d, speed:%d\r\n", cnt_diff, speed);
        // printf("motor1_encoder_value:%d circle_count:%d\r\n",Motor1_Encoder_Data, circle_count);
        
        vTaskDelay(dt);
    }
}



//设置目标值
void set_pos_pid_target(float temp_val)
{
	PosionPID.target_val = temp_val;
}
void set_speed_pid_target(float temp_val)
{
    SpeedPID.target_val = temp_val;
}

//获取目标值
float get_pos_pid_target(void)
{
	return PosionPID.target_val;
}
float get_speed_pid_target(void)
{
    return SpeedPID.target_val;
}

//设置pid参数
void set_pos_pid(float kp, float ki, float kd)
{
	PosionPID.Kp = kp;
	PosionPID.Ki = ki;
	PosionPID.Kd = kd;
}
void set_pos_kp(float kp)
{
    PosionPID.Kp = kp;
}
void set_pos_ki(float ki)
{
    PosionPID.Ki = ki;
}
void set_pos_kd(float kd)
{
    PosionPID.Kd = kd;
}

void set_speed_kp(float kp)
{
    SpeedPID.Kp = kp;
}
void set_speed_ki(float ki)
{
    SpeedPID.Ki = ki;
}
void set_speed_kd(float kd)
{
    SpeedPID.Kd = kd;
}