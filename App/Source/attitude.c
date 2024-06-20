//姿态控制任务
#include "attitude.h"
#include "bsp_usart.h"
#include "sensors.h"
#include "delay.h"
#include "pid.h"
#include "motor.h"
#include "bsp_pwm.h"
/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"

attitude_t attitude;

//姿态控制pid结构体
PidObject roll_pid;
PidObject pitch_pid;

//电机转速控制pid结构体
PidObject motor1_pid;
PidObject motor2_pid;

//姿态控制PID参数初始化
void attitude_pid_init(void)
{
    PID_Param_Init(&roll_pid,  0,   2, 0.0, 1.7, 50, 200);
    PID_Param_Init(&pitch_pid, 0, 100,   0, 0.1, 50, 10000);
}

void attitudeTask(void *parameter)
{
    float roll_out, pitch_out;
    //姿态控制允许误差
    float roll_relax = 0;
    float pitch_relax = 0;
    //姿态控制目标值
    float roll_target = 0;
    float pitch_target = 0;
    attitude_pid_init();
    motor_init();

    vTaskDelay(2000);  //等待传感器初始化完成
    while (1)
    {
        vTaskDelay(20); //控制频率为50Hz
        sensorsAcquire(&attitude);
    
        //姿态控制 外环角度环控制
        // roll_out = PosionPID_Realize(&roll_pid, attitude.roll);
        // pitch_out = PosionPID_Realize(&pitch_pid, attitude.pitch);
        pitch_out = AttitudePID_Realize(&pitch_pid, attitude.pitch, pitch_relax, -attitude.gyro_y);
        roll_out = AttitudePID_Realize(&roll_pid, attitude.roll, roll_relax, attitude.gyro_x);
        motor1_out(pitch_out);
        //内环电机转速控制
        motor1_pid.target_val = pitch_out;
        motor2_pid.target_val = roll_out;
        // motor1_speed();
        // motor2_speed();

        // printf("roll:%.2f, %.2f\n", attitude.roll, roll_pid.target_val);
        printf("pitch:%.2f, %.2f\n", attitude.pitch, pitch_pid.target_val);
        // printf("roll:%.2f, %.2f, %.2f\n", attitude.roll, attitude.pitch, attitude.yaw);
    }
    
}