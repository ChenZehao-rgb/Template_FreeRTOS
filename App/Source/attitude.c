//姿态控制任务
#include "attitude.h"
#include "bsp_usart.h"
#include "sensors.h"
#include "delay.h"
#include "pid.h"
#include "motor.h"
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
    PID_Param_Init(&roll_pid, 0, 0.5, 0.1, 0.1, 0, 0);
    PID_Param_Init(&pitch_pid, 0, 0.5, 0.1, 0.1, 0, 0);
}

void attitudeTask(void *parameter)
{
    float roll_out, pitch_out;
    //姿态控制允许误差
    float roll_relax = 5.0;
    float pitch_relax = 5.0;
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
        roll_out = PosionPID_Realize(&roll_pid, attitude.roll);
        pitch_out = PosionPID_Realize(&pitch_pid, attitude.pitch);

        //内环电机转速控制
        motor1_pid.target_val = pitch_out;
        motor2_pid.target_val = roll_out;
        motor1_speed();
        motor2_speed();

        // printf("roll:%.2f pitch:%.2f yaw:%.2f\r\n", attitude.roll, attitude.pitch, attitude.yaw);
    }
    
}