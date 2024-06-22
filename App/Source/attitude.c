//姿态控制任务
#include "attitude.h"
#include "bsp_usart.h"
#include "sensors.h"
#include "delay.h"
#include "pid.h"
#include "motor.h"
#include "bsp_pwm.h"
#include "encoder.h"
/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"

attitude_t attitude;

//姿态控制pid结构体
PidObject Angle_x_pid;
PidObject Speed_x_pid;  //轮子驱动，前后平衡
PidObject Angle_y_pid;
PidObject Speed_y_pid;  //动量轮驱动，左右平衡

float zero_angle_x = 0; //机械零点坐标
float zero_angle_y = 0;

int speed_x = 0, speed_y = 0; //速度环输入值,电机编码器读数之差
float setpoint_x = 0, setpoint_y = 0; //速度环输出值
int angle_out_x = 0, angle_out_y = 0; //角度环输出值
//姿态控制PID参数初始化
void attitude_pid_init(void)
{
    PID_Param_Init(&Angle_x_pid, 0, 0, 
                                    0, 
                                    0, 0, 0);
    PID_Param_Init(&Speed_x_pid, 0, 0,
                                    0,
                                    0, 0, 0);
    PID_Param_Init(&Angle_y_pid, 0, 300,
                                    0,
                                    5, 0, 5000);
    PID_Param_Init(&Speed_y_pid, 0, -0.01,
                                    0,
                                    0, 0, 10);
}

void attitudeTask(void *parameter)
{
    uint32_t tick = 0; //时间戳
    
    
    attitude_pid_init();
    motor_init();

    vTaskDelay(2000);  //等待传感器初始化完成
    while (1)
    {
        vTaskDelay(20); //运行周期5ms

        sensorsAcquire(&attitude);

        speed_x = get_motor2_speed();
        setpoint_x = ctrl_speed_realize(&Speed_x_pid, zero_angle_x, speed_x);
        ctrl_angle_realize(&Angle_x_pid, setpoint_x, attitude.roll, attitude.gyro_x);
        // motor2_out(angle_out_x);
        
        speed_y = get_motor1_speed();
        setpoint_y = ctrl_speed_realize(&Speed_y_pid, zero_angle_y, speed_y);
        // setpoint_y = ctrl_speed_realize2(&Speed_y_pid, zero_angle_y, speed_y, Motor1_Encoder_Value());
        angle_out_y = ctrl_angle_realize(&Angle_y_pid, setpoint_y, attitude.pitch, attitude.gyro_y);
        motor1_out(angle_out_y);
            
        
        
        // printf("roll:%.2f, %.2f\n", attitude.roll, roll_pid.target_val);
        printf("pitch:%.2f, %.2f\n", attitude.pitch, zero_angle_y);
        // printf("roll:%.2f, %.2f, %.2f\n", attitude.roll, attitude.pitch, attitude.yaw);
        
    }
    
}

