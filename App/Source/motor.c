//电机控制，用于调电机转速pid和电机控制函数编写
#include "motor.h"
#include "bsp_pwm.h"
#include "bsp_usart.h"
#include "encoder.h"
#include "pid.h"
/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"

int32_t palse_motor1 = 0;
int32_t palse_motor2 = 0;
extern encoder_counter motor1_encoder;
extern encoder_counter motor2_encoder;

extern PidObject motor1_pid;
extern PidObject motor2_pid;

//电机转速闭环控制
void motor1_speed(void)
{
    motor1_encoder.current_cnt = Motor1_Encoder_Value();   //获取当前编码器值
    motor1_encoder.cnt_diff = motor1_encoder.current_cnt - motor1_encoder.last_cnt;      //计算编码器差值
    motor1_encoder.current_circle = motor1_encoder.circle_count;          //获取当前圈数

    motor1_encoder.cnt_diff += (motor1_encoder.current_circle - motor1_encoder.last_circle) * 396;   //根据圈数补充编码器差值
    
    motor1_encoder.cnt_speed = PosionPID_Realize(&motor1_pid, motor1_encoder.cnt_diff);    //编码器差值作为实际值，计算PID输出

    motor1_out(motor1_encoder.cnt_speed);
    // velocity = get_motor_velocity(cnt_diff, dt, 396.0); // circle/s
    
    motor1_encoder.last_circle = motor1_encoder.current_circle;
    motor1_encoder.last_cnt = motor1_encoder.current_cnt;

    // printf("real:%d, %.2f, %d\n", cnt_diff, PosionPID.target_val, cnt_speed);
}

void motor2_speed(void)
{
    motor2_encoder.current_cnt = Motor2_Encoder_Value();   //获取当前编码器值
    motor2_encoder.cnt_diff = motor2_encoder.current_cnt - motor2_encoder.last_cnt;      //计算编码器差值
    motor2_encoder.current_circle = motor2_encoder.circle_count;          //获取当前圈数

    motor2_encoder.cnt_diff += (motor2_encoder.current_circle - motor2_encoder.last_circle) * 396;   //根据圈数补充编码器差值
    
    motor2_encoder.cnt_speed = PosionPID_Realize(&motor2_pid, motor2_encoder.cnt_diff);    //编码器差值作为实际值，计算PID输出

    motor2_out(motor2_encoder.cnt_speed);
    // velocity = get_motor_velocity(cnt_diff, dt, 396.0); // circle/s
    
    motor2_encoder.last_circle = motor2_encoder.current_circle;
    motor2_encoder.last_cnt = motor2_encoder.current_cnt;

    // printf("real:%d, %.2f, %d\n", cnt_diff, PosionPID.target_val, cnt_speed);
}

//电机初始化
void motor_init(void)
{
    //pid参数初始化
    PID_Param_Init(&motor1_pid, 50, 7,   1.7, 0.5, 500, 10000);
    PID_Param_Init(&motor2_pid, 50, 7,   1.8, 0.5, 500, 2000);
    //初始化编码器
    Motor_Encoder_Init();
    //初始化PWM
    motor_config();
    motor1_out(0);
    motor2_out(0);
}

//电机测试
void motor_test_Task(void *parameter)
{
    motor_init();
    // uint32_t palse = 10000;
    
    delay_ms(1000);
    while (1)
    {
        vTaskDelay(20); //控制频率为50Hz
        // motor1_out(palse_motor1);
        // motor2_out(palse_motor2);
        // motor1_speed();
        // printf("motor1:%d,%.2f\n",motor1_encoder.cnt_diff, motor1_pid.target_val);
        motor2_speed();
        printf("motor2:%d,%.2f\n",motor2_encoder.cnt_diff, motor2_pid.target_val);
        // timer_channel_output_pulse_value_config(MOTOR_PWM_TIMER, MOTOR1_PWM_CHANNEL, palse);
    }
}
