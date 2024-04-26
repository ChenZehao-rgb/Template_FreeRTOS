#ifndef _BSP_PWM_H
#define _BSP_PWM_H

#include "gd32f4xx.h"
#include "sys.h"

//定时器
#define MOTOR_PWM_TIMER_RCU  	RCU_TIMER4      // 定时器时钟
#define MOTOR_PWM_TIMER      	TIMER4          // 定时器

//motor1
#define MOTOR1_PWM_RCU          RCU_GPIOA       // pwm引脚
#define MOTOR1_PWM_PORT         GPIOA
#define MOTOR1_PWM_PIN          GPIO_PIN_1
#define MOTOR1_PWM_AF           GPIO_AF_2
#define MOTOR1_PWM_CHANNEL      TIMER_CH_1      // 定时器通道
#define MOTOR1_DIR_RCU          RCU_GPIOC       // dir引脚
#define MOTOR1_DIR_PORT         GPIOC
#define MOTOR1_DIR_PIN          GPIO_PIN_4
#define MOTOR1_DIR              PCout(4)
//motor2
#define MOTOR2_PWM_RCU          RCU_GPIOA       // pwm引脚
#define MOTOR2_PWM_PORT         GPIOA
#define MOTOR2_PWM_PIN          GPIO_PIN_2
#define MOTOR2_PWM_AF           GPIO_AF_2
#define MOTOR2_PWM_CHANNEL      TIMER_CH_2      // 定时器通道
#define MOTOR2_DIR_RCU          RCU_GPIOC       // DIR引脚
#define MOTOR2_DIR_PORT         GPIOC
#define MOTOR2_DIR_PIN          GPIO_PIN_5
#define MOTOR2_DIR              PCout(5)

void motor1_out(int32_t speed);
void motor2_out(int32_t speed);
void motor_config(void);              // 电机初始化
#endif  /* BSP_PWM_H */