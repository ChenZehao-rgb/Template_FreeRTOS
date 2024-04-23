#ifndef _ENCODER_H
#define _ENCODER_H

#include "gd32f4xx.h"
#include "sys.h"

//motor1
#define MOTOR1_encoder_TIMER_RCU  	 RCU_TIMER1      // 定时器时钟
#define MOTOR1_encoder_TIMER      	 TIMER1          // 定时器
#define MOTOR1_encoderA_RCU          RCU_GPIOA       // ENCODER引脚
#define MOTOR1_encoderA_PORT         GPIOA
#define MOTOR1_encoderA_PIN          GPIO_PIN_15
#define MOTOR1_encoderA_AF           GPIO_AF_1
#define MOTOR1_encoderA_CHANNEL      TIMER_CH_0      // 定时器通道
#define MOTOR1_encoderB_RCU          RCU_GPIOB       // pwm引脚
#define MOTOR1_encoderB_PORT         GPIOB
#define MOTOR1_encoderB_PIN          GPIO_PIN_3
#define MOTOR1_encoderB_AF           GPIO_AF_1
#define MOTOR1_encoderB_CHANNEL      TIMER_CH_1      // 定时器通道
#define MOTOR1_encoder_IRQ           TIMER1_IRQn
#define Motor1_Encoder_IRQHandler    TIMER1_IRQHandler
//motor2
#define MOTOR2_encoder_TIMER_RCU  	 RCU_TIMER2      // 定时器时钟
#define MOTOR2_encoder_TIMER      	 TIMER2          // 定时器
#define MOTOR2_encoderA_RCU          RCU_GPIOC       // pwm引脚
#define MOTOR2_encoderA_PORT         GPIOC
#define MOTOR2_encoderA_PIN          GPIO_PIN_6
#define MOTOR2_encoderA_AF           GPIO_AF_2
#define MOTOR2_encoderA_CHANNEL      TIMER_CH_0      // 定时器通道
#define MOTOR2_encoderB_RCU          RCU_GPIOC       // pwm引脚
#define MOTOR2_encoderB_PORT         GPIOC
#define MOTOR2_encoderB_PIN          GPIO_PIN_7
#define MOTOR2_encoderB_AF           GPIO_AF_2
#define MOTOR2_encoderB_CHANNEL      TIMER_CH_1      // 定时器通道
#define MOTOR2_encoder_IRQ           TIMER2_IRQn
#define Motor2_Encoder_IRQHandler    TIMER2_IRQHandler

void Motor_Encoder_Init(void);
uint32_t Motor1_Encoder_Value(void);
uint32_t Motor2_Encoder_Value(void);
#endif  /* ENCODER_H */