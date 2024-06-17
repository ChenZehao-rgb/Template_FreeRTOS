#ifndef _MOTOR_H
#define _MOTOR_H

#include "gd32f4xx.h"
#include "sys.h"

void motor1_speed(void);
void motor2_speed(void);
void motor_init(void);
void motor_test_Task(void *parameter);
#endif  /* MOTOR_H */