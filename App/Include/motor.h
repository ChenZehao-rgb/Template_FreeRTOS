#ifndef _MOTOR_H
#define _MOTOR_H

#include "gd32f4xx.h"
#include "sys.h"







void motorTask(void *parameter);
void set_pid_target(float temp_val);
float get_pid_target(void);
void set_pid(float kp, float ki, float kd);
#endif  /* MOTOR_H */