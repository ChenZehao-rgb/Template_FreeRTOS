#ifndef _MOTOR_H
#define _MOTOR_H

#include "gd32f4xx.h"
#include "sys.h"

void motorTask(void *parameter);

void set_pos_pid_target(float temp_val);
void set_speed_pid_target(float temp_val);
float get_pos_pid_target(void);
float get_speed_pid_target(void);
void set_pos_pid(float kp, float ki, float kd);
void set_pos_kp(float kp);
void set_pos_ki(float ki);
void set_pos_kd(float kd);
void set_speed_kp(float kp);
void set_speed_ki(float ki);
void set_speed_kd(float kd);
#endif  /* MOTOR_H */