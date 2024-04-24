#ifndef _PID_H
#define _PID_H

#include "gd32f4xx.h"
#include "sys.h"
#include "filter.h"

typedef struct
{
	float target_val; 	//目标值
	float Error;		//第k次误差
	float LastError;	//Error[-1],k-1次误差
	float PrevError;	//Error[-2],k-2次误差
	float Kp,Ki,Kd;		//比例，积分，微分系数
	float integral;		//积分值
	float output_val;	//输出值
	float iLimit;		//积分项限幅
	float oLimit;		//输出限幅
} PidObject;

typedef struct 
{
	float kp;
	float ki;
	float kd;
} pidInit_t;

void PID_Param_Init(PidObject *pid, float target, float kp, float ki, float kd, float iLimit, float oLimit);
float PosionPID_Realize(PidObject *pid, float actual_val);


#endif  /* PID_H */