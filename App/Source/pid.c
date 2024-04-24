#include "pid.h"
#include "maths.h"

void PID_Param_Init(PidObject *pid, float target, float kp, float ki, float kd, float iLimit, float oLimit)
{
	pid->target_val = target;
	pid->output_val = 0.0;
	pid->Error = 0.0;
	pid->LastError = 0.0;
	pid->integral = 0.0;
	pid->iLimit = iLimit;
	pid->oLimit = oLimit;

	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
}

//位置式pid
float PosionPID_Realize(PidObject *pid, float actual_val)
{
	//计算目标值与实际误差
	pid->Error = pid->target_val - actual_val;
	//积分项
	pid->integral += pid->Error;
	//积分项限幅
	if(pid->iLimit != 0)
	{
		pid->integral = constrainf(pid->integral, -pid->iLimit, pid->iLimit);
	}
	//pid实现
	pid->output_val = pid->Kp*pid->Error + pid->Ki*pid->integral + pid->Kd*(pid->Error-pid->LastError);
	//误差传递
	//输出限幅
	if (pid->oLimit != 0)
	{
		pid->output_val = constrainf(pid->output_val, -pid->oLimit, pid->oLimit);
	}
	pid->LastError = pid->Error;
	//返回输出值
	return pid->output_val;
}