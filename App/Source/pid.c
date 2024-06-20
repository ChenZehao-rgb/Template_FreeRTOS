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

//姿态位置式pid，添加允许误差
float AttitudePID_Realize(PidObject *pid, float actual_val, float relax, short gyro)
{
	//计算目标值与实际误差
	pid->Error = pid->target_val - actual_val;
	if(relax != 0)
	{
		pid->Error = applyDeadbandf(pid->Error, relax); //添加允许误差
	
	}
	//积分项
	pid->integral += pid->Error;
	//积分项限幅
	if (pid->iLimit != 0)
	{
		pid->integral = constrainf(pid->integral, -pid->iLimit, pid->iLimit);
	}
	//pid实现
	// pid->output_val = pid->Kp*pid->Error + pid->Ki*pid->integral + pid->Kd*(pid->Error - pid->LastError);
	pid->output_val = pid->Kp*pid->Error + pid->Ki*pid->integral + pid->Kd*gyro;
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

//设置目标值
void set_pid_target(PidObject *PID, float temp_val)
{
	PID->target_val = temp_val;
}


//获取目标值
float get_pid_target(PidObject *PID)
{
	return PID->target_val;
}

//设置pid参数
void set_pid(PidObject *PID, float kp, float ki, float kd)
{
	PID->Kp = kp;
	PID->Ki = ki;
	PID->Kd = kd;
}

void set_kp(PidObject *PID, float kp)
{
	PID->Kp = kp;
}

void set_ki(PidObject *PID, float ki)
{
	PID->Ki = ki;
}

void set_kd(PidObject *PID, float kd)
{
	PID->Kd = kd;
}
