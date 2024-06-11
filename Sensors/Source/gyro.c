#include "gyro.h"
#include "sensors_types.h"
#include "mpu6050.h"
#include <maths.h>
#include <axis.h>
#include <filter.h>

/*陀螺仪校准阈值*/
#define GYRO_CALIBRATION_THRESHOLD	4.0f

/*低通滤波参数*/
#define GYRO_LPF_CUTOFF_FREQ  	80.0f

//mpu6000初始化陀螺仪量程为FSR_2000DPS，即
#define GYRO_SCALE	16.4f

typedef struct gyroCalibration_s 
{
    Axis3i32 gyroSum;
	Axis3i16 gyroZero;
    stdev_t var[XYZ_AXIS_COUNT];
    uint16_t cycleCount;
} gyroCalibration_t;


//陀螺仪校准结构体参数
gyroCalibration_t gyroCalibration = 
{
	.cycleCount = CALIBRATING_GYRO_CYCLES,
};	

Axis3i16 gyroADCRaw; //陀螺仪ADC原始数据
Axis3i16 gyroADC;    //校准后数据
Axis3f gyrof;        //转换为度每秒

biquadFilter_t gyroFilterLPF[XYZ_AXIS_COUNT];//二阶低通滤波器

void gyroSetCalibrationCycles(uint16_t calibrationCyclesRequired)
{
    gyroCalibration.cycleCount = calibrationCyclesRequired;
}

bool gyroIsCalibrationComplete(void)
{
    return gyroCalibration.cycleCount == 0;
}

static bool isOnFinalGyroCalibrationCycle(void)
{
    return gyroCalibration.cycleCount == 1;
}

static bool isOnFirstGyroCalibrationCycle(void)
{
    return gyroCalibration.cycleCount == CALIBRATING_GYRO_CYCLES;
}

/// @brief 陀螺仪校准函数
/// @param gyroADCSample 
void performGyroCalibration(Axis3i16 gyroADCSample)
{
    for (int axis = 0; axis < 3; axis++) 
	{
        //复位数据
        if (isOnFirstGyroCalibrationCycle()) 
		{
            gyroCalibration.gyroSum.axis[axis] = 0;
            devClear(&gyroCalibration.var[axis]);
        }

        //陀螺数据累加
        gyroCalibration.gyroSum.axis[axis] += gyroADCSample.axis[axis];
        devPush(&gyroCalibration.var[axis], gyroADCSample.axis[axis]);

        //复位零偏 
        gyroCalibration.gyroZero.axis[axis] = 0;

        if (isOnFinalGyroCalibrationCycle()) 
		{
            const float stddev = devStandardDeviation(&gyroCalibration.var[axis]);
			
            //检测方差值是否大于陀螺仪受到移动的阈值
			//如果大于设定阈值则返回重新校准
            if ((stddev > GYRO_CALIBRATION_THRESHOLD) || (stddev == 0)) 
			{
                gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
                return;
            }
			
            //校准完成
            gyroCalibration.gyroZero.axis[axis] = (gyroCalibration.gyroSum.axis[axis] + (CALIBRATING_GYRO_CYCLES / 2)) / CALIBRATING_GYRO_CYCLES;
        }
    }
	
    gyroCalibration.cycleCount--;
}

bool gyroInit(float gyroUpdateRate)
{
	if(mpu6000Init() == false)
		return false;
	
	//初始化陀螺仪零偏校准
	gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
	
	//初始化二阶低通滤波
	for (int axis = 0; axis < 3; axis++)
	{
		biquadFilterInitLPF(&gyroFilterLPF[axis], gyroUpdateRate, GYRO_LPF_CUTOFF_FREQ);
	}
	return true;
}

void gyroUpdate(Axis3f *gyro)
{
    //读取原始数据
    if(mpu6050GyroRead(&gyroADCRaw))  //返回0表示读取成功
    {
        return;
    }

    //陀螺仪校准
	if (!gyroIsCalibrationComplete()) 
	{
		performGyroCalibration(gyroADCRaw);
		gyrof.x = 0.0f;
		gyrof.y = 0.0f;
		gyrof.z = 0.0f;
		*gyro = gyrof;
		return;
	}

	//计算gyroADC值，减去零偏
	gyroADC.x = gyroADCRaw.x - gyroCalibration.gyroZero.x;
	gyroADC.y = gyroADCRaw.y - gyroCalibration.gyroZero.y;
	gyroADC.z = gyroADCRaw.z - gyroCalibration.gyroZero.z;
	
	
	//转换为单位 °/s 
	gyrof.x = (float)gyroADC.x / GYRO_SCALE;
	gyrof.y = (float)gyroADC.y / GYRO_SCALE;
	gyrof.z = (float)gyroADC.z / GYRO_SCALE;
	
	//软件二阶低通滤波
	for (int axis = 0; axis < 3; axis++) 
	{
		gyrof.axis[axis] = biquadFilterApply(&gyroFilterLPF[axis], gyrof.axis[axis]);
	}
	
	*gyro = gyrof;
}
    
