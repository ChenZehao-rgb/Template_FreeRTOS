#include "sensors.h"
#include "mpu6050.h"
#include "sensors_types.h"
#include <stdbool.h>

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


Axis3i16 accADCRaw; //加速度计ADC原始数据
Axis3i16 accADC;    //校准后数据



void sensorsUpdate(Axis3f *acc, Axis3f *gyro)
{
    //读取原始数据
    if(mpu6050GyroRead(&gyroADCRaw) || mpu6050AccRead(&accADCRaw))
    {
        return;
    }
}