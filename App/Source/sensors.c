#include "sensors.h"
#include "mpu6050.h"
#include "sensors_types.h"
#include <stdbool.h>

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

Axis3i16 gyroADCRaw; //陀螺仪ADC原始数据
Axis3i16 gyroADC;    //校准后数据
Axis3f gyrof;        //转换为度每秒
Axis3i16 accADCRaw; //加速度计ADC原始数据
Axis3i16 accADC;    //校准后数据

bool mpu6050GyroRead(Axis3i16 *gyroRaw)
{
    uint8_t buf[6];
    uint8_t reg = 0;
    //MPU6050_GYRO_OUT = MPU6050陀螺仪数据寄存器地址
    //陀螺仪数据输出寄存器总共由6个寄存器组成，
    //输出X/Y/Z三个轴的陀螺仪传感器数据，高字节在前，低字节在后。
    //每一个轴16位，按顺序为xyz
    reg = MPU6050_ReadData(0x68,MPU6050_GYRO_OUT,6,buf);
    if( reg == 0 )
    {
        gyroRaw->x = (((int16_t)buf[0]) << 8) | buf[1];
        gyroRaw->y = (((int16_t)buf[2]) << 8) | buf[3];
        gyroRaw->z = (((int16_t)buf[4]) << 8) | buf[5];
    }
    return reg;  //读取正确，返回0
}

bool mpu6050AccRead(Axis3i16 *accRaw)
{
    uint8_t buf[6];
    uint8_t reg = 0;
    //MPU6050_ACC_OUT = MPU6050加速度数据寄存器地址
    //加速度数据输出寄存器总共由6个寄存器组成，
    //输出X/Y/Z三个轴的加速度传感器数据，高字节在前，低字节在后。
    //每一个轴16位，按顺序为xyz
    reg = MPU6050_ReadData(0x68,MPU6050_ACC_OUT,6,buf);
    if( reg == 0 )
    {
        accRaw->x = (((int16_t)buf[0]) << 8) | buf[1];
        accRaw->y = (((int16_t)buf[2]) << 8) | buf[3];
        accRaw->z = (((int16_t)buf[4]) << 8) | buf[5];
    }
    return reg; //读取正确，返回0
}

void sensorsUpdate(Axis3f *acc, Axis3f *gyro)
{
    //读取原始数据
    if(mpu6050GyroRead(&gyroADCRaw) || mpu6050AccRead(&accADCRaw))
    {
        return;
    }
}