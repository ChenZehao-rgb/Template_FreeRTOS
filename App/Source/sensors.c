#include "sensors.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "delay.h"
#include "bsp_usart.h"
/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


Axis3i16 accADCRaw; //加速度计ADC原始数据
Axis3i16 accADC;    //校准后数据

Axis3i16 gyro_sensortask;
    Axis3i16 acc_sensortask;
		float pitch = 0, roll = 0, yaw = 0; //欧拉角
static xQueueHandle gyrodataQueue = NULL;
static xQueueHandle accdataQueue = NULL;

int tick_sensortask = 0;
//从队列读取数据
bool sensorsReadGyro(Axis3f *gyro)
{
    if (xQueueReceive(gyrodataQueue, gyro, 0) == pdTRUE)
    {
        return true;
    }
    return false;
}

bool sensorsReadAcc(Axis3f *acc)
{
    if (xQueueReceive(accdataQueue, acc, 0) == pdTRUE)
    {
        return true;
    }
    return false;
}

//传感器初始化
void sensorsInit(void)
{
    //MPU6050初始化
    MPU6050_Init();
    //DMP初始化
    while (mpu_dmp_init())
    {
        printf("dmp error\r\n");
        delay_ms(200);
    }
    printf("Initialization Data Succeed \r\n");

    //创建队列
    gyrodataQueue = xQueueCreate(1, sizeof(Axis3f));
    accdataQueue = xQueueCreate(1, sizeof(Axis3f));
}

//传感器任务
void sensorsTask(void *parameter)
{
    
    
    
    sensorsInit();
    while (1)
    {
        vTaskDelay(2);

        //获取欧拉角
        if(tick_sensortask % 20 == 0)
        {
            if (mpu_dmp_get_data(&pitch, &roll, &yaw) == 0)
            {
                // printf("dmp_pitch:%.2f, %.2f, %.2f\n", pitch, roll, yaw);
            }
        }

        //获取陀螺仪数据
        if (!mpu6050GyroRead(&gyro_sensortask))
        {
            xQueueSend(gyrodataQueue, &gyro_sensortask, 0);
        }

        //获取加速度计数据
        if (!mpu6050AccRead(&acc_sensortask))
        {
            xQueueSend(accdataQueue, &acc_sensortask, 0);
        }
        tick_sensortask++;
    }
}

/*获取传感器数据*/
void sensorsAcquire(sensorData_t *sensors)
{
	sensorsReadGyro(&sensors->gyro);
	sensorsReadAcc(&sensors->acc);
}
