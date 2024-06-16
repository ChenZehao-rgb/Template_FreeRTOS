//姿态控制任务
#include "attitude.h"
#include "bsp_usart.h"
#include "sensors.h"
#include "delay.h"
/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"

sensorData_t sensorData;
attitude_t attitude;

void attitudeTask(void *parameter)
{
    
    vTaskDelay(2000);  //等待传感器初始化完成
    while (1)
    {
        vTaskDelay(20); //控制频率为50Hz
        sensorsAcquire(&attitude);
        // printf("roll:%.2f pitch:%.2f yaw:%.2f\r\n", attitude.roll, attitude.pitch, attitude.yaw);
    }
    
}