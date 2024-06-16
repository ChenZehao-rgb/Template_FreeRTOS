#include "attitude.h"
#include "bsp_usart.h"
#include "sensors.h"

#include "imu.h"
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
        vTaskDelay(2); //姿态解算频率为500Hz
        sensorsAcquire(&sensorData);
        imuUpdateAttitude(&sensorData, &attitude, 0.002f);
        // printf("roll:%.2f pitch:%.2f yaw:%.2f\r\n", attitude.roll, attitude.pitch, attitude.yaw);
    }
    
}