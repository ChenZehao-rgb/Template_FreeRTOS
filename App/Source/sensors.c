//传感器读取任务
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
		
static xQueueHandle PitchDataQueue = NULL;
static xQueueHandle RollDataQueue = NULL;
static xQueueHandle YawDataQueue = NULL;

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
    PitchDataQueue = xQueueCreate(1, sizeof(float));
    RollDataQueue = xQueueCreate(1, sizeof(float));
    YawDataQueue = xQueueCreate(1, sizeof(float));
}

//传感器任务
void sensorsTask(void *parameter)
{
    float pitch = 0, roll = 0, yaw = 0; //欧拉角
    sensorsInit();
    while (1)
    {
        vTaskDelay(20);

        //获取欧拉角
        if (mpu_dmp_get_data(&pitch, &roll, &yaw) == 0)
        {
            //将数据发送到队列
            xQueueSend(PitchDataQueue, &pitch, 0);
            xQueueSend(RollDataQueue, &roll, 0);
            xQueueSend(YawDataQueue, &yaw, 0);
            printf("dmp_pitch:%.2f, %.2f, %.2f\n", pitch, roll, yaw);
        }
    }
}

/*获取传感器数据*/
void sensorsAcquire(attitude_t *attitude)
{
	xQueueReceive(PitchDataQueue, &attitude->pitch, 0);
    xQueueReceive(RollDataQueue, &attitude->roll, 0);
    xQueueReceive(YawDataQueue, &attitude->yaw, 0);
}
