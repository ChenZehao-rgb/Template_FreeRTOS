#include "attitude.h"
#include "bsp_usart.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "delay.h"
/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"

void Read_attitudeTask(void *parameter)
{
    float pitch=0,roll=0,yaw=0;                 //欧拉角

    //MPU6050初始化
    MPU6050_Init();
    //DMP初始化
    while( mpu_dmp_init() )
    {
        printf("dmp error\r\n");
        delay_ms(200);
    }
    printf("Initialization Data Succeed \r\n");
    while(1) 
    {
        //获取欧拉角
        if( mpu_dmp_get_data(&pitch,&roll,&yaw) == 0 )
        { 
            printf("\r\npitch =%.2f\r\n", pitch);
            printf("\r\nroll =%.2f\r\n", roll);
            printf("\r\nyaw =%.2f\r\n", yaw);
        }      
        vTaskDelay(20);//根据设置的采样率，不可设置延时过大
    }
}