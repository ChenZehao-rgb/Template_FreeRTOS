#include "system.h"

TaskHandle_t startTaskHandle;
void startTask(void *parameter);
static void LED_Task(void *parameter);

int main(void)
{
    uint16_t uicount = 0;
    float fcount = 0;
    systeminit();
    xTaskCreate(startTask, "START_TASK", 300, NULL, 1, &startTaskHandle);
    vTaskStartScheduler();
  
	while (1)
    {
        /* pwm_breathing_lamp();
        if(g_recv_complete_flag)                        //数据接收完成
        {
            g_recv_complete_flag = 0;
            printf("g_recv_length:%d ",g_recv_length); 
            printf("g_recv_buff:%s\r\n ",g_recv_buff);
            memset(g_recv_buff, 0, g_recv_length);      //清空数组
            g_recv_length = 0;
        } */
    };

    /* while (1)
    {
        pwm_breathing_lamp();
    } */
    
    /* while(1) 
    {
        
        gpio_bit_write(PORT_LED2, PIN_LED2, SET);
        delay_1ms(1000);
        gpio_bit_write(PORT_LED2, PIN_LED2, RESET);
        delay_1ms(1000);
        uicount++;
        fcount += 0.1;
        printf("uicount = %d, fcount = %.2f\n\r", uicount, fcount);                    
    } */

    /* while (1)
    {
        key_scan();      //按键扫描
    } */

    
}

void startTask(void *parameter)
{
    taskENTER_CRITICAL();

    xTaskCreate(LED_Task, "LED_Task", 150, NULL, 2, NULL);
    
    printf("Free heap: %d bytes\n", xPortGetFreeHeapSize());			/*打印剩余堆栈大小*/
	
	vTaskDelete(startTaskHandle);										/*删除开始任务*/

	taskEXIT_CRITICAL();	/*退出临界区*/
}
static void LED_Task(void *parameter)
    {
        while (1)
        {
            gpio_bit_write(PORT_LED2, PIN_LED2, SET);
            vTaskDelay(500);
            
            gpio_bit_write(PORT_LED2, PIN_LED2, RESET);
            vTaskDelay(500);
            
        }
        
    }