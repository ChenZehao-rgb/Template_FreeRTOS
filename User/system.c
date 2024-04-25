#include "system.h"

/*底层硬件初始化*/
void systeminit(void)
{
    delay_init(240);
    nvicInit();
    led_gpio_config();
    usart_gpio_config(115200U);
    motor_config();
    Motor_Encoder_Init();
    protocol_init();
    /* 
    key_gpio_config();
    basic_timer_config(20000,10000);
    pwm_config(200, 10000);
    dma_config(); */
}