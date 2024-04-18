#include "system.h"

/*底层硬件初始化*/
void systeminit(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
    systick_config();
    led_gpio_config();
    usart_gpio_config(9600U);
    /* 
    key_gpio_config();
    basic_timer_config(20000,10000);
    pwm_config(200, 10000);
    dma_config(); */
}