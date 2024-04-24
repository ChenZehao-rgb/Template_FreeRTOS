#ifndef _BSP_USART_H
#define _BSP_USART_H

#include "gd32f4xx.h"
#include "systick.h"

#define BSP_USART_TX_RCU        RCU_GPIOA           // 串口TX的端口时钟
#define BSP_USART_RX_RCU        RCU_GPIOA           // 串口RX的端口时钟
#define BSP_USART_RCU           RCU_USART0          // 串口0的时钟

#define BSP_USART_TX_PORT       GPIOA		        // 串口TX的端口
#define BSP_USART_RX_PORT       GPIOA		        // 串口RX的端口
#define BSP_USART_AF 			GPIO_AF_7           // 串口0的复用功能
#define BSP_USART_TX_PIN        GPIO_PIN_9          // 串口TX的引脚
#define BSP_USART_RX_PIN        GPIO_PIN_10         // 串口RX的引脚

#define BSP_USART 				USART0              // 串口0
#define BSP_USART_IRQ           USART0_IRQn         // 串口0中断
#define BSP_USART_IRQHandler    USART0_IRQHandler   // 串口0中断服务函数

//缓冲区数据长度
#define USART_RECEIVE_LENGTH    4096

/*0：中断接收   1：DMA接收*/
#define RX_USART_DMA  0

extern uint8_t  g_recv_buff[USART_RECEIVE_LENGTH];
extern uint16_t g_recv_length;
extern uint8_t  g_recv_complete_flag;

void usart_gpio_config(uint32_t band_rate);  // 配置串口
void usart_send_data(uint8_t ucch);          // 发送一个字符
void usart_send_string(uint8_t *ucstr);      // 发送一个字符串
void usart1_send(uint8_t *data, uint8_t len);
#endif  /* BSP_USART_H */