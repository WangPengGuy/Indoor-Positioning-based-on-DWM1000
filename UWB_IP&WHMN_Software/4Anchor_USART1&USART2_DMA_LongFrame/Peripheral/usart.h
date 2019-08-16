#ifndef __USART_H__
#define __USART_H__

#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

#define USART1_REC_LEN      1
#define USART2_REC_LEN  	  1000  		// 定义最大接收字节数 200
#define USART1_REC_LEN_DMA  1       // 定义USART1_DMA每次接收或发送的字节数
#define USART2_REC_LEN_DMA  810      // 定义USART2_DMA每次接收或发送的字节数，和上面的不一样，主要用在HAL_UART_Receive_DMA()函数中

extern void _Error_Handler(char*, int);
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;

extern volatile uint8_t USART1_RX_STA;         			   // USART1接收状态标记	
extern volatile uint8_t USART2_RX_STA;         			   // USART2接收状态标记	
extern uint8_t USART1_RX_BUF[USART1_REC_LEN];  //串口1的接受缓冲，使用DMA接收，每次只接受一个字节
extern uint8_t USART2_RX_BUF[USART2_REC_LEN];  //接收缓冲,最大USART2_REC_LEN个字节.

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

#endif
