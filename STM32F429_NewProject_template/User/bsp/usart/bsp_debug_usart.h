#ifndef __DEBUG_USART_H
#define	__DEBUG_USART_H

#include "stm32f4xx.h"
#include <stdio.h>


//���Ŷ���
/*******************************************************/
#define DEBUG_USART                             USART3//USART1
#define DEBUG_USART_CLK                         RCC_APB1Periph_USART3//RCC_APB2Periph_USART1
#define DEBUG_USART_BAUDRATE                    256000//115200  //���ڲ�����

#define DEBUG_USART_RX_GPIO_PORT                GPIOD//GPIOA
#define DEBUG_USART_RX_GPIO_CLK                 RCC_AHB1Periph_GPIOD//RCC_AHB1Periph_GPIOA
#define DEBUG_USART_RX_PIN                      GPIO_Pin_9//GPIO_Pin_10
#define DEBUG_USART_RX_AF                       GPIO_AF_USART3//GPIO_AF_USART1
#define DEBUG_USART_RX_SOURCE                   GPIO_PinSource9//GPIO_PinSource10

#define DEBUG_USART_TX_GPIO_PORT                GPIOD//GPIOA
#define DEBUG_USART_TX_GPIO_CLK                 RCC_AHB1Periph_GPIOD//RCC_AHB1Periph_GPIOA
#define DEBUG_USART_TX_PIN                      GPIO_Pin_8//GPIO_Pin_9
#define DEBUG_USART_TX_AF                       GPIO_AF_USART3//GPIO_AF_USART1
#define DEBUG_USART_TX_SOURCE                   GPIO_PinSource8//GPIO_PinSource9

#define DEBUG_USART_IRQHandler                  USART3_IRQHandler//USART1_IRQHandler
#define DEBUG_USART_IRQ                 		USART3_IRQn//USART1_IRQn
/************************************************************/

void Debug_USART_Config(void);
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);
void Usart_SendString( USART_TypeDef * pUSARTx, char *str);

void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch);

#endif /* __USART1_H */
