#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

#include "stm32f4xx.h"
#include <stdio.h>


/*SPI接口定义   *************************************************/
#define FLASH_SPI                           SPI3
#define FLASH_SPI_CLK                       RCC_APB1Periph_SPI3
#define FLASH_SPI_CLK_INIT                  RCC_APB1PeriphClockCmd

#define FLASH_SPI_SCK_PIN                   GPIO_Pin_3                  
#define FLASH_SPI_SCK_GPIO_PORT             GPIOB                       
#define FLASH_SPI_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOB
#define FLASH_SPI_SCK_PINSOURCE             GPIO_PinSource3
#define FLASH_SPI_SCK_AF                    GPIO_AF_SPI3

#define FLASH_SPI_MISO_PIN                  GPIO_Pin_4                
#define FLASH_SPI_MISO_GPIO_PORT            GPIOB                   
#define FLASH_SPI_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define FLASH_SPI_MISO_PINSOURCE            GPIO_PinSource4
#define FLASH_SPI_MISO_AF                   GPIO_AF_SPI3

#define FLASH_SPI_MOSI_PIN                  GPIO_Pin_5                
#define FLASH_SPI_MOSI_GPIO_PORT            GPIOB                      
#define FLASH_SPI_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define FLASH_SPI_MOSI_PINSOURCE            GPIO_PinSource5
#define FLASH_SPI_MOSI_AF                   GPIO_AF_SPI3

#define FLASH_CS_PIN                        GPIO_Pin_4               
#define FLASH_CS_GPIO_PORT                  GPIOA                     
#define FLASH_CS_GPIO_CLK                   RCC_AHB1Periph_GPIOA

#define SPI_FLASH_CS_LOW()      {FLASH_CS_GPIO_PORT->BSRRH=FLASH_CS_PIN;}
#define SPI_FLASH_CS_HIGH()     {FLASH_CS_GPIO_PORT->BSRRL=FLASH_CS_PIN;}


/*SPI接口定义-结尾****************************/

/*等待超时时间*/
#define SPIT_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define SPIT_LONG_TIMEOUT         ((uint32_t)(10 * SPIT_FLAG_TIMEOUT))

/*信息输出*/
#define Dummy_Byte 0xFF

#define SPI_ERROR(fmt,arg...)          printf("<<-SPI-ERROR->> "fmt"\n",##arg)

u8 SPI3_ReadByte(void);
u8 SPI3_SendByte(u8 byte);


/****************************************************
****************************************************/

#endif /* __SPI_FLASH_H */

