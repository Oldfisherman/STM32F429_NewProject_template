/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   串口接发测试，串口接收到数据后马上回传。
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火  STM32 F429 开发板
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "stm32f4xx.h"
#include "./bsp/usart/bsp_debug_usart.h"
#include "./bsp/rcc/bsp_clkconfig.h"
#include "./bsp/spi/bsp_spi_flash.h"
#include "./bsp/systick/bsp_SysTick.h"


void Delay(__IO u32 nCount); 

/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{	
    HSE_SetSysClock(8, 336, 2, 7);//修改第一项 分频值 25 对串口是否正确设置波特率无影响，外部时钟引用仍旧需要修改 头文件中的宏定义
    //HSI_SetSysClock(16, 360, 2, 7);
    /*初始化USART 配置模式为 115200 8-N-1，中断接收*/
    Debug_USART_Config();
	
	/* 发送一个字符串 */
	//Usart_SendString( DEBUG_USART,"as");
	//Usart_SendByte(DEBUG_USART,'a');
	//printf("ks");
	
    SPI_FLASH_Init();
    
    while(1)
	{
		//Usart_SendString(DEBUG_USART,"as\n");
		Usart_SendByte(DEBUG_USART,0x01);
        //printf("ks");
		Delay(0x0fffff);
	}	
}

void Delay(__IO uint32_t nCount)	 //简单的延时函数
{
	for(; nCount != 0; nCount--);
}
/*********************************************END OF FILE**********************/

