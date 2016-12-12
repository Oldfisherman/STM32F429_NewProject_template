/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   ���ڽӷ����ԣ����ڽ��յ����ݺ����ϻش���
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:����  STM32 F429 ������
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
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
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{	
    HSE_SetSysClock(8, 336, 2, 7);//�޸ĵ�һ�� ��Ƶֵ 25 �Դ����Ƿ���ȷ���ò�������Ӱ�죬�ⲿʱ�������Ծ���Ҫ�޸� ͷ�ļ��еĺ궨��
    //HSI_SetSysClock(16, 360, 2, 7);
    /*��ʼ��USART ����ģʽΪ 115200 8-N-1���жϽ���*/
    Debug_USART_Config();
	
	/* ����һ���ַ��� */
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

void Delay(__IO uint32_t nCount)	 //�򵥵���ʱ����
{
	for(; nCount != 0; nCount--);
}
/*********************************************END OF FILE**********************/

