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
#include "./bsp/systick/bsp_SysTick.h"
#include "./bsp/tim/bsp_basic_tim.h"
#include "./bsp/led/bsp_led.h"
#include "./bsp/delay/bsp_delay.h"
#include "./bsp/spi/bsp_spi3.h"
#include "./My_Peripheral_Driver/ltc6804/ltc68041.h"


/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{	 
    HSE_SetSysClock(8, 336, 2, 7);
    /*初始化USART 配置模式为 115200 8-N-1，中断接收*/
    Debug_USART_Config();
	
	/* 发送一个字符串 */
	//Usart_SendString( DEBUG_USART,"as");
	//Usart_SendByte(DEBUG_USART,'a');
	
    /* LED 端口初始化 */
	LED_GPIO_Config();
    
    /* 初始化基本定时器定时，10us产生一次中断 */
	TIMx_Configuration();
    
    /* SPI Initial */
    LTC68041_init( );
    LTC68041_print_cells();
    //LTC68041_write_config();

	/* 控制LED灯 */
	while (1)
	{
        //LTC68041_read_register();
        LTC68041_measurement_loop();
//        LTC68041_wakeup_sleep();
//        error = LTC68041_rdcfg(TOTAL_IC,rx_cfg);
//        if (error == -1)
//        {
//            printf("A PEC error was detected in the received data");
//        }
//        LTC68041_print_rxconfig();		
//		Delay_10_us(100000);
        //LED1( ON );			 // 亮 
        //printf("ks");
		//LED1( OFF );		  // 灭
        //Delay_10_us(100000);
        //SPI_FLASH_CS_HIGH();
	}
    
    //SPI_FLASH_Init();
    
    
    //while(1)
	//{
		//Usart_SendString(DEBUG_USART,"as\n");
		//Usart_SendByte(DEBUG_USART,0x01);
        //printf("ks");
		//Delay(0x0fffff);
	//}	
}


/*********************************************END OF FILE**********************/

