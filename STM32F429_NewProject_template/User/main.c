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
#include "./bsp/spi/bsp_spi_flash.h"

//
//
//
const uint8_t TOTAL_IC = 1;//!<number of ICs in the daisy chain

/******************************************************
 *** Global Battery Variables received from 6804 commands
 These variables store the results from the LTC6804
 register reads and the array lengths must be based
 on the number of ICs on the stack
 ******************************************************/
uint16_t cell_codes[TOTAL_IC][12];
/*!<
  The cell codes will be stored in the cell_codes[][12] array in the following format:

  |  cell_codes[0][0]| cell_codes[0][1] |  cell_codes[0][2]|    .....     |  cell_codes[0][11]|  cell_codes[1][0] | cell_codes[1][1]|  .....   |
  |------------------|------------------|------------------|--------------|-------------------|-------------------|-----------------|----------|
  |IC1 Cell 1        |IC1 Cell 2        |IC1 Cell 3        |    .....     |  IC1 Cell 12      |IC2 Cell 1         |IC2 Cell 2       | .....    |
****/

uint16_t aux_codes[TOTAL_IC][6];
/*!<
 The GPIO codes will be stored in the aux_codes[][6] array in the following format:

 |  aux_codes[0][0]| aux_codes[0][1] |  aux_codes[0][2]|  aux_codes[0][3]|  aux_codes[0][4]|  aux_codes[0][5]| aux_codes[1][0] |aux_codes[1][1]|  .....    |
 |-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|---------------|-----------|
 |IC1 GPIO1        |IC1 GPIO2        |IC1 GPIO3        |IC1 GPIO4        |IC1 GPIO5        |IC1 Vref2        |IC2 GPIO1        |IC2 GPIO2      |  .....    |
*/

uint8_t tx_cfg[TOTAL_IC][6];
/*!<
  The tx_cfg[][6] stores the LTC6804 configuration data that is going to be written
  to the LTC6804 ICs on the daisy chain. The LTC6804 configuration data that will be
  written should be stored in blocks of 6 bytes. The array should have the following format:

 |  tx_cfg[0][0]| tx_cfg[0][1] |  tx_cfg[0][2]|  tx_cfg[0][3]|  tx_cfg[0][4]|  tx_cfg[0][5]| tx_cfg[1][0] |  tx_cfg[1][1]|  tx_cfg[1][2]|  .....    |
 |--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|-----------|
 |IC1 CFGR0     |IC1 CFGR1     |IC1 CFGR2     |IC1 CFGR3     |IC1 CFGR4     |IC1 CFGR5     |IC2 CFGR0     |IC2 CFGR1     | IC2 CFGR2    |  .....    |

*/

uint8_t rx_cfg[TOTAL_IC][8];
/*!<
  the rx_cfg[][8] array stores the data that is read back from a LTC6804-1 daisy chain.
  The configuration data for each IC  is stored in blocks of 8 bytes. Below is an table illustrating the array organization:

|rx_config[0][0]|rx_config[0][1]|rx_config[0][2]|rx_config[0][3]|rx_config[0][4]|rx_config[0][5]|rx_config[0][6]  |rx_config[0][7] |rx_config[1][0]|rx_config[1][1]|  .....    |
|---------------|---------------|---------------|---------------|---------------|---------------|-----------------|----------------|---------------|---------------|-----------|
|IC1 CFGR0      |IC1 CFGR1      |IC1 CFGR2      |IC1 CFGR3      |IC1 CFGR4      |IC1 CFGR5      |IC1 PEC High     |IC1 PEC Low     |IC2 CFGR0      |IC2 CFGR1      |  .....    |
*/
//
//
//


//
//
//
/*!***********************************
 \brief Initializes the configuration array
 **************************************/
void init_cfg()
{
  for (int i = 0; i<TOTAL_IC; i++)
  {
    tx_cfg[i][0] = 0xFE;
    tx_cfg[i][1] = 0x00 ;
    tx_cfg[i][2] = 0x00 ;
    tx_cfg[i][3] = 0x00 ;
    tx_cfg[i][4] = 0x00 ;
    tx_cfg[i][5] = 0x00 ;
  }
}

void serial_print_hex(uint8_t data)
{ 
    printf("%02X",data); 
}

//
void LTC68041_print_rxconfig( void )
{
    printf("Received Configuration: \n");
    for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
    {
        printf(" IC ");
        printf("%02d",current_ic+1);
        printf(": 0x");
        serial_print_hex(rx_cfg[current_ic][0]);
        printf(", 0x");
        serial_print_hex(rx_cfg[current_ic][1]);
        printf(", 0x");
        serial_print_hex(rx_cfg[current_ic][2]);
        printf(", 0x");
        serial_print_hex(rx_cfg[current_ic][3]);
        printf(", 0x");
        serial_print_hex(rx_cfg[current_ic][4]);
        printf(", 0x");
        serial_print_hex(rx_cfg[current_ic][5]);
        printf(", Received PEC: 0x");
        serial_print_hex(rx_cfg[current_ic][6]);
        printf(", 0x");
        serial_print_hex(rx_cfg[current_ic][7]);
        printf(" ");
    }
    printf("\n");
}



//
//
//
/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{	 
    int8_t error=0;
    HSE_SetSysClock(8, 336, 2, 7);
    /*初始化USART 配置模式为 115200 8-N-1，中断接收*/
    Debug_USART_Config();
	
	/* 发送一个字符串 */
	//Usart_SendString( DEBUG_USART,"as");
	//Usart_SendByte(DEBUG_USART,'a');
	printf("ks");
	
    /* LED 端口初始化 */
	LED_GPIO_Config();
    
    /* 初始化基本定时器定时，10us产生一次中断 */
	TIMx_Configuration();
    
    /* SPI Initial */
    SPI_FLASH_Init();

	/* 控制LED灯 */
	while (1)
	{
        LTC68041_wakeup_sleep();
        error = LTC6804_rdcfg(TOTAL_IC,rx_cfg);
        if (error == -1)
        {
            printf("A PEC error was detected in the received data");
        }
        LTC68041_print_rxconfig();		
		Delay_10_us(100000);
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

