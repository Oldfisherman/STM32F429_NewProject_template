 /**
  ******************************************************************************
  * @file    bsp_spi_flash.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   spi flash 底层应用函数bsp 
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火STM32 F429 开发板
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "./bsp_spi3.h"
#include "../delay/bsp_delay.h"
#include "../../My_Peripheral_Driver/ltc6804/LTC68041.h"
#include <stdlib.h>


static __IO uint32_t  SPITimeout = SPIT_LONG_TIMEOUT;   

static uint16_t SPI_TIMEOUT_UserCallback(uint8_t errorCode);


 /**
  * @brief  SPI_FLASH初始化
  * @param  无
  * @retval 无
  */
void SPI_FLASH_Init(void) //LTC6804 Initialize
{
    SPI_InitTypeDef  SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
  
    /* 使能 FLASH_SPI 及GPIO 时钟 */
    /*!< SPI_FLASH_SPI_CS_GPIO, SPI_FLASH_SPI_MOSI_GPIO, 
    SPI_FLASH_SPI_MISO_GPIO,SPI_FLASH_SPI_SCK_GPIO 时钟使能 */
    RCC_AHB1PeriphClockCmd (FLASH_SPI_SCK_GPIO_CLK | FLASH_SPI_MISO_GPIO_CLK|FLASH_SPI_MOSI_GPIO_CLK|FLASH_CS_GPIO_CLK, ENABLE);

    /*!< SPI_FLASH_SPI 时钟使能 */
    FLASH_SPI_CLK_INIT(FLASH_SPI_CLK, ENABLE);
 
    //设置引脚复用
    GPIO_PinAFConfig(FLASH_SPI_SCK_GPIO_PORT,FLASH_SPI_SCK_PINSOURCE,FLASH_SPI_SCK_AF); 
	GPIO_PinAFConfig(FLASH_SPI_MISO_GPIO_PORT,FLASH_SPI_MISO_PINSOURCE,FLASH_SPI_MISO_AF); 
	GPIO_PinAFConfig(FLASH_SPI_MOSI_GPIO_PORT,FLASH_SPI_MOSI_PINSOURCE,FLASH_SPI_MOSI_AF); 
  
    /*!< 配置 SPI_FLASH_SPI 引脚: SCK */
    GPIO_InitStructure.GPIO_Pin = FLASH_SPI_SCK_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  
  
    GPIO_Init(FLASH_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);
  
    /*!< 配置 SPI_FLASH_SPI 引脚: MISO */
    GPIO_InitStructure.GPIO_Pin = FLASH_SPI_MISO_PIN;
    GPIO_Init(FLASH_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);
  
	/*!< 配置 SPI_FLASH_SPI 引脚: MOSI */
    GPIO_InitStructure.GPIO_Pin = FLASH_SPI_MOSI_PIN;
    GPIO_Init(FLASH_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);  

	/*!< 配置 SPI_FLASH_SPI 引脚: CS */
    GPIO_InitStructure.GPIO_Pin = FLASH_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(FLASH_CS_GPIO_PORT, &GPIO_InitStructure);

    /*CS引脚高电平*/
    SPI_FLASH_CS_HIGH();

    /* FLASH_SPI 模式配置 */
    // FLASH芯片 支持SPI模式0及模式3，据此设置CPOL CPHA
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    
    //
    //LTC6804 SCK最大通信速率为1MHz
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
    //
    //
    
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(FLASH_SPI, &SPI_InitStructure);

    /* 使能 FLASH_SPI  */
    SPI_Cmd(FLASH_SPI, ENABLE);

}

 /**
  * @brief  使用SPI发送一个字节的数据
  * @param  byte：要发送的数据
  * @retval 返回接收到的数据
  */
u8 SPI3_SendByte(u8 byte)
{
    SPITimeout = SPIT_FLAG_TIMEOUT;
    uint8_t r;

    /* 等待发送缓冲区为空，TXE事件 */
    while (SPI_I2S_GetFlagStatus(FLASH_SPI, SPI_I2S_FLAG_TXE) == RESET)
    {
        if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(0);
    }

    /* 写入数据寄存器，把要写入的数据写入发送缓冲区 */
    SPI_I2S_SendData(FLASH_SPI, byte);

    SPITimeout = SPIT_FLAG_TIMEOUT;

    /* 等待接收缓冲区非空，RXNE事件 */
    while (SPI_I2S_GetFlagStatus(FLASH_SPI, SPI_I2S_FLAG_RXNE) == RESET)
    {
        if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(1);
    }

    /* 读取数据寄存器，获取接收缓冲区数据 */
    r = SPI_I2S_ReceiveData(FLASH_SPI);
    //return SPI_I2S_ReceiveData(FLASH_SPI);
    //printf ("%02x ",r);
    return r;
}

 /**
  * @brief  使用SPI读取一个字节的数据
  * @param  无
  * @retval 返回接收到的数据
  */
u8 SPI3_ReadByte(void)
{
  return (SPI3_SendByte(Dummy_Byte));
}


/**
  * @brief  等待超时回调函数
  * @param  None.
  * @retval None.
  */
static  uint16_t SPI_TIMEOUT_UserCallback(uint8_t errorCode)
{
  /* 等待超时后的处理,输出错误信息 */
  SPI_ERROR("SPI overtime errorCode = %d",errorCode);
  return 0;
}
   
/*********************************************END OF FILE**********************/



/*!*******************************************************************************************************************
 \brief Maps  global ADC control variables to the appropriate control bytes for each of the different ADC commands

@param[in] uint8_t MD The adc conversion mode
@param[in] uint8_t DCP Controls if Discharge is permitted during cell conversions
@param[in] uint8_t CH Determines which cells are measured during an ADC conversion command
@param[in] uint8_t CHG Determines which GPIO channels are measured during Auxiliary conversion command

Command Code:
-------------

|command  |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|ADCV:      |   0   |   0   |   0   |   0   |   0   |   0   |   1   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CH[2] | CH[1] | CH[0] |
|ADAX:      |   0   |   0   |   0   |   0   |   0   |   1   |   0   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CHG[2]| CHG[1]| CHG[0]|
 ******************************************************************************************************************/
//void set_adc(uint8_t MD, //ADC Mode
//             uint8_t DCP, //Discharge Permit
//             uint8_t CH, //Cell Channels to be measured
//             uint8_t CHG //GPIO Channels to be measured
//            )
//{
//    uint8_t md_bits;

//    md_bits = (MD & 0x02) >> 1;
//    ADCV[0] = md_bits + 0x02;
//    md_bits = (MD & 0x01) << 7;
//    ADCV[1] =  md_bits + 0x60 + (DCP<<4) + CH;

//    md_bits = (MD & 0x02) >> 1;
//    ADAX[0] = md_bits + 0x04;
//    md_bits = (MD & 0x01) << 7;
//    ADAX[1] = md_bits + 0x60 + CHG ;
//}



///*!****************************************************
//  \brief Wake the LTC6804 from the sleep state

// Generic wakeup commannd to wake the LTC6804 from sleep
// *****************************************************/
//void LTC68041_wakeup_sleep( void )
//{
//  SPI_LTC6804_CS_LOW();
//  Delay_10_us(10); // Guarantees the LTC6804 will be in standby
//  SPI_LTC6804_CS_HIGH();
//}

///*!****************************************************
//  \brief Wake isoSPI up from idle state
// Generic wakeup commannd to wake isoSPI up out of idle
// *****************************************************/
//void LTC68041_wakeup_idle()
//{
//    SPI_LTC6804_CS_LOW();
//    //delayMicroseconds(2); Guarantees the isoSPI will be in ready mode
//    Delay_10_us(200);
//    SPI_LTC6804_CS_HIGH();
//}

///*!******************************************************
// \brief Reads configuration registers of a LTC6804 daisy chain
// 
//@param[in] uint8_t total_ic: number of ICs in the daisy chain

//@param[out] uint8_t r_config[][8] is a two dimensional array that the function stores the read configuration data. The configuration data for each IC 
//is stored in blocks of 8 bytes with the configuration data of the lowest IC on the stack in the first 8 bytes 
//block of the array, the second IC in the second 8 byte etc. Below is an table illustrating the array organization:

//|r_config[0][0]|r_config[0][1]|r_config[0][2]|r_config[0][3]|r_config[0][4]|r_config[0][5]|r_config[0][6]  |r_config[0][7] |r_config[1][0]|r_config[1][1]|  .....    |
//|--------------|--------------|--------------|--------------|--------------|--------------|----------------|---------------|--------------|--------------|-----------|
//|IC1 CFGR0     |IC1 CFGR1     |IC1 CFGR2     |IC1 CFGR3     |IC1 CFGR4     |IC1 CFGR5     |IC1 PEC High    |IC1 PEC Low    |IC2 CFGR0     |IC2 CFGR1     |  .....    |


//@return int8_t, PEC Status.
// 
//	0: Data read back has matching PEC
// 
//	-1: Data read back has incorrect PEC


//Command Code:
//-------------

//|CMD[0:1]		|  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   | 
//|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
//|RDCFG:	        |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   0   |   1   |   0   |
//********************************************************/
//int8_t LTC6804_rdcfg(uint8_t total_ic, //Number of ICs in the system
//				     uint8_t r_config[][8] //A two dimensional array that the function stores the read configuration data.
//					 )
//{
//    const uint8_t BYTES_IN_REG = 8;
//  
//    uint8_t cmd[4];
//    uint8_t *rx_data;
//    int8_t pec_error = 0; 
//    uint16_t data_pec;
//    uint16_t received_pec;
//  
//    rx_data = (uint8_t *) malloc((8*total_ic)*sizeof(uint8_t));
//  
//    //1
//    cmd[0] = 0x00;
//    cmd[1] = 0x02;
//    cmd[2] = 0x2b;
//    cmd[3] = 0x0A;
// 
//    //2
//    //wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
//    LTC68041_wakeup_idle();
//    
//    //3
//    //output_low(LTC6804_CS);
//    SPI_LTC6804_CS_LOW();
//  
//    LTC68041_spi_write_read(cmd, 4, rx_data, (BYTES_IN_REG*total_ic));         //Read the configuration data of all ICs on the daisy chain into 
//  
//    //output_high(LTC6804_CS);													//rx_data[] array			
//    SPI_LTC6804_CS_HIGH();
// 
//    for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++) 			//executes for each LTC6804 in the daisy chain and packs the data
//    { 																			//into the r_config array as well as check the received Config data
//																				//for any bit errors	
//        //4.a																		
//        for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++)					
//        {
//            r_config[current_ic][current_byte] = rx_data[current_byte + (current_ic*BYTES_IN_REG)];
//        }
//        
//        //4.b
//        received_pec = (r_config[current_ic][6]<<8) + r_config[current_ic][7];
//        
//        //data_pec = pec15_calc(6, &r_config[current_ic][0]);
//        data_pec = LTC68041_pec15_calc(6, &r_config[current_ic][0]);
//        
//        if(received_pec != data_pec)
//        {
//            pec_error = -1;
//        }  
//  }
//  
//  free(rx_data);
//  
//  //5
//  return(pec_error);
//}

///*!
// \brief Writes and read a set number of bytes using the SPI port.

//@param[in] uint8_t tx_data[] array of data to be written on the SPI port
//@param[in] uint8_t tx_len length of the tx_data array
//@param[out] uint8_t rx_data array that read data will be written too.
//@param[in] uint8_t rx_len number of bytes to be read from the SPI port.

//*/

//void LTC68041_spi_write_read (uint8_t tx_Data[],//array of data to be written on SPI port
//                    uint8_t tx_len, //length of the tx data arry
//                    uint8_t *rx_data,//Input: array that will store the data read by the SPI port
//                    uint8_t rx_len //Option: number of bytes to be read from the SPI port
//                   )
//{
//  for (uint8_t i = 0; i < tx_len; i++)
//  {
//      //Spi_write(tx_Data[i]);
//      SPI3_SendByte(tx_Data[i]);
//  }

//  for (uint8_t i = 0; i < rx_len; i++)
//  {
//      //rx_data[i] = (uint8_t)Spi_read(0xFF);
//      rx_data[i] = SPI3_ReadByte();
//  }

//}

///*!**********************************************************
// \brief calaculates  and returns the CRC15

//  @param[in] uint8_t len: the length of the data array being passed to the function

//  @param[in] uint8_t data[] : the array of data that the PEC will be generated from


//  @returns The calculated pec15 as an unsigned int
//***********************************************************/
//uint16_t LTC68041_pec15_calc(uint8_t len, //Number of bytes that will be used to calculate a PEC
//                    uint8_t *data //Array of data that will be used to calculate  a PEC
//                   )
//{
//  uint16_t remainder,addr;

//  remainder = 16;//initialize the PEC
//  for (uint8_t i = 0; i<len; i++) // loops for each byte in data array
//  {
//    addr = ((remainder>>7)^data[i])&0xff;//calculate PEC table address
//    remainder = (remainder<<8)^crc15Table[addr];
//  }
//  return(remainder*2);//The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
//}


/*****************************************************//**
 \brief Write the LTC6804 configuration register

 This command will write the configuration registers of the LTC6804-1s
 connected in a daisy chain stack. The configuration is written in descending
 order so the last device's configuration is written first.

 @param[in] uint8_t total_ic; The number of ICs being written to.

 @param[in] uint8_t config[][6] is a two dimensional array of the configuration data that will be written, the array should contain the 6 bytes for each
 IC in the daisy chain. The lowest IC in the daisy chain should be the first 6 byte block in the array. The array should
 have the following format:
 |  config[0][0]| config[0][1] |  config[0][2]|  config[0][3]|  config[0][4]|  config[0][5]| config[1][0] |  config[1][1]|  config[1][2]|  .....    |
 |--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|-----------|
 |IC1 CFGR0     |IC1 CFGR1     |IC1 CFGR2     |IC1 CFGR3     |IC1 CFGR4     |IC1 CFGR5     |IC2 CFGR0     |IC2 CFGR1     | IC2 CFGR2    |  .....    |

 The function will calculate the needed PEC codes for the write data
 and then transmit data to the ICs on a daisy chain.


Command Code:
-------------
|               |             CMD[0]                              |                            CMD[1]                             |
|---------------|---------------------------------------------------------------|---------------------------------------------------------------|
|CMD[0:1]     |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|WRCFG:         |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |
********************************************************/
//void LTC6804_wrcfg(uint8_t total_ic, //The number of ICs being written to
//                   uint8_t config[][6] //A two dimensional array of the configuration data that will be written
//                  )
//{
//    const uint8_t BYTES_IN_REG = 6;
//    const uint8_t CMD_LEN = 4+(8*total_ic);
//    uint8_t *cmd;
//    uint16_t cfg_pec;
//    uint8_t cmd_index; //command counter

//    cmd = (uint8_t *)malloc(CMD_LEN*sizeof(uint8_t));

//    //1
//    cmd[0] = 0x00;
//    cmd[1] = 0x01;
//    cmd[2] = 0x3d;
//    cmd[3] = 0x6e;

//    //2
//    cmd_index = 4;
//    for (uint8_t current_ic = total_ic; current_ic > 0; current_ic--)       // executes for each LTC6804 in daisy chain, this loops starts with
//    {
//        // the last IC on the stack. The first configuration written is
//        // received by the last IC in the daisy chain

//        for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++) // executes for each of the 6 bytes in the CFGR register
//        {
//        // current_byte is the byte counter

//            cmd[cmd_index] = config[current_ic-1][current_byte];            //adding the config data to the array to be sent
//            cmd_index = cmd_index + 1;
//        }
//        //3
//        cfg_pec = (uint16_t)pec15_calc(BYTES_IN_REG, &config[current_ic-1][0]);   // calculating the PEC for each ICs configuration register data
//        cmd[cmd_index] = (uint8_t)(cfg_pec >> 8);
//        cmd[cmd_index + 1] = (uint8_t)cfg_pec;
//        cmd_index = cmd_index + 2;
//  }

//    //4
//    wakeup_idle ();                                 //This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.
//    //5
//    output_low(LTC6804_CS);
//    spi_write_array(CMD_LEN, cmd);
//    output_high(LTC6804_CS);
//    free(cmd);
//}













