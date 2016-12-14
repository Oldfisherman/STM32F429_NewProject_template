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
  
#include "../../bsp/spi/bsp_spi3.h"
#include "../../bsp/delay/bsp_delay.h"
#include "../../My_Peripheral_Driver/ltc6804/LTC68041.h"
#include <stdlib.h>


/**************************************************************
                My_LTC6804_C_File_CODE
**************************************************************/

/*!
  6804 conversion command variables.
*/
uint8_t ADCV[2]; //!< Cell Voltage conversion command.
uint8_t ADAX[2]; //!< GPIO conversion command.


//
//
//
const uint8_t TOTAL_IC = 2;//!<number of ICs in the daisy chain

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

void LTC68041_print_config(void)
{
  int cfg_pec;

  printf("Written Configuration: \n");
  for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
  {
    printf(" IC ");
    printf("%02d",current_ic+1);
    printf(": ");
    printf("0x");
    serial_print_hex(tx_cfg[current_ic][0]);
    printf(", 0x");
    serial_print_hex(tx_cfg[current_ic][1]);
    printf(", 0x");
    serial_print_hex(tx_cfg[current_ic][2]);
    printf(", 0x");
    serial_print_hex(tx_cfg[current_ic][3]);
    printf(", 0x");
    serial_print_hex(tx_cfg[current_ic][4]);
    printf(", 0x");
    serial_print_hex(tx_cfg[current_ic][5]);
    printf(", Calculated PEC: 0x");
    cfg_pec = LTC68041_pec15_calc(6,&tx_cfg[current_ic][0]);
    serial_print_hex((uint8_t)(cfg_pec>>8));
    printf(", 0x");
    serial_print_hex((uint8_t)(cfg_pec));
    printf("\n");
  }
  printf("\n");
}

/*!************************************************************
  \brief Prints cell coltage codes to the serial port
 *************************************************************/
void LTC68041_print_cells(void)
{
    for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
    {
        printf(" IC ");
        printf("%02d",current_ic+1);
        for (int i=0; i<12; i++)
        {
            printf(" C");
            printf("%02d",i+1);
            printf(":");
            printf("%f",cell_codes[current_ic][i]*0.0001);
            printf(",");
        }   
        printf("\n"); 
    }
        printf("\n");
}
//
//
//

 /**
  * @brief  LTC68041 SPI 初始化
  * @param  无
  * @retval 无
  */
void LTC68041_init(void) //LTC6804 Initialize
{
    init_cfg();
    LTC6804_set_adc(MD_NORMAL,0,CELL_CH_ALL,AUX_CH_ALL);  
    Spi_ltc68041_init();   
}

/**************************************************************
**************************************************************/

 /**
  * @brief  LTC68041 SPI 初始化
  * @param  无
  * @retval 无
  */
void Spi_ltc68041_init(void) //LTC6804 SPI Initialize
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

/*!****************************************************
  \brief Wake the LTC6804 from the sleep state

 Generic wakeup commannd to wake the LTC6804 from sleep
T_wakeup max 300us
 *****************************************************/
void LTC68041_wakeup_sleep( void )
{
  SPI_LTC6804_CS_LOW();
  Delay_10_us(40); // Guarantees the LTC6804 will be in standby
  SPI_LTC6804_CS_HIGH();
}

/*!****************************************************
  \brief Wake isoSPI up from idle state
 Generic wakeup commannd to wake isoSPI up out of idle
 *****************************************************/
void LTC68041_wakeup_idle( void )
{
    SPI_LTC6804_CS_LOW();
    //delayMicroseconds(2); Guarantees the isoSPI will be in ready mode
    Delay_10_us(300);
    SPI_LTC6804_CS_HIGH();
}

/*!******************************************************
 \brief Reads configuration registers of a LTC6804 daisy chain
 
@param[in] uint8_t total_ic: number of ICs in the daisy chain

@param[out] uint8_t r_config[][8] is a two dimensional array that the function stores the read configuration data. The configuration data for each IC 
is stored in blocks of 8 bytes with the configuration data of the lowest IC on the stack in the first 8 bytes 
block of the array, the second IC in the second 8 byte etc. Below is an table illustrating the array organization:

|r_config[0][0]|r_config[0][1]|r_config[0][2]|r_config[0][3]|r_config[0][4]|r_config[0][5]|r_config[0][6]  |r_config[0][7] |r_config[1][0]|r_config[1][1]|  .....    |
|--------------|--------------|--------------|--------------|--------------|--------------|----------------|---------------|--------------|--------------|-----------|
|IC1 CFGR0     |IC1 CFGR1     |IC1 CFGR2     |IC1 CFGR3     |IC1 CFGR4     |IC1 CFGR5     |IC1 PEC High    |IC1 PEC Low    |IC2 CFGR0     |IC2 CFGR1     |  .....    |


@return int8_t, PEC Status.
 
	0: Data read back has matching PEC
 
	-1: Data read back has incorrect PEC


Command Code:
-------------

|CMD[0:1]		|  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   | 
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|RDCFG:	        |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   0   |   1   |   0   |
********************************************************/
int8_t LTC68041_rdcfg(uint8_t total_ic, //Number of ICs in the system
				     uint8_t r_config[][8] //A two dimensional array that the function stores the read configuration data.
					 )
{
    const uint8_t BYTES_IN_REG = 8;
  
    uint8_t cmd[4];
    uint8_t *rx_data;
    int8_t pec_error = 0; 
    uint16_t data_pec;
    uint16_t received_pec;
  
    rx_data = (uint8_t *) malloc((8*total_ic)*sizeof(uint8_t));
  
    //1
    cmd[0] = 0x00;
    cmd[1] = 0x02;
    cmd[2] = 0x2b;
    cmd[3] = 0x0A;
 
    //2
    //wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
    LTC68041_wakeup_idle();
    
    //3
    //output_low(LTC6804_CS);
    SPI_LTC6804_CS_LOW();
  
    LTC68041_spi_write_read(cmd, 4, rx_data, (BYTES_IN_REG*total_ic));         //Read the configuration data of all ICs on the daisy chain into 
  
    //output_high(LTC6804_CS);													//rx_data[] array			
    SPI_LTC6804_CS_HIGH();
 
    for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++) 			//executes for each LTC6804 in the daisy chain and packs the data
    { 																			//into the r_config array as well as check the received Config data
																				//for any bit errors	
        //4.a																		
        for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++)					
        {
            r_config[current_ic][current_byte] = rx_data[current_byte + (current_ic*BYTES_IN_REG)];
        }
        
        //4.b
        received_pec = (r_config[current_ic][6]<<8) + r_config[current_ic][7];
        
        //data_pec = pec15_calc(6, &r_config[current_ic][0]);
        data_pec = LTC68041_pec15_calc(6, &r_config[current_ic][0]);
        
        if(received_pec != data_pec)
        {
            pec_error = -1;
        }  
  }
  
  free(rx_data);
  
  //5
  return(pec_error);
}

/*!
 \brief Writes and read a set number of bytes using the SPI port.

@param[in] uint8_t tx_data[] array of data to be written on the SPI port
@param[in] uint8_t tx_len length of the tx_data array
@param[out] uint8_t rx_data array that read data will be written too.
@param[in] uint8_t rx_len number of bytes to be read from the SPI port.

*/

void LTC68041_spi_write_read (uint8_t tx_Data[],//array of data to be written on SPI port
                    uint8_t tx_len, //length of the tx data arry
                    uint8_t *rx_data,//Input: array that will store the data read by the SPI port
                    uint8_t rx_len //Option: number of bytes to be read from the SPI port
                   )
{
  for (uint8_t i = 0; i < tx_len; i++)
  {
      //Spi_write(tx_Data[i]);
      SPI3_SendByte(tx_Data[i]);
  }

  for (uint8_t i = 0; i < rx_len; i++)
  {
      //rx_data[i] = (uint8_t)Spi_read(0xFF);
      rx_data[i] = SPI3_ReadByte();
  }

}

/*!
 \brief Writes an array of bytes out of the SPI port

 @param[in] uint8_t len length of the data array being written on the SPI port
 @param[in] uint8_t data[] the data array to be written on the SPI port

*/
void LTC68041_spi_write_array(uint8_t len, // Option: Number of bytes to be written on the SPI port
                     uint8_t data[] //Array of bytes to be written on the SPI port
                    )
{
    for (uint8_t i = 0; i < len; i++)
    {
      SPI3_SendByte(data[i]);
    }
}

/*!**********************************************************
 \brief calaculates  and returns the CRC15

  @param[in] uint8_t len: the length of the data array being passed to the function

  @param[in] uint8_t data[] : the array of data that the PEC will be generated from


  @returns The calculated pec15 as an unsigned int
***********************************************************/
uint16_t LTC68041_pec15_calc(uint8_t len, //Number of bytes that will be used to calculate a PEC
                    uint8_t *data //Array of data that will be used to calculate  a PEC
                   )
{
  uint16_t remainder,addr;

  remainder = 16;//initialize the PEC
  for (uint8_t i = 0; i<len; i++) // loops for each byte in data array
  {
    addr = ((remainder>>7)^data[i])&0xff;//calculate PEC table address
    remainder = (remainder<<8)^crc15Table[addr];
  }
  return(remainder*2);//The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}


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
void LTC68041_wrcfg(uint8_t total_ic, //The number of ICs being written to
                   uint8_t config[][6] //A two dimensional array of the configuration data that will be written
                  )
{
    const uint8_t BYTES_IN_REG = 6;
    const uint8_t CMD_LEN = 4+(8*total_ic);
    uint8_t *cmd;
    uint16_t cfg_pec;
    uint8_t cmd_index; //command counter

    cmd = (uint8_t *)malloc(CMD_LEN*sizeof(uint8_t));

    //1
    cmd[0] = 0x00;
    cmd[1] = 0x01;
    cmd[2] = 0x3d;
    cmd[3] = 0x6e;

    //2
    cmd_index = 4;
    for (uint8_t current_ic = total_ic; current_ic > 0; current_ic--)       // executes for each LTC6804 in daisy chain, this loops starts with
    {
        // the last IC on the stack. The first configuration written is
        // received by the last IC in the daisy chain

        for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++) // executes for each of the 6 bytes in the CFGR register
        {
        // current_byte is the byte counter

            cmd[cmd_index] = config[current_ic-1][current_byte];            //adding the config data to the array to be sent
            cmd_index = cmd_index + 1;
        }
        //3
        cfg_pec = (uint16_t)LTC68041_pec15_calc(BYTES_IN_REG, &config[current_ic-1][0]);   // calculating the PEC for each ICs configuration register data
        cmd[cmd_index] = (uint8_t)(cfg_pec >> 8);
        cmd[cmd_index + 1] = (uint8_t)cfg_pec;
        cmd_index = cmd_index + 2;
  }

    //4
    LTC68041_wakeup_idle ();                                 //This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.
    //5
    //output_low(LTC6804_CS);
    SPI_LTC6804_CS_LOW();
  
    //spi_write_array(CMD_LEN, cmd);
    LTC68041_spi_write_array(CMD_LEN,cmd);
  
    //output_high(LTC6804_CS);
    SPI_LTC6804_CS_HIGH();
    free(cmd);
}

/*!**********************************************************
 \brief LTC68041 measurement loop

  @returns none
***********************************************************/
void LTC68041_measurement_loop (void)
{
    int8_t error;
    
    printf("Start LTC68041 measurement loop:\n");
    
    //LTC68041_wakeup_sleep();
    LTC68041_wakeup_idle();
    
    LTC68041_wrcfg(TOTAL_IC,tx_cfg);
    
    while(1)
    {
        LTC68041_wakeup_idle();
        LTC68041_adcv();
        //Delay_10_us(10);
        LTC68041_wakeup_idle();
        
        error = LTC68041_rdcv(0,TOTAL_IC,cell_codes);
        
        if (error == -1)
        {
            printf("A PEC error was detected in the received data");
        }
        
        LTC68041_print_cells();
        //Delay_10_us(1000);
        
    }
}

/*!**********************************************************
 \brief LTC68041 read register

  @returns none
***********************************************************/
void LTC68041_read_register (void)
{
    int8_t error;
    LTC68041_wakeup_sleep();
    error = LTC68041_rdcfg(TOTAL_IC,rx_cfg);
    if (error == -1)
    {
        printf("A PEC error was detected in the received data");
    }
    LTC68041_print_rxconfig();		
    Delay_10_us(100000);
}

/*!**********************************************************
 \brief LTC68041 read register

  @returns none
***********************************************************/
void LTC68041_write_config (void)
{
    LTC68041_wakeup_sleep();
    LTC68041_wrcfg(TOTAL_IC,tx_cfg);
    LTC68041_print_config();    
}

/*!*********************************************************************************************
  \brief Starts cell voltage conversion

  Starts ADC conversions of the LTC6804 Cpin inputs.
  The type of ADC conversion executed can be changed by setting the associated global variables:
 |Variable|Function                                      |
 |--------|----------------------------------------------|
 | MD     | Determines the filter corner of the ADC      |
 | CH     | Determines which cell channels are converted |
 | DCP    | Determines if Discharge is Permitted       |

Command Code:
-------------

|CMD[0:1] |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|ADCV:      |   0   |   0   |   0   |   0   |   0   |   0   |   1   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CH[2] | CH[1] | CH[0] |
***********************************************************************************************/
void LTC68041_adcv(void)
{

    uint8_t cmd[4];
    uint16_t cmd_pec;

    //1
    cmd[0] = ADCV[0];
    cmd[1] = ADCV[1];

    //2
    cmd_pec = LTC68041_pec15_calc(2, ADCV);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    //3
    LTC68041_wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

    //4
    SPI_LTC6804_CS_LOW();
    LTC68041_spi_write_array(4,cmd);
    SPI_LTC6804_CS_HIGH();
}

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
void LTC6804_set_adc(uint8_t MD, //ADC Mode
             uint8_t DCP, //Discharge Permit
             uint8_t CH, //Cell Channels to be measured
             uint8_t CHG //GPIO Channels to be measured
            )
{
    uint8_t md_bits;

    md_bits = (MD & 0x02) >> 1;
    ADCV[0] = md_bits + 0x02;
    md_bits = (MD & 0x01) << 7;
    ADCV[1] =  md_bits + 0x60 + (DCP<<4) + CH;

    md_bits = (MD & 0x02) >> 1;
    ADAX[0] = md_bits + 0x04;
    md_bits = (MD & 0x01) << 7;
    ADAX[1] = md_bits + 0x60 + CHG ;
}

/***********************************************//**
 \brief Reads and parses the LTC6804 cell voltage registers.

 The function is used to read the cell codes of the LTC6804.
 This function will send the requested read commands parse the data
 and store the cell voltages in cell_codes variable.

 @param[in] uint8_t reg; This controls which cell voltage register is read back.

          0: Read back all Cell registers

          1: Read back cell group A

          2: Read back cell group B

          3: Read back cell group C

          4: Read back cell group D

 @param[in] uint8_t total_ic; This is the number of ICs in the daisy chain(-1 only)

 @param[out] uint16_t cell_codes[]; An array of the parsed cell codes from lowest to highest. The cell codes will
  be stored in the cell_codes[] array in the following format:
  |  cell_codes[0][0]| cell_codes[0][1] |  cell_codes[0][2]|    .....     |  cell_codes[0][11]|  cell_codes[1][0] | cell_codes[1][1]|  .....   |
  |------------------|------------------|------------------|--------------|-------------------|-------------------|-----------------|----------|
  |IC1 Cell 1        |IC1 Cell 2        |IC1 Cell 3        |    .....     |  IC1 Cell 12      |IC2 Cell 1         |IC2 Cell 2       | .....    |

  @return int8_t, PEC Status.

    0: No PEC error detected

    -1: PEC error detected, retry read


 *************************************************/
uint8_t LTC68041_rdcv(uint8_t reg, // Controls which cell voltage register is read back.
                     uint8_t total_ic, // the number of ICs in the system
                     uint16_t cell_codes[][12] // Array of the parsed cell codes
                    )
{

    const uint8_t NUM_RX_BYT = 8;
    const uint8_t BYT_IN_REG = 6;
    const uint8_t CELL_IN_REG = 3;

    uint8_t *cell_data;
    int8_t pec_error = 0;
    uint16_t parsed_cell;
    uint16_t received_pec;
    uint16_t data_pec;
    uint8_t data_counter=0; //data counter
    cell_data = (uint8_t *) malloc((NUM_RX_BYT*total_ic)*sizeof(uint8_t));
    
    //1.a
    if (reg == 0)
    {
        //a.i
        for (uint8_t cell_reg = 1; cell_reg<5; cell_reg++)                    //executes once for each of the LTC6804 cell voltage registers
        {
            data_counter = 0;
            LTC68041_rdcv_reg(cell_reg, total_ic,cell_data );                //Reads a single Cell voltage register

            for (uint8_t current_ic = 0 ; current_ic < total_ic; current_ic++)      // executes for every LTC6804 in the daisy chain
            {
                // current_ic is used as the IC counter

                //a.ii
                for (uint8_t current_cell = 0; current_cell<CELL_IN_REG; current_cell++)  // This loop parses the read back data into cell voltages, it
                {
                    // loops once for each of the 3 cell voltage codes in the register

                    parsed_cell = cell_data[data_counter] + (cell_data[data_counter + 1] << 8);//Each cell code is received as two bytes and is combined to
                    // create the parsed cell voltage code

                    cell_codes[current_ic][current_cell  + ((cell_reg - 1) * CELL_IN_REG)] = parsed_cell;
                    data_counter = data_counter + 2;                       //Because cell voltage codes are two bytes the data counter
                    //must increment by two for each parsed cell code
                }
                
        //a.iii
        received_pec = (cell_data[data_counter] << 8) + cell_data[data_counter+1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
        
        //after the 6 cell voltage data bytes
        data_pec = LTC68041_pec15_calc(BYT_IN_REG, &cell_data[current_ic * NUM_RX_BYT]);
        if (received_pec != data_pec)
        {
          pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
          //are detected in the serial data
        }
        data_counter=data_counter+2;                        //Because the transmitted PEC code is 2 bytes long the data_counter
        //must be incremented by 2 bytes to point to the next ICs cell voltage data
      }
    }
  }
//1.b
  else
  {
    //b.i
    LTC68041_rdcv_reg(reg, total_ic,cell_data);
    for (uint8_t current_ic = 0 ; current_ic < total_ic; current_ic++)        // executes for every LTC6804 in the daisy chain
    {
      // current_ic is used as the IC counter
      //b.ii
      for (uint8_t current_cell = 0; current_cell < CELL_IN_REG; current_cell++)  // This loop parses the read back data into cell voltages, it
      {
        // loops once for each of the 3 cell voltage codes in the register

        parsed_cell = cell_data[data_counter] + (cell_data[data_counter+1]<<8); //Each cell code is received as two bytes and is combined to
        // create the parsed cell voltage code

        cell_codes[current_ic][current_cell + ((reg - 1) * CELL_IN_REG)] = 0x0000FFFF & parsed_cell;
        data_counter= data_counter + 2;                       //Because cell voltage codes are two bytes the data counter
        //must increment by two for each parsed cell code
      }
      //b.iii
      received_pec = (cell_data[data_counter] << 8 )+ cell_data[data_counter + 1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
      //after the 6 cell voltage data bytes
      data_pec = LTC68041_pec15_calc(BYT_IN_REG, &cell_data[current_ic * NUM_RX_BYT]);
      if (received_pec != data_pec)
      {
          pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
          //are detected in the serial data
      }
      data_counter= data_counter + 2;                       //Because the transmitted PEC code is 2 bytes long the data_counter
      //must be incremented by 2 bytes to point to the next ICs cell voltage data
    }
  }

//2
  free(cell_data);
  return(pec_error);
}

/***********************************************//**
 \brief Read the raw data from the LTC6804 cell voltage register

 The function reads a single cell voltage register and stores the read data
 in the *data point as a byte array. This function is rarely used outside of
 the LTC6804_rdcv() command.

 @param[in] uint8_t reg; This controls which cell voltage register is read back.

          1: Read back cell group A

          2: Read back cell group B

          3: Read back cell group C

          4: Read back cell group D

 @param[in] uint8_t total_ic; This is the number of ICs in the daisy chain(-1 only)

 @param[out] uint8_t *data; An array of the unparsed cell codes

Command Code:
-------------

|CMD[0:1] |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|RDCVA:     |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   0   |
|RDCVB:     |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   1   |   0   |
|RDCVC:     |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   0   |   0   |
|RDCVD:     |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   1   |   0   |

 *************************************************/
void LTC68041_rdcv_reg(uint8_t reg, //Determines which cell voltage register is read back
                      uint8_t total_ic, //the number of ICs in the
                      uint8_t *data //An array of the unparsed cell codes
                     )
{
    const uint8_t REG_LEN = 8; //number of bytes in each ICs register + 2 bytes for the PEC
    uint8_t cmd[4];
    uint16_t cmd_pec;

    //1
    if (reg == 1)     //1: RDCVA
    {
        cmd[1] = 0x04;
        cmd[0] = 0x00;
    }
    else if (reg == 2) //2: RDCVB
    {
        cmd[1] = 0x06;
        cmd[0] = 0x00;
    }
    else if (reg == 3) //3: RDCVC
    {
        cmd[1] = 0x08;
        cmd[0] = 0x00;
    }
    else if (reg == 4) //4: RDCVD
    {
        cmd[1] = 0x0A;
        cmd[0] = 0x00;
    }

    //2
    cmd_pec = LTC68041_pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    //3
    LTC68041_wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

    //4
    SPI_LTC6804_CS_LOW();
    LTC68041_spi_write_read(cmd,4,data,(REG_LEN*total_ic));
    SPI_LTC6804_CS_HIGH();

}


