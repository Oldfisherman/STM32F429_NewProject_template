/**
  ******************************************************************************
  * @file    bsp_delay.c
  * @author  oldfisherman
  * @version V0.1
  * @date    2016-12-13
  * @brief   实现延时函数，包含软件延时与精确基本定时器延时
  ******************************************************************************
  * @attention
  *
  * 
  * 
  * 
  *
  ******************************************************************************
**/
  
#include "./bsp_delay.h"
#include "../tim/bsp_basic_tim.h"
  
static __IO u32 TimingDelay;
  
/**
  * @brief   us延时程序,10us为一个单位
  * @param  
  *	@arg nTime: Delay_us( 1 ) 则实现的延时为 1 * 10us = 10us
  * @retval  无
**/
void Delay_10_us(__IO u32 nTime)
{ 
	TimingDelay = nTime;	

    TIM_Cmd(BASIC_TIM, ENABLE);
    
	while(TimingDelay != 0)
    {
        ;
    }
    
    TIM_Cmd(BASIC_TIM, DISABLE);    
}

/**
  * @brief  获取节拍程序
  * @param  无
  * @retval 无
  * @attention  
  */
void TimingDelay_Decrement(void)
{
	if (TimingDelay != 0)
	{ 
		TimingDelay--;
	}
}

/**
  * @brief  软件延时函数
  * @param  无
  * @retval 无
  * @attention  
  */
void Delay_Soft(__IO uint32_t nCount)	 //简单的延时函数
{
	for(; nCount != 0; nCount--);
}

/**********************************end of file**********************************/
