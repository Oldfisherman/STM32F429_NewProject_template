/**
  ******************************************************************************
  * @file    bsp_delay.c
  * @author  oldfisherman
  * @version V0.1
  * @date    2016-12-13
  * @brief   ʵ����ʱ���������������ʱ�뾫ȷ������ʱ����ʱ
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
  * @brief   us��ʱ����,10usΪһ����λ
  * @param  
  *	@arg nTime: Delay_us( 1 ) ��ʵ�ֵ���ʱΪ 1 * 10us = 10us
  * @retval  ��
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
  * @brief  ��ȡ���ĳ���
  * @param  ��
  * @retval ��
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
  * @brief  �����ʱ����
  * @param  ��
  * @retval ��
  * @attention  
  */
void Delay_Soft(__IO uint32_t nCount)	 //�򵥵���ʱ����
{
	for(; nCount != 0; nCount--);
}

/**********************************end of file**********************************/
