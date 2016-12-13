/**
  ******************************************************************************
  * @file    bsp_delay.h
  * @author  oldfisherman
  * @version V0.1
  * @date    2016-12-13
  * @brief   ��ʱ��������������������
  ******************************************************************************
  * @attention
  *
  * 
  * 
  * 
  *
  ******************************************************************************
**/

#ifndef __BSP_DELAY_H__
#define __BSP_DELAY_H__

#include "stm32f4xx.h"

void Delay_Soft(__IO uint32_t nCount);
void Delay_10_us(__IO u32 nTime);

void TimingDelay_Decrement(void);       //ͨ�ö�ʱ���жϴ����ۻ�����


#endif 
/**********************************end of file**********************************/
