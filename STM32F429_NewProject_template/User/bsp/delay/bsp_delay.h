/**
  ******************************************************************************
  * @file    bsp_delay.h
  * @author  oldfisherman
  * @version V0.1
  * @date    2016-12-13
  * @brief   延时变量、函数定义与声明
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

void TimingDelay_Decrement(void);       //通用定时器中断次数累积函数


#endif 
/**********************************end of file**********************************/
