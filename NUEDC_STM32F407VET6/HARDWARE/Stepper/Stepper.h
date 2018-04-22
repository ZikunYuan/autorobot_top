#ifndef _STEPPER_H
#define _STEPPER_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//定时器 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/6/16
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
#define Dir2 PEout(6)	// DS0
#define Dir1 PEout(5)	// DS1	
void Stepper1_TurnDir_StepNum( int step1);
void Stepper2_TurnDir_StepNum(int step2);
void Stepper1_PWM_Init(u32 arr,u32 psc);
void Stepper2_PWM_Init(u32 arr, u32 psc);
void Stepper_Init(void);
#endif

