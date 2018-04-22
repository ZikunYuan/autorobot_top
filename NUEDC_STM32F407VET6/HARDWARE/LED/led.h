/***********************************************************************\
作者信息：
姓名：DAHU	        文件名称; LED.H
文件功能：
宏定义和函数声明
其他说明：
本文件仅适用于李珂画的2017电子设计PCB电路板，IO初始化都是针对这个电路板的
模块历史：
本文件在2017年10月17日重写，主要针对PCB对应的初始化
本模块的函数于当日测试，测试内容：IO初始化、所有LED的功能
stm32f4不需要考虑关闭JATG，直接初始化即可覆盖其功能
\***********************************************************************/

#ifndef __LED_H
#define __LED_H
#include "sys.h"

#define LED1 PCout(0)	
#define LED2 PCout(1)	 
#define LED3 PCout(2)	
#define LED4 PCout(3)	


void LED_Init(void);//初始化		 				    
#endif
