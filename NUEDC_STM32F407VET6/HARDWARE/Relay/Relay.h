/***********************************************************************\
作者信息：
姓名：DAHU	        文件名称; Relay.H
文件功能：
宏定义和函数声明
其他说明：
本文件仅适用于李珂画的2017电子设计PCB电路板，IO初始化都是针对这个电路板的
模块历史：
本文件在2017年11月23日创建，主要针对PCB对应的初始化
本模块的函数于当日测试，测试内容：IO初始化、以及IO功能
stm32f4不需要考虑关闭JATG，直接初始化即可覆盖其功能
\***********************************************************************/

#ifndef __relay_H
#define __relay_H
#include "sys.h"

#define Relay1 PDout(14) //大气缸
#define Relay2 PDout(13) //固定球的气缸
#define Relay3 PDout(15)	 //爪子//默认高电平 为闭合状态
#define Relay4 PCout(8)	 //电磁铁1
#define Relay5 PCout(9)	 //电磁铁2
//#define Relay6 PDout(6)	  //此继电器未用

////#define TouchSwitch1 PCin(12)
//#define TouchSwitch2 PDin(1)
//#define TouchSwitch3 PDin(3)

#define Light1 PBin(6) 	  //光纤1传感器
#define Light2 PBin(5)		//光纤2传感器

void LightEletric_Init(void);
//void TouchSwitch_Init(void);
void Relay_Init(void);//初始化

extern int Light1Flag;
#endif
