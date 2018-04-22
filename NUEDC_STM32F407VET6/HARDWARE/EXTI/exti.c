#include "RoboModule_can2.h"
#include "Relay.h"
#include "EXTI.H"
#include "delay.h"
#include "beep.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//外部中断 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/4
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
int InitPosition=0;
void EXTI9_5_IRQHandler(void)
{
	
	if(Light1==0)
	{
		delay_ms(10);
		if(Light1==0&&Light1Flag==0)
		{
		  CAN2_RoboModule_DRV_Velocity_Position_Mode(1,2,5000,1000,Real2_Position_Value[1]);
		  Light1Flag=1;
		  InitPosition=Real2_Position_Value[1];
		}
	}
	 EXTI_ClearITPendingBit(EXTI_Line6);//清除LINE2上的中断标志位 
}
//外部中断3服务程序

	   



//外部中断初始化程序
//初始化PE2~4,PA0为中断输入.
void EXTIX_Init(void)
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	
 
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource6);//PE2 连接到中断线2

	/* 配置EXTI_Line2,3,4 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line6 ;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿触发
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//中断线使能
  EXTI_Init(&EXTI_InitStructure);//配置
	 
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//外部中断4
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//抢占优先级1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置

}












