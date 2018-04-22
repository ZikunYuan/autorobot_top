/***********************************************************************\
作者：DAHU	        文件名称; Sterring.c
文件功能：
本文件用TIM11CH1来控制舵机PB9，包含IO、定时器初始化，以及舵机角度设置函数
其他说明：
本文件仅适用于李珂画的2017电子设计PCB电路板，IO初始化都是针对这个电路板的
模块历史：
本文件在2017年11月4日创建，主要针对PCB对应的初始化
本模块的函数于当日测试，测试内容：IO初始化、定时器驱动舵机旋转角度
\***********************************************************************/

#include<sterring.h>
#include<time.h>
#include<stm32f4xx.h>

void SterringInit(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);/*使能GPIOF时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);/*使能定时器11时钟*/

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM11);/*复用*/

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;/*复用*/
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;/*推挽输出*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;/*PF7*/
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;/*上拉*/
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;/**/
	GPIO_Init(GPIOB, &GPIO_InitStructure);/*初始化IO*/

	TIM_TimeBaseInitStructure.TIM_Period = 19999;/*自动重装载*/
	TIM_TimeBaseInitStructure.TIM_Prescaler = 83;/*预分频*/
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;/*时钟分频*/
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;/*向上计数*/
	TIM_TimeBaseInit(TIM11, &TIM_TimeBaseInitStructure);/*初始化*/

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;/*PWM模式*/
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;/*输出*/
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;/*起始是低*/
	TIM_OC1Init(TIM11, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM11, TIM_OCPreload_Enable);/*输出比较预装载使能*/
	TIM_ARRPreloadConfig(TIM11, ENABLE);/*自动重载预装载使能*/
	TIM_Cmd(TIM11, ENABLE);/*计数使能*/
	TIM_SetCompare1(TIM11, 15);
}

/***********************************************************************\
函数名称： SterringSet()
功能描述：
通过改变TIM11CH1的占空比来控制舵机的位置
函数参数：void                      返回值  ：void
模块历史：
本函数于2017年11月4号
相关功能于2017年11月4号测试
备注    ：
寄存器CCR1可设置的范围为：500--2500  对应舵机的0--180度（满占空比值为：20000）
对于（180度角度伺服）舵机：输入脉冲周期：20ms
          角度与高电平时间对应关系：
		                      0.5ms————0度
							  1.5ms————90度
							  2.5ms————180度
\***********************************************************************/
void SterringSet(u16 angle)
{
	//u16 num;
	//num =(int) (angle*11.1 + 500);
	TIM11->CCR1 = angle;
}


