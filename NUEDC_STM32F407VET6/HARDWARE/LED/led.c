/***********************************************************************\
作者信息：
姓名：DAHU	        文件名称; LED.C
文件功能：
本文件用于电设PCB板：LIKE的LED指示灯初始化及一些全局变量定义
其他说明：
本文件仅适用于李珂画的2017电子设计PCB电路板，IO初始化都是针对这个电路板的
模块历史：
本文件在2017年10月17日重写，主要针对PCB对应的初始化
本模块的函数于当日测试，测试内容：IO初始化、所有LED的功能
\***********************************************************************/
#include "led.h" 



/***********************************************************************\
函数名称： LED_Iint()
功能描述：
初始化电路板上的按键IO
函数参数：void
返回值： void
模块历史：
注释与2018年2月1号修改
相关功能于2月1号测试
备注：
PC0--LED1   PC1--LED2  PC2--LED3   PC3--LED4   
\***********************************************************************/
void LED_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOD时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1|GPIO_Pin_2 | GPIO_Pin_3;//PC0、PC1、PC2、PC3分别对应LED1、LED2、LED3、LED4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化
	GPIO_SetBits(GPIOC, GPIO_Pin_10);

	GPIO_SetBits(GPIOC, GPIO_Pin_0| GPIO_Pin_1|GPIO_Pin_2 | GPIO_Pin_3);//置高，灯灭
}






