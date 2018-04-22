/***********************************************************************\
作者信息：
姓名：DAHU	        文件名称; Relay.C
文件功能：
本文件用于电设PCB板：LIKE的PD6,PD4,PD2,PD0,PC11外设IO作为电磁阀控制IO
其他说明：
本文件仅适用于李珂画的2017电子设计PCB电路板，IO初始化都是针对这个电路板的
模块历史：
本文件在2017年11月23日创建，主要针对PCB对应的初始化
本模块的函数于当日测试，测试内容：IO初始化、IO的输出功能
\***********************************************************************/
#include "Relay.h"


int Light1Flag=0;
/***********************************************************************\
函数名称： Relay_Iint()
功能描述：
初始化电路板上的外设IO
函数参数：void
返回值： void
模块历史：
本模块于2017年11月23日创建
相关功能于10月17号测试
备注：
PCB上的部分丝印有误：PD11、PD10标注分别为实际IO PC11、PC10
实际线路的IO对应关系：
Relay2——PC11          relay3——PD0         relay4——PD2
relay5——PD4           relay6——PD6
\***********************************************************************/
void Relay_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOD时钟

    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化

    GPIO_SetBits(GPIOD, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);//置高，继电器关闭
    GPIO_SetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_9);     //置高，继电器关闭
}


/***********************************************************************\
函数名称： TouchSwitch_Iint()
功能描述：
初始化电路板上的预留IO作为触碰开关的控制IO
函数参数：void
返回值  ：void
模块历史：
本模块于2017年11月25日创建
备注    ：
用两个预留IO,PD5和PD3分别控制开关1和开关2
IO对应关系：
PC12——触碰开关1
PD1——触碰开关2
PD3——触碰开关3
\***********************************************************************/
//void TouchSwitch_Init(void)
//{

//    GPIO_InitTypeDef  GPIO_InitStructure;

//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOD时钟
////	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOD时钟

//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
//    GPIO_Init(GPIOD, &GPIO_InitStructure);

////	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
////	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
////	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
////	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
////	GPIO_Init(GPIOC, &GPIO_InitStructure);
//}



/***********************************************************************\
函数名称： LightEletric_Iint()
功能描述：
初始化电路板上的外设IO，即红外对管使用的IO
函数参数：void
返回值  ：void
模块历史：
本模块于2017年12月3号创建，并于当天测试功能
备注    ：
IO对应关系：
Light1——PE14  红外对管1，当接收不到信号时输出1，接收到信号时输出0
Light2——PE15	光纤传感器1
\***********************************************************************/
void LightEletric_Init(void)
{

    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOD时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}







