/***********************************************************************\
作者：DAHU	        文件名称; KEY.C
文件功能：
         本文件用于电设PCB板：LIKE的按键初始化及一些按键读取函数
其他说明：
         本文件仅适用于李达贤画的2017电子设计PCB电路板，IO初始化都是针对这个电路板的
模块历史：
         本文件在2018年2月1日重写，主要针对PCB对应的初始化
         本模块的函数于当日测试，测试内容：IO初始化、所有按键的功能
\***********************************************************************/
#include "key.h"
#include "delay.h" 


u8  KEY1_flag = 0;
u8  KEY2_flag = 0;
u8  KEY3_flag = 0;

void KEY_Init(void)
{
	
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);//使能GPIOA,GPIOE时钟
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14| GPIO_Pin_15; //PC13、PC14、PC15、对应按键K1、K2、K3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);
 
} 

/***********************************************************************************\
函数名称： KEY_Scan()
功能描述：
扫描按键情况，返回按键键值
函数参数：u8 mode               返回值： 按键键值，若无按键，则返回0
模块历史：
注释与2017年10月17号修改
相关功能于10月17号测试
备注：
此函数为正点原子自带的按键扫描函数
mode=0；不支持连按，按下按键松开，算一次，若按下按键不放开，下次扫描则不再返回键值
mode=1；支持连按，按下按键不松开，下一次扫描依然会返回相应键值
注意：此函数对按键有优先级，优先级按KEY1-KEY6递减
      此函数不支持多按，多按无效
\**********************************************************************************/
u8 KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;//按键按松开标志
	if(mode)key_up=1;  //支持连按		  
	if(key_up&&(KEY1==0||KEY2==0||KEY3==0))
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if (KEY1 == 0)return 1;
		else if (KEY2 == 0)return 2;
		else if(KEY3 == 0)return 3;
    }
	else if(KEY1==1&&KEY2==1&&KEY3==1)key_up=1; 	    
 	return 0;// 无按键按下
}


/***********************************************************************\
函数名称： KEY_MyScan()
功能描述：
扫描按键，通过改变按键标志位的值来表示是否按下按键
函数参数：void        返回值： void
模块历史：
DAHU于2017年10月17日创建此函数
相关功能于10月17号测试
备注：
     按键标志位在下次执行此函数才恢复，若需要可执行函数KEY_ClearFlag()来
	 清除标志位。
\***********************************************************************/
void KEY_MyScan(void)
{
	if (KEY1 == 0|| KEY2 == 0 || KEY3 == 0)
	{
		delay_ms(10);
		if (KEY1 == 0)
			KEY1_flag = 1;
		
		if (KEY2 == 0)
			KEY2_flag = 1;
		
		if (KEY3 == 0)
			KEY3_flag = 1;
		
	}
}

/***********************************************************************\
函数名称： KEY_ClearFlag()
功能描述：
清除按键标志位
函数参数：void        返回值： void
模块历史：
DAHU于2017年10月17日创建此函数
相关功能于10月17号测试
备注：
     需要时可调用
\***********************************************************************/
void KEY_ClearAllFlag(void)
{
	KEY1_flag = 0;
	KEY2_flag = 0;
	KEY3_flag = 0;
}



















