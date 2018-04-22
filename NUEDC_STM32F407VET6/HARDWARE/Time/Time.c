#include <string.h>
#include <stdint.h>
#include "Time.h"
#include "stm32f4xx.h"
#include "led.h"
#include "main_assistant.h"

System_Time_TypeDef System_Time;    //系统时间

/**********************************************
* 清空系统时间
**********************************************/
void System_Tiem_Clear(void)
{
    memset(&System_Time, 0, sizeof(System_Time_TypeDef));
}

/**********************************************
* 记录系统时间
**********************************************/
void System_Time_Running(void)
{
    if (System_Time.Milliseconds < 999)
    {
        System_Time.Milliseconds++;
    }
    else
    {
        System_Time.Milliseconds = 0;
        if (System_Time.Seconds<59)
        {
            System_Time.Seconds++;
        }
        else
        {
            System_Time.Seconds = 0;
            if (System_Time.Minutes<59)
            {
                System_Time.Minutes++;
            }
            else
            {
                System_Time.Minutes = 0;
                if (System_Time.Hours<23)
                {
                    System_Time.Hours++;
                }
                else
                {
                    System_Time.Hours = 0;
                }
            }
        }
    }
}


#define GET_TIME_NUM 	(5)		//设置获取时间的数组数量
volatile float Cycle_T[GET_TIME_NUM][3];

enum
{
    NOW = 0,
    OLD,
    NEW,
};

float Get_Cycle_T(uint8_t item)
{
    Cycle_T[item][OLD] = Cycle_T[item][NOW];	//上一次的时间
    Cycle_T[item][NOW] = GetSysTime_us() / 1000000.0f; //本次的时间
    Cycle_T[item][NEW] = ((Cycle_T[item][NOW] - Cycle_T[item][OLD]));//间隔的时间（周期）
    return Cycle_T[item][NEW];
}

void Cycle_Time_Init(void)
{
    uint8_t i;
    for (i = 0; i<GET_TIME_NUM; i++)
    {
        Get_Cycle_T(i);
    }
}

void TIM4_Int_Init(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  ///使能TIM4时钟

	TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);//初始化TIM3

	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM4, ENABLE); //使能定时器3

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}



void TIM6_Int_Init(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);  ///使能TIM4时钟

	TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStructure);//初始化TIM3

	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM6, ENABLE); //使能定时器3

	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


void TIM7_Int_Init(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);  ///使能TIM4时钟

	TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseInitStructure);//初始化TIM3

	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM7, ENABLE); //使能定时器3

	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM7_IRQHandler(void)   
{
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)  
		{
			CAN1_SendMsg(Compensation_Mode(TIM5->CNT),Elmo_ID);//²¹³¥±àÂëÆ÷
		  delay_ms(5);
		  TIM_ClearITPendingBit(TIM7, TIM_IT_Update  );   
		}
}


