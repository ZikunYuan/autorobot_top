#include "Stepper.h"
#include "led.h"
#include "usart.h"
 #include"stm32f4xx.h"
	 
u16 Step1 = 0, Step2 = 0;
u16 Step1_Num = 0, Step2_Num = 0;
void Stepper_Init()
{
	Stepper1_PWM_Init(2000 - 1, 84 - 1);	//84M/84=1Mhz的计数频率,重装载值2000，所以PWM频率为 1M/2000=500hz.
	Stepper2_PWM_Init(2000 - 1, 84 - 1);
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOE时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4;//LED对应引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOF时钟

}


/*定时器10 PB8用来驱动步进电机1*/
void Stepper1_PWM_Init(u32 arr, u32 psc)
{
	//此部分需手动修改IO口设置

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);  	//TIM14时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//使能PORTF时钟	

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM10); //GPIOF9复用为定时器14

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);              //初始化PF9

	TIM_TimeBaseStructure.TIM_Prescaler = psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period = arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);//初始化定时器14

												   //初始化TIM14 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
	TIM_OC1Init(TIM10, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1
	TIM_OC1PreloadConfig(TIM10, TIM_OCPreload_Enable);  //使能TIM14在CCR1上的预装载寄存器


	TIM_ARRPreloadConfig(TIM10, ENABLE);//ARPE使能 

	TIM_Cmd(TIM10, ENABLE);  //使能TIM14
	TIM_ITConfig(TIM10, TIM_IT_CC1, ENABLE); //允许定时器6更新中断
	TIM_ClearITPendingBit(TIM10, TIM_IT_CC1);

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00; //抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}



/*定时器11CH1 PB9用来驱动步进电机2*/
//注意：之前有用定时器11来控制电机，初始化时需注意
void Stepper2_PWM_Init(u32 arr, u32 psc)
{
	//此部分需手动修改IO口设置

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);  	//TIM14时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//使能PORTF时钟	

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM11); //GPIOF9复用为定时器14

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);              //初始化PF9

	TIM_TimeBaseStructure.TIM_Prescaler = psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period = arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(TIM11, &TIM_TimeBaseStructure);//初始化定时器14

													//初始化TIM14 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
	TIM_OC1Init(TIM11, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1
	TIM_OC1PreloadConfig(TIM11, TIM_OCPreload_Enable);  //使能TIM14在CCR1上的预装载寄存器


	TIM_ARRPreloadConfig(TIM11, ENABLE);//ARPE使能 

	TIM_Cmd(TIM11, ENABLE);  //使能TIM14
	TIM_ITConfig(TIM11, TIM_IT_CC1, ENABLE); //允许定时器6更新中断
	TIM_ClearITPendingBit(TIM11, TIM_IT_CC1);

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_TIM11_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00; //抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}
//void Stepper_PWM_Init(u32 arr,u32 psc)
//{		 					 
//	//此部分需手动修改IO口设置
//	
//	GPIO_InitTypeDef GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//  	TIM_OCInitTypeDef  TIM_OCInitStructure;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);  	//TIM14时钟使能    
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//使能PORTF时钟	
//	
//	GPIO_PinAFConfig(GPIOE,GPIO_PinSource5,GPIO_AF_TIM9); //GPIOF9复用为定时器14
//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9); //GPIOF9复用为定时器14
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_6;           //GPIOF9
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
//	GPIO_Init(GPIOE,&GPIO_InitStructure);              //初始化PF9
//	  
//	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
//	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
//	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
//	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
//	
//	TIM_TimeBaseInit(TIM9,&TIM_TimeBaseStructure);//初始化定时器14
//	
//	//初始化TIM14 Channel1 PWM模式	 
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
// 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
//	TIM_OC1Init(TIM9, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1
//	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);  //使能TIM14在CCR1上的预装载寄存器
// 
//	TIM_OC2Init(TIM9, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1
//	TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);  //使能TIM14在CCR1上的预装载寄存器
//
//    TIM_ARRPreloadConfig(TIM9,ENABLE);//ARPE使能 
//	
//	TIM_Cmd(TIM9, ENABLE);  //使能TIM14
// 	TIM_ITConfig(TIM9,TIM_IT_CC1,ENABLE); //允许定时器6更新中断
//	TIM_ITConfig(TIM9, TIM_IT_CC2, ENABLE); //允许定时器6更新中断
//
//
// 	NVIC_InitStructure.NVIC_IRQChannel=TIM1_BRK_TIM9_IRQn;  
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; //抢占优先级0
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01; //子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//										  
//}  

/*TIM10中断函数 用来计数，控制pwm波输出脉冲个数*/
void TIM1_UP_TIM10_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM10, TIM_IT_CC1) == SET)
	{
		TIM_ClearITPendingBit(TIM10, TIM_IT_CC1);
		Step1++;
		if (Step1 >= Step1_Num)
		{
			TIM_SetCompare1(TIM10, 0);	//修改比较值，修改占空比
			Step1 = 0;
			Step1_Num = 0;
		}

	}
}



/*TIM11中断函数 计数加一，控制pwm波脉冲输出个数*/
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM11,TIM_IT_CC1)==SET)
	{
	   TIM_ClearITPendingBit(TIM11,TIM_IT_CC1);
	   Step2++;
		if(Step2>=Step2_Num)
		{
			TIM_SetCompare1(TIM11, 0);	//修改比较值，修改占空比
			Step2=0;
			Step2_Num = 0;
         }
	
}
}




void Stepper1_TurnDir_StepNum(int step1)
{
	Step1_Num = 0;
	TIM_SetCompare1(TIM10, 0);	//修改比较值，修改占空比
	Step1 = 0;
	if (step1 < 0)
	{
		Dir1 = 0;
		Step1_Num = -step1;
	}
	else
	{
		Dir1 = 1;
		Step1_Num = step1;
	}
	   TIM_SetCompare1(TIM10, 1000);	//修改比较值，修改占空比
	   //if (step1<0)		OLED_ShowString(80, 20, "-"),
		  // OLED_ShowNumber(95, 20, -step1, 3, 12);
	   //else                 	OLED_ShowString(80, 20, "+"),
		  // OLED_ShowNumber(95, 20, step1, 3, 12);
}


void Stepper2_TurnDir_StepNum(int step2)
{
	TIM_SetCompare1(TIM11, 0);	//修改比较值，修改占空比
	Step2 = 0;
	Step2_Num = 0;
	if (step2 < 0)
	{
		Dir2 = 0;
		Step2_Num = -step2;
	}
	else
	{
		Dir2 = 1;
		Step2_Num = step2;
	}

	TIM_SetCompare1(TIM11, 1000);	//修改比较值，修改占空比
	//if (step2<0)		OLED_ShowString(80, 30, "-"),
	//	OLED_ShowNumber(95, 30, -step2, 3, 12);
	//else               		OLED_ShowString(80, 30, "+"),
	//	OLED_ShowNumber(95, 30, step2, 3, 12);

}


//void TIM1_BRK_TIM9_IRQHandler(void)
//{
//	if(TIM_GetITStatus(TIM9,TIM_IT_CC1)==SET)
//	{
//	   TIM_ClearITPendingBit(TIM9,TIM_IT_CC1);
//	   Step1++;
//		if(Step1>=Step1_Num)
//		{
//			Step1=0;
//			Step1_Num = 0;
//			TIM_SetCompare1(TIM9, 0);	//修改比较值，修改占空比
//         }
//	
//	if(TIM_GetITStatus(TIM9,TIM_IT_CC2)==SET)
//	{
//	   TIM_ClearITPendingBit(TIM9,TIM_IT_CC2);
//	   Step2++;
//		if(Step2>= Step2_Num)
//		{
//			Step2=0;
//			Step2_Num = 0;
//			TIM_SetCompare2(TIM9, 0);	//修改比较值，修改占空比
//         }
//  }
//}
//}





