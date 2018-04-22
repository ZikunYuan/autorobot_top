#include "Stepper.h"
#include "led.h"
#include "usart.h"
 #include"stm32f4xx.h"
	 
u16 Step1 = 0, Step2 = 0;
u16 Step1_Num = 0, Step2_Num = 0;
void Stepper_Init()
{
	Stepper1_PWM_Init(2000 - 1, 84 - 1);	//84M/84=1Mhz�ļ���Ƶ��,��װ��ֵ2000������PWMƵ��Ϊ 1M/2000=500hz.
	Stepper2_PWM_Init(2000 - 1, 84 - 1);
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOEʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4;//LED��Ӧ����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOFʱ��

}


/*��ʱ��10 PB8���������������1*/
void Stepper1_PWM_Init(u32 arr, u32 psc)
{
	//�˲������ֶ��޸�IO������

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);  	//TIM14ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//ʹ��PORTFʱ��	

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM10); //GPIOF9����Ϊ��ʱ��14

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOB, &GPIO_InitStructure);              //��ʼ��PF9

	TIM_TimeBaseStructure.TIM_Prescaler = psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period = arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);//��ʼ����ʱ��14

												   //��ʼ��TIM14 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�
	TIM_OC1Init(TIM10, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1
	TIM_OC1PreloadConfig(TIM10, TIM_OCPreload_Enable);  //ʹ��TIM14��CCR1�ϵ�Ԥװ�ؼĴ���


	TIM_ARRPreloadConfig(TIM10, ENABLE);//ARPEʹ�� 

	TIM_Cmd(TIM10, ENABLE);  //ʹ��TIM14
	TIM_ITConfig(TIM10, TIM_IT_CC1, ENABLE); //����ʱ��6�����ж�
	TIM_ClearITPendingBit(TIM10, TIM_IT_CC1);

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00; //��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}



/*��ʱ��11CH1 PB9���������������2*/
//ע�⣺֮ǰ���ö�ʱ��11�����Ƶ������ʼ��ʱ��ע��
void Stepper2_PWM_Init(u32 arr, u32 psc)
{
	//�˲������ֶ��޸�IO������

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);  	//TIM14ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//ʹ��PORTFʱ��	

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM11); //GPIOF9����Ϊ��ʱ��14

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOB, &GPIO_InitStructure);              //��ʼ��PF9

	TIM_TimeBaseStructure.TIM_Prescaler = psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period = arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(TIM11, &TIM_TimeBaseStructure);//��ʼ����ʱ��14

													//��ʼ��TIM14 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�
	TIM_OC1Init(TIM11, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1
	TIM_OC1PreloadConfig(TIM11, TIM_OCPreload_Enable);  //ʹ��TIM14��CCR1�ϵ�Ԥװ�ؼĴ���


	TIM_ARRPreloadConfig(TIM11, ENABLE);//ARPEʹ�� 

	TIM_Cmd(TIM11, ENABLE);  //ʹ��TIM14
	TIM_ITConfig(TIM11, TIM_IT_CC1, ENABLE); //����ʱ��6�����ж�
	TIM_ClearITPendingBit(TIM11, TIM_IT_CC1);

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_TIM11_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00; //��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}
//void Stepper_PWM_Init(u32 arr,u32 psc)
//{		 					 
//	//�˲������ֶ��޸�IO������
//	
//	GPIO_InitTypeDef GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//  	TIM_OCInitTypeDef  TIM_OCInitStructure;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);  	//TIM14ʱ��ʹ��    
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//ʹ��PORTFʱ��	
//	
//	GPIO_PinAFConfig(GPIOE,GPIO_PinSource5,GPIO_AF_TIM9); //GPIOF9����Ϊ��ʱ��14
//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9); //GPIOF9����Ϊ��ʱ��14
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_6;           //GPIOF9
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
//	GPIO_Init(GPIOE,&GPIO_InitStructure);              //��ʼ��PF9
//	  
//	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
//	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
//	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
//	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
//	
//	TIM_TimeBaseInit(TIM9,&TIM_TimeBaseStructure);//��ʼ����ʱ��14
//	
//	//��ʼ��TIM14 Channel1 PWMģʽ	 
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
// 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�
//	TIM_OC1Init(TIM9, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1
//	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);  //ʹ��TIM14��CCR1�ϵ�Ԥװ�ؼĴ���
// 
//	TIM_OC2Init(TIM9, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1
//	TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);  //ʹ��TIM14��CCR1�ϵ�Ԥװ�ؼĴ���
//
//    TIM_ARRPreloadConfig(TIM9,ENABLE);//ARPEʹ�� 
//	
//	TIM_Cmd(TIM9, ENABLE);  //ʹ��TIM14
// 	TIM_ITConfig(TIM9,TIM_IT_CC1,ENABLE); //����ʱ��6�����ж�
//	TIM_ITConfig(TIM9, TIM_IT_CC2, ENABLE); //����ʱ��6�����ж�
//
//
// 	NVIC_InitStructure.NVIC_IRQChannel=TIM1_BRK_TIM9_IRQn;  
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; //��ռ���ȼ�0
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01; //�����ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//										  
//}  

/*TIM10�жϺ��� ��������������pwm������������*/
void TIM1_UP_TIM10_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM10, TIM_IT_CC1) == SET)
	{
		TIM_ClearITPendingBit(TIM10, TIM_IT_CC1);
		Step1++;
		if (Step1 >= Step1_Num)
		{
			TIM_SetCompare1(TIM10, 0);	//�޸ıȽ�ֵ���޸�ռ�ձ�
			Step1 = 0;
			Step1_Num = 0;
		}

	}
}



/*TIM11�жϺ��� ������һ������pwm�������������*/
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM11,TIM_IT_CC1)==SET)
	{
	   TIM_ClearITPendingBit(TIM11,TIM_IT_CC1);
	   Step2++;
		if(Step2>=Step2_Num)
		{
			TIM_SetCompare1(TIM11, 0);	//�޸ıȽ�ֵ���޸�ռ�ձ�
			Step2=0;
			Step2_Num = 0;
         }
	
}
}




void Stepper1_TurnDir_StepNum(int step1)
{
	Step1_Num = 0;
	TIM_SetCompare1(TIM10, 0);	//�޸ıȽ�ֵ���޸�ռ�ձ�
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
	   TIM_SetCompare1(TIM10, 1000);	//�޸ıȽ�ֵ���޸�ռ�ձ�
	   //if (step1<0)		OLED_ShowString(80, 20, "-"),
		  // OLED_ShowNumber(95, 20, -step1, 3, 12);
	   //else                 	OLED_ShowString(80, 20, "+"),
		  // OLED_ShowNumber(95, 20, step1, 3, 12);
}


void Stepper2_TurnDir_StepNum(int step2)
{
	TIM_SetCompare1(TIM11, 0);	//�޸ıȽ�ֵ���޸�ռ�ձ�
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

	TIM_SetCompare1(TIM11, 1000);	//�޸ıȽ�ֵ���޸�ռ�ձ�
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
//			TIM_SetCompare1(TIM9, 0);	//�޸ıȽ�ֵ���޸�ռ�ձ�
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
//			TIM_SetCompare2(TIM9, 0);	//�޸ıȽ�ֵ���޸�ռ�ձ�
//         }
//  }
//}
//}





