/***********************************************************************\
���ߣ�DAHU	        �ļ�����; Sterring.c
�ļ����ܣ�
���ļ���TIM11CH1�����ƶ��PB9������IO����ʱ����ʼ�����Լ�����Ƕ����ú���
����˵����
���ļ������������滭��2017�������PCB��·�壬IO��ʼ��������������·���
ģ����ʷ��
���ļ���2017��11��4�մ�������Ҫ���PCB��Ӧ�ĳ�ʼ��
��ģ��ĺ����ڵ��ղ��ԣ��������ݣ�IO��ʼ������ʱ�����������ת�Ƕ�
\***********************************************************************/

#include<sterring.h>
#include<time.h>
#include<stm32f4xx.h>

void SterringInit(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);/*ʹ��GPIOFʱ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);/*ʹ�ܶ�ʱ��11ʱ��*/

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM11);/*����*/

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;/*����*/
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;/*�������*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;/*PF7*/
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;/*����*/
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;/**/
	GPIO_Init(GPIOB, &GPIO_InitStructure);/*��ʼ��IO*/

	TIM_TimeBaseInitStructure.TIM_Period = 19999;/*�Զ���װ��*/
	TIM_TimeBaseInitStructure.TIM_Prescaler = 83;/*Ԥ��Ƶ*/
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;/*ʱ�ӷ�Ƶ*/
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;/*���ϼ���*/
	TIM_TimeBaseInit(TIM11, &TIM_TimeBaseInitStructure);/*��ʼ��*/

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;/*PWMģʽ*/
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;/*���*/
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;/*��ʼ�ǵ�*/
	TIM_OC1Init(TIM11, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM11, TIM_OCPreload_Enable);/*����Ƚ�Ԥװ��ʹ��*/
	TIM_ARRPreloadConfig(TIM11, ENABLE);/*�Զ�����Ԥװ��ʹ��*/
	TIM_Cmd(TIM11, ENABLE);/*����ʹ��*/
	TIM_SetCompare1(TIM11, 15);
}

/***********************************************************************\
�������ƣ� SterringSet()
����������
ͨ���ı�TIM11CH1��ռ�ձ������ƶ����λ��
����������void                      ����ֵ  ��void
ģ����ʷ��
��������2017��11��4��
��ع�����2017��11��4�Ų���
��ע    ��
�Ĵ���CCR1�����õķ�ΧΪ��500--2500  ��Ӧ�����0--180�ȣ���ռ�ձ�ֵΪ��20000��
���ڣ�180�ȽǶ��ŷ�������������������ڣ�20ms
          �Ƕ���ߵ�ƽʱ���Ӧ��ϵ��
		                      0.5ms��������0��
							  1.5ms��������90��
							  2.5ms��������180��
\***********************************************************************/
void SterringSet(u16 angle)
{
	//u16 num;
	//num =(int) (angle*11.1 + 500);
	TIM11->CCR1 = angle;
}


