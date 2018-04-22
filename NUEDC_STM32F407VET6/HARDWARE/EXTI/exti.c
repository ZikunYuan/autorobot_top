#include "RoboModule_can2.h"
#include "Relay.h"
#include "EXTI.H"
#include "delay.h"
#include "beep.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//�ⲿ�ж� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/4
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
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
	 EXTI_ClearITPendingBit(EXTI_Line6);//���LINE2�ϵ��жϱ�־λ 
}
//�ⲿ�ж�3�������

	   



//�ⲿ�жϳ�ʼ������
//��ʼ��PE2~4,PA0Ϊ�ж�����.
void EXTIX_Init(void)
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
	
 
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource6);//PE2 ���ӵ��ж���2

	/* ����EXTI_Line2,3,4 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line6 ;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //�½��ش���
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//�ж���ʹ��
  EXTI_Init(&EXTI_InitStructure);//����
	 
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//�ⲿ�ж�4
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//��ռ���ȼ�1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//�����ȼ�2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure);//����

}












