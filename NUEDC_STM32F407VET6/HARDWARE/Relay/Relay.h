/***********************************************************************\
������Ϣ��
������DAHU	        �ļ�����; Relay.H
�ļ����ܣ�
�궨��ͺ�������
����˵����
���ļ������������滭��2017�������PCB��·�壬IO��ʼ��������������·���
ģ����ʷ��
���ļ���2017��11��23�մ�������Ҫ���PCB��Ӧ�ĳ�ʼ��
��ģ��ĺ����ڵ��ղ��ԣ��������ݣ�IO��ʼ�����Լ�IO����
stm32f4����Ҫ���ǹر�JATG��ֱ�ӳ�ʼ�����ɸ����书��
\***********************************************************************/

#ifndef __relay_H
#define __relay_H
#include "sys.h"

#define Relay1 PDout(14) //������
#define Relay2 PDout(13) //�̶��������
#define Relay3 PDout(15)	 //צ��//Ĭ�ϸߵ�ƽ Ϊ�պ�״̬
#define Relay4 PCout(8)	 //�����1
#define Relay5 PCout(9)	 //�����2
//#define Relay6 PDout(6)	  //�˼̵���δ��

////#define TouchSwitch1 PCin(12)
//#define TouchSwitch2 PDin(1)
//#define TouchSwitch3 PDin(3)

#define Light1 PBin(6) 	  //����1������
#define Light2 PBin(5)		//����2������

void LightEletric_Init(void);
//void TouchSwitch_Init(void);
void Relay_Init(void);//��ʼ��

extern int Light1Flag;
#endif
