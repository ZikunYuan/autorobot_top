/***********************************************************************\
������Ϣ��
������DAHU	        �ļ�����; Relay.C
�ļ����ܣ�
���ļ����ڵ���PCB�壺LIKE��PD6,PD4,PD2,PD0,PC11����IO��Ϊ��ŷ�����IO
����˵����
���ļ������������滭��2017�������PCB��·�壬IO��ʼ��������������·���
ģ����ʷ��
���ļ���2017��11��23�մ�������Ҫ���PCB��Ӧ�ĳ�ʼ��
��ģ��ĺ����ڵ��ղ��ԣ��������ݣ�IO��ʼ����IO���������
\***********************************************************************/
#include "Relay.h"


int Light1Flag=0;
/***********************************************************************\
�������ƣ� Relay_Iint()
����������
��ʼ����·���ϵ�����IO
����������void
����ֵ�� void
ģ����ʷ��
��ģ����2017��11��23�մ���
��ع�����10��17�Ų���
��ע��
PCB�ϵĲ���˿ӡ����PD11��PD10��ע�ֱ�Ϊʵ��IO PC11��PC10
ʵ����·��IO��Ӧ��ϵ��
Relay2����PC11          relay3����PD0         relay4����PD2
relay5����PD4           relay6����PD6
\***********************************************************************/
void Relay_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIODʱ��

    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��

    GPIO_SetBits(GPIOD, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);//�øߣ��̵����ر�
    GPIO_SetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_9);     //�øߣ��̵����ر�
}


/***********************************************************************\
�������ƣ� TouchSwitch_Iint()
����������
��ʼ����·���ϵ�Ԥ��IO��Ϊ�������صĿ���IO
����������void
����ֵ  ��void
ģ����ʷ��
��ģ����2017��11��25�մ���
��ע    ��
������Ԥ��IO,PD5��PD3�ֱ���ƿ���1�Ϳ���2
IO��Ӧ��ϵ��
PC12������������1
PD1������������2
PD3������������3
\***********************************************************************/
//void TouchSwitch_Init(void)
//{

//    GPIO_InitTypeDef  GPIO_InitStructure;

//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//ʹ��GPIODʱ��
////	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIODʱ��

//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
//    GPIO_Init(GPIOD, &GPIO_InitStructure);

////	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
////	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
////	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
////	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
////	GPIO_Init(GPIOC, &GPIO_InitStructure);
//}



/***********************************************************************\
�������ƣ� LightEletric_Iint()
����������
��ʼ����·���ϵ�����IO��������Թ�ʹ�õ�IO
����������void
����ֵ  ��void
ģ����ʷ��
��ģ����2017��12��3�Ŵ��������ڵ�����Թ���
��ע    ��
IO��Ӧ��ϵ��
Light1����PE14  ����Թ�1�������ղ����ź�ʱ���1�����յ��ź�ʱ���0
Light2����PE15	���˴�����1
\***********************************************************************/
void LightEletric_Init(void)
{

    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIODʱ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}







