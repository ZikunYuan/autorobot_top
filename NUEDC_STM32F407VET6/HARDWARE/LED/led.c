/***********************************************************************\
������Ϣ��
������DAHU	        �ļ�����; LED.C
�ļ����ܣ�
���ļ����ڵ���PCB�壺LIKE��LEDָʾ�Ƴ�ʼ����һЩȫ�ֱ�������
����˵����
���ļ������������滭��2017�������PCB��·�壬IO��ʼ��������������·���
ģ����ʷ��
���ļ���2017��10��17����д����Ҫ���PCB��Ӧ�ĳ�ʼ��
��ģ��ĺ����ڵ��ղ��ԣ��������ݣ�IO��ʼ��������LED�Ĺ���
\***********************************************************************/
#include "led.h" 



/***********************************************************************\
�������ƣ� LED_Iint()
����������
��ʼ����·���ϵİ���IO
����������void
����ֵ�� void
ģ����ʷ��
ע����2018��2��1���޸�
��ع�����2��1�Ų���
��ע��
PC0--LED1   PC1--LED2  PC2--LED3   PC3--LED4   
\***********************************************************************/
void LED_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIODʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1|GPIO_Pin_2 | GPIO_Pin_3;//PC0��PC1��PC2��PC3�ֱ��ӦLED1��LED2��LED3��LED4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��
	GPIO_SetBits(GPIOC, GPIO_Pin_10);

	GPIO_SetBits(GPIOC, GPIO_Pin_0| GPIO_Pin_1|GPIO_Pin_2 | GPIO_Pin_3);//�øߣ�����
}






