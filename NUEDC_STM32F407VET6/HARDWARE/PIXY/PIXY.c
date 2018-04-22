#include "PIXY.h"

#define PIXY_NUM 2      //��Ҫʶ���������
#define PIXY_counter 64   //��Ҫ���յ����ݸ����������������16*2

u8 pixy_x,pixy_y=0;
u8 Raw_Data[PIXY_counter];
u8 counter = 0;
 Pixy_Color Pixy_Color_1;
 Pixy_Color Pixy_Color_2;
 Pixy_Color Pixy_Color_3;
 Pixy_Color Pixy_Color_4;
 Pixy_Color Pixy_Color_5;
 Pixy_Color Pixy_Color_6;
 Pixy_Color Pixy_Color_7;
/*
����һ��PIXY����ͷ���ݽ��մ����ļ�
PIXY����ͷ���ô����շ�ģʽ
������Ϊ115200
�ô���2�շ�����

����PIXY����ͷ
      ������ݸ�ʽ��
      ��ͷ               55 AA 55(56) AA
      ��У��             xx xx
      ��ɫ����           xx xx
      ��������X����      xx xx
      ��������Y����      xx xx
      width              xx xx
      height             xx xx
      CCģʽ�ĽǶ�ֵ     xx xx
���16λ������8bit�ȷ��͵ĵ�λ�����͸�λ
ע�⣺��ֻʶ��һ�����壬����4+10+2�ֽ�����4��ͷ��2��У�飬10���������ݣ�2���Ƕ�ֵ��
      ��ʶ���������壬����16+14+2�ֽڵ���
	  ��ʶ���������壬����16+14+14+2�ֵ���
*/


/*      PIXY��ӦIO�ĳ�ʼ��        */
/*       PC10:TX     PC11:RX  */
void PIXY_init(u32 bound)
{
	uart4_init(bound);
}




/*      ����4�ĳ�ʼ��        */
/*       PC10:TX     PC11:RX  */
void uart4_init(u32 bound) {
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);	//ʹ��USART1��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	USART_DeInit(UART4);  //��λ����1

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOC, &GPIO_InitStructure); //��ʼ��PA9


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOC, &GPIO_InitStructure);  //��ʼ��PA10



	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

									//USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

	USART_Init(UART4, &USART_InitStructure); //��ʼ������
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//�����ж�
	USART_Cmd(UART4, ENABLE);                    //ʹ�ܴ��� 
	USART_ClearFlag(UART4, USART_FLAG_TC | USART_FLAG_TXE | USART_FLAG_RXNE);
}


/*
����һ�������жϷ�����
������ʵ��PIXY����ͷ���͵����ݵĽ���
�����������ֻд��ͬʱֻʶ��һ�����壬���ʶ��3������Ĵ��룬�����ٸ�
*/
void UART4_IRQHandler(void)//Pixy_Uart_ReadData
{
	u8 i;
	u8 s=0;    //��־����״̬���Ա�����ѭ��
	if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
	{
		Raw_Data[counter] = USART_ReceiveData(UART4);
		counter++;
		if (counter == PIXY_counter)
		{
			counter = 0;

			for (i = 0; i<PIXY_counter; i++)
			{
				if ((Raw_Data[i] == 0x55) && (Raw_Data[i + 1] == 0xaa) && (Raw_Data[i + 2] == 0x55) && (Raw_Data[i + 3] == 0xaa))
				{				
					if (Raw_Data[i + 6] == 1)
					{
						Pixy_Color_1.Pixy_Color_PosX = Raw_Data[i + 8] + Raw_Data[i + 9] * 256;
						Pixy_Color_1.Pixy_Color_PosY = Raw_Data[i + 10] + Raw_Data[i + 11] * 256;
						Pixy_Color_1.Pixy_Color_Width = Raw_Data[i + 12] + Raw_Data[i + 13] * 256;
						Pixy_Color_1.Pixy_Color_Height = Raw_Data[i + 14] + Raw_Data[i + 15] * 256;
						
					} else if (Raw_Data[i + 6] == 2)
					{
						Pixy_Color_2.Pixy_Color_PosX = Raw_Data[i + 8] + Raw_Data[i + 9] * 256;
						Pixy_Color_2.Pixy_Color_PosY = Raw_Data[i + 10] + Raw_Data[i + 11] * 256;
						Pixy_Color_2.Pixy_Color_Width = Raw_Data[i + 12] + Raw_Data[i + 13] * 256;
						Pixy_Color_2.Pixy_Color_Height = Raw_Data[i + 14] + Raw_Data[i + 15] * 256;
					
					} 
					if (Raw_Data[i + 20] == 2)
					{
						Pixy_Color_2.Pixy_Color_PosX = Raw_Data[i + 22] + Raw_Data[i + 9] * 256;
						Pixy_Color_2.Pixy_Color_PosY = Raw_Data[i + 24] + Raw_Data[i + 11] * 256;
						Pixy_Color_2.Pixy_Color_Width = Raw_Data[i + 26] + Raw_Data[i + 13] * 256;
						Pixy_Color_2.Pixy_Color_Height = Raw_Data[i + 28] + Raw_Data[i + 15] * 256;
						
					}
					
					pixy_x=Pixy_Color_1.Pixy_Color_PosX-Pixy_Color_2.Pixy_Color_PosX;
					pixy_y=Pixy_Color_1.Pixy_Color_PosY-Pixy_Color_2.Pixy_Color_PosY;
					} 
					break;
				}
			}

		}
	}

