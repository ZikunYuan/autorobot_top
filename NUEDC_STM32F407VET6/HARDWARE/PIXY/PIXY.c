#include "PIXY.h"

#define PIXY_NUM 2      //需要识别的物体数
#define PIXY_counter 64   //需要接收的数据个数，按下面的推算16*2

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
这是一个PIXY摄像头数据接收处理文件
PIXY摄像头采用串口收发模式
波特率为115200
用串口2收发数据

关于PIXY摄像头
      输出数据格式：
      包头               55 AA 55(56) AA
      和校验             xx xx
      颜色代码           xx xx
      方框中心X坐标      xx xx
      方框中心Y坐标      xx xx
      width              xx xx
      height             xx xx
      CC模式的角度值     xx xx
组成16位的两个8bit先发送的低位，后发送高位
注意：若只识别一个物体，则发送4+10+2字节数（4个头，2个校验，10个有用数据，2个角度值）
      若识别两个物体，则发送16+14+2字节的数
	  若识别三个物体，则发送16+14+14+2字的数
*/


/*      PIXY对应IO的初始化        */
/*       PC10:TX     PC11:RX  */
void PIXY_init(u32 bound)
{
	uart4_init(bound);
}




/*      串口4的初始化        */
/*       PC10:TX     PC11:RX  */
void uart4_init(u32 bound) {
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);	//使能USART1，GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	USART_DeInit(UART4);  //复位串口1

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOC, &GPIO_InitStructure); //初始化PA9


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOC, &GPIO_InitStructure);  //初始化PA10



	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

									//USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

	USART_Init(UART4, &USART_InitStructure); //初始化串口
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启中断
	USART_Cmd(UART4, ENABLE);                    //使能串口 
	USART_ClearFlag(UART4, USART_FLAG_TC | USART_FLAG_TXE | USART_FLAG_RXNE);
}


/*
这是一个串口中断服务函数
在这里实现PIXY摄像头发送的数据的接收
现在这个代码只写了同时只识别一个物体，最多识别3个物体的代码，具体再改
*/
void UART4_IRQHandler(void)//Pixy_Uart_ReadData
{
	u8 i;
	u8 s=0;    //标志处理状态，以便跳出循环
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

