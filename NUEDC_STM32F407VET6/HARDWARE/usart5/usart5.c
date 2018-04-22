#include "sys.h"
#include "usart5.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "stm32f4xx.h"
#include "My_Flag.h"
#include "usart.h"
#include "main_assistant.h"

//发送给底盘的指令
u8 GotoTZ3= 0x07;//去第三个抛射点
u8 TZ1_Done=0x08;//抛射点一完
u8 TZ2_Done=0x09;//抛射点二完
u8 Capture_Color_And_Prepare_Done=0x0A;//抓完球到准备位置

//标志位（摄像头一直发但是只接收）
//u8 TZ1_Flag=0;
//u8 TZ2_Flag=0;
//u8 TZ3_Flag=0;
//u8 Color_To_Prepare_Flag=0;
//u8 PositionGolden_Flag=0;
//u8 Capture_Color_And_Prepare_Flag=0;

u8 UART5_TX_BUF[UART5_REC_LEN]; 		
u8 UART5_RX_BUF[UART5_REC_LEN];     

extern u8 ArriveTZ3_Flag;

vu16 UART5_RX_STA=0;

Flag Flag_UART5_RX = Not_Ready;

void uart5_init(u32 bound)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    USART_DeInit(UART5);  

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
    GPIO_Init(GPIOC, &GPIO_InitStructure); 


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
    GPIO_Init(GPIOD, &GPIO_InitStructure); 

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5); 
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5); 

    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
    USART_Init(UART5, &USART_InitStructure); 

    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);

    USART_Cmd(UART5, ENABLE);                  

    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
    NVIC_Init(&NVIC_InitStructure);	

    UART5_RX_STA=0;				
}

void u5_printf(char* fmt, ...)
{
    u16 i, j;
    va_list ap;
    va_start(ap, fmt);
    vsprintf((char*)UART5_TX_BUF, fmt, ap);
    va_end(ap);
    i = strlen((const char*)UART5_TX_BUF);   
    for (j = 0; j<i; j++)
    {
        while (USART_GetFlagStatus(UART5, USART_FLAG_TC) == RESET);
        USART_SendData(UART5, (uint8_t)UART5_TX_BUF[j]);
    }
}

static void u5_putchar(u8 ch)
{
    while ((UART5->SR & 0X40) == 0);
    UART5->DR = (u8)ch;
    //return ch;
}

void u5_putbuff(u8 *buff, u32 len)
{
    while (len--)
    {
        u5_putchar(*buff);
        buff++;
    }
}

u16  USART5_RX_STA=0;
u8 NUMBER=0;
u8 gold_num=0;

void UART5_IRQHandler(void)               
{
	u8 Res;

	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)  
	{
			Res =USART_ReceiveData(UART5);
			if(Res==Capture_Color_And_Prepare)//3
			{
				  if(NUMBER==0){Prepare_Movement(0);NUMBER++;}//给底盘的指令放在函数中了
				  else if(NUMBER==1) Prepare_Movement(1);					           
			}
			else if(Res==PositionTZ1)   //0
			{
				  Launch_Movement(PositionTZ1);//扔球
					USART_SendData(UART5,TZ1_Done);//发送抛完球指令
					Init_Movement(PositionTZ2);//回到初始位置
			}
			else if(Res==PositionTZ2)   //1
			{
				  Launch_Movement(PositionTZ2);//扔球
					USART_SendData(UART5,TZ2_Done);
			}
			else if(Res==PositionGolden)  //4
			{
				  Relay1=0;//气缸回到抓球位置
			}
			else if(Res==PositionTZ3)
			{
					ArriveTZ3_Flag=1;
			}	
      else if(Res==0X0F)  //4
			{
					BEEP=1;
       	  delay_ms(500);
      	  BEEP=0;
      	  CAN1_SendMsg(Speed_Mode(0,0,Speed_Mode_Enable),Elmo_ID);//????
      	  delay_ms(5);
      	  CAN1_SendMsg(Speed_Mode(0,0,Stop),Elmo_ID);//??
      	  delay_ms(5);
      	  CAN1_SendMsg(Speed_Mode(0,0,Disable),Elmo_ID);//????
      	  Erase_Can_order();
					RM35_GoBack();
				  Relay4 = 1;//电磁铁
          Relay5 = 1;
       	  U3_printf("Emergency_stop_Done\r\n");			
			}			
  } 
} 

