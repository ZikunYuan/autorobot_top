#ifndef __UART5_H
#define __UART5_H
#include "sys.h"
#include "stdio.h"
//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//串口4初始化代码
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/5/14
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved
//********************************************************************************
//修改说明
//无
//////////////////////////////////////////////////////////////////////////////////

#include "My_Flag.h"
#include "beep.h"
#define UART5_MAX_RECV_LEN		100					//最大接收缓存字节数
#define UART5_MAX_SEND_LEN		100					//最大发送缓存字节数
#define UART5_RX_EN 			1					//0,不接收;1,接收.

#define UART5_REC_LEN  			200  	//定义最大接收字节数 200

//底盘发过来的指令
#define PositionTZ1  0X00
#define PositionTZ2  0x01
#define PositionTZ3  0x02
#define Capture_Color_And_Prepare 0x03         //抓彩球
#define PositionGolden 0x04        //抛金球

//发送给底盘的指令
extern u8 GotoTZ3;//去第三个抛射点
extern u8 TZ1_Done;//抛射点一完
extern u8 TZ2_Done;//抛射点二完
extern u8 Capture_Color_And_Prepare_Done;//抓完球到准备位置

extern Flag Flag_UART5_RX;

extern u8  UART5_RX_BUF[UART5_REC_LEN]; 		//接收缓冲,最大UART4_MAX_RECV_LEN字节
extern u8  UART5_TX_BUF[UART5_REC_LEN]; 		//发送缓冲,最大UART4_MAX_SEND_LEN字节
extern vu16 UART5_RX_STA;   						//接收数据状态

extern vs16 CameraData_Centerpoint,CameraData_Flag;
extern float CameraData_Angle;
extern u8 target_ID;
extern u16 USART5_RX_STA;         		//接收状态标记

//如果想串口中断接收，请不要注释以下宏定义
void USART_SendByte(USART_TypeDef* USARTx, uint8_t date);
void USART_SendArray(USART_TypeDef* USARTx,uint8_t *array,uint8_t num);

void USART3_IRQHandler(void);


void PRINT(void);
void Apriltag(void);
void Z_go(void);
extern u8  STAa;         //识别状态标记，识别到二维码为1，反之为0
extern u8  Symbol;       //Xa符号标记，0x01则Xa为负值，反之为正值
extern u8  IDa;          //二维码的ID值：1-4
extern u8  Direction;
extern int Xa;           //二维码中心坐标Xa，用于确定位置
extern u16 Z;

extern u8 Flag_go;
extern u8 get_ball;

void uart5_init(u32 bound);
void u5_printf(char* fmt, ...);
void u5_putbuff(u8 *buff, u32 len);
#endif


