#ifndef __UART5_H
#define __UART5_H
#include "sys.h"
#include "stdio.h"
//////////////////////////////////////////////////////////////////////////////////
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//����4��ʼ������
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/5/14
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved
//********************************************************************************
//�޸�˵��
//��
//////////////////////////////////////////////////////////////////////////////////

#include "My_Flag.h"
#include "beep.h"
#define UART5_MAX_RECV_LEN		100					//�����ջ����ֽ���
#define UART5_MAX_SEND_LEN		100					//����ͻ����ֽ���
#define UART5_RX_EN 			1					//0,������;1,����.

#define UART5_REC_LEN  			200  	//�����������ֽ��� 200

//���̷�������ָ��
#define PositionTZ1  0X00
#define PositionTZ2  0x01
#define PositionTZ3  0x02
#define Capture_Color_And_Prepare 0x03         //ץ����
#define PositionGolden 0x04        //�׽���

//���͸����̵�ָ��
extern u8 GotoTZ3;//ȥ�����������
extern u8 TZ1_Done;//�����һ��
extern u8 TZ2_Done;//��������
extern u8 Capture_Color_And_Prepare_Done;//ץ����׼��λ��

extern Flag Flag_UART5_RX;

extern u8  UART5_RX_BUF[UART5_REC_LEN]; 		//���ջ���,���UART4_MAX_RECV_LEN�ֽ�
extern u8  UART5_TX_BUF[UART5_REC_LEN]; 		//���ͻ���,���UART4_MAX_SEND_LEN�ֽ�
extern vu16 UART5_RX_STA;   						//��������״̬

extern vs16 CameraData_Centerpoint,CameraData_Flag;
extern float CameraData_Angle;
extern u8 target_ID;
extern u16 USART5_RX_STA;         		//����״̬���

//����봮���жϽ��գ��벻Ҫע�����º궨��
void USART_SendByte(USART_TypeDef* USARTx, uint8_t date);
void USART_SendArray(USART_TypeDef* USARTx,uint8_t *array,uint8_t num);

void USART3_IRQHandler(void);


void PRINT(void);
void Apriltag(void);
void Z_go(void);
extern u8  STAa;         //ʶ��״̬��ǣ�ʶ�𵽶�ά��Ϊ1����֮Ϊ0
extern u8  Symbol;       //Xa���ű�ǣ�0x01��XaΪ��ֵ����֮Ϊ��ֵ
extern u8  IDa;          //��ά���IDֵ��1-4
extern u8  Direction;
extern int Xa;           //��ά����������Xa������ȷ��λ��
extern u16 Z;

extern u8 Flag_go;
extern u8 get_ball;

void uart5_init(u32 bound);
void u5_printf(char* fmt, ...);
void u5_putbuff(u8 *buff, u32 len);
#endif


