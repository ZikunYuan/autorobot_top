#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h" 
#include "stm32f4xx_gpio.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//IIC ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/6
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
   //λ������,ʵ��51���Ƶ�GPIO���ƹ���
//����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).M4ͬM3����,ֻ�ǼĴ�����ַ����.
//IO�ڲ����궨��
//#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
//#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
//#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
////IO�ڵ�ַӳ��
//#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
//#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
//#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
//#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
//#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
//#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
//#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
//#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
//#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014     
//
//#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
//#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
//#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
//#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
//#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
//#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
//#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
//#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
//#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 
// 
////IO�ڲ���,ֻ�Ե�һ��IO��!
////ȷ��n��ֵС��16!
//#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
//#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 
//
//#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
//#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 
//
//#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
//#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 
//
//#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
//#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 
//
//#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
//#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����
//
//#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
//#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����
//
//#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
//#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����
//
//#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //��� 
//#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //����
//
//#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //��� 
//#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //����	   		   


////IO��������
//#define SDA_IN()  { GPIO_InitTypeDef  GPIO_InitStructure;RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;GPIO_InitStructure.GPIO_OType = GPIO_PuPd_NOPULL;GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;GPIO_Init(GPIOB, &GPIO_InitStructure);}
//#define SDA_OUT() { GPIO_InitTypeDef  GPIO_InitStructure;RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;GPIO_Init(GPIOB, &GPIO_InitStructure);}
#define SDA_IN()  {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=0<<11*2;}	//PB9����ģʽ
#define SDA_OUT() {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=1<<11*2;} //PB9���ģʽ
//IO��������	 
#define IIC_SCL    PBout(10) //SCL
#define IIC_SDA    PBout(11) //SDA	 
#define READ_SDA   PBin(11)  //����SDA 


void IIC_Init(void);                //��ʼ��IIC��IO��				 
int IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
int IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 
unsigned char I2C_Readkey(unsigned char I2C_Addr);

u8 I2C_ReadOneByte(u8 I2C_Addr,u8 addr);
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data);
u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data);
u8 IICwriteBit(u8 dev,u8 reg,u8 bitNum,u8 data);
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data);
//u8 IIC_Read_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);

int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
#endif

















