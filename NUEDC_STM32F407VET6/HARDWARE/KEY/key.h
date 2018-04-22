#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h" 

/****************************�⺯����ȡIO��ƽ**********************************/
#define KEY1 		GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13) //PE3
#define KEY2 		GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_14)	//PE4
#define KEY3 		GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_15) //PC13


/****************************��غ�������***************************************/
void KEY_MyScan(void);
void KEY_ClearAllFlag(void);
void KEY_Init(void);	
u8 KEY_Scan(u8 mode);  		

/*****************************������־λ����************************************/
extern u8  KEY1_flag ;
extern u8  KEY2_flag ;
extern u8  KEY3_flag ;


#endif
