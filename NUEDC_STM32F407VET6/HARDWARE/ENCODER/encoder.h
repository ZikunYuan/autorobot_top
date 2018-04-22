#ifndef __ENCODER_H
#define __ENCODER_H
#include <sys.h>	 

#define ENCODER_TIM_PERIOD (u32)(0xFFFFFFFF)   //自动重装载值 对于TIM2和TIM5是32位的
#define ENCODER1_TIMER 5                       //编码器1用的定时器5 32位
#define ENCODER2_TIMER 2                       //编码器2用的定时器2 32位
#define ENCODER3_TIMER 1                       //编码器4用的定时器1 16位
void Encoder1_Init_TIM5(void);
void Encoder2_Init_TIM2(void);
void Encoder3_Init_TIM1(void);
int Read_Encoder(u8 TIMX);
void TIM5_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM1_BRK_TIM9_IRQHandler(void);

extern int Encoder1;
extern int Encoder2;
extern int Encoder3;  

#endif
