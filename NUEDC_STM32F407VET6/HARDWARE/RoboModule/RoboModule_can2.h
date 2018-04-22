#ifndef __CAN2_H__
#define __CAN2_H__
#include "stm32f4xx.h"
#include "main_assistant.h"
#define abs(x) ((x)>0? (x):(-(x)))

#define OpenLoop_Mode                       0x01
#define Current_Mode                        0x02
#define Velocity_Mode                       0x03
#define Position_Mode                       0x04
#define Velocity_Position_Mode              0x05
#define Current_Velocity_Mode               0x06
#define Current_Position_Mode               0x07
#define Current_Velocity_Position_Mode      0x08

void CAN2_Configuration(void);
void CAN2_RoboModule_DRV_Reset(unsigned char Group,unsigned char Number);
void CAN2_RoboModule_DRV_Mode_Choice(unsigned char Group,unsigned char Number,unsigned char Mode);
void CAN2_RoboModule_DRV_OpenLoop_Mode(unsigned char Group,unsigned char Number,short Temp_PWM);
void CAN2_RoboModule_DRV_Current_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Current);
void CAN2_RoboModule_DRV_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity);
void CAN2_RoboModule_DRV_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,long Temp_Position);
void CAN2_RoboModule_DRV_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity,long Temp_Position);
void CAN2_RoboModule_DRV_Current_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity);
void CAN2_RoboModule_DRV_Current_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,long Temp_Position);
void CAN2_RoboModule_DRV_Current_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity,long Temp_Position);
void RoboModule_CAN2_Init(unsigned char Group,unsigned char Number,unsigned char Tim,unsigned char Mode,unsigned char  Ctl1_Ctl2);


void CAN2_RoboModule_DRV_Config(unsigned char Group,unsigned char Number,unsigned char Temp_Time,unsigned char Ctl1_Ctl2);
void CAN2_RoboModule_DRV_Online_Check(unsigned char Group,unsigned char Number);

extern short Real2_Current_Value[4];
extern short Real2_Velocity_Value[4];
extern long Real2_Position_Value[4];
extern char Real2_Online[4];
extern char Real2_Ctl1_Value[4];
extern char Real2_Ctl2_Value[4];

#endif
