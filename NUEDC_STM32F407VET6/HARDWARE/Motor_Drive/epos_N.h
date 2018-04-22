

#ifndef __EPOS_N_H
#define __EPOS_N_H

#include "delay.h"
#include "stm32f4xx.h"

#define EPOS_NUM 1       //EPOS数目

#define MAX_F_ERR 2000   //最大跟随误差
#define MAX_P_V 12000     //最大速度
#define QDEC 30000       //急停减速度
#define MAX_ACC 30000     //最大加速度


//#define MAX_F_ERR 2000   //最大跟随误差
//#define MAX_P_V 7200     //最大速度
//#define QDEC 15000       //急停减速度
//#define MAX_ACC 10000     //最大加速度

typedef enum
{
	FALSE = 0,
	TRUE = !FALSE
}BOOL;
/*
********************************************************************************
*@  file: epos.c
*@  author: YX_L
*@  data: 04/11/2016
*@  version: v1.0
*@  brief: 参数修改要严格按照EPOScan指令对照表！
*...........................................................................
*@
*@ Notes:     ***********************************************************
*@            *  MAXON电机型号       额定转速(RPM)  额定电流(A) 功率(W) *
*@            *  EC-4pole-30-305013  16200          9.21        200     *
*@            *  EC-40-369146        9090           12.2        150     *
*@            ***********************************************************
*@
********************************************************************************
*/

typedef struct __Epos
{
	u32 ID;
	u8 Buf[8];
	u8 NODE_ID;
	CanTxMsg eposCAN;//epos报文结构体，用于向epos写数据
	uint8_t CurMode;
	int32_t opt;    //用于PPMODE
	int32_t opt_mem;
	int32_t Position;
	int32_t Speed;
	int32_t Current;
	int32_t CMSpeed;
	int32_t Acc;
	int32_t Dec;
}Epos;

extern Epos epos[EPOS_NUM + 1];
extern Epos epos1;
//运行模式宏定义
#define PPMODE 0x01
#define PMODE 0xFF
#define PVMODE 0x03
#define VMODE 0xFE 
#define CMODE 0xFD

#define EPOS_DELAY1  6
#define EPOS_DELAY2  60
#define EPOS_DELAY3  100

//轮廓速度模式
void Profile_Velocity_Change(int v1, int v2, int v3, int v4);
void Profile_Velocity_Stop(void);


////上电清除错误
//void MAXON_Clear_Error(void);
////Maxon启动
//void MAXON_Star(void);
//
//void Epos_clear(Epos* epos);
//
//void Epos_Star(Epos* epos);
//
//void Profile_Velocity_Mode_Parameter(void);

//extern BOOL    Read(); 

extern void    Epos_Delay(int8_t time);

extern void    Epos_Write(Epos* epos, uint16_t Index, uint8_t SubIndex, int32_t param);

extern void    Epos_Read(Epos* epos, uint16_t Index, int8_t SubIndex, int32_t* param);

extern void    Epos_SInit(Epos* epos1, int8_t NODE_ID1);

extern void    Epos_Init(void);

extern void    Epos_ParamInit(Epos* epos);

extern void    Epos_Run(Epos* epos, int32_t speed, int32_t position, uint8_t mode, int32_t opt);

extern void    Epos_RunPPM(Epos* epos, int32_t speed, int32_t posi, uint8_t opt);

extern void    Epos_RunPM(Epos* epos, int32_t position, uint8_t opt);

extern void    Epos_RunPVM(Epos* epos, int32_t speed, uint8_t opt);

extern void    Epos_RunVM(Epos* epos, int32_t speed, uint8_t opt);

extern void    Epos_RunCM(Epos* epos, int32_t Current, int32_t MAX_Speed);

extern void    Epos_SetAcc(Epos* epos, int32_t Acc);

extern void    Epos_SetDec(Epos* epos, int32_t Dec);

extern void    Epos_Stop(Epos* epos);

extern void    Epos_HaltStop(Epos* epos);

extern void    Epos_ShutDown(Epos* epos);

extern void    Epos_Watch(Epos* epos);



#endif
