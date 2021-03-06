#ifndef __MAIN_ASSISTANT_H
#define __MAIN_ASSISTANT_H

#include "sys.h"
#include "encoder.h"
#include "Beep.h"
#include "delay.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "inv_mpu.h"
#include "usart.h"
#include "motor.h"
#include "key.h"
#include "MPU6050.h"
#include "oled.h"
#include "encoder.h"
#include "exti.h"
#include "show.h"
#include "ANO_DT.h"
#include "led.h"
#include "HMC5883L.h"
#include "Task_Loop.h"
#include "SysTick.h"
#include "ak8975.h"
#include "myiic.h"
#include "Time.h"
#include "math.h"
#include "Stepper.h"
#include "control.h"
#include "epos_N.h"
#include "can.h"
#include "KS103.h"
#include <Sterring.h>
#include "usart3.h"
#include "Relay.h"
#include "auto_math.h"
#include "exti.h"
#include "RoboModule_can2.h"
#include "usart5.h"

//Elmo_ID
#define Elmo_ID 0xAA
#define UnderPan_ID 0xDD
//控制ec40指令
#define Speed_Mode_Enable 0
#define Enable 1
#define Disable 2
#define Stop 3
#define Unstop 4
#define Reset 5
#define Erase_Reset_Flag 6
#define Speed_Position_Mode_Enable 7//相对位置模式
#define Parameter_Enable 8//参数表模式
#define Compensation 9//补偿

void Test_Pattern(void);
void OriginalPosition(void);
void Init_Movement(int num);//抓球位置
void Prepare_Movement(int num);//准备发射位置
void Launch_Movement(int num);//抛射的速度 抛射的位置 抛射停止位置
void NextBallPosition(void);//球架旋转爪子能抓到的位置
void RM35_GoBack(void);

struct Ejection_parameter//抛射参数结构体
{
    u16 Weight_of_Fire_Place;//抛射位置的方向（外部编码器的）
    u16 Fire_Place;//抛射的位置
    u16 Roll_Wait_Time;//甩球的停止时间
};

typedef struct Ejection_parameter Eje_value;

//倍福参数表
struct Elmo_parameter
{
    u8 capture;
    u8 prepare;
    u8 shoot;
    u8 shoot_swing;
};

typedef struct Elmo_parameter Para_value;

#endif
