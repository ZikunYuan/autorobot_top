#ifndef __TIME_H
#define __TIME_H

#include <stdint.h>
#include "stm32f4xx.h"

/* 系统时间 */
typedef struct {
    uint16_t Hours;         //时
    uint8_t  Minutes;       //分
    uint8_t  Seconds;       //秒
    uint16_t Milliseconds;  //毫秒
}System_Time_TypeDef;

extern System_Time_TypeDef System_Time; //系统时间

/**********************************************
* 清空系统时间
**********************************************/
void System_Tiem_Clear(void);

/**********************************************
* 记录系统时间
**********************************************/
void System_Time_Running(void);

/**********************************************
* 获取系统时间 单位us
* 两次获取若大于u32/1000(us),则两次差值溢出，不可取
**********************************************/
uint32_t GetSysTime_us(void);

float Get_Cycle_T(uint8_t item);
void Cycle_Time_Init(void);
void TIM4_Int_Init(u16 arr, u16 psc);
void TIM6_Int_Init(u16 arr, u16 psc);
void TIM7_Int_Init(u16 arr, u16 psc);
#endif
