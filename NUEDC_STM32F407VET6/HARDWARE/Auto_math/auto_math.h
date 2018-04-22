#ifndef __AUTO_MATH_
#define __AUTO_MATH_
#include "main_assistant.h"
#include "sys.h"

#define Can_order_lenth 8

u8* Compensation_Mode(u32 command);
u8* Parameter_Mode(u8 command,u8 c);
u8* Speed_Mode(u8 w,u16 x,u8 c);
u8* Speed_Position_Mode(u8 w,u16 x,u8 c,u8 p);
void Erase_Can_order(void);
int Judge_PorM(u16 x);

#endif
