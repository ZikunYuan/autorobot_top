#ifndef __PIXY_H__
#define __PIXY_H__

#include"stm32f10x.h"

typedef struct Pixy_Color//
{
	u16 Pixy_Color_PosX;  //0 to 319
	u16 Pixy_Color_PosY;  //0 to 210
	u16 Pixy_Color_Width; //1 to 320
	u16 Pixy_Color_Height;//1 to 210
}Pixy_Color;


extern void BSPInit(void);
extern Pixy_Color Pixy_Color_1;
extern Pixy_Color Pixy_Color_2;
extern Pixy_Color Pixy_Color_3;
extern Pixy_Color Pixy_Color_4;
extern Pixy_Color Pixy_Color_5;
extern Pixy_Color Pixy_Color_6;
extern Pixy_Color Pixy_Color_7;
extern u16 datax;
void PIXY_init(u32 bound);
void uart4_init(u32 bound);

#endif

//===========================================  End Of File  ===========================================//