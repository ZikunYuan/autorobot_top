#include "auto_math.h"

u8 Can_order[Can_order_lenth];

u8* Compensation_Mode(u32 command)
{
	Can_order[0]=0;
	Can_order[1]=0;
	Can_order[2]=0;
	Can_order[3]=Compensation;//使用参数表模式
	Can_order[4]=(command&0x0000ff00)>>8;
	Can_order[5]=command&0x000000ff;
	Can_order[6]=0;//发送指令给倍福
	Can_order[7]=0;
	return Can_order;
}

u8* Parameter_Mode(u8 command,u8 c)
{
	Can_order[0]=0;
	Can_order[1]=0;
	Can_order[2]=0;
	Can_order[3]=c;//使用参数表模式
	Can_order[4]=0;
	Can_order[5]=0;
	Can_order[6]=command;//发送指令给倍福
	Can_order[7]=0;
	return Can_order;
}

u8* Speed_Mode(u8 w,u16 x,u8 c)
{
	Can_order[0]=w;//符号位
	Can_order[1]=(x&0xff00)>>8;//取出16位数高八位
	Can_order[2]=x&0x00ff;//取出16位数第八位
	Can_order[3]=c;//使能位和停止位
	Can_order[4]=0;
	Can_order[5]=0;
	Can_order[6]=0;
	Can_order[7]=0;
	return Can_order;
}

u8* Speed_Position_Mode(u8 w,u16 x,u8 c,u8 p)
{
	Can_order[0]=w;//符号位
	Can_order[1]=(x&0xff00)>>8;//取出16位数高八位
	Can_order[2]=x&0x00ff;//取出16位数第八位
	Can_order[3]=c;//使能位和停止位
	Can_order[4]=(p&0xff00)>>8;//取位置的高八位
	Can_order[5]=p&0x00ff;//取位置的低八位
	Can_order[6]=0;
	Can_order[7]=0;
	return Can_order;
}


void Erase_Can_order(void)
{
	int i;
	for(i=0;i<Can_order_lenth;i++)
	{
		Can_order[i]=0;
	}
}

int Judge_PorM(u16 x)
{
	if(x)
		return 1;
	else 
		return -1;
}



