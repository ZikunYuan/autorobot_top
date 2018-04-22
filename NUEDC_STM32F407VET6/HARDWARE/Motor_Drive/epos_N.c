/**
******************************************************************************
* @file    epos.c
* @author  YX_L 李彪
* @version V2.0
* @date    2017-01-16
* @brief   已测试
*******************************************************************************
**/
#include "epos_N.h"
#include <math.h> 
#include "can.h"

Epos epos[EPOS_NUM + 1];
Epos epos1;
#define EPOS_DELAY1  6
#define EPOS_DELAY2  60
#define EPOS_DELAY3  100

/*********************************************************************************************************
** @function : Epos_Run(Epos* epos,int32_t speed,int32_t position,int8_t mode,int32_t opt)
** @input    : epos    要设置的epos
**		       : speed   设置速度，仅在PPMODE,PVMODE,VMODE三种模式下有用，PMODE下置0即可；
**           : mode    设置运行模式，可为PPMODE(平滑位置模式)、PMODE(位置模式)、
**           :            PVMODE(平滑速度模式)、VMODE(速度模式)，各种模式的特点请参考epos UserManul
**           : opt     平滑位置模式下的特殊参数，用于定义平滑位置模式的运行方式，仅对平滑位置模式
**                     起作用，其他模式下置0即可，
**                     定义如下:
**                     0x001f-完成当前操作（即走到前一次设定的位置）后再走到绝对位置position；
**                     0x003f-停止当前操作（即不走到前一次设定的位置），立即走到绝对位置position；
**                     0x005f-完成当前操作后，走到与当前位置相差position的位置；
**                     0x007f-停止当前操作，立即走到与当前位置相差position的位置；
** @output   : none
** @brief    : 只能使用Epos_Run()函数操作各种模式，无需调用各个子函数例如Epos_RunPVM()等操作Epos

1．平滑位移模式时，其位移计数标准是编码器的脉冲数，一圈有2000个脉冲。绝对位置是相对于初始化上电的位置，
相对位置就是相对上一次的位置。
Epos_Run(&epos[1], 2000, 42000, PPMODE, 0x007f);还需要考虑减数比，如果我们用的是21的减数比，所以外部显示就是其转了1圈。

2. 对于模式之间的切换，根据调试，最好就是先关闭输出  Epos_ShutDown(&epos[1])或者Epos_Stop(&epos[1]);
然后再对这个模式进行初始化，经过一段延时100ms左右，再对这个模式进行操作。详情可以参考我按键调试的程序1、2，
是基于原子的探索者的。
也可以不用这几个函数，先让他进入那个模式再对其进行操作。但调试中转换模式，驱动容易报错。

3．电流模式需要单独调用void Epos_RunCM(Epos* epos,int32_t Current,int32_t MAX_Speed)
他急停后，还是可以转动的，不会锁死。

*********************************************************************************************************/

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
void Epos_Run(Epos* epos,
	int32_t speed,
	int32_t position,
	uint8_t mode,
	int32_t opt)
{
	uint8_t i = 0;
	if (mode != epos->CurMode)
	{
		i = 1;
		epos->CurMode = mode;
	}

	if (mode == PPMODE)
	{
		epos->opt = opt;
		Epos_RunPPM(epos, speed, position, i);
	}
	else if (mode == PVMODE)
	{
		epos->opt = 0x000f;
		Epos_RunPVM(epos, speed, i);
	}
	else if (mode == PMODE)
	{
		Epos_RunPM(epos, position, i);
	}
	else if (mode == VMODE)
	{
		Epos_RunVM(epos, speed, i);
	}
}

/*******************************************************************************
** @function : 非精确延时函数
** @input    : time     设置延时计数值
** @output   : none
** @brief    ：无需操作，在宏定义中已经定义了几个合适的值
*******************************************************************************/
void Epos_Delay(int8_t time)
{
	int16_t i;
	for (; time>0; time--)
	{
		for (i = 0; i<50; i++);
	}
}

/*******************************************************************************
** @function : Epos     驱动参数写入
** @input    : Index    对象字典索引
**           : SubIndex 对象字典子索引
**           : param    设定参数
** @output   : none
** @brief    : 无需操作
*******************************************************************************/
void Epos_Write(Epos* epos, uint16_t Index, uint8_t SubIndex, int32_t param)
{
	if (param<0)
	{
		param = (~((-1) *param)) + 1;
	}

	epos->ID = 0x600 + epos->NODE_ID;

	epos->Buf[0] = 0x22;
	epos->Buf[1] = Index & 0xFF;
	epos->Buf[2] = (Index & 0xFF00) >> 8;
	epos->Buf[3] = SubIndex;
	epos->Buf[4] = param & 0xFF;
	epos->Buf[5] = (param & 0xFF00) >> 8;
	epos->Buf[6] = (param & 0xFF0000) >> 16;
	epos->Buf[7] = (param & 0xFF000000) >> 24;

	//delay_us(1000);
    Epos_Delay(EPOS_DELAY3);
    Epos_Delay(EPOS_DELAY3);
    Epos_Delay(EPOS_DELAY3);
    Epos_Delay(EPOS_DELAY3);

	CAN1_SendMsg(epos->Buf, epos->ID);
}

//BOOL Epos_Read(Epos* epos,uint16_t Index,int8_t SubIndex,int32_t* param)
//{   
//    int8_t  err,i;
//    uint16_t ret_Index,ret_SubIndex;
//    BOOL   r=FALSE;
//        
//    for(i=0;i<3;i++)
//    {
//				epos->buf.ID=0x600+epos->NODE_ID;
//				epos->buf.Buf[0]=0x40;
//				epos->buf.Buf[1]= Index&0xFF;
//				epos->buf.Buf[2]= (Index&0xFF00)>>8;
//				epos->buf.Buf[3]= SubIndex;
//				epos->buf.Buf[4]=0x00;
//				epos->buf.Buf[5]=0x00;
//				epos->buf.Buf[6]=0x00;
//				epos->buf.Buf[7]=0x00;
//				epos->buf.DataLen = 8;
//				epos->buf.IsExtend=0;
//				epos->buf.IsRemote=0;
//				r=Write(epos->CAN_Handles, &(epos->buf), CANTxCMD_NOR);
//				if(r==FALSE)break;
//				
//				r=Read(epos->CAN_Handles,&(epos->buf));
//				ret_Index=(((uint16_t)(epos->buf.Buf[2]))<<8)+((uint16_t)(epos->buf.Buf[1]));
//				ret_SubIndex=epos->buf.Buf[3];
//				if( ret_Index==Index && ret_SubIndex==SubIndex )
//			  {
//				    *param=(((int32_t)(epos->buf.Buf[7]))<<24)+(((int32_t)(epos->buf.Buf[6]))<<16)+(((int32_t)(epos->buf.Buf[5]))<<8) + (int32_t)(epos->buf.Buf[4]);
//						r=TRUE;
//						break;
//			  }
//				r=FALSE;
//    }
//    if(i==3)r=FALSE;
//    return r;
//}

void Epos_Star(Epos* epos)
{
	epos->ID = 0x700 + epos->NODE_ID;

	epos->Buf[0] = 0x00;
	epos->Buf[1] = 0x00;
	epos->Buf[2] = 0x00;
	epos->Buf[3] = 0x00;
	epos->Buf[4] = 0x00;
	epos->Buf[5] = 0x00;
	epos->Buf[6] = 0x00;
	epos->Buf[7] = 0x00;
    
	CAN1_SendMsg(epos->Buf, epos->ID);
}

/*******************************************************************************
** @function : EPOS2 驱动初始化
** @input    : none
** @output   : none
** @brief    ：EPOS_NUM是所需要的初始化的驱动个数
*******************************************************************************/
void Epos_Init(void)
{
	unsigned long i;

	for (i = 1; i<EPOS_NUM + 1; i++)
	{
        //epos[i].NODE_ID = pow(2, i - 1);
        epos[i].NODE_ID = 16;
        //epos[i].NODE_ID = i;
		epos[i].opt = 0x001f;
		epos[i].Acc = MAX_ACC;
		epos[i].Dec = MAX_ACC;
		epos[i].CurMode = 0x00;
		Epos_ParamInit(&epos[i]);
		Epos_Run(&epos[i], 0, 0, PVMODE, 0x003f);
	}

	Epos_Delay(EPOS_DELAY2);
	Epos_Delay(EPOS_DELAY2); //初始化后等待一段时间
}

/*******************************************************************************
** @function : EPOS2 参数初始化
** @input    : epos 要进行参数初始化的epos
** @output   : none
** @brief    ：MAX_F_ERR等宏定义不要在这里修改
*******************************************************************************/
void Epos_ParamInit(Epos* epos)
{
	Epos_Write(epos, 0x6040, 0x00, 0x0080);   //清除错误

	Epos_Delay(EPOS_DELAY3);

	Epos_Write(epos, 0x6065, 0x00, MAX_F_ERR);//最大跟随误差
	Epos_Delay(EPOS_DELAY3);

	Epos_Write(epos, 0x607F, 0x00, MAX_P_V);    //最大速度 要与上位机设置的一致
	Epos_Delay(EPOS_DELAY3);

	Epos_Write(epos, 0x6083, 0x00, MAX_ACC);  //最大加速度
	Epos_Delay(EPOS_DELAY3);

	Epos_Write(epos, 0x6084, 0x00, MAX_ACC);   //最大减速度
	Epos_Delay(EPOS_DELAY3);

	Epos_Write(epos, 0x6085, 0x00, QDEC);      //急停减速度
	Epos_Delay(EPOS_DELAY3);
}

/*******************************************************************************
** @function : EPOS2 单个驱动初始化
** @input    : epos 要设置的epos
**           ：NODE_ID1 epos节点号6
** @output   : none
** @brief    ：暂时不需要
*******************************************************************************/
void Epos_SInit(Epos* epos1, int8_t NODE_ID1)
{
	epos1->NODE_ID = NODE_ID1;
	epos1->CurMode = 0x00;
	epos1->opt = 0x001f;
	epos1->Acc = MAX_ACC;
	epos1->Dec = MAX_ACC;

	Epos_Star(epos1);
	Epos_ParamInit(epos1);
	Epos_Run(epos1, 0, 0, PPMODE, 0x007f);
}

/*******************************************************************************
** @function : EPOS2 运行平滑位置模式
** @input    : epos 要设置的epos
**           : speed 设置速度
**           : position 设置运行到的位置
**           : opt 用于定义平滑位置模式的运行方式
**           :   定义如下：
**           :              0x001f-完成当前操作（即走到前一次设定的位置）后再走到绝对位置position
**           :              0x003f-停止当前操作（即不走到前一次设定的位置），立即走到绝对位置position
**           :              0x005f-完成当前操作后，走到与当前位置相差position的位置
**           :              0x007f-停止当前操作，立即走到与当前位置相差position的位置
** @output   : none
** @brief    : 暂时不需要
*******************************************************************************/
void Epos_RunPPM(Epos* epos, int32_t speed, int32_t position, uint8_t opt)
{
	if (opt)
	{
		Epos_Write(epos, 0x6060, 0x00, 0x01);
		Epos_Delay(EPOS_DELAY1);

		Epos_Write(epos, 0x6040, 0x00, 0x06);
		Epos_Delay(EPOS_DELAY2);

		Epos_Write(epos, 0x6040, 0x00, 0x0f);
		Epos_Delay(EPOS_DELAY2);

		Epos_Write(epos, 0x6081, 0x00, speed);
		Epos_Delay(EPOS_DELAY1);

		Epos_Write(epos, 0x607A, 0x00, position);
		Epos_Delay(EPOS_DELAY1);

		Epos_Write(epos, 0x6040, 0x00, epos->opt);
		Epos_Delay(EPOS_DELAY2);

		epos->opt_mem = epos->opt;
		epos->Position = position;
	}
	else if ((epos->opt_mem != epos->opt) || (epos->Position != position))
	{
		Epos_Write(epos, 0x6081, 0x00, speed);
		Epos_Delay(EPOS_DELAY1);

		Epos_Write(epos, 0x607A, 0x00, position);
		Epos_Delay(EPOS_DELAY1);

		Epos_Write(epos, 0x6040, 0x00, epos->opt);
		Epos_Delay(EPOS_DELAY2);

		epos->opt_mem = epos->opt;
		epos->Position = position;
	}
}

/*******************************************************************************
** @function : EPOS2 运行平滑速度模式
** @input    : epos 要设置的epos
**           ：speed 要设置的速度
**           ：opt 平滑位置模式下的特殊参数，该模式下设置为0
** @output   : none
** @brief    ：这个子函数无需直接操作
*******************************************************************************/
void Epos_RunPVM(Epos* epos, int32_t speed, uint8_t opt)
{
	if (opt)
	{
		Epos_Write(epos, 0x6060, 0x00, 0x03);
		Epos_Delay(EPOS_DELAY3);

		Epos_Write(epos, 0x6040, 0x00, 0x06);
		Epos_Delay(EPOS_DELAY3);

		Epos_Write(epos, 0x6040, 0x00, 0x0f);
		Epos_Delay(EPOS_DELAY3);

		Epos_Write(epos, 0x60FF, 0x00, speed);
		Epos_Delay(EPOS_DELAY3);

		Epos_Write(epos, 0x6040, 0x00, epos->opt);
		Epos_Delay(EPOS_DELAY3);

		epos->Speed = speed;
	}

	else if (epos->Speed != speed)
	{
		Epos_Write(epos, 0x60FF, 0x00, speed);
		Epos_Delay(EPOS_DELAY3);

		Epos_Write(epos, 0x6040, 0x00, epos->opt);
		Epos_Delay(EPOS_DELAY3);

		epos->Speed = speed;
	}
}

/*******************************************************************************
** @function : EPOS2    运行位置模式
** @input    : epos     要设置的epos
**           ：position 要设置运行到的位置
**           ：opt      平滑位置模式下的特殊参数，该模式下设置为0
** @output   : none
** @brief    ：这个子函数无需直接操作
*******************************************************************************/
void Epos_RunPM(Epos* epos, int32_t position, uint8_t opt)
{
	if (opt)
	{
		Epos_Write(epos, 0x6060, 0x00, 0xFF);
		Epos_Delay(EPOS_DELAY1);

		Epos_Write(epos, 0x6065, 0x00, 2147483640);
		Epos_Delay(EPOS_DELAY1);

		Epos_Write(epos, 0x6040, 0x00, 0x06);
		Epos_Delay(EPOS_DELAY2);

		Epos_Write(epos, 0x6040, 0x00, 0x0f);
		Epos_Delay(EPOS_DELAY2);

		Epos_Write(epos, 0x2062, 0x00, position);
		Epos_Delay(EPOS_DELAY3);

		epos->Position = position;
	}
	else if (epos->Position != position)
	{
		Epos_Write(epos, 0x2062, 0x00, position);
		Epos_Delay(EPOS_DELAY3);

		epos->Position = position;
	}
}

/*******************************************************************************
** @function : EPOS2    运行速度模式
** @input    : epos     要设置的epos
**           ：speed    要设置运行速度
**           ：opt      平滑位置模式下的特殊参数，该模式下设置为0
** @output   : none
** @brief    ：这个子函数无需直接操作
*******************************************************************************/
void Epos_RunVM(Epos* epos, int32_t speed, uint8_t opt)
{
	if (opt)
	{
		Epos_Write(epos, 0x6060, 0x00, 0xFE);
		Epos_Delay(EPOS_DELAY1);

		Epos_Write(epos, 0x6040, 0x00, 0x06);
		Epos_Delay(EPOS_DELAY2);

		Epos_Write(epos, 0x6040, 0x00, 0x0f);
		Epos_Delay(EPOS_DELAY2);

		Epos_Write(epos, 0x206B, 0x00, speed);
		Epos_Delay(EPOS_DELAY3);

		epos->Speed = speed;
	}
	else if (epos->Speed != speed)
	{
		Epos_Write(epos, 0x206B, 0x00, speed);
		Epos_Delay(EPOS_DELAY3);

		epos->Speed = speed;
	}
}

/*******************************************************************************
** @function : EPOS2     运行电流模式
** @input    : epos      要设置的epos
**           ：Current   要设置运行电流 单位mA
**           ：MAX_Speed 设置在设置的电流下电机的最大转速，单位为rpm，如果转速搭到最大转速，而电流还
**           ：          没有达到最大值，电流将不再增大，如此可以在电流模式下做到限速！若要保持电流恒
**           ：          定，可以将MAX_Speed设得较大，但最大不能超过上位机设定的速度
** @output   : none
** @brief    ：这个子函数无需直接操作
*******************************************************************************/
void Epos_RunCM(Epos* epos, int32_t Current, int32_t MAX_Speed)
{
	if (epos->CurMode != CMODE)
	{
		epos->CurMode = CMODE;

		Epos_Write(epos, 0x6060, 0x00, 0xFD);
		Epos_Delay(EPOS_DELAY1);

		Epos_Write(epos, 0x6410, 0x04, MAX_Speed);
		Epos_Delay(EPOS_DELAY1);

		Epos_Write(epos, 0x6040, 0x00, 0x06);
		Epos_Delay(EPOS_DELAY3);

		Epos_Write(epos, 0x6040, 0x00, 0x0f);
		Epos_Delay(EPOS_DELAY3);

		Epos_Write(epos, 0x6410, 0x04, MAX_Speed);
		Epos_Delay(EPOS_DELAY1);

		Epos_Write(epos, 0x2030, 0x00, Current);
		Epos_Delay(EPOS_DELAY3);

		Epos_Write(epos, 0x6040, 0x00, 0x0f);
		Epos_Delay(EPOS_DELAY3);

		epos->Current = Current;
		epos->CMSpeed = MAX_Speed;
	}
	else if ((epos->Current != Current) || (epos->CMSpeed != MAX_Speed))
	{
		Epos_Write(epos, 0x6410, 0x04, MAX_Speed);
		Epos_Delay(EPOS_DELAY1);

		Epos_Write(epos, 0x2030, 0x00, Current);
		Epos_Delay(EPOS_DELAY3);

		Epos_Write(epos, 0x6040, 0x00, 0x0f);
		Epos_Delay(EPOS_DELAY3);

		epos->Current = Current;
		epos->CMSpeed = MAX_Speed;
	}
}

/*******************************************************************************
** @function : EPOS2    急停
** @input    : epos     要设置的epos
** @output   : none
** @brief    ：
*******************************************************************************/
void Epos_Stop(Epos* epos)
{
	Epos_Write(epos, 0x6040, 0x00, 0x000B);
	Epos_Delay(EPOS_DELAY2);
}

/*******************************************************************************
** @function : EPOS2    急停
** @input    : epos     要设置的epos
** @output   : none
** @brief    ：
*******************************************************************************/
void Epos_HaltStop(Epos* epos)
{
	Epos_Write(epos, 0x6040, 0x00, 0x10F);
	Epos_Delay(EPOS_DELAY2);
}


/*******************************************************************************
** @function : EPOS2    关闭输出
** @input    : epos     要设置的epos
** @output   : none
** @brief    ：
*******************************************************************************/
void Epos_ShutDown(Epos* epos)
{
	Epos_Write(epos, 0x6040, 0x00, 0x0006);
	epos->CurMode = 0x00;
	Epos_Delay(EPOS_DELAY2);
}

/*******************************************************************************
** @function : EPOS2    设置加速度
** @input    : epos     要设置的epos
**           ：acc      要设定的加速度值
** @output   : none
** @brief    ：
*******************************************************************************/
void Epos_SetAcc(Epos* epos, int32_t acc)
{
	epos->Acc = acc;
	Epos_Write(epos, 0x6083, 0x00, acc);
	Epos_Delay(EPOS_DELAY1);
}

/*******************************************************************************
** @function : EPOS2    设置减速度
** @input    : epos     要设置的epos
**           ：dec      要设定的减速度值
** @output   : none
** @brief    ：
*******************************************************************************/
void Epos_SetDec(Epos* epos, int32_t dec)
{
	epos->Dec = dec;
	Epos_Write(epos, 0x6084, 0x00, dec);
	Epos_Delay(EPOS_DELAY1);
}

/*******************************************************************************
** @function : EPOS2    监控函数
** @input    : epos     要查看的epos
** @output   : none
** @brief    ：暂时没写。。。。
*******************************************************************************/
//void Epos_Watch(Epos* epos)
//{   
//    int32_t err;
//    if ( Epos_Read(epos, 0x1001, 0x00, &err) == TRUE )
//    {
//        if (err != 0)
//				{
//				    Epos_ParamInit(epos);
//						epos->CurMode= 0x00;
//						epos->opt= 0x001f;
//				}
//    }
//}



/*    更改
函数名称：void Velocity_Change(int v1,int v2,int v3,int v4)
函数功能：  设置轮廓速度模式下速度
函数输入：  v1 v2 v3 v4
函数输出：  无
*/

int Give_v_flag=0;
void Profile_Velocity_Change(int v1, int v2, int v3, int v4)
{

	Epos_Run(&epos[1], v1, 0, PVMODE, 0);
	delay_ms(5);
	Epos_Run(&epos[2], v2, 0, PVMODE, 0);
	Epos_Run(&epos[3], v3, 0, PVMODE, 0);
	Epos_Run(&epos[4], v4, 0, PVMODE, 0);

}

/*    更改
函数名称： Profile_Velocity_Stop(void)
函数功能： 轮廓速度模式下电机急停
函数输入： 无
函数输出： 无
*/
void Profile_Velocity_Stop(void)
{
	Epos_HaltStop(&epos[1]);
	Epos_HaltStop(&epos[2]);
	Epos_HaltStop(&epos[3]);
	Epos_HaltStop(&epos[4]);
}
