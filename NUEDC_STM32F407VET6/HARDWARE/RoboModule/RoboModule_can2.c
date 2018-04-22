#include "RoboModule_can2.h"
#include "delay.h"

extern u8 NormalShuttlecock_1_Flag;
extern u16 Golden_Start_Place;
extern u8 ArriveTZ3_Flag;

unsigned int CAN2_Time_Out = 0;

static void CAN2_Delay_Us(unsigned int t)
{
    int i;
    for(i=0; i<t; i++)
    {
        int a=40;
        while(a--);
    }
}


/*----CAN2_TX-----PA12----*/
/*----CAN2_RX-----PA11----*/

//本接收数据的函数，默认为4个驱动器，都挂在0组，编号为1、2、3、4
/*************************************************************************
                          CAN2_RX0_IRQHandler
描述：CAN2的接收中断函数
*************************************************************************/
void CAN2_RX1_IRQHandler(void)
{
    CanRxMsg rx_message;

    if (CAN_GetITStatus(CAN2,CAN_IT_FMP1)!= RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP1);
        CAN_Receive(CAN2, CAN_FIFO1, &rx_message);

        if((rx_message.IDE == CAN_Id_Standard)&&(rx_message.IDE == CAN_RTR_Data)&&(rx_message.DLC == 8)) //标准帧、数据帧、数据长度为8
        {
            if(rx_message.StdId == 0x12B)
            {
                Real2_Current_Value[1] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                Real2_Velocity_Value[1] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
                Real2_Position_Value[1] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
            }
        }
    }
}

/*************************************************************************
                          CAN2_Configuration
描述：初始化CAN2配置为1M波特率
*************************************************************************/
void CAN2_Configuration(void)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);      //can1为主，CAN2为从，必须先使能CAN1

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);

    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOB, &gpio);

    nvic.NVIC_IRQChannel = CAN2_RX1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel = CAN2_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    CAN_DeInit(CAN2);
    CAN_StructInit(&can);

    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = DISABLE;
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = DISABLE;
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = ENABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_9tq;
    can.CAN_BS2 = CAN_BS2_4tq;
    can.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN2, &can);

    can_filter.CAN_FilterNumber = 14;
    can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
    can_filter.CAN_FilterScale = CAN_FilterScale_32bit;
    can_filter.CAN_FilterIdHigh = 0x0001;
    can_filter.CAN_FilterIdLow = 0x0001;
    can_filter.CAN_FilterMaskIdHigh = 0x0000;
    can_filter.CAN_FilterMaskIdLow = 0x0000;
    can_filter.CAN_FilterFIFOAssignment = 1;
    can_filter.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&can_filter);

    CAN_ITConfig(CAN2,CAN_IT_FMP1,ENABLE);
    CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);
}


/********************************
参数：Group：RoboModule的分组，取值范围0-7
      Number：RoboModule的编号，取值范围1-15
      Tim：要求驱动器返回当前位置速度等参数的时间，0为不反回参数
      Mode：初始化驱动器模式
      Ctl1_Ctl2：选择触碰开光功能，0为失能，1为使能
备注：此函数自带较长延时，延时时间较长，修改延时需谨慎，
     延时太少可能导致驱动器初始化不成功

*********************************/

void RoboModule_CAN2_Init(unsigned char Group,unsigned char Number,unsigned char Tim,unsigned char Mode,unsigned char  Ctl1_Ctl2)
{
    delay_ms(100);
    CAN2_RoboModule_DRV_Reset(Group,Number);//对0组所有驱动器进行复位
    delay_ms(500);//发送复位指令后的延时必须要有，等待驱动器再次初始化完成
    CAN2_RoboModule_DRV_Config(Group,Number,Tim,Ctl1_Ctl2);
    delay_us(500);
    CAN2_RoboModule_DRV_Mode_Choice(Group,Number,Mode); //0组的所有驱动器 都进入开环模式
    delay_ms(500); //发送模式选择指令后，要等待驱动器进入模式就绪。所以延时也不可以去掉。
}


unsigned char can2_tx_success_flag = 0;
/*************************************************************************
                          CAN2_TX_IRQHandler
描述：CAN2的发送中断函数
*************************************************************************/
void CAN2_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET)
    {
        CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
        can2_tx_success_flag=1;
    }
}

/****************************************************************************************
                                       复位指令
Group   取值范围 0-7
Number  取值范围 0-15，其中Number==0时，为广播发送
*****************************************************************************************/
void CAN2_RoboModule_DRV_Reset(unsigned char Group,unsigned char Number)
{
    unsigned short can_id = 0x000;
    CanTxMsg tx_message;

    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8

    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }

    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    tx_message.Data[0] = 0x55;
    tx_message.Data[1] = 0x55;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;

    can2_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);

    CAN2_Time_Out = 0;
    while(can2_tx_success_flag == 0)
    {
        CAN2_Delay_Us(1);
        CAN2_Time_Out++;
        if(CAN2_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                     模式选择指令
Group   取值范围 0-7
Number  取值范围 0-15，其中Number==0时，为广播发送

Mode    取值范围

OpenLoop_Mode                       0x01
Current_Mode                        0x02
Velocity_Mode                       0x03
Position_Mode                       0x04
Velocity_Position_Mode              0x05
Current_Velocity_Mode               0x06
Current_Position_Mode               0x07
Current_Velocity_Position_Mode      0x08
*****************************************************************************************/
void CAN2_RoboModule_DRV_Mode_Choice(unsigned char Group,unsigned char Number,unsigned char Mode)
{
    unsigned short can_id = 0x001;
    CanTxMsg tx_message;

    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8

    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }

    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    tx_message.Data[0] = Mode;
    tx_message.Data[1] = 0x55;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;

    can2_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);

    CAN2_Time_Out = 0;
    while(can2_tx_success_flag == 0)
    {
        CAN2_Delay_Us(1);
        CAN2_Time_Out++;
        if(CAN2_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                   开环模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = ±5000时，最大输出电压为电源电压

*****************************************************************************************/
void CAN2_RoboModule_DRV_OpenLoop_Mode(unsigned char Group,unsigned char Number,short Temp_PWM)
{
    unsigned short can_id = 0x002;
    CanTxMsg tx_message;

    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8

    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }

    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }

    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;

    can2_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);

    CAN2_Time_Out = 0;
    while(can2_tx_success_flag == 0)
    {
        CAN2_Delay_Us(1);
        CAN2_Time_Out++;
        if(CAN2_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                   电流模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_current的取值范围如下：
-32768 ~ +32767，单位mA

*****************************************************************************************/
void CAN2_RoboModule_DRV_Current_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Current)
{
    unsigned short can_id = 0x003;
    CanTxMsg tx_message;

    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8

    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }

    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }

    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }

    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;

    can2_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);

    CAN2_Time_Out = 0;
    while(can2_tx_success_flag == 0)
    {
        CAN2_Delay_Us(1);
        CAN2_Time_Out++;
        if(CAN2_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                   速度模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_velocity的取值范围如下：
-32768 ~ +32767，单位RPM

*****************************************************************************************/
void CAN2_RoboModule_DRV_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity)
{
    unsigned short can_id = 0x004;
    CanTxMsg tx_message;

    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8

    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }

    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }

    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }

    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;

    can2_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);

    CAN2_Time_Out = 0;
    while(can2_tx_success_flag == 0)
    {
        CAN2_Delay_Us(1);
        CAN2_Time_Out++;
        if(CAN2_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                   位置模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc

*****************************************************************************************/
void CAN2_RoboModule_DRV_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,long Temp_Position)
{
    unsigned short can_id = 0x005;
    CanTxMsg tx_message;

    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8

    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }

    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }

    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }

    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);

    can2_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);

    CAN2_Time_Out = 0;
    while(can2_tx_success_flag == 0)
    {
        CAN2_Delay_Us(1);
        CAN2_Time_Out++;
        if(CAN2_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                  速度位置模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_velocity的取值范围如下：
0 ~ +32767，单位RPM

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc
*****************************************************************************************/
void CAN2_RoboModule_DRV_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x006;
    CanTxMsg tx_message;

    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8

    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }

    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }

    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }

    if(Temp_Velocity < 0)
    {
        Temp_Velocity = abs(Temp_Velocity);
    }

    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);

    can2_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);

    CAN2_Time_Out = 0;
    while(can2_tx_success_flag == 0)
    {
        CAN2_Delay_Us(1);
        CAN2_Time_Out++;
        if(CAN2_Time_Out>100)
        {
            break;
        }
    }
}


/****************************************************************************************
                                  电流速度模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_current的取值范围如下：
0 ~ +32767，单位mA

temp_velocity的取值范围如下：
-32768 ~ +32767，单位RPM

*****************************************************************************************/
void CAN2_RoboModule_DRV_Current_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity)
{
    unsigned short can_id = 0x007;
    CanTxMsg tx_message;

    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8

    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }

    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }

    tx_message.Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;

    can2_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);

    CAN2_Time_Out = 0;
    while(can2_tx_success_flag == 0)
    {
        CAN2_Delay_Us(1);
        CAN2_Time_Out++;
        if(CAN2_Time_Out>100)
        {
            break;
        }
    }
}


/****************************************************************************************
                                  电流位置模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_current的取值范围如下：
0 ~ +32767，单位mA

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc

*****************************************************************************************/
void CAN2_RoboModule_DRV_Current_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,long Temp_Position)
{
    unsigned short can_id = 0x008;
    CanTxMsg tx_message;

    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8

    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }

    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID


    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }

    tx_message.Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);

    can2_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);

    CAN2_Time_Out = 0;
    while(can2_tx_success_flag == 0)
    {
        CAN2_Delay_Us(1);
        CAN2_Time_Out++;
        if(CAN2_Time_Out>100)
        {
            break;
        }
    }
}


/****************************************************************************************
                                  电流速度位置模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_current的取值范围如下：
0 ~ +32767，单位mA

temp_velocity的取值范围如下：
0 ~ +32767，单位RPM

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc

*****************************************************************************************/
void CAN2_RoboModule_DRV_Current_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x009;
    CanTxMsg tx_message;

    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8

    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }

    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }

    if(Temp_Velocity < 0)
    {
        Temp_Velocity = abs(Temp_Velocity);
    }

    tx_message.Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);

    can2_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);

    CAN2_Time_Out = 0;
    while(can2_tx_success_flag == 0)
    {
        CAN2_Delay_Us(1);
        CAN2_Time_Out++;
        if(CAN2_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                      配置指令
Temp_Time的取值范围: 0 ~ 255，为0时候，为关闭电流速度位置反馈功能
Ctl1_Ctl2的取值范围：0 or 1 ，当不为0 or 1，则认为是0，为关闭左右限位检测功能
特别提示：Ctl1，Ctl2的功能仅存在于102 301，其余版本驱动器，Ctl1_Ctl2 = 0 即可
*****************************************************************************************/
void CAN2_RoboModule_DRV_Config(unsigned char Group,unsigned char Number,unsigned char Temp_Time,unsigned char Ctl1_Ctl2)
{
    unsigned short can_id = 0x00A;
    CanTxMsg tx_message;

    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8

    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }

    if((Ctl1_Ctl2 != 0x00)&&(Ctl1_Ctl2 != 0x01))
    {
        Ctl1_Ctl2 = 0x00;
    }

    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    tx_message.Data[0] = Temp_Time;
    tx_message.Data[1] = Ctl1_Ctl2;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;

    can2_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);

    CAN2_Time_Out = 0;
    while(can2_tx_success_flag == 0)
    {
        CAN2_Delay_Us(1);
        CAN2_Time_Out++;
        if(CAN2_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                      在线检测
*****************************************************************************************/
void CAN2_RoboModule_DRV_Online_Check(unsigned char Group,unsigned char Number)
{
    unsigned short can_id = 0x00F;
    CanTxMsg tx_message;

    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8

    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }

    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    tx_message.Data[0] = 0x55;
    tx_message.Data[1] = 0x55;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;

    can2_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);

    CAN2_Time_Out = 0;
    while(can2_tx_success_flag == 0)
    {
        CAN2_Delay_Us(1);
        CAN2_Time_Out++;
        if(CAN2_Time_Out>100)
        {
            break;
        }
    }
}

short Real2_Current_Value[4] = {0};
short Real2_Velocity_Value[4] = {0};
long Real2_Position_Value[4] = {0};
char Real2_Online[4] = {0};
char Real2_Ctl1_Value[4] = {0};
char Real2_Ctl2_Value[4] = {0};

/*******************************************************
函数名：CAN2_SendMsg
函数功能：can2发送数据帧
注意事项：ID禁用0x0102和0x00xx
时间：2018/3/10
*/
u8 CAN2_SendMsg(u8* msg, u32 TX_STD_ID)
{
	  CanTxMsg tx_message;
    u8 mbox;
    u16 i = 0;
    tx_message.StdId = TX_STD_ID;				 //?????ID
    tx_message.IDE = CAN_ID_STD;					 //????
    tx_message.RTR = CAN_RTR_DATA;				 //??????
    tx_message.DLC = 8;							     //?????2??
    for (i = 0; i<8; i++)
    {
        tx_message.Data[i] = msg[i];
    }
    mbox = CAN_Transmit(CAN2, &tx_message);
    i = 0;
    while ((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i<0XFFF))i++;	//??????
    if (i >= 0XFFF)return 1;//????
    return 0;		         //????
}
