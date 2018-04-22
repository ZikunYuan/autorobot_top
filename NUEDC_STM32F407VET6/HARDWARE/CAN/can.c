/**
******************************************************************************
* @file    bsp_can.c
* @author  YX_L 李彪
* @date    2017-02-10
* @brief   这个文件提供CAN通信的初始化、数据收发送
******************************************************************************
* @attention
*          本文件适用于F103 库函数V3.5.0 和F407 库函数V1.4.0
*          使用时只需直接拷贝进相关文件夹即可，以下是详细使用说明
*          如果用F4:
*          1.首先调用 void CAN_Config(void)进行初始化
*          2.CAN1发送数据调用 u8 CAN1_SendMsg(u8* msg,u32 TX_STD_ID)
*          3.CAN1接收数据调用 u8 CAN1_ReceiveMsg(u8* buf)
*          4.使用CAN2时首先在 bsp_can.h 中将 CAN2_USE_ENABLE 设置为1
*          5.CAN2发送数据调用 u8 CAN2_SendMsg(u8* msg,u32 TX_STD_ID)
*          6.CAN2接收数据调用 u8 CAN2_ReceiveMsg(u8* buf)
*          7.使用接收中断时，请先在 bsp_can.h 中使能
*
*          如果用F1:
*          1.首先调用 u8 CAN_Config()进行初始化
*          2.CAN1发送数据调用 u8 CAN1_SendMsg(u8* msg, u32 TX_STD_ID)
*          3.CAN1接收数据调用 u8 CAN1_ReceiveMsg(u8 *buf)
*          7.使用接收中断时，请先在 bsp_can.h 中使能 CAN_RX0_INT_ENABLE
*
******************************************************************************
*/
#include "can.h"
#include "beep.h"

CanTxMsg TxMessage;
/**********************************使用F4 CAN通信*******************************/

static void CAN_GPIO_Config(void);
static void CAN_NVIC_Config(void);
static void CAN_Mode_Config(void);
static void CAN_Filter_Config(void);

extern u8 NormalShuttlecock_1_Flag;
extern u16 Golden_Start_Place;

/*
* 函数名：CAN_Config
* 描述  ：完整配置CAN的功能
* 输入  ：无
* 输出  ：无
* 调用  ：外部调用
*/
void CAN_Config(void)
{
    CAN_GPIO_Config();
    CAN_NVIC_Config();
    CAN_Mode_Config();
    CAN_Filter_Config();
}

/*
* 函数名：CAN1_SendMsg
* 描述  ：使用CAN1发送数据
* 输入  ：msg ：数据指针,最大为8个字节
*         TX_STD_ID ：发送ID
* 输出  ：1 发送成功
*         0 发送失败
* 调用  ：外部调用
*/
u8 CAN1_SendMsg(u8* msg, u32 TX_STD_ID)
{
    u8 mbox;
    u16 i = 0;
    TxMessage.StdId = TX_STD_ID;				 //使用的标准ID
    TxMessage.IDE = CAN_ID_STD;					 //标准模式
    TxMessage.RTR = CAN_RTR_DATA;				 //发送的是数据
    TxMessage.DLC = 8;							     //数据长度为2字节
    for (i = 0; i<8; i++)
    {
        TxMessage.Data[i] = msg[i];
    }
    mbox = CAN_Transmit(CAN1, &TxMessage);
    i = 0;
    while ((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i<0XFFF))i++;	//等待发送结束
    if (i >= 0XFFF)return 1;//发送失败
    return 0;		         //发送成功
}

/*
* 函数名：CAN1_ReceiveMsg
* 描述  ：使用CAN1接收数据
* 输入  ：buf:数据指针,最大为8个字节
* 输出  ：接收数据长度
* 调用  ：外部调用
*/
u8 CAN1_ReceiveMsg(u8* buf)
{
    CanRxMsg RxMessage;
    u16 i;
    if (CAN_MessagePending(CAN1, CAN_FIFO0) == 0)return 0;		//查询FIFO0是否有信息 没有就直接退出
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据
    for (i = 0; i<RxMessage.DLC; i++)
    {
        buf[i] = RxMessage.Data[i];
    }
    return RxMessage.DLC;
}

/*
* 函数名：CAN1_RX0_IRQHandler
* 描述  ：使用CAN1中断服务函数，测试使用
* 输入  ：无
* 输出  ：无
* 调用  ：外部调用
*/

//中断服务函数
extern u8 ArriveTZ1_Flag,ArriveTZ2_Flag,ArriveTZ3_Flag,
	        NormalShuttlecock_Flag,GoldenShuttlecock_Flag;

void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg rx_message;
    
    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx_message);     
        if((rx_message.IDE == CAN_Id_Standard)&&(rx_message.IDE == CAN_RTR_Data)&&(rx_message.DLC == 8)) //标准帧、数据帧、数据长度为8
        {
        }             
    }
}


/*
* 函数名：CAN_GPIO_Config
* 描述  ：CAN的GPIO 配置
* 输入  ：无
* 输出  ：无
* 调用  ：内部调用
*/
static void CAN_GPIO_Config(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能PORTA时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); //使能CAN1时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;         //复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;       //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;   //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;         //上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);               //初始化PA11,PA12
    //引脚复用映射配置
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1); //GPIOA11复用为CAN1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1); //GPIOA12复用为CAN1

}

/*
* 函数名：CAN_NVIC_Config
* 描述  ：CAN1的NVIC 配置,CAN1第1优先级组，0优先级 CAN2第2优先级组，1优先级
* 输入  ：无
* 输出  ：无
* 调用  ：内部调用
*/
static void CAN_NVIC_Config(void)
{



    NVIC_InitTypeDef  NVIC_InitStructure;

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);     //FIFO0消息挂号中断允许.
    /*
    *CAN_IT_FMP0   FIFO0消息挂号中断允许
    *CAN_IT_FF0    FIFO0消息满中断允许
    *CAN_IT_FOV0   FIFO0消息上溢中断允许
    */
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;           //CAN1 FIFO0 接收中断
    /*
    *CAN1_TX_IRQn  CAN1 发送中断
    *CAN1_RX0_IRQn CAN1 FIFO0中断
    *CAN1_RX1_IRQn CAN1 FIFO1中断
    *CAN1_SCE_IRQn CAN1 状态改变中断
    *其他中断详见stm32f4xx_can.h 537行
    */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

/*
* 函数名：CAN_Mode_Config
* 描述  ：CAN的模式 配置
* 输入  ：无
* 输出  ：无
* 调用  ：内部调用
*/
static void CAN_Mode_Config(void)
{
    CAN_InitTypeDef    CAN_InitStructure;

    CAN_InitStructure.CAN_TTCM = DISABLE;	             //非时间触发通信模式
    CAN_InitStructure.CAN_ABOM = DISABLE;	             //软件自动离线管理
    CAN_InitStructure.CAN_AWUM = DISABLE;              //睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
    CAN_InitStructure.CAN_NART = ENABLE;	             //禁止报文自动传送
    CAN_InitStructure.CAN_RFLM = DISABLE;	             //报文不锁定,新的覆盖旧的
    CAN_InitStructure.CAN_TXFP = DISABLE;	             //优先级由报文标识符决定
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;	   //模式设置
    //设置波特率
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;	         //BTR-SJW 重新同步跳跃宽度 2个时间单元
    CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;           //BTR-TS1 时间段1 占用了6个时间单元
    CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;           //BTR-TS1 时间段2 占用了3个时间单元
    CAN_InitStructure.CAN_Prescaler = 3;               //BTR-BRP 波特率分频器  定义了时间单元的时间 36/(1+6+3)/4=0.9Mbps
    CAN_Init(CAN1, &CAN_InitStructure);              // 初始化CAN1
}

/*
* 函数名：CAN_Filter_Config
* 描述  ：CAN的过滤器 配置
* 输入  ：无
* 输出  ：无
* 调用  ：内部调用
*/
static void CAN_Filter_Config(void)
{
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;

    /***********************1个32位的标识符屏蔽位模式的过滤器********************************/
    CAN_FilterInitStructure.CAN_FilterNumber = 0;	                      //过滤器0
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;    //32位

    CAN_FilterInitStructure.CAN_FilterIdHigh = ((u16)CAN1_RX_STD_ID << 5) & 0xFFFF;
    //CAN_FilterIdHigh包含的是STD[10:0]和EXID[17:13],标准CAN ID本身是不包含扩展ID数据，
    //因此为了要将标准CAN ID放入此寄存器，标准CAN ID首先应左移5位后才能对齐.
    CAN_FilterInitStructure.CAN_FilterIdLow = CAN_ID_STD;

    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;              //过滤器高16位每位必须匹配
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;               //过滤器低16位每位必须匹配
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;//过滤器0关联到FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;              //激活过滤器0
    CAN_FilterInit(&CAN_FilterInitStructure);                         //滤波器初始化

}
