#include "main_assistant.h"

/**************************************************************************
函数功能: Ec_spos
入口参数：无
返回  值：无
**************************************************************************/
extern u8 gold_num;
u32 pwm_num=0;
int Location1Num, Location2Num, Speed1Num;//设置PID停止位置以及期望速度
u8 RecevieFlag = 0;//自动车接收到才球架标志
u8 NextBall = 0;//彩球架切换球标志
u8 Speed2Flag = 0;//速度环PID调节标志
u8 Location2Flag = 0;   //位置环PID调节标志
u8 Light1_Flag = 0;
int Sign_Flag=1;             //用于匿名正负号标志位
//u8 ArriveTZ1_Flag=0;
//u8 ArriveTZ2_Flag=0;
u8 ArriveTZ3_Flag=0;
//u8 NormalShuttlecock_Flag=0;
//u8 NormalShuttlecock_1_Flag=0;
u8 GoldenShuttlecock_Flag=0;

//抛射位置的参数
u16 Release=0;//解除即速度为零
//抛射位置正负 抛射的位置（还是编码器位置）甩球的停止时间
Eje_value Eje[7]= {{1,0*4000+1905,150},
    {1,1*4000+1855,200},
    {1,2*4000+1760,200},
    {1,3*4000+1760,200},
    {1,4*4000+1760,200},
    {1,5*4000+1760,200},
    {1,6*4000+1760,200},
};
Para_value Para[7]= {{3,4,5,6},
    {7,8,9,10},
    {11,12,13,14},
    {19,23,27,31},
    {20,24,28,32},
    {21,25,29,33},
    {22,26,30,34},
};
//capture prepare shoot shoot_swing
u8 clear_pos=0;
u8 color_to_gold=1;
//以上为抛射位置的参数

u32 BallPosition[]= {0x1fd3,0x456c,0x6c0d,0x8e27,0xb2b8}; //每个值对应球架上的每个球的位置
int Shot_Flag = 0;
int BallNum=0;//用来表示第几个球,0表示第一个球
extern int Moto1,Moto2;
extern int Now1_Encoder, Last1_Encoder;//用于速度环时记录前后编码器读数

extern void System_Init(void);

/**************************************************************************
函数功能：扔球的一系列动作的函数
入口参数：无
返回  值：无
**************************************************************************/
//到初始化步骤也就是抓球的前一个动作
void Init_Movement(int num)//抓球位置
{
    if(num==2) {
        Relay1=0;    //第三次抛射将气缸退回
        delay_ms(700);
    }
    delay_ms(5);//没有实际用处 但不能删
    CAN1_SendMsg(Compensation_Mode(TIM5->CNT),Elmo_ID);//补偿编码器
    delay_ms(5);
    CAN1_SendMsg(Parameter_Mode(Para[num+gold_num].capture,Parameter_Enable),Elmo_ID);
    delay_ms(300);
}

//彩球：到竖直位置准备扔球 金球：抓球并且进入竖直位置准备扔球
void Prepare_Movement(int num)//准备发射位置
{
    if(num<2)//彩球
    {
        Relay3 = 1;//Init_Movement已经到了抓球位置 直接抓球就可以到准备位置了
        delay_ms(50);//等待夹住了再退车
        USART_SendData(UART5,Capture_Color_And_Prepare_Done);
        CAN1_SendMsg(Compensation_Mode(TIM5->CNT),Elmo_ID);//补偿编码器
        delay_ms(20);
        CAN1_SendMsg(Parameter_Mode(Para[num+gold_num].prepare,Parameter_Enable),Elmo_ID);//发送指令 执行恢复竖直位置
        delay_ms(400);
        Relay2=1;//将球固定住的气缸打开将球固定住
    }
    else if(num==2)//金球
    {

        Relay3 = 1;//Init_Movement已经到了抓球位置 所以这里先抓球
        delay_ms(100);//延时一下等到球抓紧了再进行下一步
        CAN1_SendMsg(Compensation_Mode(TIM5->CNT),Elmo_ID);//补偿编码器
        delay_ms(5);
        CAN1_SendMsg(Parameter_Mode(Para[num+gold_num].prepare,Parameter_Enable),Elmo_ID);//发送指令 执行恢复竖直位置
        delay_ms(500);//延时一下等到球抓紧了再进行下一步
        Relay1 = 1;//将气缸打到安全位置
        delay_ms(200);//延时一下再换球
        Relay2 = 1;//将球固定住的气缸打开将球固定住
        delay_ms(100);
        NextBallPosition();//换下一个球
        delay_ms(100);
    }
}

//开始发射彩球及抛射球知道停止位停止
void Launch_Movement(int num)//抛射的速度 抛射的位置 抛射停止位置
{
    delay_ms(5);//先将球稳定后再抛射
    Relay2=0;//开始抛射固定球的气缸就打回
    delay_ms(5);
    CAN1_SendMsg(Compensation_Mode(TIM5->CNT),Elmo_ID);//补偿编码器
    delay_ms(5);
    CAN1_SendMsg(Parameter_Mode(Para[num+gold_num].shoot_swing,Parameter_Enable),Elmo_ID);
    delay_ms(400);
    delay_ms(Eje[num].Roll_Wait_Time);//Eje[num].Roll_Wait_Time
    CAN1_SendMsg(Parameter_Mode(Para[num+gold_num].shoot,Parameter_Enable),Elmo_ID);
    Location1Num=Judge_PorM(Eje[num+gold_num].Weight_of_Fire_Place)*Eje[num+gold_num].Fire_Place;
    do {
        Encoder1=TIM5->CNT;
    }
    while (Encoder1 <= Location1Num);
    Relay3 = 0;//爪子释放 球被抛出
}



/**************************************************************************
函数功能：将旋转台旋转到可以接收球架的位置
入口参数：无
返回  值：无
备    注：在初始化完成后就要执行此步，每次执行一次此函数就会使电磁铁绕一圈
**************************************************************************/
void OriginalPosition(void)
{
    int temp=100000;
    while(Light1Flag==0)
    {
        CAN2_RoboModule_DRV_Velocity_Position_Mode(1,2,5000,1000,temp);
        temp+=100000;
        delay_ms(500);
    }
}

void RM35_GoBack(void)
{
    CAN2_RoboModule_DRV_Velocity_Position_Mode(1,2,5000,1000,0);
    delay_ms(500);
}
/**************************************************************************
函数功能：将球架旋转到爪子可以抓到球的位置
入口参数：无
返回  值：无
备    注：此函数必须在位置初始化函数（void OriginalPosition（））之后使用，
          每次调用自动从第一个球到最后一个球
**************************************************************************/
void NextBallPosition(void)
{
    if(BallNum<5)
        CAN2_RoboModule_DRV_Velocity_Position_Mode(1,2,5000,600,(InitPosition+BallPosition[BallNum]));
    delay_ms(10);
    BallNum++;
}

/**************************************************************************
函数功能：蓝牙手动控制
入口参数：无
返回  值：无
**************************************************************************/
void Test_Pattern(void)
{
    if(USART3_RX_BUF[0]=='P')//位置抛射PositionTZ1
    {
        BEEP=1;
        delay_ms(500);
        BEEP=0;
        if(USART3_RX_BUF[10]=='1')//第一个抛射位置
        {
            Launch_Movement(PositionTZ1);
            // CAN1_SendMsg(TZ1_Done,UnderPan_ID);
            Init_Movement(PositionTZ2);//在主函数中执行
        }
        else if(USART3_RX_BUF[10]=='2')//第二抛射位置
        {
            Launch_Movement(PositionTZ2);
            //CAN1_SendMsg(TZ2_Done,UnderPan_ID);
        }
        else if(USART3_RX_BUF[10]=='3')//第三个抛射位置
        {
            ArriveTZ3_Flag=1;
            Init_Movement(PositionTZ3);
            Prepare_Movement(2);//准备位置
            U3_printf("ArriveTZ3_Flag=%d\r\n",ArriveTZ3_Flag);
        }
        if(USART3_RX_BUF[10]=='r')//彩球到准备位置步骤
        {
            if(USART3_RX_BUF[12]=='0')
                Prepare_Movement(0);
            else Prepare_Movement(1);
        }
        U3_printf("Simulation_Can_Done\r\n");
    }
    else if(USART3_RX_BUF[0]=='A')//气缸Air_cylinder
    {
        BEEP=1;
        delay_ms(500);
        BEEP=0;
        Relay1=~Relay1;
        U3_printf("Air_cylinder_Done\r\n");
    }
    else if(USART3_RX_BUF[0]=='g')//气缸guding_ball
    {
        BEEP=1;
        delay_ms(500);
        BEEP=0;
        Relay2=~Relay2;
        U3_printf("guding_ball_Done\r\n");
    }
    else if(USART3_RX_BUF[0]=='c')//爪子clap
    {
        BEEP=1;
        delay_ms(500);
        BEEP=0;
        Relay3=~Relay3;
        U3_printf("clap_Done\r\n");
    }
    else if(USART3_RX_BUF[0]=='O')//旋转rm35-2 OriginalPosition
    {
        BEEP=1;
        delay_ms(500);
        BEEP=0;
        OriginalPosition();
        U3_printf("OriginalPosition_Done\r\n");
    }
    else if(USART3_RX_BUF[0]=='N')//旋转rm35-2 NextBallPosition
    {
        BEEP=1;
        delay_ms(500);
        BEEP=0;
        NextBallPosition();
        U3_printf("NextBallPosition_Done\r\n");
    }
    else if(USART3_RX_BUF[0]=='E')//紧急停止 所有PID控制清零 电机不动Emergency_stop
    {
        BEEP=1;
        delay_ms(500);
        BEEP=0;
        CAN1_SendMsg(Speed_Mode(0,Release,Speed_Mode_Enable),Elmo_ID);//速度归零
        delay_ms(5);
        CAN1_SendMsg(Speed_Mode(0,Release,Stop),Elmo_ID);//停止
        delay_ms(5);
        CAN1_SendMsg(Speed_Mode(0,Release,Disable),Elmo_ID);//再不使能
        Erase_Can_order();
        Speed2Flag = 0;//速度环PID调节标志
        Location2Flag = 0;   //位置环PID调节标志
        U3_printf("Emergency_stop_Done\r\n");
    }
    else if(USART3_RX_BUF[0] == 's')//set0
    {
        BEEP=1;
        delay_ms(500);
        BEEP=0;
        BallNum=0;
        U3_printf("set0_Done\r\n");
    }
    else if(USART3_RX_BUF[0] == 'C')//Capture
    {
        BEEP=1;
        delay_ms(500);
        BEEP=0;
        Init_Movement(PositionTZ3);
    }
    else if(USART3_RX_BUF[0] == 'p')//prepare
    {
        BEEP=1;
        delay_ms(500);
        BEEP=0;
        Prepare_Movement(PositionTZ3);
    }
    else if(USART3_RX_BUF[0] == 'S')//Shoot
    {
        BEEP=1;
        delay_ms(500);
        BEEP=0;
        Launch_Movement(PositionTZ3);
    }
    else if(USART3_RX_BUF[0] == 'o')//one by one
    {
        BEEP=1;
        delay_ms(500);
        BEEP=0;
        Init_Movement(PositionTZ3);
        delay_ms(200);
        Launch_Movement(PositionTZ3);
        delay_ms(200);
        Launch_Movement(PositionTZ3);
    }
    else if(USART3_RX_BUF[0] == 'G')//Gold_num
    {
        BEEP=1;
        delay_ms(500);
        BEEP=0;
        if(USART3_RX_BUF[2] == 'l')
        {
            gold_num=USART3_RX_BUF[9]-48;
            U3_printf("gold_num=%d\r\n",gold_num);
        }
        else if(USART3_RX_BUF[2] == 't')//GotoTZ3
        {
            BEEP=1;//让蜂鸣器叫一下告诉手动车操作手可以将球架爪子张开了
            delay_ms(500);
            BEEP=0;
            USART_SendData(UART5,GotoTZ3);
            NextBallPosition();
            delay_ms(500);
            Init_Movement(PositionTZ3);//回到初始位置
            Prepare_Movement(PositionTZ3);//准备位置
            RecevieFlag = 1;//此时接收到彩球架,打开电磁铁，固定住球架
            U3_printf("GotoTZ3\r\n");
        }
    }
    else if(USART3_RX_BUF[0] == '_')//_RM35_Back
    {
        BEEP=1;
        delay_ms(500);
        BEEP=0;
        RM35_GoBack();
        U3_printf("_RM35_Back\r\n");
    }
    USART3_RX_STA=0;//清除串口的标志位
}










