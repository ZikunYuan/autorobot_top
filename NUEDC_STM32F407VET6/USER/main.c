#include "main_assistant.h"

/**************************************************************************
程序使用提示：
2018.2.1.
程序所有用于2018年自动车功能的引脚改成2018年李达贤所画板子的引脚
**************************************************************************/

//没用到的变量
float Home_Flag =0;
//以上为没用到的变量

extern u8 RecevieFlag;       //自动车接收到才球架标志
extern u8 GoldenShuttlecock_Flag;
extern u8 Origin_Flag;
extern u8 ArriveTZ3_Flag;
extern int BallNum;
extern u16 Golden_Start_Place;
extern int Shot_Flag;
extern u8 clear_pos;
extern u8 gold_num;

void System_Init(void)
{
    SysTick_Init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
    BEEP_Init();
    delay_init(168);    //初始化延时函数
    KEY_Init();
    LED_Init();
    Relay_Init();
    CAN_Config();
    CAN2_Configuration();
    Encoder1_Init_TIM5();
    Encoder2_Init_TIM2();
    LightEletric_Init();
    uart_init(115200);     //匿名调数据
    uart3_init(115200);    //串口助手接收数据
    uart5_init(115200);    //与底盘通信
    //TIM7_Int_Init(999, 8399);//4999,8399
    RoboModule_CAN2_Init(1,2,10,Velocity_Position_Mode,0);
    EXTIX_Init();
    Relay1=1;//一开始气缸开到安全位置（tip：彩球没有球架）
    Relay2=0;//一开始固定球的气缸不打出来
    Relay3=0;//一开始爪子就是张开的
    TIM2->CNT=0;//旋转RM35电机
    //使能Elmo
    CAN1_SendMsg(Speed_Mode(0,0,Erase_Reset_Flag),Elmo_ID);//电机复位指令
    delay_ms(5);
    CAN1_SendMsg(Speed_Mode(0,0,Reset),Elmo_ID);//电机复位指令
    delay_ms(5);
    CAN1_SendMsg(Speed_Mode(0,0,Enable),Elmo_ID);
    delay_ms(300);
    CAN1_SendMsg(Speed_Mode(0,0,Unstop),Elmo_ID);
    delay_ms(5);
    CAN1_SendMsg(Parameter_Mode(clear_pos,Parameter_Enable),Elmo_ID);
    delay_ms(5);
    U3_printf("START\r\n");
    Init_Movement(0);//到初始位置抓球
    OriginalPosition();//球架转到初始化位置
//		Argument_Load();//匿名读取FLASH参数
    BEEP=1;//启动完成标志
    delay_ms(500);
    BEEP=0;
    delay_ms(50);
    BEEP=1;
    delay_ms(100);
    BEEP=0;
}

int main()
{
//	  SCB->VTOR=FLASH_BASE|0X10000;//设置偏移量（IAP）
    System_Init();

    while (1)
    {
        //手动车传递球架部分 只有金球才有球架
        if (Light2==0&&RecevieFlag==0)//Light2是输入检测
        {
            delay_ms(50);
            if(Light2==0&&RecevieFlag==0)//Light2是输入检测)
            {
                Relay4 = 0;//电磁铁
                Relay5 = 0;
                BEEP=1;//让蜂鸣器叫一下告诉手动车操作手可以将球架爪子张开了
                delay_ms(1000);
                BEEP=0;
                USART_SendData(UART5,GotoTZ3);
                NextBallPosition();
                delay_ms(500);
                Init_Movement(PositionTZ3);//回到初始位置
							  delay_ms(100);
                Prepare_Movement(PositionTZ3);//准备位置
                RecevieFlag = 1;//此时接收到彩球架,打开电磁铁，固定住球架
            }
        }
        if(ArriveTZ3_Flag==1)
        {
            while(1)
            {
                Launch_Movement(PositionTZ3);//抛射
							  delay_ms(100);
                gold_num++;
                Init_Movement(PositionTZ3);//回到抓球位置
							  delay_ms(100);
                Prepare_Movement(PositionTZ3);//准备位置
							  delay_ms(400);
                if(BallNum==6)
                {
                    Launch_Movement(PositionTZ3);//抛射
                    while(1);
                }
            }
        }
    }
}


