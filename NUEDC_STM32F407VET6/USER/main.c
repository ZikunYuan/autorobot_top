#include "main_assistant.h"

/**************************************************************************
����ʹ����ʾ��
2018.2.1.
������������2018���Զ������ܵ����Ÿĳ�2018��������������ӵ�����
**************************************************************************/

//û�õ��ı���
float Home_Flag =0;
//����Ϊû�õ��ı���

extern u8 RecevieFlag;       //�Զ������յ�����ܱ�־
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
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
    BEEP_Init();
    delay_init(168);    //��ʼ����ʱ����
    KEY_Init();
    LED_Init();
    Relay_Init();
    CAN_Config();
    CAN2_Configuration();
    Encoder1_Init_TIM5();
    Encoder2_Init_TIM2();
    LightEletric_Init();
    uart_init(115200);     //����������
    uart3_init(115200);    //�������ֽ�������
    uart5_init(115200);    //�����ͨ��
    //TIM7_Int_Init(999, 8399);//4999,8399
    RoboModule_CAN2_Init(1,2,10,Velocity_Position_Mode,0);
    EXTIX_Init();
    Relay1=1;//һ��ʼ���׿�����ȫλ�ã�tip������û����ܣ�
    Relay2=0;//һ��ʼ�̶�������ײ������
    Relay3=0;//һ��ʼצ�Ӿ����ſ���
    TIM2->CNT=0;//��תRM35���
    //ʹ��Elmo
    CAN1_SendMsg(Speed_Mode(0,0,Erase_Reset_Flag),Elmo_ID);//�����λָ��
    delay_ms(5);
    CAN1_SendMsg(Speed_Mode(0,0,Reset),Elmo_ID);//�����λָ��
    delay_ms(5);
    CAN1_SendMsg(Speed_Mode(0,0,Enable),Elmo_ID);
    delay_ms(300);
    CAN1_SendMsg(Speed_Mode(0,0,Unstop),Elmo_ID);
    delay_ms(5);
    CAN1_SendMsg(Parameter_Mode(clear_pos,Parameter_Enable),Elmo_ID);
    delay_ms(5);
    U3_printf("START\r\n");
    Init_Movement(0);//����ʼλ��ץ��
    OriginalPosition();//���ת����ʼ��λ��
//		Argument_Load();//������ȡFLASH����
    BEEP=1;//������ɱ�־
    delay_ms(500);
    BEEP=0;
    delay_ms(50);
    BEEP=1;
    delay_ms(100);
    BEEP=0;
}

int main()
{
//	  SCB->VTOR=FLASH_BASE|0X10000;//����ƫ������IAP��
    System_Init();

    while (1)
    {
        //�ֶ���������ܲ��� ֻ�н���������
        if (Light2==0&&RecevieFlag==0)//Light2��������
        {
            delay_ms(50);
            if(Light2==0&&RecevieFlag==0)//Light2��������)
            {
                Relay4 = 0;//�����
                Relay5 = 0;
                BEEP=1;//�÷�������һ�¸����ֶ��������ֿ��Խ����צ���ſ���
                delay_ms(1000);
                BEEP=0;
                USART_SendData(UART5,GotoTZ3);
                NextBallPosition();
                delay_ms(500);
                Init_Movement(PositionTZ3);//�ص���ʼλ��
							  delay_ms(100);
                Prepare_Movement(PositionTZ3);//׼��λ��
                RecevieFlag = 1;//��ʱ���յ������,�򿪵�������̶�ס���
            }
        }
        if(ArriveTZ3_Flag==1)
        {
            while(1)
            {
                Launch_Movement(PositionTZ3);//����
							  delay_ms(100);
                gold_num++;
                Init_Movement(PositionTZ3);//�ص�ץ��λ��
							  delay_ms(100);
                Prepare_Movement(PositionTZ3);//׼��λ��
							  delay_ms(400);
                if(BallNum==6)
                {
                    Launch_Movement(PositionTZ3);//����
                    while(1);
                }
            }
        }
    }
}


