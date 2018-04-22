#include "main_assistant.h"

/**************************************************************************
��������: Ec_spos
��ڲ�������
����  ֵ����
**************************************************************************/
extern u8 gold_num;
u32 pwm_num=0;
int Location1Num, Location2Num, Speed1Num;//����PIDֹͣλ���Լ������ٶ�
u8 RecevieFlag = 0;//�Զ������յ�����ܱ�־
u8 NextBall = 0;//������л����־
u8 Speed2Flag = 0;//�ٶȻ�PID���ڱ�־
u8 Location2Flag = 0;   //λ�û�PID���ڱ�־
u8 Light1_Flag = 0;
int Sign_Flag=1;             //�������������ű�־λ
//u8 ArriveTZ1_Flag=0;
//u8 ArriveTZ2_Flag=0;
u8 ArriveTZ3_Flag=0;
//u8 NormalShuttlecock_Flag=0;
//u8 NormalShuttlecock_1_Flag=0;
u8 GoldenShuttlecock_Flag=0;

//����λ�õĲ���
u16 Release=0;//������ٶ�Ϊ��
//����λ������ �����λ�ã����Ǳ�����λ�ã�˦���ֹͣʱ��
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
//����Ϊ����λ�õĲ���

u32 BallPosition[]= {0x1fd3,0x456c,0x6c0d,0x8e27,0xb2b8}; //ÿ��ֵ��Ӧ����ϵ�ÿ�����λ��
int Shot_Flag = 0;
int BallNum=0;//������ʾ�ڼ�����,0��ʾ��һ����
extern int Moto1,Moto2;
extern int Now1_Encoder, Last1_Encoder;//�����ٶȻ�ʱ��¼ǰ�����������

extern void System_Init(void);

/**************************************************************************
�������ܣ������һϵ�ж����ĺ���
��ڲ�������
����  ֵ����
**************************************************************************/
//����ʼ������Ҳ����ץ���ǰһ������
void Init_Movement(int num)//ץ��λ��
{
    if(num==2) {
        Relay1=0;    //���������佫�����˻�
        delay_ms(700);
    }
    delay_ms(5);//û��ʵ���ô� ������ɾ
    CAN1_SendMsg(Compensation_Mode(TIM5->CNT),Elmo_ID);//����������
    delay_ms(5);
    CAN1_SendMsg(Parameter_Mode(Para[num+gold_num].capture,Parameter_Enable),Elmo_ID);
    delay_ms(300);
}

//���򣺵���ֱλ��׼������ ����ץ���ҽ�����ֱλ��׼������
void Prepare_Movement(int num)//׼������λ��
{
    if(num<2)//����
    {
        Relay3 = 1;//Init_Movement�Ѿ�����ץ��λ�� ֱ��ץ��Ϳ��Ե�׼��λ����
        delay_ms(50);//�ȴ���ס�����˳�
        USART_SendData(UART5,Capture_Color_And_Prepare_Done);
        CAN1_SendMsg(Compensation_Mode(TIM5->CNT),Elmo_ID);//����������
        delay_ms(20);
        CAN1_SendMsg(Parameter_Mode(Para[num+gold_num].prepare,Parameter_Enable),Elmo_ID);//����ָ�� ִ�лָ���ֱλ��
        delay_ms(400);
        Relay2=1;//����̶�ס�����״򿪽���̶�ס
    }
    else if(num==2)//����
    {

        Relay3 = 1;//Init_Movement�Ѿ�����ץ��λ�� ����������ץ��
        delay_ms(100);//��ʱһ�µȵ���ץ�����ٽ�����һ��
        CAN1_SendMsg(Compensation_Mode(TIM5->CNT),Elmo_ID);//����������
        delay_ms(5);
        CAN1_SendMsg(Parameter_Mode(Para[num+gold_num].prepare,Parameter_Enable),Elmo_ID);//����ָ�� ִ�лָ���ֱλ��
        delay_ms(500);//��ʱһ�µȵ���ץ�����ٽ�����һ��
        Relay1 = 1;//�����״򵽰�ȫλ��
        delay_ms(200);//��ʱһ���ٻ���
        Relay2 = 1;//����̶�ס�����״򿪽���̶�ס
        delay_ms(100);
        NextBallPosition();//����һ����
        delay_ms(100);
    }
}

//��ʼ�������������֪��ֹͣλֹͣ
void Launch_Movement(int num)//������ٶ� �����λ�� ����ֹͣλ��
{
    delay_ms(5);//�Ƚ����ȶ���������
    Relay2=0;//��ʼ����̶�������׾ʹ��
    delay_ms(5);
    CAN1_SendMsg(Compensation_Mode(TIM5->CNT),Elmo_ID);//����������
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
    Relay3 = 0;//צ���ͷ� ���׳�
}



/**************************************************************************
�������ܣ�����ת̨��ת�����Խ�����ܵ�λ��
��ڲ�������
����  ֵ����
��    ע���ڳ�ʼ����ɺ��Ҫִ�д˲���ÿ��ִ��һ�δ˺����ͻ�ʹ�������һȦ
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
�������ܣ��������ת��צ�ӿ���ץ�����λ��
��ڲ�������
����  ֵ����
��    ע���˺���������λ�ó�ʼ��������void OriginalPosition������֮��ʹ�ã�
          ÿ�ε����Զ��ӵ�һ�������һ����
**************************************************************************/
void NextBallPosition(void)
{
    if(BallNum<5)
        CAN2_RoboModule_DRV_Velocity_Position_Mode(1,2,5000,600,(InitPosition+BallPosition[BallNum]));
    delay_ms(10);
    BallNum++;
}

/**************************************************************************
�������ܣ������ֶ�����
��ڲ�������
����  ֵ����
**************************************************************************/
void Test_Pattern(void)
{
    if(USART3_RX_BUF[0]=='P')//λ������PositionTZ1
    {
        BEEP=1;
        delay_ms(500);
        BEEP=0;
        if(USART3_RX_BUF[10]=='1')//��һ������λ��
        {
            Launch_Movement(PositionTZ1);
            // CAN1_SendMsg(TZ1_Done,UnderPan_ID);
            Init_Movement(PositionTZ2);//����������ִ��
        }
        else if(USART3_RX_BUF[10]=='2')//�ڶ�����λ��
        {
            Launch_Movement(PositionTZ2);
            //CAN1_SendMsg(TZ2_Done,UnderPan_ID);
        }
        else if(USART3_RX_BUF[10]=='3')//����������λ��
        {
            ArriveTZ3_Flag=1;
            Init_Movement(PositionTZ3);
            Prepare_Movement(2);//׼��λ��
            U3_printf("ArriveTZ3_Flag=%d\r\n",ArriveTZ3_Flag);
        }
        if(USART3_RX_BUF[10]=='r')//����׼��λ�ò���
        {
            if(USART3_RX_BUF[12]=='0')
                Prepare_Movement(0);
            else Prepare_Movement(1);
        }
        U3_printf("Simulation_Can_Done\r\n");
    }
    else if(USART3_RX_BUF[0]=='A')//����Air_cylinder
    {
        BEEP=1;
        delay_ms(500);
        BEEP=0;
        Relay1=~Relay1;
        U3_printf("Air_cylinder_Done\r\n");
    }
    else if(USART3_RX_BUF[0]=='g')//����guding_ball
    {
        BEEP=1;
        delay_ms(500);
        BEEP=0;
        Relay2=~Relay2;
        U3_printf("guding_ball_Done\r\n");
    }
    else if(USART3_RX_BUF[0]=='c')//צ��clap
    {
        BEEP=1;
        delay_ms(500);
        BEEP=0;
        Relay3=~Relay3;
        U3_printf("clap_Done\r\n");
    }
    else if(USART3_RX_BUF[0]=='O')//��תrm35-2 OriginalPosition
    {
        BEEP=1;
        delay_ms(500);
        BEEP=0;
        OriginalPosition();
        U3_printf("OriginalPosition_Done\r\n");
    }
    else if(USART3_RX_BUF[0]=='N')//��תrm35-2 NextBallPosition
    {
        BEEP=1;
        delay_ms(500);
        BEEP=0;
        NextBallPosition();
        U3_printf("NextBallPosition_Done\r\n");
    }
    else if(USART3_RX_BUF[0]=='E')//����ֹͣ ����PID�������� �������Emergency_stop
    {
        BEEP=1;
        delay_ms(500);
        BEEP=0;
        CAN1_SendMsg(Speed_Mode(0,Release,Speed_Mode_Enable),Elmo_ID);//�ٶȹ���
        delay_ms(5);
        CAN1_SendMsg(Speed_Mode(0,Release,Stop),Elmo_ID);//ֹͣ
        delay_ms(5);
        CAN1_SendMsg(Speed_Mode(0,Release,Disable),Elmo_ID);//�ٲ�ʹ��
        Erase_Can_order();
        Speed2Flag = 0;//�ٶȻ�PID���ڱ�־
        Location2Flag = 0;   //λ�û�PID���ڱ�־
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
            BEEP=1;//�÷�������һ�¸����ֶ��������ֿ��Խ����צ���ſ���
            delay_ms(500);
            BEEP=0;
            USART_SendData(UART5,GotoTZ3);
            NextBallPosition();
            delay_ms(500);
            Init_Movement(PositionTZ3);//�ص���ʼλ��
            Prepare_Movement(PositionTZ3);//׼��λ��
            RecevieFlag = 1;//��ʱ���յ������,�򿪵�������̶�ס���
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
    USART3_RX_STA=0;//������ڵı�־λ
}










