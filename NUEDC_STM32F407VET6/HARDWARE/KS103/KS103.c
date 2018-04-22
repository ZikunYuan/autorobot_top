#include "KS103.h"
#include "myiic.h"
#include <stdint.h>
#include "delay.h"
#include <stdio.h>

#define SET_MODE 0x18

static uint8_t KS103_ReadOneByte(uint8_t address, uint8_t reg);
static void KS103_WriteOneByte(uint8_t address, uint8_t reg, uint8_t command);

// KS103��ʼ��
void KS103_Init(void)
{
    IIC_Init();
}

// ��ȡָ���������ľ���
uint16_t KS103_Read_Distance(uint8_t KS103_Num)
{
    KS103_WriteOneByte(KS103_Num, 0x02, SET_MODE);
    delay_ms(30);

    //��ȡ��8λ
    uint16_t value_1;
    value_1 = KS103_ReadOneByte(KS103_Num, 0x02);
    //printf("H : %d\r\n", value_1);
     
    //��ȡ��8λ
    uint16_t value_2;
    value_2 = KS103_ReadOneByte(KS103_Num, 0x03);
    //printf("L : %d\r\n", value_2);

    uint16_t value_3;
    value_3 = value_1 << 8;
    value_3 += value_2;

    value_3 *= 0.174;
    return value_3;
}

// ��ȡ�ĸ�������
void KS103_Read_All_Distance(uint16_t *KS103_temp)
{
    KS103_WriteOneByte(KS103_3_Addr, 0x02, SET_MODE);
    delay_ms(3);
    KS103_WriteOneByte(KS103_4_Addr, 0x02, SET_MODE);
    delay_ms(30);

    uint16_t value_1 = 0;
    value_1 = KS103_ReadOneByte(KS103_3_Addr, 0x02);

    uint16_t value_2 = 0;
    value_2 = KS103_ReadOneByte(KS103_3_Addr, 0x03);

    uint16_t value_3 = 0;
    value_3 = value_1 << 8;
    value_3 += value_2;

    value_3 *= 0.174;
    *(KS103_temp + 0) = value_3;

    value_1 = KS103_ReadOneByte(KS103_4_Addr, 0x02);
    value_2 = KS103_ReadOneByte(KS103_4_Addr, 0x03);
    value_3 = value_1 << 8;
    value_3 += value_2;
    value_3 *= 0.174;
    *(KS103_temp + 1) = value_3;
}

// �޸�KS103��ַ
void KS103_Change_Addr(uint8_t OldAdddr, uint8_t NewAddr)
{
    KS103_WriteOneByte(OldAdddr, 0x02, 0x9a);             //Ĭ��ԭ��ַ��0x00;
    delay_ms(3);
    KS103_WriteOneByte(OldAdddr, 0x02, 0x92);
    delay_ms(3);
    KS103_WriteOneByte(OldAdddr, 0x02, 0x9e);
    delay_ms(3);
    KS103_WriteOneByte(OldAdddr, 0x02, NewAddr);
    delay_ms(100);
}

//��KS103ĳ�Ĵ����ж�ȡ8λ����
//ReadAddr:��ʼ�����ĵ�ַ  
//����ֵ  :����������
static uint8_t KS103_ReadOneByte(uint8_t address, uint8_t reg)
{
    uint8_t temp = 0;
    IIC_Start();
    IIC_Send_Byte(address);     //���͵͵�ַ
    IIC_Wait_Ack();
    IIC_Send_Byte(reg);         //���ͼĴ������
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(address + 1); //�������ģʽ			   
    IIC_Wait_Ack();

    delay_us(57);	            //���Ӵ˴���ͨ�ųɹ�������
    temp = IIC_Read_Byte(0);	//��ȡ
    IIC_Stop();                 //����һ��ֹͣ����
    return temp;
}

//��KS103ĳ�Ĵ�����д��8λ����
//address  : д�����ݵ�Ŀ�ĵ�ַ
//reg      : �Ĵ������
//command  : ����
static void KS103_WriteOneByte(uint8_t address, uint8_t reg, uint8_t command)
{
    IIC_Start();
    IIC_Send_Byte(address);	    //���͵�ַ
    IIC_Wait_Ack();
    IIC_Send_Byte(reg);         //���ͼĴ������	  
    IIC_Wait_Ack();
    IIC_Send_Byte(command);     //��������
    IIC_Wait_Ack();
    IIC_Stop();                 //����һ��ֹͣ���� 
}
