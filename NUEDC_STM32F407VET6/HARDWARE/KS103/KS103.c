#include "KS103.h"
#include "myiic.h"
#include <stdint.h>
#include "delay.h"
#include <stdio.h>

#define SET_MODE 0x18

static uint8_t KS103_ReadOneByte(uint8_t address, uint8_t reg);
static void KS103_WriteOneByte(uint8_t address, uint8_t reg, uint8_t command);

// KS103初始化
void KS103_Init(void)
{
    IIC_Init();
}

// 读取指定传感器的距离
uint16_t KS103_Read_Distance(uint8_t KS103_Num)
{
    KS103_WriteOneByte(KS103_Num, 0x02, SET_MODE);
    delay_ms(30);

    //获取高8位
    uint16_t value_1;
    value_1 = KS103_ReadOneByte(KS103_Num, 0x02);
    //printf("H : %d\r\n", value_1);
     
    //获取低8位
    uint16_t value_2;
    value_2 = KS103_ReadOneByte(KS103_Num, 0x03);
    //printf("L : %d\r\n", value_2);

    uint16_t value_3;
    value_3 = value_1 << 8;
    value_3 += value_2;

    value_3 *= 0.174;
    return value_3;
}

// 读取四个传感器
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

// 修改KS103地址
void KS103_Change_Addr(uint8_t OldAdddr, uint8_t NewAddr)
{
    KS103_WriteOneByte(OldAdddr, 0x02, 0x9a);             //默认原地址是0x00;
    delay_ms(3);
    KS103_WriteOneByte(OldAdddr, 0x02, 0x92);
    delay_ms(3);
    KS103_WriteOneByte(OldAdddr, 0x02, 0x9e);
    delay_ms(3);
    KS103_WriteOneByte(OldAdddr, 0x02, NewAddr);
    delay_ms(100);
}

//从KS103某寄存器中读取8位数据
//ReadAddr:开始读数的地址  
//返回值  :读到的数据
static uint8_t KS103_ReadOneByte(uint8_t address, uint8_t reg)
{
    uint8_t temp = 0;
    IIC_Start();
    IIC_Send_Byte(address);     //发送低地址
    IIC_Wait_Ack();
    IIC_Send_Byte(reg);         //发送寄存器编号
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(address + 1); //进入接收模式			   
    IIC_Wait_Ack();

    delay_us(57);	            //增加此代码通信成功！！！
    temp = IIC_Read_Byte(0);	//读取
    IIC_Stop();                 //产生一个停止条件
    return temp;
}

//在KS103某寄存器中写入8位数据
//address  : 写入数据的目的地址
//reg      : 寄存器编号
//command  : 数据
static void KS103_WriteOneByte(uint8_t address, uint8_t reg, uint8_t command)
{
    IIC_Start();
    IIC_Send_Byte(address);	    //发送地址
    IIC_Wait_Ack();
    IIC_Send_Byte(reg);         //发送寄存器编号	  
    IIC_Wait_Ack();
    IIC_Send_Byte(command);     //发送数据
    IIC_Wait_Ack();
    IIC_Stop();                 //产生一个停止条件 
}
