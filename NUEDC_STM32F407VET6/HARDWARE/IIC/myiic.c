#include "myiic.h"
#include "delay.h"
#include "SysTick.h"
#include "stm32f4xx_gpio.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//IIC ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/6
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//��ʼ��IIC
void IIC_Init(void)
{			
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��

  //GPIOB8,B9��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
	IIC_SCL=1;
	IIC_SDA=1;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Start(void)
*��������:		����IIC��ʼ�ź�
*******************************************************************************/
int IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA=1;
	if(!READ_SDA)return 0;	
	IIC_SCL=1;
	delay_us(1);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	if(READ_SDA)return 0;
	delay_us(1);
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
	return 1;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Stop(void)
*��������:	    //����IICֹͣ�ź�
*******************************************************************************/	  
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(1);
	IIC_SCL=1; 
	IIC_SDA=1;//����I2C���߽����ź�
	delay_us(1);							   	
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IIC_Wait_Ack(void)
*��������:	    �ȴ�Ӧ���źŵ��� 
//����ֵ��0������Ӧ��ʧ��
//        1������Ӧ��ɹ�
*******************************************************************************/
int IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA=1;
	delay_us(1);	   
	IIC_SCL=1;
	delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop();
			return 0;
		}
	  delay_us(1);
	}
	IIC_SCL=0;//ʱ�����0 	   
	return 1;  
} 

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Ack(void)
*��������:	    ����ACKӦ��
*******************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}
	
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_NAck(void)
*��������:	    ����NACKӦ��
*******************************************************************************/	    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Send_Byte(u8 txd)
*��������:	    IIC����һ���ֽ�
*******************************************************************************/		  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	  SDA_OUT();	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(1);   
		IIC_SCL=1;
		delay_us(1); 
		IIC_SCL=0;	
		delay_us(1);
    }	 
} 	 
  
/**************************ʵ�ֺ���********************************************
*����ԭ��:		bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
*��������:		
*******************************************************************************/
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
		int i;
    if (!IIC_Start())
        return 1;
    IIC_Send_Byte(addr << 1 );
    if (!IIC_Wait_Ack()) {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
		for (i = 0; i < len; i++) {
        IIC_Send_Byte(data[i]);
        if (!IIC_Wait_Ack()) {
            IIC_Stop();
            return 0;
        }
    }
    IIC_Stop();
    return 0;
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
*��������:		
*******************************************************************************/
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!IIC_Start())
        return 1;
    IIC_Send_Byte(addr << 1);
    if (!IIC_Wait_Ack()) {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte((addr << 1)+1);
    IIC_Wait_Ack();
    while (len) {
        if (len == 1)
            *buf = IIC_Read_Byte(0);
        else
            *buf = IIC_Read_Byte(1);
        buf++;
        len--;
    }
    IIC_Stop();
    return 0;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IIC_Read_Byte(unsigned char ack)
*��������:	    //��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK 
*******************************************************************************/  
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(2); 
    }					 
    if (ack)
        IIC_Ack(); //����ACK 
    else
        IIC_NAck();//����nACK  
    return receive;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*��������:	    ��ȡָ���豸 ָ���Ĵ�����һ��ֵ
����	I2C_Addr  Ŀ���豸��ַ
		addr	   �Ĵ�����ַ
����   ��������ֵ
*******************************************************************************/ 
u8 I2C_ReadOneByte(u8 I2C_Addr,u8 addr)
{
	u8 res=0;
	
	IIC_Start();	
	IIC_Send_Byte(I2C_Addr);	   //����д����
	res++;
	IIC_Wait_Ack();
	IIC_Send_Byte(addr); res++;  //���͵�ַ
	IIC_Wait_Ack();	  
	//IIC_Stop();//����һ��ֹͣ����	
	IIC_Start();
	IIC_Send_Byte(I2C_Addr+1); res++;          //�������ģʽ			   
	IIC_Wait_Ack();
	res=IIC_Read_Byte(0);	   
    IIC_Stop();//����һ��ֹͣ����

	return res;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ����� length��ֵ
����	dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫ�����ֽ���
		*data  ���������ݽ�Ҫ��ŵ�ָ��
����   ���������ֽ�����
*******************************************************************************/ 
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data){
    u8 count = 0;
	
	IIC_Start();
	IIC_Send_Byte(dev);	   //����д����
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //���͵�ַ
    IIC_Wait_Ack();	  
	IIC_Start();
	IIC_Send_Byte(dev+1);  //�������ģʽ	
	IIC_Wait_Ack();
	
    for(count=0;count<length;count++){
		 
		 if(count!=length-1)data[count]=IIC_Read_Byte(1);  //��ACK�Ķ�����
		 	else  data[count]=IIC_Read_Byte(0);	 //���һ���ֽ�NACK
	}
    IIC_Stop();//����һ��ֹͣ����
    return count;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*��������:	    ������ֽ�д��ָ���豸 ָ���Ĵ���
����	dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫд���ֽ���
		*data  ��Ҫд�����ݵ��׵�ַ
����   �����Ƿ�ɹ�
*******************************************************************************/ 
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data){
  
 	u8 count = 0;
	IIC_Start();
	IIC_Send_Byte(dev);	   //����д����
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //���͵�ַ
    IIC_Wait_Ack();	  
	for(count=0;count<length;count++){
		IIC_Send_Byte(data[count]); 
		IIC_Wait_Ack(); 
	 }
	IIC_Stop();//����һ��ֹͣ����

    return 1; //status == 0;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICreadByte(u8 dev, u8 reg, u8 *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ�����һ��ֵ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		*data  ���������ݽ�Ҫ��ŵĵ�ַ
����   1
*******************************************************************************/ 
u8 IICreadByte(u8 dev, u8 reg, u8 *data){
	*data=I2C_ReadOneByte(dev, reg);
    return 1;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
*��������:	    д��ָ���豸 ָ���Ĵ���һ���ֽ�
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		data  ��Ҫд����ֽ�
����   1
*******************************************************************************/ 
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data){
    return i2cWrite(dev, reg, 1, &data);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �еĶ��λ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		bitStart  Ŀ���ֽڵ���ʼλ
		length   λ����
		data    ��Ÿı�Ŀ���ֽ�λ��ֵ
����   �ɹ� Ϊ1 
 		ʧ��Ϊ0
*******************************************************************************/ 
u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{

    u8 b;
    if (IICreadByte(dev, reg, &b) != 0) {
        u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    } else {
        return 0;
    }
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �е�1��λ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		bitNum  Ҫ�޸�Ŀ���ֽڵ�bitNumλ
		data  Ϊ0 ʱ��Ŀ��λ������0 ���򽫱���λ
����   �ɹ� Ϊ1 
 		ʧ��Ϊ0
*******************************************************************************/ 
u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data){
    u8 b;
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}



// IIC��n�ֽ�����
u8 IIC_Read_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
{
	IIC_Start();
	IIC_Send_Byte(SlaveAddress << 1);
	if (IIC_Wait_Ack())
	{
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(REG_Address);
	IIC_Wait_Ack();

	IIC_Start();
	IIC_Send_Byte(SlaveAddress << 1 | 0x01);
	IIC_Wait_Ack();
	while (len)
	{
		if (len == 1)
		{
			*buf = IIC_Read_Byte(0);
		}
		else
		{
			*buf = IIC_Read_Byte(1);
		}
		buf++;
		len--;
	}
	IIC_Stop();
	return 0;
}



//float Single_Write (unsigned char SlaveAddress, unsigned char REG_Address, unsigned char REG_data)
//{
//    if (!IIC_Start())
//        return false;
//    I2C_SendByte(SlaveAddress<< 1 | I2C_Direction_Transmitter); //
//    if (!IIC_WaitAck()) {
//        I2C_Stop();
//        return false;
//    }
//    I2C_SendByte(REG_Address);
//    I2C_WaitAck();
//    I2C_SendByte(REG_data);
//    I2C_WaitAck();
//    I2C_Stop();
//    return true;
//}

//------------------End of File----------------------------



















