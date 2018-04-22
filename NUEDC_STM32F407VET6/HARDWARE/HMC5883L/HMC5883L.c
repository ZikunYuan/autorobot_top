#include"HMC5883L.h"
#include "mpu6050.h"
#include <math.h>
#include "Sensor_Param.h"
#include "MPU6050.h"
extern float Pitch, Roll, Yaw_Mpu, Yaw_Com;
void Init_HMC5883(void)
{
	Write_HMC5883(0x00, 0x14);		//配置寄存器A：采样平均数1 输出速率75Hz 正常测量
	Write_HMC5883(0x01, 0x20);		 //配置寄存器B：增益控制
	Write_HMC5883(0x02, 0x00);		 //模式寄存器：连续测量模式
	

	//IIC_Start();                  //起始信号
	//IIC_Send_Byte(SlaveAddress);
	//IIC_Send_Byte(0x00);   //指针指向00，配置寄存器A 
	//IIC_Send_Byte(0x78);   //发送设备地址+写信号
	//IIC_Start();
	//IIC_Send_Byte(SlaveAddress);
	//IIC_Send_Byte(0x02);
	//IIC_Send_Byte(0x00);
	//IIC_Stop();

	//Write_HMC5883(0x02, 0x00);		//连续测量模式
}

void Write_HMC5883(u8 add, u8 da)
{
    IIC_Start();                  //起始信号
    IIC_Send_Byte(SlaveAddress);   //发送设备地址+写信号
	IIC_Wait_Ack();

    IIC_Send_Byte(add);    //内部寄存器地址，请参考中文pdf 
	IIC_Wait_Ack();

    IIC_Send_Byte(da);       //内部寄存器数据，请参考中文pdf
	IIC_Wait_Ack();

    IIC_Stop();                   //发送停止信号
}

u8 Read_HMC5883(u8 REG_Address)
{   
	u8 REG_data;
    IIC_Start();                          //起始信号
    IIC_Send_Byte(SlaveAddress);           //发送设备地址+写信号
	IIC_Wait_Ack();

    IIC_Send_Byte(REG_Address);                   //发送存储单元地址，从0开始	
	IIC_Wait_Ack();

    IIC_Start();                          //起始信号
    IIC_Send_Byte(SlaveAddress+1);         //发送设备地址+读信号
	IIC_Wait_Ack();

    REG_data=IIC_Read_Byte(0);              //读出寄存器数据
	IIC_Stop();                           //停止信号
    return REG_data; 
}

//******************************************************
//
//连续读出HMC5883内部角度数据，地址范围0x3~0x5
//
//******************************************************
void Multiple_read_HMC5883(void)
{   u8 i;
    u8 BUF[6];
    int x, y, z;
	double angle_Yaw;
    IIC_Start();                          //起始信号
    IIC_Send_Byte(SlaveAddress);           //发送设备地址+写信号
	IIC_Wait_Ack();
    IIC_Send_Byte(0x03);                   //发送存储单元地址，从0x3开始	
	IIC_Wait_Ack();
    IIC_Start();                          //起始信号
    IIC_Send_Byte(SlaveAddress+1);         //发送设备地址+读信号
	IIC_Wait_Ack();
	 for (i=0; i<6; i++)                      //连续读取6个地址数据，存储中BUF
    {
        
        if (i == 5)
        {
           BUF[i] = IIC_Read_Byte(0);          //最后一个数据需要回NOACK
        }
        else
        {
          BUF[i] = IIC_Read_Byte(1);          //返回ACK
       }
   }
    IIC_Stop();                          //停止信号
	x = BUF[0] << 8 | BUF[1]; //Combine MSB and LSB of X Data output register
	z = BUF[2] << 8 | BUF[3]; //Combine MSB and LSB of Z Data output register
	y = BUF[4] << 8 | BUF[5]; //Combine MSB and LSB of Y Data output register
	if (x>32768)
		x = -(0xFFFF - x + 1);
	if (z>32768)
		z = -(0xFFFF - z + 1);
	if (y>32768)
		y = -(0xFFFF - y + 1);
	angle_Yaw = (atan2((double)y, (double)x) * (180 / 3.14159265) + 180); // angle in degrees
	Yaw_Com = angle_Yaw;
}
//
//
//u8 Mag_CALIBRATED = 0;
////磁力计中点矫正
//
//void ANO_AK8975_CalOffset_Mag(void)
//{
//	static xyz_f_t	MagMAX = { -100 , -100 , -100 }, MagMIN = { 100 , 100 , 100 }, MagSum;
//	static uint16_t cnt_m = 0;
//
//	if (Mag_CALIBRATED)
//	{
//
//		if (ABS(ak8975.Mag_Adc.x)<400 && ABS(ak8975.Mag_Adc.y)<400 && ABS(ak8975.Mag_Adc.z)<400)
//		{
//			MagMAX.x = _MAX(ak8975.Mag_Adc.x, MagMAX.x);
//			MagMAX.y = _MAX(ak8975.Mag_Adc.y, MagMAX.y);
//			MagMAX.z = _MAX(ak8975.Mag_Adc.z, MagMAX.z);
//
//			MagMIN.x = _MIN(ak8975.Mag_Adc.x, MagMIN.x);
//			MagMIN.y = _MIN(ak8975.Mag_Adc.y, MagMIN.y);
//			MagMIN.z = _MIN(ak8975.Mag_Adc.z, MagMIN.z);
//
//			if (cnt_m == CALIBRATING_MAG_CYCLES)
//			{
//				ak8975.Mag_Offset.x = (int16_t)((MagMAX.x + MagMIN.x) * 0.5f);
//				ak8975.Mag_Offset.y = (int16_t)((MagMAX.y + MagMIN.y) * 0.5f);
//				ak8975.Mag_Offset.z = (int16_t)((MagMAX.z + MagMIN.z) * 0.5f);
//
//				MagSum.x = MagMAX.x - MagMIN.x;
//				MagSum.y = MagMAX.y - MagMIN.y;
//				MagSum.z = MagMAX.z - MagMIN.z;
//
//				ak8975.Mag_Gain.y = MagSum.x / MagSum.y;
//				ak8975.Mag_Gain.z = MagSum.x / MagSum.z;
//
//				Param_SaveMagOffset(&ak8975.Mag_Offset);//param_Save();//保存数据
//				cnt_m = 0;
//				Mag_CALIBRATED = 0;
//	/*			f.msg_id = 3;
//				f.msg_data = 1;*/
//			}
//		}
//		cnt_m++;
//	}
//	else
//	{
//
//	}
//}
//
//
//u8 ak8975_ok;
//void ANO_AK8975_Read_Mag_Data(void)
//{
//	int16_t mag_temp[3];
//	u8 ak8975_buffer[6]; //接收数据缓存
//
//
//
//	//IIC_Read_1Byte(AK8975_ADDRESS, AK8975_HXL, &ak8975_buffer[0]);
//	//IIC_Read_1Byte(AK8975_ADDRESS, AK8975_HXH, &ak8975_buffer[1]);
//	//mag_temp[1] = ((((int16_t)ak8975_buffer[1]) << 8) | ak8975_buffer[0]);  //磁力计X轴
//
//	//IIC_Read_1Byte(AK8975_ADDRESS, AK8975_HYL, &ak8975_buffer[2]);
//	//IIC_Read_1Byte(AK8975_ADDRESS, AK8975_HYH, &ak8975_buffer[3]);
//	//mag_temp[0] = ((((int16_t)ak8975_buffer[3]) << 8) | ak8975_buffer[2]);  //磁力计Y轴
//
//	//IIC_Read_1Byte(AK8975_ADDRESS, AK8975_HZL, &ak8975_buffer[4]);
//	//IIC_Read_1Byte(AK8975_ADDRESS, AK8975_HZH, &ak8975_buffer[5]);
//	//mag_temp[2] = -((((int16_t)ak8975_buffer[5]) << 8) | ak8975_buffer[4]);  //磁力计Z轴	
//
//	ak8975.Mag_Adc.x = mag_temp[0];
//	ak8975.Mag_Adc.y = mag_temp[1];
//	ak8975.Mag_Adc.z = mag_temp[2];
//
//	ak8975.Mag_Val.x = (ak8975.Mag_Adc.x - ak8975.Mag_Offset.x);
//	ak8975.Mag_Val.y = (ak8975.Mag_Adc.y - ak8975.Mag_Offset.y);
//	ak8975.Mag_Val.z = (ak8975.Mag_Adc.z - ak8975.Mag_Offset.z);
//	//磁力计中点矫正	
//	ANO_AK8975_CalOffset_Mag();
//
//	//AK8975采样触发
//	ANO_AK8975_Run();
//}
//
//xyz_f_t ANO_AK8975_Get_Mag(void)
//{
//	return ak8975.Mag_Val;
//}