/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：ak8975.c
 * 描述    ：电子罗盘驱动
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/
#include "mpu6050.h"
#include "ak8975.h"
//#include "parameter.h"
#include "mymath.h"
//#include "include.h"
#include "myiic.h"
#include "Sensor_Param.h"
#include "inv_mpu.h"
#include "dmpmap.h"
#include "math.h"
// 	xyz_f_t Mag_Offset = { -1 , -1 , -1 };
// 	xyz_f_t Mag_Gain   = { 1 , 0.8538 , 0.9389 };

ak8975_t ak8975 = { {0,0,0},{-1,-1,-1},{1,0.8538,0.9389},{0,0,0} };
u8 ak8975_buffer[6]; //接收数据缓存
//bool ANO_AK8975_Run(void)
//{
//    return IIC_Write_One_Byte(AK8975_ADDRESS,AK8975_CNTL,0x01);
//}

xyz_f_t XYZ_STRUCT_COPY(float x,float y, float z)
{
    xyz_f_t m ;
    m.x = x;
    m.y = y;
    m.z = z;
    return m;
}
u8 ak8975_ok;
float init_ax, init_ay, init_az, init_gx, init_gy, init_gz, init_mx, init_my, init_mz;
unsigned long timestamp;
signed short int mag[3];

void ANO_AK8975_Read_Mag_Data(void)
{
    IICwriteByte(AK8975_ADDRESS, AK8975_CNTL, 0x01);
    int16_t mag_temp[3];
    IICwriteByte(AK8975_ADDRESS, AK8975_CNTL, 0x01);
    i2cRead(AK8975_ADDRESS,AK8975_HXL,1,&ak8975_buffer[0]);
    i2cRead(AK8975_ADDRESS,AK8975_HXH,1,&ak8975_buffer[1]);
    mag_temp[1] = ((((int16_t)ak8975_buffer[1]) << 8) | ak8975_buffer[0]) ;  //磁力计X轴

    i2cRead(AK8975_ADDRESS,AK8975_HYL,1,&ak8975_buffer[2]);
    i2cRead(AK8975_ADDRESS,AK8975_HYH,1,&ak8975_buffer[3]);
    mag_temp[0] = ((((int16_t)ak8975_buffer[3]) << 8) | ak8975_buffer[2]) ;  //磁力计Y轴

    i2cRead(AK8975_ADDRESS,AK8975_HZL,1,&ak8975_buffer[4]);
    i2cRead(AK8975_ADDRESS,AK8975_HZH,1,&ak8975_buffer[5]);
    ak8975_buffer[4]=I2C_ReadOneByte(AK8975_ADDRESS,AK8975_HZL);
    ak8975_buffer[5]=I2C_ReadOneByte(AK8975_ADDRESS,AK8975_HZH);
    mag_temp[2] = -((((int16_t)ak8975_buffer[5]) << 8) | ak8975_buffer[4]) ;  //磁力计Z轴

    int i;
    unsigned char data_write[2];
    unsigned char data_write1[2];
	// 或者加一个for循环
//    for(i=0; i<5; i++)

//    {
        mpu_set_bypass(1);   //开启bypass，必须有这句代码
        Delay(24000);            //这俩句之间的延迟至少2400
//		data_write[0] = 0x00;
//		data_write[1] = 0x02;
//		i2cWrite(0x68, 0x6A, 1, data_write);	 //关闭MPU9150的I2C_MASTER模式，必须要有这句
//		Delay(34000);            //这俩句之间的延迟至少24000
//		i2cWrite(0x68, 0x37, 1, data_write + 1);
        mpu_get_compass_reg(mag, &timestamp);  //读取compass数据
        //进行x y轴的校准，未对z轴进行校准，参考MEMSense的校准方法
        init_mx = (float)mag[1] - 8;
        init_my = (float)1.046632*mag[0] - 1.569948;
        init_mz = (float)-mag[2];
//		data_write1[0] = 0x20;
//		data_write1[1] = 0x00;
//		i2cWrite(0x68, 0x6A, 1, data_write1); //开启MPU9150的I2C_MASTER模式，必须要有这句
//		Delay(34000);		   //这俩句之间的延迟至少24000
//		i2cWrite(0x68, 0x37, 1, data_write1 + 1);//关
//		Delay(34000);            //这俩句之间的延迟至少24000
        mpu_set_bypass(0);						//关闭bypass，必须有这句代码
        Yaw_Com = (atan2((double)init_my, (double)init_mx) * (180 / 3.14159265) + 180); // angle in degrees

        ak8975.Mag_Adc.x = init_mx;
        ak8975.Mag_Adc.y = init_my;
        ak8975.Mag_Adc.z = init_mz;
//    }
    //ak8975.Mag_Adc.x = mag_temp[0];
    //ak8975.Mag_Adc.y = mag_temp[1];
    //ak8975.Mag_Adc.z = mag_temp[2];


    ak8975.Mag_Val.x = (ak8975.Mag_Adc.x - ak8975.Mag_Offset.x) ;
    ak8975.Mag_Val.y = (ak8975.Mag_Adc.y - ak8975.Mag_Offset.y) ;
    ak8975.Mag_Val.z = (ak8975.Mag_Adc.z - ak8975.Mag_Offset.z) ;
    //磁力计中点矫正
    ANO_AK8975_CalOffset_Mag();
    //Yaw_Com = (atan2((double)ak8975.Mag_Val.y, (double)ak8975.Mag_Val.x) * (180 / 3.14159265) + 180); // angle in degrees

}

xyz_f_t ANO_AK8975_Get_Mag(void)
{
    return ak8975.Mag_Val;
}




void Param_SaveMagOffset(xyz_f_t *offset)
{
    memcpy(&ak8975.Mag_Offset, offset, sizeof(xyz_f_t));

}



u8 Mag_CALIBRATED = 0;
//磁力计中点矫正

void ANO_AK8975_CalOffset_Mag(void)
{
    static xyz_f_t	MagMAX = { -100, -100, -100 }, MagMIN = { 100, 100, 100 }, MagSum;
    static uint16_t cnt_m=0;

    if(Mag_CALIBRATED)
    {

        if(ABS(ak8975.Mag_Adc.x)<400&&ABS(ak8975.Mag_Adc.y)<400&&ABS(ak8975.Mag_Adc.z)<400)
        {
            MagMAX.x = _MAX(ak8975.Mag_Adc.x, MagMAX.x);
            MagMAX.y = _MAX(ak8975.Mag_Adc.y, MagMAX.y);
            MagMAX.z = _MAX(ak8975.Mag_Adc.z, MagMAX.z);

            MagMIN.x = _MIN(ak8975.Mag_Adc.x, MagMIN.x);
            MagMIN.y = _MIN(ak8975.Mag_Adc.y, MagMIN.y);
            MagMIN.z = _MIN(ak8975.Mag_Adc.z, MagMIN.z);

            if(cnt_m == CALIBRATING_MAG_CYCLES)
            {
                ak8975.Mag_Offset.x = (int16_t)((MagMAX.x + MagMIN.x) * 0.5f);
                ak8975.Mag_Offset.y = (int16_t)((MagMAX.y + MagMIN.y) * 0.5f);
                ak8975.Mag_Offset.z = (int16_t)((MagMAX.z + MagMIN.z) * 0.5f);

                MagSum.x = MagMAX.x - MagMIN.x;
                MagSum.y = MagMAX.y - MagMIN.y;
                MagSum.z = MagMAX.z - MagMIN.z;

                ak8975.Mag_Gain.y = MagSum.x / MagSum.y;
                ak8975.Mag_Gain.z = MagSum.x / MagSum.z;

                Param_SaveMagOffset(&ak8975.Mag_Offset);//param_Save();//保存数据
                cnt_m = 0;
                Mag_CALIBRATED = 0;
                //f.msg_id = 3;
                //f.msg_data = 1;
            }
        }
        cnt_m++;

    }
    else
    {

    }
}

void ANO_AK8975_Read(void)
{
    //读取磁力计
    ANO_AK8975_Read_Mag_Data();
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

