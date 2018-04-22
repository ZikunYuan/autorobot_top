/*
 * ANO_DT_User_Settings.h
 *
 *  Created on: 2016.11.25
 *      Author: DGVY
 */
#ifndef __ANO_DT_USER_SETTINGS_H_
#define __ANO_DT_USER_SETTINGS_H_
#include "main_assistant.h"
extern Eje_value Eje[3];
/************************************************************************/
// 模块内部头文件，外部不要引用
/************************************************************************/


/**************用于修改地面站的各种设定，由用户指定********************/

// -----用到的UART口-----
// USART1 -- 1
// USART2 -- 2
// USART3 -- 3
// USART4 -- 4
#define ANO_DT_UART_PORT_SELECT 1

// -----指定各按钮事件-----
//加速度计校准按钮
#define ANO_DT_ACCELEROMETER_ADJUSTING()	printf("accelerometer_adj\r\n");

//陀螺仪校准按钮
#define ANO_DT_GYRO_CALIBRATED()			printf("gyro_adj\r\n");

//罗盘校准按钮
#define ANO_DT_COMPASS_CALIBRATED()			printf("compass_adj\r\n");

//气压计校准按钮
#define ANO_DT_BAROMETER_CALIBRATED()		printf("barometer_adj\r\n");


// -----全部数据写入单片机成功指示-----
#define ANO_DT_DATA_RECEIVE_WRITE_SUCCEED() \
        printf("\r\nPID_Data_Upate is successful!!\r\n");


// 参数放大比例，单片机内的参数将乘以此数，再存入FLASH或发送到上位机
#define ANO_DT_PARAMETER_SCALING   1

// 单片机内部参数类型
#define ANO_DT_PARAMETER_TYPE (double)

// 单片机内部数据存储地址，默认为FLASH的第八扇区
#define ANO_DT_PID_DATA_SAVE_FLASH_ADDRESS ADDR_FLASH_SECTOR_5


// -----要调节的数据，以下数据与匿名地面站"飞控参数"一栏一一对应----

#define USE_PID_01 1
#define USE_PID_02 1
#define USE_PID_03 1
//#define USE_PID_04 0
//#define USE_PID_05 0
//#define USE_PID_06 0
//#define USE_PID_07 0
//#define USE_PID_08 0
//#define USE_PID_09 0
//#define USE_PID_10 0
//#define USE_PID_11 0
//#define USE_PID_12 0
//#define USE_PID_13 0
//#define USE_PID_14 0
//#define USE_PID_15 0
//#define USE_PID_16 0
//#define USE_PID_17 0
//#define USE_PID_18 0



#if defined(USE_PID_01)
#define PID_01_P    Eje[0].Weight_of_Fire_Place
#define PID_01_I    Eje[0].Fire_Place
#define PID_01_D    Eje[0].Roll_Wait_Time
#endif

#if defined(USE_PID_02)
#define PID_02_P    Eje[1].Weight_of_Fire_Place
#define PID_02_I    Eje[1].Fire_Place
#define PID_02_D    Eje[1].Roll_Wait_Time
#endif

#if defined(USE_PID_03)
#define PID_03_P    Eje[2].Weight_of_Fire_Place
#define PID_03_I    Eje[2].Fire_Place
#define PID_03_D    Eje[2].Roll_Wait_Time
#endif

//#if defined(USE_PID_04)
//#define PID_04_P    Eje[0].Roll_Wait_Time
//#define PID_04_I    Eje[1].Capture_Ball_Location
//#define PID_04_D    Eje[1].Prepare_Shoot_Location
//#endif

//#if defined(USE_PID_05)
//#define PID_05_P    Eje[1].Weight_of_Shoot_S
//#define PID_05_I    Eje[1].Shoot_Speed
//#define PID_05_D    Eje[1].Weight_of_Fire_Place
//#endif

//#if defined(USE_PID_06)
//#define PID_06_P    Eje[1].Fire_Place
//#define PID_06_I    Eje[1].Stop_Place
//#define PID_06_D    Eje[1].Weight_of_Roll_Speed
//#endif

//#if defined(USE_PID_07)
//#define PID_07_P    Eje[1].Roll_Speed
//#define PID_07_I    Eje[1].Roll_Wait_Time
//#define PID_07_D    Eje[2].Capture_Ball_Location
//#endif

//#if defined(USE_PID_08)
//#define PID_08_P    Eje[2].Prepare_Shoot_Location
//#define PID_08_I    Eje[2].Weight_of_Shoot_S
//#define PID_08_D    Eje[2].Shoot_Speed 
//#endif

//#if defined(USE_PID_09)
//#define PID_09_P    Eje[2].Weight_of_Fire_Place
//#define PID_09_I    Eje[2].Fire_Place
//#define PID_09_D    Eje[2].Stop_Place
//#endif

//#if defined(USE_PID_10)
//#define PID_10_P    Eje[2].Weight_of_Roll_Speed
//#define PID_10_I    Eje[2].Roll_Speed
//#define PID_10_D    Eje[2].Roll_Wait_Time
//#endif

//#if defined(USE_PID_11)
//#define PID_11_P    
//#define PID_11_I    
//#define PID_11_D    
//#endif

//#if defined(USE_PID_12)
//#define PID_12_P    
//#define PID_12_I    
//#define PID_12_D    
//#endif

//#if defined(USE_PID_13)
//#define PID_13_P    
//#define PID_13_I    
//#define PID_13_D    
//#endif

//#if defined(USE_PID_14)
//#define PID_14_P    
//#define PID_14_I    
//#define PID_14_D    
//#endif

//#if defined(USE_PID_15)
//#define PID_15_P    
//#define PID_15_I    
//#define PID_15_D    
//#endif

//#if defined(USE_PID_16)
//#define PID_16_P   
//#define PID_16_I    
//#define PID_16_D    
//#endif

//#if defined(USE_PID_17)
//#define PID_17_P    kp_b
//#define PID_17_I    ki_b
//#define PID_17_D    kd_b
//#endif

//#if defined(USE_PID_18)
//#define PID_18_P    kp_b
//#define PID_18_I    ki_b
//#define PID_18_D    kd_b
//#endif

#endif /* ANO_DT_ANO_DT_USER_SETTINGS_H_ */
