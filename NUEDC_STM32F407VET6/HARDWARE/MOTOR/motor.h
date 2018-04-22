#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 

#define PWMA   TIM4->CCR2  //PD13
#define PWMB   TIM3->CCR4  //PB1

#define AIN1   PDout(14)
#define AIN2   PDout(15)

#define BIN1   PAout(7)
#define BIN2   PBout(0)

#define Motor1 1
#define Motor2 2
#define TurnRight 1
#define TurnLeft 0

void Motor_Init(void);

void Motor1_Init(void);
void Motor2_Init(void);

void Motor1_GPIO_Init(void);
void Motor1_PWM_Init(u16 arr, u16 psc);

void Motor2_GPIO_Init(void);
void Motor2_PWM_Init(u16 arr, u16 psc);

void Motor_Control(u8 MotorNum, u8 TurnDir, u16 Speed);
#endif
