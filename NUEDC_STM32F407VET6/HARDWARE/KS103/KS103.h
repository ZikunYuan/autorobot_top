#ifndef __KS103_H
#define __KS103_H

#include <stdint.h>


#define KS103_1_Addr    0xE8 
#define KS103_2_Addr    0xD0
#define KS103_3_Addr    0xD2
#define KS103_4_Addr    0xD4

// KS103��ʼ��
void KS103_Init(void);

// ��ȡָ���������ľ���
uint16_t KS103_Read_Distance(uint8_t KS103_Num);

// �޸�KS103��ַ
void KS103_Change_Addr(uint8_t OldAdddr, uint8_t NewAddr);

// ��ȡ�ĸ�������
void KS103_Read_All_Distance(uint16_t *KS103_temp);

#endif
