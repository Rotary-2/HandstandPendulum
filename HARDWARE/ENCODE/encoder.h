#ifndef __ENCODER_H
#define __ENCODER_H

#include "sys.h"
#include "stdlib.h"
#define ENCODER_TIM_PERIOD (u16)(65535) //��ʱ��2 CNT�Ĵ�����16λ��
void Encoder_Init_TIM2(void);
int Read_Encoder(u8 TIMX);

#endif
