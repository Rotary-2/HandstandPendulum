#ifndef __MOTOR_H
#define __MOTOR_H

#include "sys.h"
#include "stdlib.h"
void Motor_PWM_Init(u16 arr,u16 psc);

//�����������
#define AIN1 PCout(4)
#define AIN2 PCout(5)


//���PWM����
#define PWMA TIM3->CCR2
#define PWMB TIM3->CCR1

#endif
