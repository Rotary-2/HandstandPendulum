#ifndef __MOTOR_H
#define __MOTOR_H

#include "sys.h"
#include "stdlib.h"
void Motor_PWM_Init(u16 arr,u16 psc);

//电机方向引脚
#define AIN1 PCout(4)
#define AIN2 PCout(5)


//电机PWM引脚
#define PWMA TIM3->CCR2
#define PWMB TIM3->CCR1

#endif
