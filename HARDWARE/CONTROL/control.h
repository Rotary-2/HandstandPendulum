#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"	 
#include "stdlib.h"
#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "lcd.h"
#include "adc.h"
#include "encoder.h"
#include "motor.h"
#include "tim.h"
#define PI 3.14159f


//ʹ��5��LED״̬��ʾϵͳ��5�����
/*��LED��˸2�Σ�ϵͳδ�������궨
 *��LED��˸3�Σ�������δ����
 *��LED��˸4�Σ���λ����δ��ȷ���ӻ���λ���ع���
 *��LED����   ��ϵͳ�����������������
 *��LED����   ��ϵͳ������ֵ����ģʽ
*/
#define LED_ZeroNoFound \
					do{ LED_winkTimes=2,\
						LED_ONTime=50,  \
						LED_OFFTime=200,\
						LED_TaskFre=400;\
					}while(0)

//�궨���ʱ�������󣬱�����δ����
#define LED_EncoderERROR \
					do{ LED_winkTimes=3,\
						LED_ONTime=50,  \
						LED_OFFTime=200,\
						LED_TaskFre=400;\
					}while(0)
					
//�궨���ʱ����������λ����δ����
#define LED_ZeroERROR \
					do{ LED_winkTimes=4,\
						LED_ONTime=50,  \
						LED_OFFTime=200,\
						LED_TaskFre=400;\
					}while(0)		
					
//ϵͳ�����������Ȱ�					
#define LED_SystemReady \
					do{ LED_winkTimes=1,\
						LED_ONTime=500,  \
						LED_OFFTime=500,\
						LED_TaskFre=0;\
					}while(0)	

//ϵͳ������ֵ����ģʽ					
#define LED_AdjustMid \
					do{ LED_winkTimes=1,\
						LED_ONTime=120,  \
						LED_OFFTime=180,\
						LED_TaskFre=0;\
					}while(0)		

//����������ӳ١�Flash�������
extern volatile u8 delay_flag , Flash_SaveFlag;

//�����ṩ��OLED��ʾ
extern u8 select_mid , tips , Flag_Stop ;
extern u16 Angle_ADC_1 , Angle_ADC_2 , mid_1 , mid_2 ;				
extern int Moto;
extern float theta_1 , theta_2 , theta_dot_1 , theta_dot_2 , last_theta_1 , last_theta_2 \
	         , x , x_dot , k1 , k2 , k3 , k4 , k5 , k6 , Voltage , Target_Moto , use_time , x_diff , u ;

//�ڲ���������
static void Key(u16 times);
static float angle_count(float Angle_ADC, float mid);
static void PWM_Limit(void);
static void Set_PWM(int moto);
static void Find_Zero(void);
static void Adjust_Mid(void);
static int Incremental_PI(float CurrentVal,float TargetVal);
static void Fre_or_Time_Debug_Begin(void);
static void Fre_or_Time_Debug_end(void);
static void MotorStall_Check(float dis);
static u8 Turn_OFF(void);
static int Sgn(float x);
static float float_abs(float x);

#endif

