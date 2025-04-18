#include "motor.h"

/**************************************************************************
�������ܣ����Ƶ������ת���ų�ʼ��
��ڲ�������
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
static void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //ʹ�ܶ˿�ʱ��

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;  //�˿�����
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //�������
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;      //50M
//	GPIO_Init(GPIOA, &GPIO_InitStructure);				  //�����趨������ʼ��GPIO

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;  //�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     //50M
	GPIO_Init(GPIOC, &GPIO_InitStructure);				  //�����趨������ʼ��GPIO

//	GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6); //io���1����ͣ���
	GPIO_SetBits(GPIOC,GPIO_Pin_4|GPIO_Pin_5); //io���1����ͣ���
}


/**************************************************************************
�������ܣ����PWM���ų�ʼ��
��ڲ�����Ԥװ��ֵ��Ԥ����ϵ��
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
void Motor_PWM_Init(u16 arr,u16 psc)
{		 		
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	Motor_Init();//�������IO��ʼ��
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);  //ʹ�ܶ�ʱ��8 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //ʹ��GPIO����ʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; //�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    //50M
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = arr;      //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 
	TIM_TimeBaseStructure.TIM_Prescaler =psc;    //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ����Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);            //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0;                            //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //�������:TIM����Ƚϼ��Ը�
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set; 
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);                     //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);            //CH1Ԥװ��ʹ��	 

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0;                            //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //�������:TIM����Ƚϼ��Ը�
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set; 
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);                      //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);             //CH2Ԥװ��ʹ��	

	TIM_ARRPreloadConfig(TIM3, ENABLE);                           //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���	
	TIM_Cmd(TIM3, ENABLE);                                        //ʹ��TIM8
	//TIM_CtrlPWMOutputs(TIM3,ENABLE);                              //�߼���ʱ���������ʹ�����	
	
	//��ֹ���ת��
	PWMA = 0;
	PWMB = 0;
} 
