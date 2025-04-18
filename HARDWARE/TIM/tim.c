#include "tim.h"


void TIM6_Init(u16 psc,u16 arr)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//������Ӧʱ�ӣ����ö�ӦGPIO
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	
	//��ʱ���������Գ�ʼ��
	TIM_TimeBaseStructure.TIM_Prescaler = psc; //Ԥ��Ƶ�� 	
	TIM_TimeBaseStructure.TIM_Period = arr;//�趨�������Զ���װֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 	
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure); 

	
	//�жϷ����ʼ��
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM6, ENABLE);//ʹ�ܶ�ʱ��6
}

