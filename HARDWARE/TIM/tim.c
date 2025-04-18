#include "tim.h"


void TIM6_Init(u16 psc,u16 arr)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//开启对应时钟，配置对应GPIO
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	
	//定时器基础属性初始化
	TIM_TimeBaseStructure.TIM_Prescaler = psc; //预分频器 	
	TIM_TimeBaseStructure.TIM_Period = arr;//设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 	
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure); 

	
	//中断分组初始化
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM6, ENABLE);//使能定时器6
}

