#include "encoder.h"

/**************************************************************************
函数功能：将定时器2初始化为编码器模式3
入口参数：无
返回  值：无
作    者：WHEELTEC
**************************************************************************/
void Encoder_Init_TIM2(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//使能定时器2的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能PA端口时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;  //端口配置 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);				  //根据设定参数初始化GPIOA
  
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x00; // 预分频器 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//边沿计数模式 
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);  //初始化定时器2
	
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
	
	TIM_ICStructInit(&TIM_ICInitStructure); //把TIM_ICInitStruct 中的每一个参数按缺省值填入
	TIM_ICInitStructure.TIM_ICFilter = 10;  //设置滤波器长度
	TIM_ICInit(TIM2, &TIM_ICInitStructure);//根据 TIM_ICInitStruct 的参数初始化外设	TIMx
 
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);//清除TIM的更新标志位
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);//使能定时器中断
	
	TIM_SetCounter(TIM2,0);
	TIM_Cmd(TIM2, ENABLE); //使能定时器2
}

/**************************************************************************
函数功能：单位时间读取编码器计数
入口参数：定时器
返回  值：速度值
作    者：WHEELTEC
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
	int Encoder_TIM;    
	switch(TIMX)
	{
		case 2:  Encoder_TIM= (short)TIM2 -> CNT;   TIM2 -> CNT=0;  break;
		case 3:  Encoder_TIM= (short)TIM3 -> CNT;   TIM3 -> CNT=0;  break;
		case 4:  Encoder_TIM= (short)TIM4 -> CNT;   TIM4 -> CNT=0;  break;	
		case 5:  Encoder_TIM= (short)TIM5 -> CNT;   TIM5 -> CNT=0;  break;	
		default:  Encoder_TIM=0;
	}
	return Encoder_TIM;
}

/**************************************************************************
*  函数功能：TIM2中断服务函数
*  入口参数：无
*  返 回 值：无
*  作    者：WHEELTEC
**************************************************************************/
void TIM2_IRQHandler(void)
{ 		    		  			    
	if(TIM2->SR&0X0001)//溢出中断
	{    				   				     	    	
	}				   
	TIM2->SR&=~(1<<0);//清除中断标志位 	    
}

