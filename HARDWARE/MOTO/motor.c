#include "motor.h"

/**************************************************************************
函数功能：控制电机正反转引脚初始化
入口参数：无
返回  值：无
作    者：WHEELTEC
**************************************************************************/
static void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能端口时钟

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;  //端口配置
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;      //50M
//	GPIO_Init(GPIOA, &GPIO_InitStructure);				  //根据设定参数初始化GPIO

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;  //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     //50M
	GPIO_Init(GPIOC, &GPIO_InitStructure);				  //根据设定参数初始化GPIO

//	GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6); //io输出1，关停电机
	GPIO_SetBits(GPIOC,GPIO_Pin_4|GPIO_Pin_5); //io输出1，关停电机
}


/**************************************************************************
函数功能：电机PWM引脚初始化
入口参数：预装载值、预分配系数
返回  值：无
作    者：WHEELTEC
**************************************************************************/
void Motor_PWM_Init(u16 arr,u16 psc)
{		 		
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	Motor_Init();//电机方向IO初始化
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);  //使能定时器8 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能GPIO外设时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    //50M
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = arr;      //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 
	TIM_TimeBaseStructure.TIM_Prescaler =psc;    //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);            //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             //选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0;                            //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set; 
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);                     //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);            //CH1预装载使能	 

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             //选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0;                            //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set; 
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);                      //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);             //CH2预装载使能	

	TIM_ARRPreloadConfig(TIM3, ENABLE);                           //使能TIMx在ARR上的预装载寄存器	
	TIM_Cmd(TIM3, ENABLE);                                        //使能TIM8
	//TIM_CtrlPWMOutputs(TIM3,ENABLE);                              //高级定时器输出必须使能这句	
	
	//防止电机转动
	PWMA = 0;
	PWMB = 0;
} 
