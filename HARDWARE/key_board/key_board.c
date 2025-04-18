#include "key_board.h"
#include "delay.h"
#include "usart.h"	  
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK Mini STM32开发板
//按键输入 驱动代码		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/3/06
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									   
//////////////////////////////////////////////////////////////////////////////////	

int C = 0;
int R = 0;
u8 i;
 	    
//按键初始化函数 
//PA15和PC5 设置成输入
void KEY_Init(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;

 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC,ENABLE);//使能PORTA,PORTC时钟

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//关闭jtag，使能SWD，可以用SWD模式调试
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 	GPIO_Init(GPIOC, &GPIO_InitStructure);
} 
//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
//返回值：
//0，没有任何按键按下
//KEY0_PRES，KEY0按下
//KEY1_PRES，KEY1按下
//WKUP_PRES，WK_UP按下 
//注意此函数有响应优先级,KEY0>KEY1>WK_UP!!
u8 KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;//按键按松开标志
	if(mode)key_up=1;  //支持连按		  
	
	R1(0);
	R2(0);
	R3(0);
	R4(0);
	if (key_up&&(C1 == 0 || C2 == 0 || C3 == 0 || C4 == 0))
	{
		R1(0);
		R2(1);
		R3(1);
		R4(1);
		if (!C1) return 1;
		else if (!C2) return 2;
		else if (!C3) return 3;
		else if (!C4) return 4;
		
		R1(1);
		R2(0);
		R3(1);
		R4(1);
		if (!C1) return 5;
		else if (!C2) return 6;
		else if (!C3) return 7;
		else if (!C4) return 8;
		
		R1(1);
		R2(1);
		R3(0);
		R4(1);
		if (!C1) return 9;
		else if (!C2) return 10;
		else if (!C3) return 11;
		else if (!C4) return 12;
		
		R1(1);
		R2(1);
		R3(1);
		R4(0);
		if (!C1) return 13;
		else if (!C2) return 14;
		else if (!C3) return 15;
		else if (!C4) return 16;
	} else if(C1 == 1 && C2 == 1 && C3 == 1 && C4 == 1)key_up=1; 	 
	return 0;// 无按键按下
}
