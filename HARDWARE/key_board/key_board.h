#ifndef __KEY_BOARD_H
#define __KEY_BOARD_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK miniSTM32开发板
//按键驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/3
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////   	 


//#define KEY0 PCin(5)   	
//#define KEY1 PAin(15)	 
//#define WK_UP  PAin(0)	 

#define C1  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2)//读取按键0
#define C2  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3)//读取按键1
#define C3  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)//读取按键0
#define C4  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)//读取按键1
#define R1(x)  GPIO_WriteBit(GPIOA,GPIO_Pin_8,(BitAction)(x))//读取按键0
#define R2(x)  GPIO_WriteBit(GPIOA,GPIO_Pin_11,(BitAction)(x))//读取按键1
#define R3(x)  GPIO_WriteBit(GPIOA,GPIO_Pin_12,(BitAction)(x))//读取按键0
#define R4(x)  GPIO_WriteBit(GPIOC,GPIO_Pin_3,(BitAction)(x))//读取按键1
// #define WK_UP   GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)//读取按键2 
 

// #define KEY0_PRES	1		//KEY0  
// #define KEY1_PRES	2		//KEY1  

void KEY_Init(void);//IO初始化
u8 KEY_Scan(u8 mode);  	//按键扫描函数					    
#endif
