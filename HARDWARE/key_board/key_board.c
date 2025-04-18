#include "key_board.h"
#include "delay.h"
#include "usart.h"	  
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK Mini STM32������
//�������� ��������		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/3/06
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									   
//////////////////////////////////////////////////////////////////////////////////	

int C = 0;
int R = 0;
u8 i;
 	    
//������ʼ������ 
//PA15��PC5 ���ó�����
void KEY_Init(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;

 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC,ENABLE);//ʹ��PORTA,PORTCʱ��

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//�ر�jtag��ʹ��SWD��������SWDģʽ����
	
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
//����������
//���ذ���ֵ
//mode:0,��֧��������;1,֧��������;
//����ֵ��
//0��û���κΰ�������
//KEY0_PRES��KEY0����
//KEY1_PRES��KEY1����
//WKUP_PRES��WK_UP���� 
//ע��˺�������Ӧ���ȼ�,KEY0>KEY1>WK_UP!!
u8 KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;//�������ɿ���־
	if(mode)key_up=1;  //֧������		  
	
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
	return 0;// �ް�������
}
