#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "lcd.h"
#include "adc.h"
#include "encoder.h"
#include "motor.h"
#include "tim.h"
#include "key_board.h"

unsigned keyword;

//ALIENTEK Mini STM32�����巶������11
//TFTLCD��ʾʵ��   
//����֧�֣�www.openedv.com
//������������ӿƼ����޹�˾ 


u16 Angle;
int main(void)
{ 
	u8 lcd_id[12];			//���LCD ID�ַ���		 
	delay_init();	    	 //��ʱ������ʼ��	  
	uart_init(9600);	 	//���ڳ�ʼ��Ϊ9600
	LED_Init();		  		//��ʼ����LED���ӵ�Ӳ���ӿ�
 	LCD_Init();
	KEY_Init();
	 
	Adc1_Init();
	Adc2_Init();
	Motor_PWM_Init(7199,0);    //���PWM���ų�ʼ��
	Encoder_Init_TIM2();       //�����������ʼ��
	
//	TIM6_Init(19,7199);        //��ʱ��6��ʼ��(500Hz����)
	
	POINT_COLOR=BLACK; 
	printf((char*)lcd_id,"LCD ID:%04X",lcddev.id);//��LCD ID��ӡ��lcd_id���顣		

	lcd_show_xfont(10,10, "��", 16, 0, BLACK);
	lcd_show_xfont(26,10, "ת", 16, 0, BLACK);
	lcd_show_xfont(42,10, "��", 16, 0, BLACK);
	lcd_show_xfont(58,10, "��", 16, 0, BLACK);
	lcd_show_xfont(74,10, "��", 16, 0, BLACK);
	lcd_show_xfont(90,10, ":", 16, 0, BLACK);
	lcd_show_xfont(10,30, "��", 16, 0, BLACK);
	lcd_show_xfont(26,30, "��", 16, 0, BLACK);
	lcd_show_xfont(42,30, "��", 16, 0, BLACK);
	lcd_show_xfont(58,30, "��", 16, 0, BLACK);
	lcd_show_xfont(90,30, ":", 16, 0, BLACK);
	lcd_show_xfont(10,50, "��", 16, 0, BLACK);
	lcd_show_xfont(26,50, "ת", 16, 0, BLACK);
	lcd_show_xfont(42,50, "��", 16, 0, BLACK);
	lcd_show_xfont(58,50, "��", 16, 0, BLACK);
	lcd_show_xfont(90,50, ":", 16, 0, BLACK);
  while(1) 
	{		 
		POINT_COLOR=BLACK; 
		keyword = KEY_Scan(0);
		if(keyword)
		LCD_ShowNum(10, 70, keyword, 2, 16);   
		printf("%d\r\n", keyword);

	} 
}
