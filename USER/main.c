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
#define buffer_size 220

unsigned keyword;

//ALIENTEK Mini STM32开发板范例代码11
//TFTLCD显示实验   
//技术支持：www.openedv.com
//广州市星翼电子科技有限公司 

u16 Angle;
u16 angle_buf[buffer_size] = {0};
u16 temp;
u8 Angleindex = 0;
int main(void)
{ 
	u8 lcd_id[12];			//存放LCD ID字符串		 
	delay_init();	    	 //延时函数初始化	  
	uart_init(9600);	 	//串口初始化为9600
	LED_Init();		  		//初始化与LED连接的硬件接口
 	LCD_Init();
	KEY_Init();
	 
	Adc1_Init();
	Adc2_Init();
	Motor_PWM_Init(7199,0);    //电机PWM引脚初始化
	Encoder_Init_TIM2();       //电机编码器初始化
	
//	TIM6_Init(19,7199);        //定时器6初始化(500Hz任务)
	
	POINT_COLOR=BLACK; 
	printf((char*)lcd_id,"LCD ID:%04X",lcddev.id);//将LCD ID打印到lcd_id数组。		

	lcd_show_xfont(10,10, "旋", 16, 0, BLACK);
	lcd_show_xfont(26,10, "转", 16, 0, BLACK);
	lcd_show_xfont(42,10, "臂", 16, 0, BLACK);
	lcd_show_xfont(58,10, "角", 16, 0, BLACK);
	lcd_show_xfont(74,10, "度", 16, 0, BLACK);
	lcd_show_xfont(90,10, ":", 16, 0, BLACK);
	lcd_show_xfont(10,30, "摆", 16, 0, BLACK);
	lcd_show_xfont(26,30, "杆", 16, 0, BLACK);
	lcd_show_xfont(42,30, "角", 16, 0, BLACK);
	lcd_show_xfont(58,30, "度", 16, 0, BLACK);
	lcd_show_xfont(90,30, ":", 16, 0, BLACK);
	lcd_show_xfont(10,50, "旋", 16, 0, BLACK);
	lcd_show_xfont(26,50, "转", 16, 0, BLACK);
	lcd_show_xfont(42,50, "角", 16, 0, BLACK);
	lcd_show_xfont(58,50, "度", 16, 0, BLACK);
	lcd_show_xfont(90,50, ":", 16, 0, BLACK);
  while(1) 
	{		 
		POINT_COLOR=BLACK; 
		keyword = KEY_Scan(0);
		if(keyword)
		LCD_ShowNum(10, 70, keyword, 2, 16);   
		printf("%d\r\n", keyword);
		                                
		Angle = Get_Adc1_Average(11,16);
		LCD_ShowNum(106,30, Angle, 3, 16);
	
		temp = angle_buf[Angleindex];
		angle_buf[Angleindex] = Angle / 10 - 200;
		printf("angle_buf[%d] = %d\r\n",Angleindex, angle_buf[Angleindex]);
		Angleindex = (Angleindex + 1) % buffer_size;
		
		LCD_DrawLine(10, 100, 11, 310);    
		LCD_DrawLine(10, 309, 230, 310);
		
//		if (Angleindex == 0) LCD_Fill(10, 100, 230, 310, WHITE);
		for(int i = Angleindex + 1; i < buffer_size; i++) 
		{	
        int x = i - Angleindex - 1 + 10;
        int y = 310 - angle_buf[i];
				int last_y = 310 - angle_buf[i - 1];
				LCD_Fast_DrawPoint(x, last_y, WHITE);   
				LCD_Fast_DrawPoint(buffer_size, angle_buf[0], WHITE); 			
				LCD_Fast_DrawPoint(x, y, BLACK);    
				if (Angleindex == buffer_size - 1) LCD_Fast_DrawPoint(x, y, WHITE);   
    }
		for(int i = 0; i < Angleindex; i++) 
		{
        int x = i + buffer_size - Angleindex + 10;
        int y = 310 - angle_buf[i];
				int last_y = 310 - angle_buf[i - 1];
				LCD_Fast_DrawPoint(x, last_y, WHITE);     
				LCD_Fast_DrawPoint(buffer_size, angle_buf[0], WHITE); 	
        LCD_Fast_DrawPoint(x, y, BLACK);     
				if (Angleindex == buffer_size - 1) LCD_Fast_DrawPoint(x, y, WHITE);   
    }
	} 
}
