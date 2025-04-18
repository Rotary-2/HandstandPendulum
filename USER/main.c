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

double Angle;
double Angle_org;
u16 angle_buf[buffer_size] = {0};
u16 temp;
u8 Angleindex = 0;
/*---------------电机控制相关---------------*/
float u = 0; //LQR输出量
float Target_Moto = 0;//电机目标速度
int Moto;//输入到电机的PWM数值
int Moto_Kp = -200 , Moto_Ki = -400 ;//PID参数
/*-----------------------------------------*/

/*---------------编码器读取相关---------------*/
int Encoder;//编码器原始值
float dx;   //某时刻的滑块位移
float x_dot;//某时刻的滑块速度
float x_dot_1;//控制时的滑块速度
float x;    //积分累计的位移
float last_x;//上一次的位移
float x_diff;//滑块位移的控制量
//齿轮旋转一圈的周长（齿数*齿距 = 20 * 0.00508 = 0.1016 m）
#define GearWheel_Girth 0.1016f
/*------------------------------------------*/

/*-------------角位移传感器相关数据-------------*/
#define ADC_1_Ch 11  //角位移传感器1数据采集通道
#define ADC_2_Ch 12  //角位移传感器2数据采集通道
u16 mid_1 = 3095 , mid_2 = 0 ; //角位移1、2中值
u16 Angle_ADC_1,Angle_ADC_2;//角位移传感器ADC原始值
float theta_1,theta_2; //经过计算转换后的实际弧度值
float theta_dot_1,theta_dot_2;//角速度
float last_theta_1,last_theta_2;//上一次的弧度
/*------------------------------------------*/

/*-------------时间、频率相关参数-------------*/
u8 COUNT = 0; //用于延迟控制
float t = 0.01f;  //动作时间间隔
float t1 = 0.002f; //采集数据时间间隔
/*------------------------------------------*/

/*-------------延迟相关参数-------------*/
volatile u8 delay_flag = 1;//用于配合主函数完成延迟
u8 delay_50 = 0;//计时变量，用于配合主函数完成延迟
/*------------------------------------------*/

/*-------------LED状态相关参数-------------*/
u8 LED_winkTimes = 2;//LED闪烁的次数
u16 LED_ONTime = 50,LED_OFFTime = 200,LED_TaskFre = 400;//LED点亮时间，关闭时间，执行任务的间隔		
/*------------------------------------------*/

/*--------------------中断自检相关参数--------------------*/
//当定义Check_Fre为1时,use_time输出的是中断的频率时间（单位ms）
//当定义Check_UseTime为1时,use_time输出的是执行中断消耗的时间（单位ms）
#define Check_UseTime 0 //定义监测频率或检测中断用时
#if Check_Fre || Check_UseTime
u32 temp_check_begin,temp_check_end;
int64_t temp_check_time;
u8 time_count_flag=0;
float use_time;//记录用时，单位ms
#endif
/*--------------------------------------------------------*/

/* 软件使能位相关参数 */
u8 Flag_Stop=1;//电机软件使能标志位 0使能1失能
u8 MotorDisable=0;//用于辅助失能滤波

/* 电压测量相关参数 */
float Voltage; //电压值
#define Battery_Ch 12 //电池电压采集通道

/* 零点标定相关参数 */
u8 ZeroFlag = 0;//表示是否已找到零点
u8 FindOnce = 0;//首次开机时自动寻找零点使用
u8 ZeroStep = 0;//自动寻找零点的步骤记录

/* 中值调节相关参数 */
u8 Mid_Adjust = 0; //用于表示是否处于中值调节模式
u8 select_mid = 0; //用于选择需要调节中值1/2
u16 mul=1;//调节倍率
u8 tips=3;//OLED提示相关

/* 是否进行掉电保存位 */
volatile u8 Flash_SaveFlag = 0;

/* 状态反馈系数 */
//float k1 = 52.0602f, k2 = -253.6011f, k3 = 416.9505f, k4 = 43.1534f, k5 = -5.8431f, k6 = 51.2644f;
float k1 = -78.4629f, k2 = -127.6699f, k3 = -50.0307f, k4 = -18.2386f ;

//自动起摆步骤记录器
u8 START_0, START_1;

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
	TIM6_Init(19,7199);        //定时器6初始化(500Hz任务)
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

void TIM6_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM6, TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update); //清除中断标志位

		//辅助主函数完成延迟
		if(delay_flag==1)
		{	//延迟 2ms * 25
			if(++delay_50==25) delay_50=0,delay_flag=0;
		}

		//电池电压采集
//		Voltage = Get_Adc2(Battery_Ch)/4096.0f*3.3f*11;
//		Voltage = Get_Adc2(Battery_Ch);
		
		POINT_COLOR=BLACK;
		
//		keyword = KEY_Scan(0);
//		if(keyword)
//		LCD_ShowNum(10, 70, keyword, 2, 16);   
   
		Angle_org = Get_Adc1_Average(11,16);   
		//Angle = gauss6_fit(Angle_org)*10;

		printf("Angle_org = %lf Voltage = %f\r\n", Angle_org, Voltage);

		LCD_ShowNum(106,130, Angle_org, 5, 16);
	}
}
