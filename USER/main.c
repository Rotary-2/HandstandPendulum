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

//ALIENTEK Mini STM32�����巶������11
//TFTLCD��ʾʵ��   
//����֧�֣�www.openedv.com
//������������ӿƼ����޹�˾ 

double Angle;
double Angle_org;
u16 angle_buf[buffer_size] = {0};
u16 temp;
u8 Angleindex = 0;
/*---------------����������---------------*/
float u = 0; //LQR�����
float Target_Moto = 0;//���Ŀ���ٶ�
int Moto;//���뵽�����PWM��ֵ
int Moto_Kp = -200 , Moto_Ki = -400 ;//PID����
/*-----------------------------------------*/

/*---------------��������ȡ���---------------*/
int Encoder;//������ԭʼֵ
float dx;   //ĳʱ�̵Ļ���λ��
float x_dot;//ĳʱ�̵Ļ����ٶ�
float x_dot_1;//����ʱ�Ļ����ٶ�
float x;    //�����ۼƵ�λ��
float last_x;//��һ�ε�λ��
float x_diff;//����λ�ƵĿ�����
//������תһȦ���ܳ�������*�ݾ� = 20 * 0.00508 = 0.1016 m��
#define GearWheel_Girth 0.1016f
/*------------------------------------------*/

/*-------------��λ�ƴ������������-------------*/
#define ADC_1_Ch 11  //��λ�ƴ�����1���ݲɼ�ͨ��
#define ADC_2_Ch 12  //��λ�ƴ�����2���ݲɼ�ͨ��
u16 mid_1 = 3095 , mid_2 = 0 ; //��λ��1��2��ֵ
u16 Angle_ADC_1,Angle_ADC_2;//��λ�ƴ�����ADCԭʼֵ
float theta_1,theta_2; //��������ת�����ʵ�ʻ���ֵ
float theta_dot_1,theta_dot_2;//���ٶ�
float last_theta_1,last_theta_2;//��һ�εĻ���
/*------------------------------------------*/

/*-------------ʱ�䡢Ƶ����ز���-------------*/
u8 COUNT = 0; //�����ӳٿ���
float t = 0.01f;  //����ʱ����
float t1 = 0.002f; //�ɼ�����ʱ����
/*------------------------------------------*/

/*-------------�ӳ���ز���-------------*/
volatile u8 delay_flag = 1;//�����������������ӳ�
u8 delay_50 = 0;//��ʱ�����������������������ӳ�
/*------------------------------------------*/

/*-------------LED״̬��ز���-------------*/
u8 LED_winkTimes = 2;//LED��˸�Ĵ���
u16 LED_ONTime = 50,LED_OFFTime = 200,LED_TaskFre = 400;//LED����ʱ�䣬�ر�ʱ�䣬ִ������ļ��		
/*------------------------------------------*/

/*--------------------�ж��Լ���ز���--------------------*/
//������Check_FreΪ1ʱ,use_time��������жϵ�Ƶ��ʱ�䣨��λms��
//������Check_UseTimeΪ1ʱ,use_time�������ִ���ж����ĵ�ʱ�䣨��λms��
#define Check_UseTime 0 //������Ƶ�ʻ����ж���ʱ
#if Check_Fre || Check_UseTime
u32 temp_check_begin,temp_check_end;
int64_t temp_check_time;
u8 time_count_flag=0;
float use_time;//��¼��ʱ����λms
#endif
/*--------------------------------------------------------*/

/* ���ʹ��λ��ز��� */
u8 Flag_Stop=1;//������ʹ�ܱ�־λ 0ʹ��1ʧ��
u8 MotorDisable=0;//���ڸ���ʧ���˲�

/* ��ѹ������ز��� */
float Voltage; //��ѹֵ
#define Battery_Ch 12 //��ص�ѹ�ɼ�ͨ��

/* ���궨��ز��� */
u8 ZeroFlag = 0;//��ʾ�Ƿ����ҵ����
u8 FindOnce = 0;//�״ο���ʱ�Զ�Ѱ�����ʹ��
u8 ZeroStep = 0;//�Զ�Ѱ�����Ĳ����¼

/* ��ֵ������ز��� */
u8 Mid_Adjust = 0; //���ڱ�ʾ�Ƿ�����ֵ����ģʽ
u8 select_mid = 0; //����ѡ����Ҫ������ֵ1/2
u16 mul=1;//���ڱ���
u8 tips=3;//OLED��ʾ���

/* �Ƿ���е��籣��λ */
volatile u8 Flash_SaveFlag = 0;

/* ״̬����ϵ�� */
//float k1 = 52.0602f, k2 = -253.6011f, k3 = 416.9505f, k4 = 43.1534f, k5 = -5.8431f, k6 = 51.2644f;
float k1 = -78.4629f, k2 = -127.6699f, k3 = -50.0307f, k4 = -18.2386f ;

//�Զ���ڲ����¼��
u8 START_0, START_1;

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
	TIM6_Init(19,7199);        //��ʱ��6��ʼ��(500Hz����)
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
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update); //����жϱ�־λ

		//��������������ӳ�
		if(delay_flag==1)
		{	//�ӳ� 2ms * 25
			if(++delay_50==25) delay_50=0,delay_flag=0;
		}

		//��ص�ѹ�ɼ�
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
