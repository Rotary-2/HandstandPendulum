#include "encoder.h"

/**************************************************************************
�������ܣ�����ʱ��2��ʼ��Ϊ������ģʽ3
��ڲ�������
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
void Encoder_Init_TIM2(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//ʹ�ܶ�ʱ��2��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ��PA�˿�ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;  //�˿����� 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);				  //�����趨������ʼ��GPIOA
  
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x00; // Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //�趨�������Զ���װֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//ѡ��ʱ�ӷ�Ƶ������Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ؼ���ģʽ 
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);  //��ʼ����ʱ��2
	
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//ʹ�ñ�����ģʽ3
	
	TIM_ICStructInit(&TIM_ICInitStructure); //��TIM_ICInitStruct �е�ÿһ��������ȱʡֵ����
	TIM_ICInitStructure.TIM_ICFilter = 10;  //�����˲�������
	TIM_ICInit(TIM2, &TIM_ICInitStructure);//���� TIM_ICInitStruct �Ĳ�����ʼ������	TIMx
 
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);//ʹ�ܶ�ʱ���ж�
	
	TIM_SetCounter(TIM2,0);
	TIM_Cmd(TIM2, ENABLE); //ʹ�ܶ�ʱ��2
}

/**************************************************************************
�������ܣ���λʱ���ȡ����������
��ڲ�������ʱ��
����  ֵ���ٶ�ֵ
��    �ߣ�WHEELTEC
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
*  �������ܣ�TIM2�жϷ�����
*  ��ڲ�������
*  �� �� ֵ����
*  ��    �ߣ�WHEELTEC
**************************************************************************/
void TIM2_IRQHandler(void)
{ 		    		  			    
	if(TIM2->SR&0X0001)//����ж�
	{    				   				     	    	
	}				   
	TIM2->SR&=~(1<<0);//����жϱ�־λ 	    
}

