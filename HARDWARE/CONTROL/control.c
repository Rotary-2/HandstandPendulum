//#include "control.h"
//#include "math.h"

///*---------------电机控制相关---------------*/
//float u = 0; //LQR输出量
//float Target_Moto = 0;//电机目标速度
//int Moto;//输入到电机的PWM数值
//int Moto_Kp = -200 , Moto_Ki = -400 ;//PID参数
///*-----------------------------------------*/

///*---------------编码器读取相关---------------*/
//int Encoder;//编码器原始值
//float dx;   //某时刻的滑块位移
//float x_dot;//某时刻的滑块速度
//float x_dot_1;//控制时的滑块速度
//float x;    //积分累计的位移
//float last_x;//上一次的位移
//float x_diff;//滑块位移的控制量
//齿轮旋转一圈的周长（齿数*齿距 = 20 * 0.00508 = 0.1016 m）
//#define GearWheel_Girth 0.1016f
///*------------------------------------------*/

///*-------------角位移传感器相关数据-------------*/
//#define ADC_1_Ch 11  //角位移传感器1数据采集通道
//#define ADC_2_Ch 12  //角位移传感器2数据采集通道
//u16 mid_1 = 3095 , mid_2 = 0 ; //角位移1、2中值
//u16 Angle_ADC_1,Angle_ADC_2;//角位移传感器ADC原始值
//float theta_1,theta_2; //经过计算转换后的实际弧度值
//float theta_dot_1,theta_dot_2;//角速度
//float last_theta_1,last_theta_2;//上一次的弧度
///*------------------------------------------*/

///*-------------时间、频率相关参数-------------*/
//u8 COUNT = 0; //用于延迟控制
//float t = 0.01f;  //动作时间间隔
//float t1 = 0.002f; //采集数据时间间隔
///*------------------------------------------*/

///*-------------延迟相关参数-------------*/
//volatile u8 delay_flag = 1;//用于配合主函数完成延迟
//u8 delay_50 = 0;//计时变量，用于配合主函数完成延迟
///*------------------------------------------*/

///*-------------LED状态相关参数-------------*/
//u8 LED_winkTimes = 2;//LED闪烁的次数
//u16 LED_ONTime = 50,LED_OFFTime = 200,LED_TaskFre = 400;//LED点亮时间，关闭时间，执行任务的间隔		
///*------------------------------------------*/

///*--------------------中断自检相关参数--------------------*/
//当定义Check_Fre为1时,use_time输出的是中断的频率时间（单位ms）
//当定义Check_UseTime为1时,use_time输出的是执行中断消耗的时间（单位ms）
//#define Check_UseTime 0 //定义监测频率或检测中断用时
//#if Check_Fre || Check_UseTime
//u32 temp_check_begin,temp_check_end;
//int64_t temp_check_time;
//u8 time_count_flag=0;
//float use_time;//记录用时，单位ms
//#endif
///*--------------------------------------------------------*/

///* 软件使能位相关参数 */
//u8 Flag_Stop=1;//电机软件使能标志位 0使能1失能
//u8 MotorDisable=0;//用于辅助失能滤波

///* 电压测量相关参数 */
//float Voltage; //电压值
//#define Battery_Ch 13 //电池电压采集通道

///* 零点标定相关参数 */
//u8 ZeroFlag = 0;//表示是否已找到零点
//u8 FindOnce = 0;//首次开机时自动寻找零点使用
//u8 ZeroStep = 0;//自动寻找零点的步骤记录

///* 中值调节相关参数 */
//u8 Mid_Adjust = 0; //用于表示是否处于中值调节模式
//u8 select_mid = 0; //用于选择需要调节中值1/2
//u16 mul=1;//调节倍率
//u8 tips=3;//OLED提示相关

///* 是否进行掉电保存位 */
//volatile u8 Flash_SaveFlag = 0;

///* 状态反馈系数 */
//float k1 = 52.0602f, k2 = -253.6011f, k3 = 416.9505f, k4 = 43.1534f, k5 = -5.8431f, k6 = 51.2644f;
//float k1 = -78.4629f, k2 = -127.6699f, k3 = -50.0307f, k4 = -18.2386f ;

//自动起摆步骤记录器
//u8 START_0, START_1;

///**************************************************************************
//函数功能：主要逻辑控制部分(定时器中断函数)
//入口参数：无
//返 回 值：无
//作    者：WHEELTEC
//**************************************************************************/
//void TIM6_IRQHandler(void)
//{
//	if(TIM_GetITStatus(TIM6, TIM_IT_Update)!=RESET)
//	{
//		TIM_ClearITPendingBit(TIM6, TIM_IT_Update); //清除中断标志位
//		
//		//辅助主函数完成延迟
//		if(delay_flag==1)
//		{	//延迟 2ms * 25
//			if(++delay_50==25) delay_50=0,delay_flag=0;
//		}
//		
//		//按键扫描任务
//		//Key(500);
//		
//		//中值调节
//		//Adjust_Mid();
//		
//		//寻找零点
//		//Find_Zero();
//		
//		//电池电压采集
//		Voltage = Get_Adc2(Battery_Ch)/4096.0f*3.3f*11;
//		
//		//角度采集
//		Angle_ADC_1 = Get_Adc1_Average(ADC_1_Ch,30);
//		
//		
//		//读取编码器数值
//		Encoder = -Read_Encoder(2);
//		
//		//计算时刻电机转动的距离
//		//电机旋转1圈的编码器脉冲数 = 编码器精度 * 电机减速比 * 倍频数 = 500*5.18*4 = 10360
//		//滑块位移 = 电机旋转的圈数 * 齿轮转一圈的长度 
//		//dx = (Encoder/10360.0f)*GearWheel_Girth;
//		
//		//计算滑块速度,单位 m/s
//		x_dot = dx / t1;
//		
//		//计算滑块位移
//		x += dx;
//		
//		//时间变量
//		COUNT += 1;
//		
//		if(COUNT == 5) // 历经0.01s发布一次新的目标速度
//		{
//			COUNT = 0;
//			theta_1 = angle_count(Angle_ADC_1, mid_1) /180.0f * PI;  //计算第一阶弧度
//			theta_dot_1 = (theta_1 - last_theta_1) / t;//计算第一阶角速度
//			last_theta_1 = theta_1;                    //保存本次数据提供给下一次使用
//			
//			/* 改装1阶后第二阶数据不再需要
//			theta_2 = ( angle_count(Angle_ADC_1, mid_1) + angle_count(Angle_ADC_2, mid_2)) /180.0f * PI;//计算第二阶弧度
//			theta_dot_2 = (theta_2 - last_theta_2) / t;//计算第二阶角速度	
//			last_theta_2 = theta_2;
//			*/
//			
//			x_dot_1 = (x-last_x) / t;                  //计算发布目标速度时的滑块线速度

//			if(Angle_ADC_1>(mid_1-200) && Angle_ADC_1<(mid_1+200))
//			{
//				if (START_1 == 0) x_dot_1=0, u = 0, START_1 = 1;//稳摆瞬间，清空起摆时的变量
//				else u = -(k1 * ( x + x_diff ) + k2 * theta_1 + k3 * x_dot_1 + k4 * theta_dot_1 );//稳摆（计算当前需要的加速度）
//			}
//			else
//			{				
//				if ((START_0 == 0 && Flag_Stop==0 )|| float_abs(x) > 0.15f)
//				{
//					u = 1; //在第一次起摆或者摆杆静止时，需要赋值一个加速度
//					if( x>=0.02 ) x_dot_1=0, u = 0, START_0 = 1;
//					
//					//位置X超限时，手动调整
//					if(float_abs(x)>0.15f && Flag_Stop == 0) x_dot_1 = 0, u = -1*Sgn(x); //起摆位置超限，让电机反转
//				}
//				else u  = 4.0 * Sgn( -theta_dot_1 * cos(theta_1) ) + Sgn(x) * log( 1 - float_abs(x)/0.15 ); //起摆过程
//				
//				START_1 = 0;
//			}

//			
//			last_x = x;                                //保存本次数据提供给下一次使用	
//			if(ZeroFlag) Target_Moto = x_dot_1 + u*t;  //计算当前时刻的目标速度发布（已经进行零点标定的前提下）	
//		}
//		
//		//对电机目标速度进行PID闭环计算
//		Moto = Incremental_PI(x_dot,Target_Moto);
//		
//		//电机控制部分
//		if(Flag_Stop==0&&EN==1)
//		{
//			//角度、位移、电压不满足时，关停电机
//			if( Turn_OFF() )
//			{
//				MotorDisable++;
//				if(MotorDisable>5)//不满足稳摆条件超过5次，关停电机
//				{
//					MotorDisable=5;
//					if(ZeroFlag) Moto=0,Flag_Stop=1;//未寻找完零点时不受角度与位移限制
//				}					
//			}
//			else
//				if(MotorDisable>0) MotorDisable--;
//		}
//		else
//		{   //当急停开关使能后，软件急停也需要使能，防止开关拨动到ON后电机瞬间启动
//			if(EN==0) Flag_Stop=1;
//			MotorDisable=0;//清零积累量
//			Moto=0;
//		}
//		
//		PWM_Limit();        //PWM限幅
//		Set_PWM(Moto);      //PWM发送	
//		MotorStall_Check(x);//电机堵转检测保护
//		
//		Fre_or_Time_Debug_end();//时间或频率debug监测
//	}
//}


///**************************************************************************
//函数功能：对float类型取绝对值
//入口参数：float数据
//返回  值：取绝对值后的数据
//作    者：WHEELTEC
//**************************************************************************/
//static float float_abs(float x)
//{
//	float y;
//	if (x>=0) y=x;
//	else y=-x;
//	return y;
//}


///**************************************************************************
//函数功能：符号函数
//入口参数：需要去符号的数据
//返回  值：符号
//作    者：WHEELTEC
//**************************************************************************/
//static int Sgn (float x)
//{
//	int y;
//	if ( x>0 ) y = 1;
//	else if ( x<0 ) y = -1;
//	else y = 0;
//	return y;
//}

///**************************************************************************
//函数功能：增量式PI控制器
//入口参数：测量速度、目标速度
//返 回 值：PWM值
//作    者：WHEELTEC
//**************************************************************************/
//static int Incremental_PI(float CurrentVal,float TargetVal)
//{
//	float Bias;
//	static float  Last_bias;
//	static int PWM;
//	
//	Bias = CurrentVal - TargetVal;
//	PWM += Moto_Ki*Bias + Moto_Kp*(Bias-Last_bias);
//	Last_bias=Bias;
//	
//	if(Flag_Stop==1||EN==0) PWM = 0;//电机关闭时，清空累计量
//	
//	return PWM;
//}

///**************************************************************************
//函数功能：自动寻找位移零点
//入口参数：无
//返 回 值：无
//作    者：WHEELTEC
//**************************************************************************/
//static void Find_Zero(void)
//{
//	if(FindOnce==1) //第一次开机时，用户按键被单击，则开始寻找零点
//	{
//		if(ZeroStep==0)//第一步：滑块向左移动寻找限位开关
//		{
//			Target_Moto = -0.2f;
//			if(LIMIT==0)
//			{
//				x = -0.205f;//限位开关处，距离中点0.205米
//				ZeroStep = 1;
//			}
//		}
//		else if(ZeroStep==1)//第二步：回到中点
//		{
//			Target_Moto = 0.2f;
//			if(x>=0)
//			{
//				Target_Moto = 0;
//				ZeroStep = 2;
//				ZeroFlag=1;//标记零点查找完毕		
//				Flag_Stop = 1;//关闭电机
//				LED_SystemReady;//系统就绪时LED的状态
//			}
//		}
//		else
//			return;
//	}
//}

///**************************************************************************
//函数功能：平衡条件检测
//入口参数：无
//返 回 值：返回1则不满足平衡条件，返回0表示满足平衡条件
//作    者：WHEELTEC
//**************************************************************************/
//static u8 Turn_OFF(void)
//{
//	u8 temp = 0;
//	
//	//判断第一阶角度是否在平衡范围内
//	if(Angle_ADC_1>(mid_1-400) && Angle_ADC_1<(mid_1+400)) ++temp;
//	
//	//判断第二阶角度是否在平衡范围内
//	//if(Angle_ADC_2>(mid_2-400) && Angle_ADC_2<(mid_2+400)) ++temp;
//	
//	//判断位移是否有超出边界
//	if(x>=-0.22f && x<=0.22f) ++temp;
//	
//	//判断适配器电压是否足够
//	if(Voltage>20) ++temp;
//		
//	//判断限位开关是否被接触到
//	if(LIMIT==1) ++temp;
//	
//	if(temp==3) return 0; //5个平衡条件均满足，返回0代表满足平衡条件
//	else        return 1; //出现任意不满足的条件，返回1代表不满足平衡条件

//}

///**************************************************************************
//函数功能：按键扫描
//入口参数：执行该函数的频率
//返 回 值：无
//作    者：WHEELTEC
//**************************************************************************/
//static void Key(u16 times)
//{	
//	u8 tmp = key_stateless;         //接收用户按键状态的变量
//	u8 L_Control = 0 ,R_Control = 0;//接收左按键、右按键状态的变量
//	
//	//控制滑块移动的逻辑变量
//	static u8 MoveL,MoveR; 
//	static u16 TimeL,TimeR;
//	
//	float Count_time = (((float)(1.0f/(float)times))*1000.0f);//算出计1需要多少个毫秒
//	
//	/*------------------- 按键扫描 ------------------- */
//	tmp=KEY_Scan(times,0);//用户按键扫描
//	if(Mid_Adjust==0)//非调节模式使用此方法扫描按键
//	{
//		L_Control = L_click();//左控制按键单击扫描
//		R_Control = R_KEY_Scan(times);//右控制按键扫描		
//	}
//	/*---------------------------------------------- */
//	
//	//用户按键处理
//	if(tmp==single_click)
//	{
//		if(Mid_Adjust==1) //调节模式，单击按键选择调节的倍率
//		{
//			mul *= 10;
//			if(mul>1000) mul=1;
//			ReflashPart=1;
//			
//			//OLED指示使用变量
//			if(mul==1) tips=3;
//			else if(mul==10) tips=2;
//			else if(mul==100) tips=1;
//			else if(mul==1000) tips=0;
//		}
//		else //非调节模式，按键用于控制系统启停
//		{
//			if(ZeroFlag==0) FindOnce = 1;//第一次开机，单击用户按键开始寻找零点
//			Flag_Stop = !Flag_Stop;
//		}		
//	}
//	
//	else if(tmp==double_click)
//	{
//		if(Mid_Adjust==1) //在调节模式下，双击选择要调节的中值
//		{
//			select_mid = !select_mid;
//		}
//		else//非调节模式下，双击手动标定零点
//		{
//			//防止用户在自动标定零点期间出现手动标定行为
//			if(FindOnce==1&&ZeroStep<2) return; 
//			
//			ZeroFlag=1;//标定为已找到零点
//			x = 0; 
//			LED_SystemReady;//系统就绪时LED的状态
//		}

//	}
//	
//	else if(tmp==long_click)
//	{
//		if(Mid_Adjust==1) //从调节模式切换回非调节模式，则保存中值到Flash
//		{
//			Flash_SaveFlag = 1;
//		}
//		
//		Mid_Adjust = !Mid_Adjust;// 进入/退出 调节模式
//		Flag_Stop=1; //关停电机
//		Flag_Show=0; //显示恢复
//		if(Mid_Adjust==1)//进入调节模式页
//		{
//			show_page = 1;
//			LED_AdjustMid;//系统进入调节模式时LED的状态
//		}
//		else //退出调节模式页
//		{
//			show_page = 0;
//			if(ZeroFlag==1) LED_SystemReady;//系统就绪时LED的状态
//			else if(ZeroFlag==2) LED_ZeroERROR;//限位开关不正常时的LED状态
//			else LED_ZeroNoFound;//系统未标定零点时LED的状态
//		}
//		
//	}
//	
//	//左控制按键处理
//	if(L_Control) MoveL=1,MoveR=0;
//	if(MoveL)
//	{
//		TimeL++;
//		x_diff += 0.0001f; //步进值0.0001,2ms步进1次,共步进1秒 = 1000/2 * 0.0001 = 0.05m
//		if(TimeL>=(u16)(1000/Count_time)) //1秒
//		{
//			TimeL=0;
//			MoveL = 0;
//		}
//	}
//	else
//		TimeL = 0;
//	
//	//右控制按键处理
//	if(R_Control == single_click) MoveR=1,MoveL=0;
//	if(MoveR)
//	{
//		TimeR++;
//		x_diff -= 0.0001f;
//		if(TimeR>=(u16)(1000/Count_time)) //1秒
//		{
//			TimeR=0;
//			MoveR = 0;
//		}
//	}
//	else
//		TimeR = 0; 
//	
//	//右控制按键长按处理
//	if(R_Control == long_click) Flag_Show = !Flag_Show;//显示模式切换

//}


///**************************************************************************
//函数功能：通过按键调节中值
//入口参数：无
//返 回 值：无
//作    者：WHEELTEC
//**************************************************************************/
//static void Adjust_Mid(void)
//{
//	static u8 LowFre;//用于降低扫描频率
//	
//	if(++LowFre==100)
//	{
//		LowFre=0;
//		
//		if(Mid_Adjust==1) //进入了调节模式
//		{
//			if(L_KEY==0) //左按键按下
//			{
//				if(select_mid) mid_2-=1*mul;
//				else	mid_1-=1*mul;
//			}		
//			if(R_KEY==0) //右按键按下
//			{
//				if(select_mid) mid_2+=1*mul;
//				else	mid_1+=1*mul;
//			}
//			
//			//中值限幅
//			if(mid_1<1) mid_1=0;
//			if(mid_2<1) mid_2=0;
//			if(mid_1>4095) mid_1=4095;
//			if(mid_2>4095) mid_2=4095;
//		}
//	}

//}


///**************************************************************************
//函数功能：通过输入的ADC采集值计算角度
//入口参数：当前ADC采集值、计算时参考的零点（中值）
//返 回 值：当前角度
//作    者：WHEELTEC
//**************************************************************************/
//static float angle_count(float Angle_ADC, float mid)
//{
//	float Angle;
//	Angle = (mid - Angle_ADC)*0.0879f;
//	if (Angle >  180) Angle -= 360;
//	if (Angle < -180) Angle += 360;
//	return Angle;
//}

///**************************************************************************
//函数功能：PWM限幅，防止满占空出现电机不稳定
//入口参数：无
//返 回 值：无
//作    者：WHEELTEC
//**************************************************************************/
//static void PWM_Limit(void)
//{	
//	int Amplitude=6900;    //===PWM满幅是7200 限制在6900
//	
//	if(ZeroFlag==0) Amplitude=1300;//在寻找零点时，PWM限幅在1200,防止用户在未接入\
//	                                 编码器的情况启动零点查找，导致电机飞转
//	
//	if(Moto<-Amplitude) Moto=-Amplitude;	
//	if(Moto>Amplitude)  Moto=Amplitude;		
//}

///**************************************************************************
//函数功能：PWM发送，控制电机运动
//入口参数：PWM数值大小
//返 回 值：无
//作    者：WHEELTEC
//**************************************************************************/
//#define USE_MOTOR1 0 //2路电机驱动选择其一（对应驱动接口MOTOR1、MOTOR2）
//#define USE_MOTOR2 1
//static void Set_PWM(int moto)
//{
//	#if USE_MOTOR1
//	if(moto>0)		AIN1 = 1,AIN2 = 0;
//	else 	        moto=-moto,AIN1 = 0,AIN2 = 1;
//	
//	if(EN==0) moto = 0,AIN1 = 1,AIN2 = 1;//使能开关释放电机
//	
//	PWMA = moto;
//	#endif
//	
//	#if USE_MOTOR2
//	if(moto>0)		BIN1 = 1,BIN2 = 0;
//	else 	        moto=-moto,BIN1 = 0,BIN2 = 1;
//	
//	if(EN==0) moto = 0,BIN1 = 1,BIN2 = 1;//使能开关释放电机
//	
//	PWMB = moto;
//	#endif	
//}


///**************************************************************************
//函数功能：①电机堵转检测，检测到堵转及时关停电机
//         ②编码器检测，检测到编码器未接入时关停电机
//入口参数：滑块位移
//返 回 值：无
//作    者：WHEELTEC
//**************************************************************************/
//static void MotorStall_Check(float dis)
//{
//	static float Last_X = 0;
//	float diff_x;
//	static u16 MotorError;//堵转时间
//	
//	diff_x = dis - Last_X; //记录本次位移与上一次位移偏差
//	Last_X = dis; //保存本次位移
//	
//	if(Moto!=0) //当前存在控制量
//	{
//		//存在控制量时且两次位移偏差很小，说明电机此时堵转、或电机编码器未连接
//		if(diff_x<=0.01f&&diff_x>=-0.01f) 
//		{
//			if(ZeroFlag) //零点标定完成时
//			{
//				if(Moto>6500||Moto<-6500) MotorError++;
//				else 
//				{
//					if(MotorError>0) MotorError--;//防止运行时出现短暂性的较大控制量造成误判
//				}
//			}
//			else //零点标定未完成时
//			{
//				if(Moto>=1300||Moto<=-1300) MotorError++;
//				else
//				{
//					if(MotorError>0) MotorError--;//防止运行时出现短暂性的较大控制量造成误判
//				}
//			}
//		}
//	}
//	else
//		MotorError = 0;
//	
//	//2ms中断 
//	//堵转超过0.5s后，关闭电机
//	if(MotorError>250)
//	{
//		if(ZeroFlag==0)//如果是标定过程中出现的堵转，则停止标定
//		{
//			Target_Moto=0;
//			if(x>=-0.01f&&x<=0.01f) //若是编码器未连接
//			{
//				LED_EncoderERROR;//错误提示，编码器未连接
//			}
//			else //若是限位开关未连接
//			{
//				if(ZeroStep==0)
//				{
//					ZeroFlag=2;//标记已找到零点
//					ZeroStep=2;//结束步骤
//					x = -0.205f;//标定已知的位置
//					LED_ZeroERROR;//错误提示，告知用户限位开关未连接或故障
//				}
//				else
//					LED_EncoderERROR;//错误提示，编码器未连接
//			}	
//		}
//		
//		Flag_Stop = 1; //停转电机
//	}
//}


///**************************************************************************
//函数功能：利用DWT计时器计算中断频率或中断内用时时间（单位ms）
//入口参数：无
//返 回 值：无
//作    者：WHEELTEC
//**************************************************************************/
//static void Fre_or_Time_Debug_Begin(void)
//{
//	#if Check_Fre || Check_UseTime
//	time_count_flag = !time_count_flag; //监测频率时，用于记录两次翻转的时间间隔
//	if(time_count_flag==1) temp_check_begin = DWT_GetTick_us();//第1次翻转 || 记录中断最开始的时间
//	#endif
//	
//	#if Check_Fre
//	else temp_check_end = DWT_GetTick_us();//第二次翻转
//	
//	if(time_count_flag==0)//第二次翻转后即可计算出频率
//	{
//		temp_check_time = temp_check_end - temp_check_begin;//翻转总时间
//		if(temp_check_time<0) 
//		{
//			temp_check_time = temp_check_begin + 4294967295u - temp_check_end;//DWT为32位寄存器，如果小于0则加上32位寄存器最大值得出正确时间
//		}
//		use_time = (float)(temp_check_time/72.0f)/1000.0f;// DWT计时器频率为72M，÷72表示用了多少 us, ÷1000把单位转换为ms
//	}
//	#endif
//}

///**************************************************************************
//函数功能：利用DWT计时器计算中断频率或中断内用时时间（单位ms）
//入口参数：无
//返 回 值：无
//作    者：WHEELTEC
//**************************************************************************/
//static void Fre_or_Time_Debug_end(void)
//{
//	#if Check_UseTime
//	if(time_count_flag==1) temp_check_end = DWT_GetTick_us();//记录中断结束时间
//	temp_check_time = temp_check_end - temp_check_begin;//计算出本次中断内用时
//	if(temp_check_time<0) 
//	{
//		temp_check_time = temp_check_begin + 4294967295u - temp_check_end;//DWT为32位寄存器，如果小于0则加上32位寄存器最大值得出正确时间
//	}
//	use_time = (float)(temp_check_time/72.0f)/1000.0f;// DWT计时器频率为72M，÷72表示用了多少us, ÷1000把单位转换为ms
//	#endif
//}



