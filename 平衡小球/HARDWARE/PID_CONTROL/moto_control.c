#include "moto_control.h"
#include "usart.h"	 
#include "pid.h"
#include "led.h"
#include "oled.h"
#include "math.h"

//点坐标
u16 point_location[9][2]={{0,0},{0,0},{0,0},{0,0},{173,107},{0,0},{0,0},{0,0},{0,0}};


u32 time_control = 0;
u16 now_y = 0,now_x = 0;
u16 last_y = 0,last_x = 0;
int X_speedbuf[3]={0,0,0},Y_speedbuf[3]={0,0,0};
float speed_x = 0.0f,speed_y = 0.0f;
float Espeed_x = 0,Espeed_y = 0;

float aim_y = 0.0f,aim_x = 0.0f,duty_x = 0,duty_y = 0;
char dis[20];
u8 start_flag = 0,mode_flag = 4,mode_change = 0;
void TIM5_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); //时钟使能

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(  //使能或者失能指定的TIM中断
		TIM5, //TIM2
		TIM_IT_Update ,
		ENABLE  //使能
		);
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	TIM_Cmd(TIM5, ENABLE);  //使能TIMx外设
							 
}


void TIM5_IRQHandler(void)   //TIM3中断
{
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
		{
			time_control+=5;
			if(read_data())
			{
				moto_driver(0,0);
				if(start_flag == 1)
				start_flag = 0;
			}
			else 
			{
				
				switch(mode_flag)
				{
					case 1:
						if(mode_change == 1)
						{
							time_control = 0;
							mode_change = 0;
						}
							
						mode1();
						break;
					case 2:
						if(mode_change == 1)
						{
							time_control = 0;
							mode_change = 0;
						}
							
						mode2();
						break;
					case 3:
						if(mode_change == 1)
						{
							time_control = 0;
							mode_change = 0;
						}
						mode3();
						break;
					case 4:
						if(mode_change == 1)
						{
							time_control = 0;
							mode_change = 0;
						}
						mode4();
						break;
					case 5:
						if(mode_change == 1)
						{
							time_control = 0;
							mode_change = 0;
						}
						mode5();
						break;
					case 6:
						if(mode_change == 1)
						{
							time_control = 0;
							mode_change = 0;
						}
						mode6();
						break;
					case 7:
						if(mode_change == 1)
						{
							time_control = 0;
							mode_change = 0;
						}
						mode7();
						break;
					case 8:
						if(mode_change == 1)
						{
							time_control = 0;
							mode_change = 0;
						}
						mode8();
						break;
				}
				
				
				
				if(start_flag == 0)
				start_flag = 1;
			}
			
			

			
			
			clear(dis,20);
			sprintf(dis,"loca_x:%5d",now_x);	
			OLED_ShowString(0,0,(u8 *)dis);
			clear(dis,20);
			sprintf(dis,"loca_y:%5d",now_y);	
			OLED_ShowString(0,2,(u8 *)dis);
			sprintf(dis,"speed_x:%3.1f",speed_x);	
			OLED_ShowString(0,4,(u8 *)dis);
			sprintf(dis,"speed_y:%3.1f",speed_y);	
			OLED_ShowString(0,6,(u8 *)dis);
			
			//OLED_Refresh_Gram();
			
			
			
			LED0=!LED0;
		}
	TIM_ClearITPendingBit(TIM5,TIM_IT_Update);  //清除TIMx的中断待处理位:TIM 中断源 
}


void mode1(void)
{
	//PID_calculate(y,aim_y,(float)now_y,&aim_x);
	//PID_calculate(x,aim_x,(float)now_x,&moto_dif);
	
	aim_y = 100.0;
	aim_x = 220.0;
	
	
	if(now_x ==0 &&now_y == 0 )
		moto_driver(0,0);
	else
	{
	PID_calculate(&locaPID_x,(float)aim_x,(float)now_x,&Espeed_x);
	
	PID_calculate(&speedPID_x,Espeed_x,speed_x,(float*)&duty_x);
	
	PID_calculate(&locaPID_y,(float)aim_y,(float)now_y,&Espeed_y);
	
	PID_calculate(&speedPID_y,Espeed_y,speed_y,(float*)&duty_y);
	
	
	
	moto_driver(duty_x,duty_y);
	//moto_driver(500,500);
	}

}


void mode2(void)
{
		
	aim_y = point_location[4][1];
	aim_x = point_location[4][0];
	
	
	if(now_x ==0 &&now_y == 0 )
		moto_driver(0,0);
	else
	{
	PID_calculate(&locaPID_x,(float)aim_x,(float)now_x,&Espeed_x);
	
	PID_calculate(&speedPID_x,Espeed_x,speed_x,(float*)&duty_x);
	
	PID_calculate(&locaPID_y,(float)aim_y,(float)now_y,&Espeed_y);
	
	PID_calculate(&speedPID_y,Espeed_y,speed_y,(float*)&duty_y);
	
	
	
	moto_driver(duty_x,duty_y);
	
	}


}
void mode3(void)
{
		if(time_control<3000)
		{
			aim_y = 100.0;
			aim_x = 220.0;
		}
		else
		{
			aim_y = 100.0;
			aim_x = 175.0;
		}
	
	
	if(now_x ==0 &&now_y == 0 )
		moto_driver(0,0);
	else
	{
	PID_calculate(&locaPID_x,(float)aim_x,(float)now_x,&Espeed_x);
	
	PID_calculate(&speedPID_x,Espeed_x,speed_x,(float*)&duty_x);
	
	PID_calculate(&locaPID_y,(float)aim_y,(float)now_y,&Espeed_y);
	
	PID_calculate(&speedPID_y,Espeed_y,speed_y,(float*)&duty_y);
	
	
	
	moto_driver(duty_x,duty_y);
	
	}



}
void mode4(void)
{
		if(time_control<2000)
		{
			aim_y = point_location[4][1]+30;
			aim_x = point_location[4][0];
		}
		else if(time_control<4000)
		{
			aim_y = point_location[4][1];
			aim_x = point_location[4][0]+30;
		}
		else if(time_control<6000)
		{
			aim_y = point_location[4][1]-30;
			aim_x = point_location[4][0];
		}
		else if(time_control<8000)
		{
			aim_y = point_location[4][1];
			aim_x = point_location[4][0]-30;
		}
		else time_control = 0;
//	
	
	
	if(now_x ==0 &&now_y == 0 )
		moto_driver(0,0);
	else
	{
	PID_calculate(&locaPID_x,(float)aim_x,(float)now_x,&Espeed_x);
	
	PID_calculate(&speedPID_x,Espeed_x,speed_x,(float*)&duty_x);
	
	PID_calculate(&locaPID_y,(float)aim_y,(float)now_y,&Espeed_y);
	
	PID_calculate(&speedPID_y,Espeed_y,speed_y,(float*)&duty_y);
	
	
	
	moto_driver(duty_x,duty_y);
	
	}

	

}
void mode5(void)
{

}
void mode6(void)
{

}
void mode7(void)
{
	const float priod = 600;//1250.0;  //单摆周期(毫秒)
	static uint32_t MoveTimeCnt = 0;
	float A = 30.0;
	float phase = 0.0;
	float Normalization = 0.0;
	float Omega = 0.0;
	
	MoveTimeCnt += 5;							 //每5ms运算1次
	Normalization = (float)MoveTimeCnt / priod;	 //对单摆周期归一化
	Omega = 2.0*3.14159*Normalization;			 //对2π进行归一化处理				
	  
	phase = 3.141592*1.4/2.0;		 //逆时针旋转相位差90° 

	
	aim_y = A*sin(Omega) + point_location[4][1];		//计算出X方向当前摆角
	aim_x = A*cos(Omega) + point_location[4][0];
	//aim_x = A*sin(Omega+phase)+point_location[4][0]; 	//计算出Y方向当前摆角
	
	
	PID_calculate(&locaPID_x,(float)aim_x,(float)now_x,&Espeed_x);
	
	PID_calculate(&speedPID_x,Espeed_x,speed_x,(float*)&duty_x);
	
	PID_calculate(&locaPID_y,(float)aim_y,(float)now_y,&Espeed_y);
	
	PID_calculate(&speedPID_y,Espeed_y,speed_y,(float*)&duty_y);
	
	
	
	moto_driver(duty_x,duty_y);
	

}
void mode8(void)
{

}


void moto_driver(float duty_x,float duty_y)
{
	s16 moto_x,moto_y;
	moto_x = Midell_x - duty_x;//lowestDuty_x-(lowestDuty_x - HighestDuty_x)*((420.0+duty_x)/1000.0);
	moto_y = Midell_y + duty_y;//lowestDuty_y+(HighestDuty_y - lowestDuty_y)*((400.0+duty_y)/1000.0);

	
	moto_x = LIMIT(moto_x,HighestDuty_x,lowestDuty_x);
	moto_y = LIMIT(moto_y,lowestDuty_y,HighestDuty_y);
		
	
	TIM_SetCompare1(TIM3,(int)moto_x);	
	TIM_SetCompare2(TIM3,(int)moto_y);	
	
}
s16 LIMIT(s16 a,s16 min,s16 max)
{
	if(a<min)
		return min;
	else if(a>max)
		return max;
	else
		return a;
}
void moto_io_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOD, ENABLE);	 //使能PA,PD端口时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;				 //LED0-->PA.8 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIOA.8
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;				 //LED0-->PA.8 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(GPIOD, &GPIO_InitStructure);					 //根据设定参数初始化GPIOA.8
	
	
	GPIO_SetBits(GPIOD,GPIO_Pin_2);						 //PA.8 输出高
	GPIO_SetBits(GPIOB,GPIO_Pin_4);						 //PA.8 输出高
	GPIO_ResetBits(GPIOB,GPIO_Pin_3|GPIO_Pin_5);						 //PA.8 输出高
	

}


void clear(char *string,u16 leng)
{
	for(leng--;leng>0;leng--)
	string[leng] = 0; 
}







u8 read_data(void)
{
	u8 t,len = 0,data_flag = 0;
	u16 x_buf = 0,y_buf = 0;
	if(USART_RX_STA&0x8000)
		{
			len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
			if(USART_RX_BUF[0] == 0x5b && USART_RX_BUF[len-1] == 0x5d)
			{
				for(t=1;t<len-1;t++)
				{
					if((USART_RX_BUF[t]&0xf0) == 0x30 && data_flag == 0)
					{
						x_buf*=10;
						x_buf += (USART_RX_BUF[t]-0x30);
					}
					else if((USART_RX_BUF[t]&0xf0) == 0x30 && data_flag == 1)
					{
						y_buf*=10;
						y_buf += (USART_RX_BUF[t]-0x30);
					}
					else if(USART_RX_BUF[t] == 0x2c && USART_RX_BUF[t+1] == 0x20 && data_flag == 0)
					{
						data_flag = 1;
					}
				}
				
				now_x = x_buf;
				now_y = y_buf;
				
				if(((now_x - last_x)>50 || (now_x - last_x)<-50 || (now_y  - last_y)>50 || (now_y  - last_y)<-50) && start_flag )
				{
					now_x = last_x;
					now_y = last_y;
				}
				else
				{		
					X_speedbuf[2] = X_speedbuf[1];
					X_speedbuf[1] = X_speedbuf[0];
					X_speedbuf[0] = now_x - last_x;
					
					Y_speedbuf[2] = Y_speedbuf[1];
					Y_speedbuf[1] = Y_speedbuf[0];
					Y_speedbuf[0] = now_y  - last_y;
					
					speed_x = (float)(X_speedbuf[0] + X_speedbuf[1] + X_speedbuf[2])/3;
					speed_y = (float)(Y_speedbuf[0] + Y_speedbuf[1] + Y_speedbuf[2])/3;
					
					last_y=now_y;
					last_x=now_x;
				}
				
				
			}
			USART_RX_STA = 0;
			USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
			return 0;//读取成功
		}
		else return 1; //读取失败
		
}





