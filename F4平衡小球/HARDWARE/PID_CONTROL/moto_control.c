#include "moto_control.h"
#include "usart.h"	 
#include "usart2.h"
#include "pid.h"
#include "led.h"
#include "lcd.h"
#include "math.h"

//点坐标
u16 point_location[9][2]={{115,72},{169,70},{221,69},{117,125},{169,122},{223,123},{119,178},{171,177},{225,175}};
u16 low = 0,high = 0;
u8 abcd_num[4] = {1,9,3,7};
u32 time_control = 0;
u16 now_y = 0,now_x = 0;
u16 buf_y = 0,buf_x = 0;
u16 last_y = 0,last_x = 0;
int X_speedbuf[3]={0,0,0},Y_speedbuf[3]={0,0,0};
float speed_x = 0.0f,speed_y = 0.0f;
float Espeed_x = 0,Espeed_y = 0;

float aim_y = 0.0f,aim_x = 0.0f,duty_x = 0,duty_y = 0;
char dis[20];
u8 start_flag = 0,mode_flag = 0,mode_change = 0;
void TIM3_control_Init(u32 arr,u32 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟
	
    TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{
		
			if(read_data())
			{
				moto_driver(0,0);
				if(start_flag == 1)
				start_flag = 0;
			}
			else 
			{
				time_control+=5;
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
			sprintf(dis,"loca_x:%4d",now_x);	
			LCD_ShowString(10,32,100,16,16,(u8 *)dis);
			clear(dis,20);
			sprintf(dis,"loca_y:%4d",now_y);	
			LCD_ShowString(120,32,100,16,16,(u8 *)dis);
			clear(dis,20);
			sprintf(dis,"speed_x:%3.1f",speed_x);	
			LCD_ShowString(10,48,100,16,16,(u8 *)dis);
			clear(dis,20);
			sprintf(dis,"speed_y:%3.1f",speed_y);	
			LCD_ShowString(120,48,100,16,16,(u8 *)dis);
			
		
			if(time_control>50000)time_control = 0;
			
			
			LED0=!LED0;
		}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
}


void mode1(void)
{
	
	//	moto_driver(0,0);
	//PID_calculate(y,aim_y,(float)now_y,&aim_x);
	//PID_calculate(x,aim_x,(float)now_x,&moto_dif);
	
	aim_y = point_location[1][1];
	aim_x = point_location[1][0];
	
//	
//	if(now_x ==0 &&now_y == 0 )
//		moto_driver(0,0);
//	else
//	{
	PID_calculate(&locaPID_x,(float)aim_x,(float)now_x,&Espeed_x);
	
	PID_calculate(&speedPID_x,Espeed_x,speed_x,(float*)&duty_x);
	
	PID_calculate(&locaPID_y,(float)aim_y,(float)now_y,&Espeed_y);
	
	PID_calculate(&speedPID_y,Espeed_y,speed_y,(float*)&duty_y);
	
	
//	
	moto_driver(duty_x,duty_y);
//	//moto_driver(500,500);
//	}

}


void mode2(void)
{
		
	aim_y = point_location[4][1];
	aim_x = point_location[4][0];
	
	
	
	PID_calculate(&locaPID_x,(float)aim_x,(float)now_x,&Espeed_x);
	
	PID_calculate(&speedPID_x,Espeed_x,speed_x,(float*)&duty_x);
	
	PID_calculate(&locaPID_y,(float)aim_y,(float)now_y,&Espeed_y);
	
	PID_calculate(&speedPID_y,Espeed_y,speed_y,(float*)&duty_y);
	
	
	
	moto_driver(duty_x,duty_y);
	


}
void mode3(void)
{
		if(time_control<2000)
		{
			aim_y = point_location[3][1];
			aim_x = point_location[3][0];
		}
		else if(time_control<4000)
		{
			aim_y = point_location[4][1];
			aim_x = point_location[4][0];
		}
		else time_control = 4000;
	
	

	
	
	
//	Pid_set(&locaPID_x,0.01,0.0008,0.07);
//	Pid_set(&speedPID_x,55,0,0 );//2.7,0,6	
//	Pid_set(&locaPID_y,0.015,0,0);//0.04,0,0
//	Pid_set(&speedPID_y,53,0,0);//6

		PID_calculate(&locaPID_x,(float)aim_x,(float)now_x,&Espeed_x);
		
		PID_calculate(&speedPID_x,Espeed_x,speed_x,(float*)&duty_x);
		
		PID_calculate(&locaPID_y,(float)aim_y,(float)now_y,&Espeed_y);
		
		PID_calculate(&speedPID_y,Espeed_y,speed_y,(float*)&duty_y);
		
		
		
		moto_driver(duty_x,duty_y);//*(now_x-point_location[3][0])/(point_location[4][0]-point_location[3][0])
	

}
void mode5(void)
{
	
//	if(now_x>135 && now_x<225 && now_y>75 && now_y<165)
//	{
	
	
	
//	}
//	else 
//	{
	
	
//	
//		if(now_x>point_location[8][0]-30 && now_x<point_location[8][0]+30 && now_y>point_location[8][1]-30 && now_y<point_location[8][1]+30)
//	{
//		aim_y = point_location[4][1];
//		aim_x = point_location[4][0];
////		
//		
//			moto_driver(30,-30);
//		return;
////		//moto_driver(duty_x,duty_y);
////		Pid_set(&locaPID_x,0.001,0.0008,0.009);
////		Pid_set(&speedPID_x,35,0,0 );//2.7,0,6
////	
////	
////		Pid_set(&locaPID_y,0.008,0.0005,0.009);//0.04,0,0
//////		Pid_set(&speedPID_y,39,0,0);//6
////	}
////	else{
//		aim_y = point_location[5][1];
//		aim_x = point_location[5][0];
//	
////	}
//	else
////	{
//		Pid_set(&locaPID_x,0.01,0.0008,0.009);
//	//Pid_set(&locaPID_x,0.0,0.0,0.0);
//		Pid_set(&speedPID_x,70,0,0 );//2.7,0,6
//	
//		Pid_set(&locaPID_y,0.01,0.0005,0.009);//0.04,0,0
//	//Pid_set(&locaPID_y,0.0,0.0,0.0);//0.04,0,0
//		Pid_set(&speedPID_y,62,0,0);//6

//	}
//	
//	}
	
	
		//Pid_set(&locaPID_x,0.015,0.00038,0.001);//0.00035
		//Pid_set(&locaPID_y,0.015,0.0004 ,0);//
		if(time_control<1000)
		{
			path(1,2);
			aim_y = buf_y;
			aim_x = buf_x;
		}
		else if(time_control<2000)
		{
			aim_y = point_location[1][1];
			aim_x = point_location[1][0];
		}
		else if(time_control<3000)
		{
			path(2,6);
			aim_y = buf_y;
			aim_x = buf_x;
		}
		else if(time_control<4000)
		{
			aim_y = point_location[5][1];
			aim_x = point_location[5][0];
		}
		else if(time_control<5000)
		{
			path(6,9);
			aim_y = buf_y;
			aim_x = buf_x;
		}
		else if(time_control<6000)
		{
			aim_y = point_location[8][1];
			aim_x = point_location[8][0];
		}
		else 
		{
			time_control = 6000;
//			moto_driver(10,10);
//			return ;
	
		}
		
//		
//		if(time_control<2000)
//		{
//			aim_y = point_location[4][1]-50;
//			aim_x = point_location[4][0];
//		}
//		else if(time_control<4000)
//		{
//			aim_y = point_location[4][1];
//			aim_x = point_location[4][0]+50;
//		}
//		else if(time_control<6000)
//		{
//			aim_y = point_location[8][1]-30;
//			aim_x = point_location[8][0]-30;
//		}
//		else if(time_control<8000)
//		{
			
//		}
	

	
		PID_calculate(&locaPID_x,(float)aim_x,(float)now_x,&Espeed_x);
		
		PID_calculate(&speedPID_x,Espeed_x,speed_x,(float*)&duty_x);
		
		PID_calculate(&locaPID_y,(float)aim_y,(float)now_y,&Espeed_y);
		
		PID_calculate(&speedPID_y,Espeed_y,speed_y,(float*)&duty_y);
		
	
		//LIMIT(duty_x,1440,1480);
		//LIMIT(duty_y,1500,1530);
		moto_driver(duty_x,duty_y);
	
	
}
void mode4(void)
{
		if(time_control<1500)
		{
			aim_y = point_location[1][1]+25;
			aim_x = point_location[1][0];
		}
		else if(time_control<3000)
		{
			aim_y = point_location[5][1];
			aim_x = point_location[5][0]-25;
		}
		else if(time_control<4500)
		{
			aim_y = point_location[8][1]-10;
			aim_x = point_location[8][0]-10;
		}
		else if(time_control<6000)
		{
			aim_y = point_location[8][1];
			aim_x = point_location[8][0];
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
void path(u16 now,u16 exp)
{
	u16 x_dis = 0,y_dis = 0;
	x_dis= my_abs(point_location[exp-1][0]-point_location[now-1][0]);
	y_dis= my_abs(point_location[exp-1][1]-point_location[now-1][1]);
	
	if(x_dis<65 && y_dis<65)
	{
		buf_x = (point_location[exp-1][0] + point_location[now-1][0])/2;
		buf_y = (point_location[exp-1][1] + point_location[now-1][1])/2;
	}
	else if(exp+now == 10)
	{	
		if(exp == 2 || now == 2)
		{
			buf_x =  point_location[4][0]+25;
			buf_y =  point_location[4][1];
		}
		else if(exp == 4 || now == 4)
		{
			buf_x =  point_location[4][0];
			buf_y =  point_location[4][1]-25;
		}
		else if(exp == 3 || now == 3)
		{
			buf_x =  point_location[4][0]+25;
			buf_y =  point_location[4][1]+25;
		}
		else if(exp == 1 || now == 1)
		{
			buf_x =  point_location[4][0]+25;
			buf_y =  point_location[4][1]-25;
		}
	}
	else if(x_dis<10||y_dis<10)
	{
		buf_x = (point_location[exp-1][0] + point_location[now-1][0] + point_location[4][0])/3;
		buf_y = (point_location[exp-1][1] + point_location[now-1][1] + point_location[4][1])/3;
	
	}
	else 
	{
		buf_x = (point_location[exp-1][0] + point_location[now-1][0])/2;
		buf_y = (point_location[exp-1][1] + point_location[now-1][1])/2;
	
	}
	
	
}
void mode6(void)
{
//	switch(abcd_num[0])
//	{
//		case 1:
//		{
//		};break;
//		case 2:
//		{
//		};break;
//		case 3:
//		{
//		};break;
//		case 4:
//		{
//		};break;
//	}
	Pid_set(&locaPID_x,0.03,0.0001,0);
	Pid_set(&locaPID_y,0.03,0.0003,0);
		if(time_control<2000)
		{
			
			path(abcd_num[0],abcd_num[1]);
			aim_y = buf_y;
			aim_x = buf_x;
		}
		else if(time_control<4000)
		{
			
			
			aim_y = point_location[abcd_num[1]-1][1];
			aim_x = point_location[abcd_num[1]-1][0];
		}
		else if(time_control<6000)
		{
			path(abcd_num[1],abcd_num[2]);
			aim_y = buf_y;
			aim_x = buf_x;
		}
		else if(time_control<8000)
		{
			
			aim_y = point_location[abcd_num[2]-1][1];
			aim_x = point_location[abcd_num[2]-1][0];
		}
		else if(time_control<10000)
		{
			path(abcd_num[2],abcd_num[3]);
			aim_y = buf_y;
			aim_x = buf_x;
		}
		else if(time_control<12000)
		{
			aim_y = point_location[abcd_num[3]-1][1];
			aim_x = point_location[abcd_num[3]-1][0];
		}
		else time_control = 12000;
	
	
	
	PID_calculate(&locaPID_x,(float)aim_x,(float)now_x,&Espeed_x);
	
	PID_calculate(&speedPID_x,Espeed_x,speed_x,(float*)&duty_x);
	
	PID_calculate(&locaPID_y,(float)aim_y,(float)now_y,&Espeed_y);
	
	PID_calculate(&speedPID_y,Espeed_y,speed_y,(float*)&duty_y);
	
	
	
	moto_driver(duty_x,duty_y);
	
}
void mode7(void)
{
	const float priod = 700;//1250.0;  //单摆周期(毫秒)
	static uint32_t MoveTimeCnt = 0;
	float A = 20.0;
	//float phase = 0.0;
	float Normalization = 0.0;
	float Omega = 0.0;
	
	MoveTimeCnt += 5;							 //每5ms运算1次
	Normalization = (float)MoveTimeCnt / priod;	 //对单摆周期归一化
	Omega = 2.0*3.14159*Normalization;			 //对2π进行归一化处理				
	  
	//phase = 3.141592*1.4/2.0;		 //逆时针旋转相位差90° 
	if(time_control<3500)
	{
		aim_y = A*sin(Omega) + point_location[4][1];		//计算出X方向当前摆角
		aim_x = A*cos(Omega) + point_location[4][0];
		//aim_x = A*sin(Omega+phase)+point_location[4][0]; 	//计算出Y方向当前摆角
	}
	else if(time_control<6000)
	{
			aim_y = point_location[8][1]-20;
			aim_x = point_location[8][0]-20;
	}
		else if(time_control<7000)
	{
			aim_y = point_location[8][1];
			aim_x = point_location[8][0];
	}
	
	PID_calculate(&locaPID_x,(float)aim_x,(float)now_x,&Espeed_x);
	
	PID_calculate(&speedPID_x,Espeed_x,speed_x,(float*)&duty_x);
	
	PID_calculate(&locaPID_y,(float)aim_y,(float)now_y,&Espeed_y);
	
	PID_calculate(&speedPID_y,Espeed_y,speed_y,(float*)&duty_y);
	
	
	
	moto_driver(duty_x,duty_y);
	

}
void mode8(void)
{
		Pid_set(&locaPID_x,0.03,0.0001,0);
		Pid_set(&locaPID_y,0.03,0.0003,0);
		if(time_control<1000)
		{
			
			path(5,8);
			aim_y = buf_y;
			aim_x = buf_x;
		}
		else if(time_control<2000)
		{
			path(5,4);
			aim_y = buf_y;
			aim_x = buf_x;
		}
		else if(time_control<3000)
		{
			path(5,2);
			aim_y = buf_y;
			aim_x = buf_x;
		}
		else if(time_control<4000)
		{
			path(5,6);
			aim_y = buf_y;
			aim_x = buf_x;
		}
		else if(time_control<5000)
		{
			aim_y = point_location[5][1];
			aim_x = point_location[5][0];
		}
		else if(time_control<6000)
		{
			aim_y = point_location[2][1];
			aim_x = point_location[2][0];
		}
		else if(time_control<7000)
		{
			aim_y = point_location[1][1];
			aim_x = point_location[1][0];
		}
		else if(time_control<8000)
		{
			aim_y = point_location[0][1];
			aim_x = point_location[0][0];
		}
		else if(time_control<9000)
		{
			aim_y = point_location[3][1];
			aim_x = point_location[3][0];
		}
		else if(time_control<10000)
		{
			aim_y = point_location[6][1];
			aim_x = point_location[6][0];
		}
		else if(time_control<11000)
		{
			aim_y = point_location[7][1];
			aim_x = point_location[7][0];
		}
		else if(time_control<12000)
		{
			aim_y = point_location[8][1];
			aim_x = point_location[8][0];
		}
		else if(time_control<13000)
		{
			aim_y = point_location[4][1];
			aim_x = point_location[4][0];
		}
		else time_control = 13000;
	
	
	PID_calculate(&locaPID_x,(float)aim_x,(float)now_x,&Espeed_x);
	
	PID_calculate(&speedPID_x,Espeed_x,speed_x,(float*)&duty_x);
	
	PID_calculate(&locaPID_y,(float)aim_y,(float)now_y,&Espeed_y);
	
	PID_calculate(&speedPID_y,Espeed_y,speed_y,(float*)&duty_y);
	
	
	
	moto_driver(duty_x,duty_y);
	

}


void moto_driver(float duty_x,float duty_y)
{
	s16 moto_x,moto_y;
	moto_x = Midell_x - duty_x;//lowestDuty_x-(lowestDuty_x - HighestDuty_x)*((420.0+duty_x)/1000.0);
	moto_y = Midell_y + duty_y;//lowestDuty_y+(HighestDuty_y - lowestDuty_y)*((400.0+duty_y)/1000.0);

	
	moto_x = LIMIT(moto_x,HighestDuty_x,lowestDuty_x);
	moto_y = LIMIT(moto_y,lowestDuty_y,HighestDuty_y);
		
	
	TIM_SetCompare1(TIM4,(int)moto_x);	
	TIM_SetCompare2(TIM4,(int)moto_y);	
	
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


void clear(char *string,u16 leng)
{
	for(leng--;leng>0;leng--)
	string[leng] = 0; 
}







u8 read_data(void)
{
	u8 t,len = 0,data_flag = 0;
	u16 x_buf = 0,y_buf = 0;
	if(USART2_RX_STA&0x8000)
		{
			len=USART2_RX_STA&0x3fff;//得到此次接收到的数据长度
			if(USART2_RX_BUF[0] == 0x5b && USART2_RX_BUF[len-1] == 0x5d)
			{
				for(t=1;t<len-1;t++)
				{
					if((USART2_RX_BUF[t]&0xf0) == 0x30 && data_flag == 0)
					{
						x_buf*=10;
						x_buf += (USART2_RX_BUF[t]-0x30);
					}
					else if((USART2_RX_BUF[t]&0xf0) == 0x30 && data_flag == 1)
					{
						y_buf*=10;
						y_buf += (USART2_RX_BUF[t]-0x30);
					}
					else if(USART2_RX_BUF[t] == 0x2c && USART2_RX_BUF[t+1] == 0x20 && data_flag == 0)
					{
						data_flag = 1;
					}
				}
				
				now_x = x_buf;
				now_y = y_buf;
				
				if(((now_x - last_x)>20 || (now_x - last_x)<-20 || (now_y  - last_y)>20 || (now_y  - last_y)<-20) && start_flag )
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
			USART2_RX_STA = 0;
			USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启相关中断
			return 0;//读取成功
		}
		else return 1; //读取失败
		
}

u16 my_abs(int num)
{
	return num>0?num:(-1)*num;
}



