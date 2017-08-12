#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "usart2.h"
#include "led.h"
#include "lcd.h"
#include "pwm.h"
#include "moto_control.h"
#include "pid.h"
#include "key.h"
#include "touch.h" 

void user_guide_init(void);
void value_check(void);
u8 returnval(u16 x,u16 y);
u8 touch_val = 0,select_num = 0,press_flag = 0,check_num = 0;

int main(void)
{ 

	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);      //初始化延时函数
	uart_init(115200);		//初始化串口波特率为115200
	uart2_init(115200);
	LED_Init();					  //初始化LED
 	LCD_Init();           //初始化LCD FSMC接口
	tp_dev.init();				//触摸屏初始化
	user_guide_init();	 
	
	
	
	

	//Pid_set(&locaPID_x,0.028,0,0);//0.15
	
	Pid_set(&locaPID_x,0.038,0.00025,0);//0.00035
	Pid_set(&speedPID_x,80,0,0 );//2.7,0,6

	//Pid_set(&locaPID_y,0.028,0,0);
	
	Pid_set(&locaPID_y,0.038,0.0005,0);//
	Pid_set(&speedPID_y,80,0,0);//6	
	
	
	
	
	
	
//	Pid_set(&locaPID_x,0.02,0.0008,0.009);
//  Pid_set(&locaPID_x,0,0,0);

	
	
	//Pid_set(&locaPID_y,0.04,0.0002,0);//0.04,0,0
	//Pid_set(&speedPID_y,80,0,0);//6

	
//	Pid_set(&speedPID_x,35,0,0 );//2.7,0,6
//		
//	Pid_set(&locaPID_y,0,0,0);//0.04,0,


	TIM4_PWM_Init(19999,84-1); //50HZ 周期20ms
	TIM3_control_Init(12499,84-1);
	
  	while(1) 
	{		
		tp_dev.scan(0); 		 
		if(tp_dev.sta&TP_PRES_DOWN)			//触摸屏被按下
		{	
		 	if(tp_dev.x[0]<lcddev.width && tp_dev.y[0]<lcddev.height && press_flag == 0)
			{	
				touch_val = returnval(tp_dev.x[0],tp_dev.y[0]);	
				value_check();
			}
			press_flag = 1;
		}
		else 
		{
			press_flag = 0;
			delay_ms(10);	//没有按键按下的时候 	
		}		
	} 
}


void user_guide_init(void)
{
//	LCD_Clear(WHITE);//清屏  
	POINT_COLOR=RED;      //画笔颜色：红色	
	LCD_ShowString(50,0,240,16,16,"Balance_Ball GUIDE");	
	LCD_ShowString(50,16,240,16,16,"Please select mode!");	
//	LCD_ShowString(20,32,200,16,16,"X:");	
//	LCD_ShowString(140,32,200,16,16,"Y:");	
//	LCD_ShowString(20,48,200,16,16,"X_speed:");
//	LCD_ShowString(140,48,200,16,16,"Y_speed:");
	LCD_ShowString(20,70,200,16,16,"NOW_mode:");		
	LCD_Fill(0,160,240,160,BLUE);		 
	LCD_Fill(0,200,240,200,BLUE);	
	LCD_Fill(0,240,240,240,BLUE);	
	LCD_Fill(0,280,240,280,BLUE);	
	LCD_Fill(0,320,240,320,BLUE);	
	LCD_Fill(0,160,0,320,BLUE);	
	LCD_Fill(40,160,40,320,BLUE);	
	LCD_Fill(80,160,80,320,BLUE);	
	LCD_Fill(120,160,120,320,BLUE);	
	LCD_Fill(160,160,160,320,BLUE);	
	LCD_Fill(200,160,200,320,BLUE);	
	LCD_Fill(240,160,240,320,BLUE);	
	LCD_ShowString(15,167,200,24,24,"7");
	LCD_ShowString(15,207,200,24,24,"4");		
	LCD_ShowString(15,247,200,24,24,"1");		
	LCD_ShowString(55,167,200,24,24,"8");
	LCD_ShowString(55,207,200,24,24,"5");		
	LCD_ShowString(55,247,200,24,24,"2");		
//	LCD_ShowString(55,287,200,24,24,"0");	
	LCD_ShowString(95,167,200,24,24,"9");
	LCD_ShowString(95,207,200,24,24,"6");		
	LCD_ShowString(95,247,200,24,24,"3");		
	LCD_ShowString(130,167,200,24,24,"M1");		
	LCD_ShowString(170,167,200,24,24,"M2");	
	LCD_ShowString(210,167,200,24,24,"M3");
	LCD_ShowString(130,207,200,24,24,"M4");	
	LCD_ShowString(170,207,200,24,24,"M5");
	LCD_ShowString(210,207,200,24,24,"M6");
	LCD_ShowString(130,247,200,24,24,"M7");
	LCD_ShowString(170,247,200,24,24,"M8");
	LCD_ShowString(210,247,200,24,24,"RS");
	LCD_ShowString(130,287,200,24,24,"CK");
	
}


u8 returnval(u16 x,u16 y)
{
	if((x>0&&x<40)&&(y>160&&y<200)) return 7;
	else if((x>0&&x<40)&&(y>200&&y<240)) return 4;
	else if((x>0&&x<40)&&(y>240&&y<280)) return 1;
	else if((x>40&&x<80)&&(y>160&&y<200)) return 8;
	else if((x>40&&x<80)&&(y>200&&y<240)) return 5;
	else if((x>40&&x<80)&&(y>240&&y<280)) return 2;
//	else if((x>40&&x<80)&&(y>280&&y<320)) return 0;
	else if((x>80&&x<120)&&(y>160&&y<200)) return 9;
	else if((x>80&&x<120)&&(y>200&&y<240)) return 6;
	else if((x>80&&x<120)&&(y>240&&y<280)) return 3;
	else if((x>120&&x<160)&&(y>160&&y<200)) return 11;//M1
	else if((x>160&&x<200)&&(y>160&&y<200)) return 22;//M2
	else if((x>200&&x<240)&&(y>160&&y<200)) return 33;//M3
	else if((x>120&&x<160)&&(y>200&&y<240)) return 44;//M4
	else if((x>160&&x<200)&&(y>200&&y<240)) return 55;//M5
	else if((x>200&&x<240)&&(y>200&&y<240)) return 66;//M6
	else if((x>120&&x<160)&&(y>240&&y<280)) return 77;//M7
	else if((x>160&&x<200)&&(y>240&&y<280)) return 88;//M8
	else if((x>200&&x<240)&&(y>240&&y<280)) return 99;//RST
	else if((x>120&&x<160)&&(y>280&&y<320)) return 110;//check
	else return 0;
	
}

void value_check(void)
{
	switch(touch_val)
	{
		case 11:
			if(mode_flag != 1)
			{
				mode_flag = 1;
				mode_change = 1;
				LCD_ShowString(100,70,200,16,16,"M1");		
			}
			break;
		case 22:
			if(mode_flag != 2)
			{
				mode_flag = 2;
				mode_change = 1;
				LCD_ShowString(100,70,200,16,16,"M2");		
			}
			break;
		case 33:
			if(mode_flag != 3)
			{
				mode_flag = 3;
				mode_change = 1;
				LCD_ShowString(100,70,200,16,16,"M3");		
			}
			break;
		case 44:
			if(mode_flag != 4)
			{
				mode_flag = 4;
				mode_change = 1;
				LCD_ShowString(100,70,200,16,16,"M4");		
			}
			break;
		case 55:
			if(mode_flag != 5)
			{
				mode_flag = 5;
				mode_change = 1;
				LCD_ShowString(100,70,200,16,16,"M5");		
			}
			break;
		case 66:
			if(mode_flag != 6)
			{
				mode_flag = 6;
				mode_change = 1;
				LCD_ShowString(100,70,200,16,16,"M6");		
			}
			break;
		case 77:
			if(mode_flag != 7)
			{
				mode_flag = 7;
				mode_change = 1;
				LCD_ShowString(100,70,200,16,16,"M7");		
			}
			break;
		case 88:
			if(mode_flag != 8)
			{
				mode_flag = 8;
				mode_change = 1;
				LCD_ShowString(100,70,200,16,16,"M8");		
			}
			break;
		case 99:
			if(mode_flag != 0)
			{
				mode_flag = 0;
				select_num = 0;
				check_num = 0;
				mode_change = 1;
				LCD_Fill(20,90,200,106,WHITE);
				LCD_ShowString(100,70,200,16,16,"  ");		
			}
			break;
			
	}
	
	if(mode_flag == 6 && select_num < 4 && touch_val>0 && touch_val <10)
	{
		abcd_num[select_num] = touch_val;
		switch(select_num)
		{
			case 0: 
				clear(dis,20);
				sprintf(dis,"A:%1d",touch_val);	
				LCD_ShowString(20,90,100,16,16,(u8 *)dis);
				break;
			case 1: 
				clear(dis,20);
				sprintf(dis,"B:%1d",touch_val);	
				LCD_ShowString(60,90,100,16,16,(u8 *)dis);
				break;
			case 2: 
				clear(dis,20);
				sprintf(dis,"C:%1d",touch_val);	
				LCD_ShowString(100,90,100,16,16,(u8 *)dis);
				break;
			case 3: 
				clear(dis,20);
				sprintf(dis,"D:%1d",touch_val);	
				LCD_ShowString(140,90,100,16,16,(u8 *)dis);
				break;
		}
		select_num++;
	}
	if(mode_flag == 0 && check_num<9 && touch_val == 110)
	{
		point_location[check_num][0] = now_x;
		point_location[check_num][1] = now_y;
		LCD_ShowxNum(20,120,check_num,3,16,0);
		LCD_ShowxNum(60,120,point_location[check_num][0],3,16,0);
		LCD_ShowxNum(120,120,point_location[check_num][1],3,16,0);
		check_num++;
	}


}



