#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "key.h"
#include "ov7670.h"
#include "exti.h"
#include "oled.h"
#include "timer.h"
#include "PIC_process.h"
#include "Figure.h"
#include "olsm.h"
u8 chuli_flag = 1;

char dis[20];
extern u8 ov_sta;	//在exit.c里 面定义
void camera_refresh(void);
void data_calculate(void);
void clear(char *string,u16 leng);


int main(void)
{ 
	

	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);      //初始化延时函数
	uart_init(115200);		//初始化串口波特率为115200	
	LED_Init();					  //初始化LED
	KEY_Init();					//初始化按键
	TIM3_Int_Init(500-1,8400-1);	//定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数5000次为500ms     
 	LCD_Init();           //初始化LCD FSMC接口
	//OLED_Init();
	POINT_COLOR=RED;			//设置字体为红色 
	LCD_ShowString(30,50,200,16,16,"ELITE STM32F103 ^_^");	
	LCD_ShowString(30,70,200,16,16,"OV7670 TEST");	
	LCD_ShowString(30,90,200,16,16,"ATOM@ALIENTEK");
	LCD_ShowString(30,110,200,16,16,"2015/1/18"); 
	LCD_ShowString(30,130,200,16,16,"KEY0:Light Mode");
	LCD_ShowString(30,150,200,16,16,"KEY1:Saturation");
	LCD_ShowString(30,170,200,16,16,"KEY_UP:Contrast");
	LCD_ShowString(30,190,200,16,16,"TPAD:Effects");	 
  	LCD_ShowString(30,210,200,16,16,"OV7670 Init...");	  
	while(OV7670_Init())//初始化OV7670
	{
		LCD_ShowString(30,210,200,16,16,"OV7670 Error!!");
		delay_ms(200);
	    LCD_Fill(30,210,239,246,WHITE);
		delay_ms(200);
	}
 	LCD_ShowString(30,210,200,16,16,"OV7670 Init OK");
	delay_ms(1500);	 	   
	OV7670_Light_Mode(0);
	OV7670_Color_Saturation(2);
	OV7670_Contrast(2);
 	OV7670_Special_Effects(0);	 
	EXTI6_Init();						//使能定时器捕获
	OV7670_Window_Set(12,176,240,320);	//设置窗口	  
  	OV7670_CS=0;			
	LCD_Clear(BLACK);	
	LCD_Scan_Dir(U2D_L2R); 	

  	while(1) 
	{		 
			
		
		data_calculate();
		
	} 
}

//定时器3中断服务函数
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{
		camera_refresh();//更新显示
		
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
}

	
void data_calculate(void)
{
   u32 j = 0;
//	u8 hh;
	float k = 0,d = 0;
	PointType centrol;
	if(!chuli_flag)
	{
		
		//picture_filter();
		//get_blackline();
		//blackline_filter();
		//get_blackmid();	
		//ordinary_lsm(&k,&d);
		
		//Figure_edge();
		Figure_FindSingleCircle(&centrol);
		//Figure_FindCircles(&centrol);
		
		if(lcddev.id==0X1963)LCD_Set_Window((lcddev.width-240)/2,(lcddev.height-320)/2,240,320);//将显示区域设置到屏幕中央
		else if(lcddev.id==0X5510||lcddev.id==0X5310)LCD_Set_Window((lcddev.width-320)/2,(lcddev.height-240)/2,320,240);//将显示区域设置到屏幕中央
		LCD_SetCursor((lcddev.width-320)/2,(lcddev.height-240)/2);	
		LCD_WriteRAM_Prepare();     //开始写入GRAM	
		/*
		for(j=0;j<76800;j++)
		{
			if(image[j/640][(j%320)/2]==1)
				LCD->LCD_RAM = 0;
			else
				LCD->LCD_RAM = 65535;
		}
		LCD_Draw_Circle(centrol.x*2,centrol.y*2,5);
		*/
		//Figure_Clean();
//		POINT_COLOR=RED;			//设置字体为红色 
//		for(hh = 0;hh<HEIGHT;hh++)
//		{
////			LCD_DrawPoint(blackline[hh][0]*2,hh*2);
////			LCD_DrawPoint(blackline[hh][1]*2,hh*2);
////			LCD_DrawPoint(blackline[hh][2]*2,hh*2);
//		}

		
		//LCD_Scan_Dir(DFT_SCAN_DIR);	//恢复默认扫描方向 
	
//		if(whiteline<50)
//		{
//			k *= 100;
//			k-=2;
//			d-=61;
//		}
//		else 
//		{
//			k = 0;
//			d = 0;
//		}
//		
     	printf("%c%c%c%c%c%c%c%c",0xaa,0xff,centrol.x>>8,centrol.x&0xff,centrol.y>>8,centrol.y&0xff,0x0d,0x0a);	
//		LCD_ShowString(0,24,320,24,24,"K:");	
//		LCD_ShowString(0,48,320,24,24,"D:");
//		clear(dis,20);
//		sprintf(dis,"%6.2f",k);	
//		LCD_ShowString(48,24,320,24,24,dis);	
//		clear(dis,20);
//		sprintf(dis,"%6.2f",d);	
//		LCD_ShowString(48,48,320,24,24,dis);
		
		
	
		chuli_flag = 1;
	}


}


void camera_refresh(void)
{
	u32 j;
 	u16 color,h = 0,l = 0;
	
	if(ov_sta == 2)//有帧中断更新？
	{
		//LCD_Scan_Dir(U2D_L2R);		//从上到下,从左到右  
		if(lcddev.id==0X1963)LCD_Set_Window((lcddev.width-240)/2,(lcddev.height-320)/2,240,320);//将显示区域设置到屏幕中央
		else if(lcddev.id==0X5510||lcddev.id==0X5310)LCD_Set_Window((lcddev.width-320)/2,(lcddev.height-240)/2,320,240);//将显示区域设置到屏幕中央
		LCD_SetCursor((lcddev.width-320)/2,(lcddev.height-240)/2);	
		LCD_WriteRAM_Prepare();     //开始写入GRAM	
		OV7670_RRST=0;				//开始复位读指针 
		OV7670_RCK_L;
		OV7670_RCK_H;
		OV7670_RCK_L;
		OV7670_RRST=1;				//复位读指针结束 
		OV7670_RCK_H;
		for(j=0;j<76800;j++)
		{
			OV7670_RCK_L;
			color=GPIOF->IDR&0XFF;	//读数据
			OV7670_RCK_H; 
			color<<=8;  
			OV7670_RCK_L;
			color|=GPIOF->IDR&0XFF;	//读数据
			OV7670_RCK_H; 
			l = j%320;
			h = j/320;
		
			if(color>32000)//白
			{
				color=65535;
				if(chuli_flag)
				{
					if(!(l%2) && !(h%2))
					image[h/2][l/2] = 0;
				}
			}
			else //黑
			{
				color = 0;
				if(chuli_flag)
				{
					if(!(l%2) && !(h%2))
					{
						image[h/2][l/2] = 1;
					}
				}
			}
			//LCD->LCD_RAM=color; 	
			   
		}
		if(chuli_flag)
		chuli_flag = 0;
		
		//LCD_Scan_Dir(DFT_SCAN_DIR);	//恢复默认扫描方向 
 		ov_sta=0;					//清零帧中断标记
	} 
}	   

void clear(char *string,u16 leng)
{
	for(leng--;leng>0;leng--)
	string[leng] = 0; 
}



