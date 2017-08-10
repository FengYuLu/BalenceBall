#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "usart2.h"
#include "led.h"
#include "lcd.h"


u16 x,y;
char dis[20];

void clear(char *string,u16 leng);
void read_data(u16 *x_data,u16 *y_data);


int main(void)
{ 

	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);      //初始化延时函数
	uart_init(115200);		//初始化串口波特率为115200
	uart2_init(115200);
	LED_Init();					  //初始化LED
 	LCD_Init();           //初始化LCD FSMC接口
	POINT_COLOR=RED;      //画笔颜色：红色		 	
	LCD_ShowString(0,0,320,24,24,"openmv_f4_Test By Mr.Xu ^_^ ");	
	

	
  	while(1) 
	{		
		
		read_data(&x,&y);
		LCD_ShowString(0,24,320,24,24,"X:");	
		LCD_ShowString(0,48,320,24,24,"Y:");
		clear(dis,20);
		sprintf(dis,"%d",x);	
		LCD_ShowString(48,24,320,24,24,dis);	
		clear(dis,20);
		sprintf(dis,"%d",y);	
		LCD_ShowString(48,48,320,24,24,dis);
		
		LED0=!LED0;	 
		delay_ms(100);	
	} 
}
void clear(char *string,u16 leng)
{
	for(leng--;leng>0;leng--)
	string[leng] = 0; 
}

void read_data(u16 *x_data,u16 *y_data)
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
						data_flag++;
					}
				}
				
				*x_data = x_buf;
				*y_data = y_buf;
			}
			
			USART2_RX_STA = 0;
		}
}



