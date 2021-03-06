#include "led.h"
#include "delay.h"
#include "usart.h"	 
#include "moto_control.h"
#include "sys.h"
#include "pwm.h"
#include "pid.h"
#include "oled.h"

//ALIENTEK Mini STM32开发板范例代码8
//PWM输出实验   
//技术支持：www.openedv.com
//广州市星翼电子科技有限公司
 int main(void)
 {	
	 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	delay_init();	    	 //延时函数初始化	  
	uart_init(115200);	 //串口初始化为9600
	LED_Init();		  	//初始化与LED连接的硬件接口
	OLED_Init();
	OLED_Display_On();
	moto_io_init();
	 
	 
//	Pid_set(&locaPID_x,0.23,0.0015,0.58);//0.15
//	Pid_set(&speedPID_x,2.35,0.15,2 );//2.7,0,6
//	Pid_set(&locaPID_y,0.23,0.0015,0.58);	  //0.6
//	Pid_set(&speedPID_y,2.5,0.1,2.5 );//6
	 
	 
	 
	 
	Pid_set(&locaPID_x,0.038,0.00018,0);//0.15
	Pid_set(&speedPID_x,88,0,0 );//2.7,0,6
	 
	 Pid_set(&locaPID_y,0.04,0.0002,0);//0.04,0,0
	Pid_set(&speedPID_y,80,0,0);//6
	 //恩毅参数调好了 你试一下各个题吧
	 //你原来的参数在桌面 PID.TXT里
	TIM3_PWM_Init(19999,71);//720分频，50HZ 周期20ms
	TIM5_Int_Init(12499,71);
  
   	while(1)
	{
		;
	} 
}

