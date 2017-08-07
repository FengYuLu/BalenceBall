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
	 
	 
	 //
	 //
//	Pid_set(&y,2,0,0);
//	Pid_set(&x,8,0,0);
	TIM5_PWM_Init(19999,71);//720分频，50HZ 周期20ms
	TIM3_Int_Init(4999,71);
  
   	while(1)
	{
		;
	} 
}

