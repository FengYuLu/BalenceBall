#include "led.h"
#include "delay.h"
#include "usart.h"	 
#include "moto_control.h"
#include "sys.h"
#include "pwm.h"
#include "pid.h"
#include "oled.h"

//ALIENTEK Mini STM32�����巶������8
//PWM���ʵ��   
//����֧�֣�www.openedv.com
//������������ӿƼ����޹�˾
 int main(void)
 {	
	 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// �����ж����ȼ�����2
	delay_init();	    	 //��ʱ������ʼ��	  
	uart_init(115200);	 //���ڳ�ʼ��Ϊ9600
	LED_Init();		  	//��ʼ����LED���ӵ�Ӳ���ӿ�
	OLED_Init();
	OLED_Display_On();
	moto_io_init();
	Pid_set(&Weiyi,2,0,0);
	Pid_set(&Angle,8,0,0);
	TIM5_PWM_Init(9999,0);//����Ƶ��PWMƵ��=72000/(899+1)=80Khz 
	TIM3_Int_Init(4999,71);
  
   	while(1)
	{
		;
	} 
}

