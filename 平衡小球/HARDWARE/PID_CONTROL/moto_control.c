#include "moto_control.h"
#include "usart.h"	 
#include "pid.h"
#include "led.h"
#include "oled.h"

int now_y = 0,now_x = 0;
int last_y = 0,last_x = 0;
int speed_x = 0,speed_y = 0;
float Espeed_x = 0,Espeed_y = 0;
float aim_y = 56.0,aim_x = 71.0,duty_x = 0,duty_y = 0;
char dis[20];
u8 start_flag = 0;
void TIM5_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); //ʱ��ʹ��

	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(  //ʹ�ܻ���ʧ��ָ����TIM�ж�
		TIM5, //TIM2
		TIM_IT_Update ,
		ENABLE  //ʹ��
		);
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	TIM_Cmd(TIM5, ENABLE);  //ʹ��TIMx����
							 
}


void TIM5_IRQHandler(void)   //TIM3�ж�
{
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
		{
			
			data_read();
			mode1();
			clear(dis,20);
			sprintf(dis,"duty_x:%4.2f",duty_x);	
			OLED_ShowString(0,0,(u8 *)dis);
			clear(dis,20);
			sprintf(dis,"duty_y:%4.2f",duty_y);	
			OLED_ShowString(0,2,(u8 *)dis);
			sprintf(dis,"speed_x:%6d",speed_x);	
			OLED_ShowString(0,4,(u8 *)dis);
			sprintf(dis,"speed_y:%6d",speed_y);	
			OLED_ShowString(0,6,(u8 *)dis);
			
			//OLED_Refresh_Gram();
			
			
			
			LED0=!LED0;
		}
	TIM_ClearITPendingBit(TIM5,TIM_IT_Update);  //���TIMx���жϴ�����λ:TIM �ж�Դ 
}


void mode1(void)
{
	//PID_calculate(y,aim_y,(float)now_y,&aim_x);
	//PID_calculate(x,aim_x,(float)now_x,&moto_dif);
	PID_calculate(&locaPID_x,(float)aim_x,(float)now_x,&Espeed_x);
	
	PID_calculate(&speedPID_x,Espeed_x,(float)speed_x,(float*)&duty_x);
	
	PID_calculate(&locaPID_y,(float)aim_y,(float)now_y,&Espeed_y);
	
	PID_calculate(&speedPID_y,Espeed_x,(float)speed_y,(float*)&duty_y);
	
	
	
	moto_driver(duty_x,duty_y);
	//moto_driver(500,500);


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
 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOD, ENABLE);	 //ʹ��PA,PD�˿�ʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;				 //LED0-->PA.8 �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOA.8
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;				 //LED0-->PA.8 �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOD, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOA.8
	
	
	GPIO_SetBits(GPIOD,GPIO_Pin_2);						 //PA.8 �����
	GPIO_SetBits(GPIOB,GPIO_Pin_4);						 //PA.8 �����
	GPIO_ResetBits(GPIOB,GPIO_Pin_3|GPIO_Pin_5);						 //PA.8 �����
	data_read();

}

void data_read(void)
{
		if(USART_RX_STA&0x8000)
		{	
			start_flag=1;
			if(USART_RX_BUF[0]==0xaa && USART_RX_BUF[1]==0xff)
			{
				
					
					
				now_x = ((USART_RX_BUF[2]<<8)|USART_RX_BUF[3]);				   
				now_y = ((USART_RX_BUF[4]<<8)|USART_RX_BUF[5]);	
					
				//if(now_x>32767)now_x=0xffff0000|now_x;//ת���ɸ���
				//if(now_y>32767)now_y=0xffff0000|now_y;//ת���ɸ���
				speed_x = now_x - last_x;
				speed_y =last_y - now_y ;
				
				  
//				sprintf(dis,"loca_x:%5d",now_x);
//				OLED_ShowString(0,0,dis);
//				sprintf(dis,"loca_y:%5d",now_y);
//				OLED_ShowString(0,10,dis);
				
//				if((now_y - last_y)>50 || (last_y - now_y)>50)
//				{
//					now_y = last_y;
//				}
				//else	now_y = (10*now_y + 90*last_y)/100;
					
//				if((now_x - last_x)>50 || (last_x - now_x)>50)
//				{
//					now_x = last_x;
//				}
//				now_x = (10*now_x + 90*last_x)/100;
//				now_y = (10*now_y + 90*last_y)/100;
				last_y=now_y;
				last_x=now_x;
			}				
			
			USART_RX_STA=0;
			
			USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
		}

}
void zhuanwan(void)
{
	
	TIM_SetCompare1(TIM5,0);	
	TIM_SetCompare2(TIM5,7370);	

}
void clear(char *string,u16 leng)
{
	for(leng--;leng>0;leng--)
	string[leng] = 0; 
}
