#include "moto_control.h"
#include "usart.h"	 
#include "pid.h"
#include "led.h"
#include "oled.h"

int now_y = 0,now_x = 0;
int last_y = 0,last_x = 0;
float aim_weiyi = 0,aim_angle = 0,moto_dif = 0,moto1 = 0,moto2 = 0;
char dis[20];
u8 start_flag = 0;
void TIM3_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��

	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(  //ʹ�ܻ���ʧ��ָ����TIM�ж�
		TIM3, //TIM2
		TIM_IT_Update ,
		ENABLE  //ʹ��
		);
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx����
							 
}


void TIM3_IRQHandler(void)   //TIM3�ж�
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
		{
			
			data_read();
			mode1();
			clear(dis,20);
			sprintf(dis,"%6d",now_x);	
			OLED_ShowString(80,0,dis);
			clear(dis,20);
			sprintf(dis,"%6d",now_y);	
			OLED_ShowString(80,16,dis);			
			OLED_Refresh_Gram();
			
			
			
			LED0=!LED0;
		}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //���TIMx���жϴ�����λ:TIM �ж�Դ 
}


void mode1(void)
{
	PID_calculate(Weiyi,aim_weiyi,(float)now_y,&aim_angle);
	PID_calculate(Angle,aim_angle,(float)now_x,&moto_dif);
	moto_driver(moto_dif);



}

void moto_driver(float difference)
{
	moto1 = 7000 - difference;
	moto2 = 7320 + difference;

	if(moto1>9999)moto1=9999;
	else if(moto1<1000)moto1=1000;
	if(moto2>9999)moto2=9999;
	else if(moto2<1000)moto2=1000;
	TIM_SetCompare1(TIM5,(int)moto1);	
	TIM_SetCompare2(TIM5,(int)moto2);	
	
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
	

}
	
void data_read(void)
{
		if(USART_RX_STA&0x8000)
		{	
			start_flag=1;
			if(USART_RX_BUF[0]==0xaa && USART_RX_BUF[1]==0xff)
			{
				last_y=now_y;
				last_x=now_x;
					
					
				now_x = ((USART_RX_BUF[2]<<8)|USART_RX_BUF[3]);				   
				now_y = ((USART_RX_BUF[4]<<8)|USART_RX_BUF[5]);	
					
				if(now_x>32767)now_x=0xffff0000|now_x;//ת���ɸ���
				if(now_y>32767)now_y=0xffff0000|now_y;//ת���ɸ���
				
				
				if((now_y - last_y)>50 || (last_y - now_y)>50)
				{
					now_y = last_y;
				}
				//else	now_y = (10*now_y + 90*last_y)/100;
					
				if((now_x - last_x)>50 || (last_x - now_x)>50)
				{
					now_x = last_x;
				}
//				now_x = (10*now_x + 90*last_x)/100;
//				now_y = (10*now_y + 90*last_y)/100;
				
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
