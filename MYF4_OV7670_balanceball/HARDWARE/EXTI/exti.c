#include "exti.h"
#include "delay.h" 
#include "led.h" 
#include "key.h"
#include "ov7670.h"
//#include "ov7670cfg.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//�ⲿ�ж� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/4
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 


u8 ov_sta;	//֡�жϱ��
 //�ⲿ�ж�5~9�������
void EXTI9_5_IRQHandler(void)
{		 		
	if(EXTI_GetITStatus(EXTI_Line5)==SET)	//��8�ߵ��ж�
	{   
		if(ov_sta == 1)
		{
			OV7670_WRST=0;	//��λдָ��		  		 
			//OV7670_WRST=1;	
			OV7670_WREN=0;	//����д��FIFO 	
			ov_sta++;		//֡�жϼ�1 			
		}
		else if(!ov_sta)
		{
			OV7670_WRST=0;	//��λдָ��		  		 
			OV7670_WRST=1;	
			OV7670_WREN=1;	//����д��FIFO 	 
			ov_sta++;		//֡�жϼ�1 
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line5);  //���EXTI8��·����λ						  
} 
//�ⲿ�ж�9��ʼ��
void EXTI6_Init(void)
{												  
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
	
 
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOG, EXTI_PinSource5);//PE6 ���ӵ��ж���2
	
	 EXTI_InitStructure.EXTI_Line = EXTI_Line5;//LINE6
	 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
	 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //�����ش��� 
	 EXTI_InitStructure.EXTI_LineCmd = ENABLE;//ʹ��LINE6
	 EXTI_Init(&EXTI_InitStructure);//����
	
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//�ⲿ�ж�0
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//��ռ���ȼ�0
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//�����ȼ�2
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
	  NVIC_Init(&NVIC_InitStructure);//����
	
}









