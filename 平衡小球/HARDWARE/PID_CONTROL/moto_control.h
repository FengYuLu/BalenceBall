#ifndef __MOTO_CONTROL_H
#define __MOTO_CONTROL_H	
#include "sys.h" 
#include "delay.h"

#define HighestDuty_y	1636//1.7ms
#define lowestDuty_y	1340//
#define Midell_y		1500

#define HighestDuty_x	1302//
#define lowestDuty_x	1602//
#define Midell_x		1461

//
//			|
//			|	
//	|		x---PA0
//	|------------
//	|	|
//		|	
//		y-------PA1
//
//



extern u8 start_flag;
extern u16 now_y,now_x;
void TIM5_Int_Init(u16 arr,u16 psc);
void moto_driver(float duty_x,float duty_y);
void moto_io_init(void);
void mode1(void);
void mode2(void);
void mode3(void);
void mode4(void);
void mode5(void);
void mode6(void);
void mode7(void);
void mode8(void);
void clear(char *string,u16 leng);

s16 LIMIT(s16 a,s16 min,s16 max);
u8 read_data(void);



#endif 


