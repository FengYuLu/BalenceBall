#ifndef __MOTO_CONTROL_H
#define __MOTO_CONTROL_H	
#include "sys.h" 
#include "delay.h"

#define HighestDuty_x	1700//1.7ms
#define lowestDuty_x	1000//

#define HighestDuty_y	1000//
#define lowestDuty_y	1700//

//
//			|
//			|	
//	|		x
//	|------------
//	|	|
//		|	
//		y
//
//



extern u8 start_flag;
extern int now_y,now_x;
void data_read(void);
void TIM3_Int_Init(u16 arr,u16 psc);
void moto_driver(float duty_x,float duty_y);
void moto_io_init(void);
void mode1(void);
void clear(char *string,u16 leng);
void zhuanwan(void);
s16 LIMIT(s16 a,s16 min,s16 max);
#endif 


