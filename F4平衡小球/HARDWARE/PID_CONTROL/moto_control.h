#ifndef __MOTO_CONTROL_H
#define __MOTO_CONTROL_H	
#include "sys.h" 
#include "delay.h"

#define HighestDuty_y	1815//1600//1.7ms
#define lowestDuty_y	1215//1430//
#define Midell_y		1500

#define HighestDuty_x	1160//1400//
#define lowestDuty_x	1760//1500//
#define Midell_x		1465

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

extern u16 low,high;
extern char dis[20];
extern u8 start_flag,mode_flag,mode_change,abcd_num[4];
extern u16 now_y,now_x,point_location[9][2];
void TIM3_control_Init(u32 arr,u32 psc);
void moto_driver(float duty_x,float duty_y);
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
void path(u16 now,u16 exp);
u16 my_abs(int num);

#endif 


