#ifndef __MOTO_CONTROL_H
#define __MOTO_CONTROL_H	
#include "sys.h" 
#include "delay.h"
extern u8 start_flag;
extern int now_y,now_x;
void data_read(void);
void TIM3_Int_Init(u16 arr,u16 psc);
void moto_driver(float difference);
void moto_io_init(void);
void mode1(void);
void clear(char *string,u16 leng);
void zhuanwan(void);
#endif 


