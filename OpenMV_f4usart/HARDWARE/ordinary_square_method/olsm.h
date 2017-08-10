#ifndef __OLSM_H
#define __OLSM_H
#include "sys.h"
#include "math.h"
#define SAMPLE_NUM 100

typedef struct 
{
	int x,y;
}MY_POINT;

extern MY_POINT sample[SAMPLE_NUM];
extern float k,d;

void ordinary_lsm(float *kk,float *dd);









#endif
