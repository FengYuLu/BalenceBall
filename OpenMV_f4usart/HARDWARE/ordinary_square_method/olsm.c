#include "olsm.h"

MY_POINT sample[SAMPLE_NUM];



void ordinary_lsm(float *kk,float *dd)
{
	float Ex = 0.0f,Ey = 0.0f,Exy = 0.0f,Ex2 = 0.0f,my_k = 0.0f,my_d = 0.0f;
	int num = 0;
	for(num = 0;num<SAMPLE_NUM;num++)
	{
		Ex += (float)sample[num].x;
		Ey += (float)sample[num].y;
		Exy += (float)sample[num].x * (float)sample[num].y;
		Ex2 += (float)sample[num].x * (float)sample[num].x;
	}
	
	my_k = (num*Exy-Ex*Ey)/(num*Ex2-Ex*Ex);
	my_d = (Ey-my_k*Ex)/(float)num;
	
	*kk = my_k;
	*dd = my_d;
	
}







