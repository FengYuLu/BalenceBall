#ifndef __PID_H
#define __PID_H	
#include "sys.h" 
#include "delay.h"

typedef struct
{
float Ek_now,EK_last,Integral,Kp,Ki,Kd,P_out,I_out,D_out,PID_out;
}PID_Date;

extern PID_Date locaPID_y;
extern PID_Date locaPID_x;
extern PID_Date speedPID_y;
extern PID_Date speedPID_x;

void PID_calculate(PID_Date Select_pid,float aim,float measure,float *outval);
void Pid_set(PID_Date *Select_pid,float kp,float ki,float kd);

#endif 


