#include "pid.h"

PID_Date locaPID_y={0,0,0,0,0,0,0,0,0,0};
PID_Date locaPID_x={0,0,0,0,0,0,0,0,0,0};
PID_Date speedPID_y={0,0,0,0,0,0,0,0,0,0};
PID_Date speedPID_x={0,0,0,0,0,0,0,0,0,0};



void PID_calculate(PID_Date Select_pid,float aim,float measure,float *outval)
{
	Select_pid.EK_last = Select_pid.Ek_now;
	Select_pid.Ek_now = aim - measure;
	Select_pid.Integral += Select_pid.Ek_now;
	if(Select_pid.Integral > 500)Select_pid.Integral = 500;
	else if(Select_pid.Integral < -500)Select_pid.Integral = -500;
	Select_pid.P_out = Select_pid.Kp*Select_pid.Ek_now;
	Select_pid.I_out = Select_pid.Ki*Select_pid.Integral;
	Select_pid.D_out = Select_pid.Kd*(Select_pid.Ek_now - Select_pid.EK_last);
	Select_pid.PID_out = Select_pid.P_out+Select_pid.I_out+Select_pid.D_out;
	*outval = Select_pid.PID_out;
}


void Pid_set(PID_Date *Select_pid,float kp,float ki,float kd)
{
	Select_pid->Kp = kp;
	Select_pid->Ki = ki;
	Select_pid->Kd = kd;
}






