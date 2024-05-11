/*
 * PID.c
 *
 *  Created on: Dec 17, 2022
 *      Author: Federico Brunella
 */
#include<PID.h>

void init_PID(PID* p, float Tc, float u_max, float u_min){
	//Set PID parameters
	p->Tc = Tc;
	p->u_max = u_max;
	p->u_min = u_min;
}

//This function sets PID tuning parameters (standard form)
void tune_PID(PID* p, float Kp, float Ki, float Kd){
	//Set tuning parameters (standard form)
	p->Kp = Kp; //gain
	p->Ki = Ki; //integral time
	p->Kd = Kd; //derivative time
}

float PID_controller(PID* p, float y, float r){
	static float e_old = 0, Iterm = 0;
	float u;
	float newIterm;

	float e = r - y;

	float Pterm = p->Kp *e;
	newIterm = Iterm + (p->Ki)*p->Tc*e_old;
	float Dterm = (p->Kd/p->Tc)*(e - e_old);

	e_old = e;

	u = Pterm + newIterm + Dterm;

	if(u > p->u_max){
		u = p->u_max;
	}else if(u < p->u_min){
		u = p->u_min;
	}else{
		Iterm = newIterm; //Clapping anti-windup
	}

	return u;
}

