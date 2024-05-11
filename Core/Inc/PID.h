/*
 * PID.h
 *
 *  Created on: Dec 17, 2022
 *      Author: Federico Brunella
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <main.h>

typedef struct PID{
	float Kp; //P gain
	float Ki; //I gain
	float Kd; //D gain
	// Other param
	float Tc; //sampling period
	float u_max; // PID output upper limit
	float u_min; // PID output lower limit
}PID;


//Function declaration
void init_PID(PID*, float, float, float);
void tune_PID(PID* , float, float, float);
float PID_controller(PID* , float, float);

#endif /* INC_PID_H_ */
