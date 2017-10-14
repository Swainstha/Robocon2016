/*
 * PID.cpp
 *
 * Created: 1/29/2016 1:04:42 PM
 *  Author: 
 */  
#include "PID.h"
#include "generalFunctions.h"

void PID::Initialize()
{
	kp=0;
	ki=0;
	kd=0;
	errSum=0;
	lasterror=0;
	offset=0;
}
void PID::Set_PID(float KP,float KI,float KD)
{
	kp=KP;
	ki=KI;
	kd=KD;
}
float PID::Compute_PID(float input,float setpoint)
{
	float error=setpoint-input;
	errSum += error;
	
	constrain(errSum,-500,500);
	
	float dErr = (error - lasterror);
	
	//Compute PID Output
	float output;
	if (fabs(error)>3)
	{
		output = kp * error + ki * errSum + kd * dErr+offset;
	}
	else
	output=0;
	
	constrain(output,minOut,maxOut);
	//Remember some variables for next time
	lasterror = error;
	return output;
}