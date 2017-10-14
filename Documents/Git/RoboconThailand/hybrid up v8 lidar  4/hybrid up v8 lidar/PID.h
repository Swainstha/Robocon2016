/*
 * PID.h
 *
 * Created: 1/29/2016 1:09:27 PM
 *  Author: 
 */ 


#ifndef PID_H_
#define PID_H_

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <math.h>

class PID
{
	private:
			float kp,ki,kd;
			float errSum,maxOut,minOut;
	
	public:
			float lasterror;
			float offset;
			void Initialize();
			void Set_Range(float min,float max){minOut=min;maxOut=max;}
			void SetKp(float KP){kp=KP;}
			void SetKi(float KI){ki=KI;}
			void SetKd(float KD){kd=KD;}
			float GetKp(){return kp;}
			float GetKi(){return ki;}
			float GetKd(){return kd;}
			void Set_PID(float KP,float KI,float KD);
			float Compute_PID(float input,float setPoint);
};
#endif /* PID_H_ */