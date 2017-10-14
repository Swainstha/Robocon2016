/*
 * ductfan.cpp
 *
 * Created: 5/9/2016 11:51:01 PM
 *  Author
 */ 

#include "ductfan.h"

void Ductfan::Initialize()
{
	minvalue=900;
	maxvalue=2100;
	Duct_DDR |= (1<<Duct_Pin);
	//	TCCR1A TCCR1B ICR1 ALL ARE IN SERVO.H
}
void Ductfan::SetRange_Us(unsigned int min,unsigned int max)
{
	minvalue = min;
	maxvalue = max;
}
void Ductfan::calibrate()
{
	run(100);
	_delay_ms(2000);
	run(0);
	_delay_ms(5000);
}
void Ductfan::run(float dutycycle)
{
	float var;
	var = minvalue+dutycycle*(float)(maxvalue-minvalue)/100.0;
	OCR1B=(unsigned int)var*2;
}
