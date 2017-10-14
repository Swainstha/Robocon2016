/*
 * Lidar_Up.cpp
 *
 * Created: 7/20/2016 3:10:52 AM
 *  Author: grunze
 65535
 */ 

#include "Lidar up.h"

volatile uint8_t timeoutFlagUp=0;
float timeDataUp=0;

void Lidar_Up::Init_timer5()
{
	TCCR5B |= (1<<WGM52);
	TIMSK5 |= (1<<OCIE5A);
	TIFR5 |= (1<<OCF5A);
	OCR5A = 10000;
	TCNT5 = 0;
}

void Lidar_Up::Init_Lidar_Up()
{
	Init_timer5();
	TRIGGER_DDR |= (1<<TRIGGER_PIN);				//trigger pin as output
	TRIGGER_PORT |= (1<<TRIGGER_PIN);				//trigger pin initially high
	SIGNAL_DDR &=~ (1<<SIGNAL_PIN);				//signal_out pin as input
	TCNT5=0;
}

float Lidar_Up::Lidar_Up_Get_Data()
{
	
	TRIGGER_PORT&=~(1<<TRIGGER_PIN);
	_delay_us(10);

	Start_Timer5;
	
	while((!(SIGNAL_CHNL&(1<<SIGNAL_PIN)))&&(!timeoutFlagUp));
	
	if(timeoutFlagUp)
	{
		timeoutFlagUp=0;
		Stop_Timer5;
		TRIGGER_PORT|=(1<<TRIGGER_PIN);
		return ERROR;
	}
	
	Start_Timer5;

	while((SIGNAL_CHNL&(1<<SIGNAL_PIN))&&(!timeoutFlagUp));
	
	if(timeoutFlagUp)
	{
		timeoutFlagUp=0;
		Stop_Timer5;
		TRIGGER_PORT|=(1<<TRIGGER_PIN);
		return ERROR;
	}
	
	timeDataUp=((float)TCNT5)/20.0;
	Stop_Timer5;
	
	TRIGGER_PORT|=(1<<TRIGGER_PIN);
	return timeDataUp;
	
	_delay_ms(7);
}


ISR(TIMER5_COMPA_vect)
{
	timeoutFlagUp=1;
}