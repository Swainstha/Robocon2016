/*
 * Lidar_Down.cpp
 *
 * Created: 7/20/2016 3:10:34 AM
 *  Author: grunze
 */ 

#include "Lidar down.h"

volatile uint8_t timeoutFlagDown=0;
float timeDataDown=0;

void Lidar_Down::Init_timer0()
{
	TIMSK0 |= (1<<TOIE0);
	TCNT0=0;
}

void Lidar_Down::Init_Lidar_Down()
{
	Init_timer0();
	Trigger_DDR |= (1<<Trigger_PIN);				//trigger pin as output
	Trigger_PORT &=~ (1<<Trigger_PIN);			//trigger pin initially low
	Signal_DDR &=~ (1<<Signal_PIN);				//signal_out pin as input
	TCNT0=0;
}

float Lidar_Down::Lidar_Down_Get_Data()
{
	Start_Timer0;
	
	while((Signal_CHNL&(1<<Signal_PIN))&&(!timeoutFlagDown));
	
	if(timeoutFlagDown)
	{
		timeoutFlagDown=0;
		Stop_Timer0;
		return ERROR;
	}
	
	Start_Timer0;
	
	while((!(Signal_CHNL&(1<<Signal_PIN)))&&(!timeoutFlagDown));
	
	if(timeoutFlagDown)
	{
		timeoutFlagDown=0;
		Stop_Timer0;
		return ERROR;
	}
	
	Start_Timer0;

	while((Signal_CHNL&(1<<Signal_PIN))&&(!timeoutFlagDown));
	
	if(timeoutFlagDown)
	{
		timeoutFlagDown=0;
		Stop_Timer0;
		return ERROR;
	}
	
	timeDataDown=((float)TCNT0*16)/10;
	timeDataDown=timeDataDown-30;
	Stop_Timer0;
	
	if(timeDataDown<0)
	{
		timeDataDown=0;
	}
	return timeDataDown;
	_delay_ms(7);
}

ISR(TIMER0_OVF_vect)
{
	timeoutFlagDown=1;
}