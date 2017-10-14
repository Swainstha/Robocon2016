/*
 * adc.cpp
 *
 * Created: 6/20/2016 5:48:41 PM
 *  Author: grunze
 */ 

#include "adc.h"

//adc read from pololu initilization
void Analog::ADC_Init()
{
	previous_motor_A=0;
	previous_wiper=0;
	previous_motor_B=0;
	ADCSRA |= (1<<ADEN); //enblaing
	ADMUX |= (1<<REFS0);
	//ADMUX |= (1<<ADLAR); //avcc reference
	ADCSRA |= (1<<ADPS1) | (1<<ADPS2) | (1<<ADPS0); //128 bit presclaing
	ADCSRA |=(1<<ADSC);	//start conversion
	while (ADCSRA & (1<<ADSC));
	ADCSRA &= ~(1<<ADEN);
}

//check of actual data from the sensor pololu
int Analog::Get_Junction_Sensor(uint8_t ch,int Threshold,int difference)
{
	unsigned char ch1=ch;
	if (ch>7)
	{
		ADCSRB|=(1<<MUX5);
	}
	else
	{
		ADCSRB&=~(1<<MUX5);
	}
	ch &= 0b0000111;
	ADMUX = (ADMUX & 0XF8) | ch;
	ADCSRA |= (1<<ADSC) | (1<<ADEN);
	while (ADCSRA & (1<<ADSC));
	adc=ADCW;
	
	if (ch1==MOTOR_A)
	{
		if (adc>Threshold)
		{
			previous_motor_A=1;
		}
		else if (adc<(Threshold-difference))
		{
			previous_motor_A=0;
		}
		return previous_motor_A;
		
	}
	else if (ch1==MOTOR_B)  
	{
		if (adc>Threshold)
		{
			previous_motor_B=1;
		}
		else if (adc<(Threshold-difference))
		{
			previous_motor_B=0;
		}
		return previous_motor_B;

	}
	else if (ch1==WIPER)  //wiper
	{
		if (adc>Threshold)
		{
			previous_wiper=1;
		}
		else if (adc<(Threshold-difference))
		{
			previous_wiper=0;
		}
		return previous_wiper;

	}
	
	_delay_ms(5);
}

int Analog::Get_Junction_Sensor1(uint8_t ch)
{
	if (ch>7)
	{
		ADCSRB|=(1<<MUX5);
	}
	else
	{
		ADCSRB&=~(1<<MUX5);
	}
	ch &= 0b0000111;
	ADMUX = (ADMUX & 0XF8) | ch;
	ADCSRA |= (1<<ADSC) | (1<<ADEN);
	while (ADCSRA & (1<<ADSC));
	adc=ADCW;
	
	return adc;
}