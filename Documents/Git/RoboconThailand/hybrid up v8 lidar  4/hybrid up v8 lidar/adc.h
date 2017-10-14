/*
 * adc.h
 *
 * Created: 6/20/2016 5:48:59 PM
 *  Author: grunze
 */ 


#ifndef ADC_H_
#define ADC_H_

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <stdlib.h>
#include "util/delay.h"

#define WIPER 13
#define MOTOR_A 8
#define MOTOR_B 0

class Analog
{
	private:
	unsigned int previous_motor_B;
	unsigned int previous_motor_A;
	unsigned int previous_wiper;
	int C_adc;
	int C_P_Adc;
	
	public:
	int adcl,adc_value;
	unsigned int adc;
	uint16_t Adc_Data;
	void ADC_Init();
	int Get_Junction_Sensor(uint8_t ch,int range,int difference);
	int Get_Junction_Sensor1(uint8_t ch);
};




#endif /* ADC_H_ */