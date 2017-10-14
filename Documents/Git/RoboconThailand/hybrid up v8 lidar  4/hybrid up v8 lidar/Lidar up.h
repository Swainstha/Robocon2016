/*
 * Lidar_up.h
 *
 * Created: 7/20/2016 3:02:47 AM
 *  Author: grunze
 */ 


#ifndef LIDAR_UP_H_
#define LIDAR_UP_H_

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define ERROR 0

#define TRIGGER_DDR DDRH
#define TRIGGER_PORT PORTH
#define TRIGGER_CHNL PINH
#define TRIGGER_PIN PH5

#define SIGNAL_DDR DDRB
#define SIGNAL_PORT PORTB
#define SIGNAL_PIN PB4
#define SIGNAL_CHNL PINB


#define Start_Timer5 TCNT5=0 ; TCCR5B |= ((1<<CS51))
#define Stop_Timer5 TCCR5B &=~ ((1<<CS51)) ; TCNT5=0

class Lidar_Up
{
	private:
	void Init_timer5();
	
	public:
	void Init_Lidar_Up();
	float Lidar_Up_Get_Data();
	
};


#endif /* LIDAR UP_H_ */