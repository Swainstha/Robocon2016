/*
 * Lidar_down.h
 *
 * Created: 7/20/2016 3:03:06 AM
 *  Author: grunze
 */ 


#ifndef LIDAR_DOWN_H_
#define LIDAR_DOWN_H_

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define ERROR 0

#define Trigger_DDR DDRH
#define Trigger_PORT PORTH
#define Trigger_PIN PINH4

#define Signal_DDR DDRH
#define Signal_PORT PORTH
#define Signal_PIN PINH6
#define Signal_CHNL PINH

#define Start_Timer0 TCNT0=0 ; TCCR0B |= (1<<CS02)
#define Stop_Timer0 TCCR0B &=~ (1<<CS02) ; TCNT0=0

class Lidar_Down
{
	private:
	void Init_timer0();
	
	public:
	void Init_Lidar_Down();
	float Lidar_Down_Get_Data();
	
};

#endif /* LIDAR DOWN_H_ */