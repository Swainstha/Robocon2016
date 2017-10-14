/*
 * encoder.h
 *
 * Created: 6/2/2016 9:58:43 PM
 *  Author: Swain
 */ 


#ifndef ENCODER_H_
#define ENCODER_H_



#define F_CPU 16000000UL
#define BAUD_RATE 9600
#include <avr/io.h>
#include "encoder.h"

void init_extinterrupt();
void init_timer();

volatile int counter=0;
volatile int last_counter=0;
volatile unsigned int distance=0;
volatile unsigned int last_dist=0;
volatile unsigned int lastdist=0;
volatile unsigned int diffdist=0;
volatile int speed=0;
int displayflag=0;
int compass_flag=0;
int ending_flag=0;
int compass_flag2=0;

ISR(INT0_vect)
{
	if(bit_is_set(PIND,5))
	{
		counter--;
		if(counter % 8==0)
		{
			distance--;
		}
	}
	else
	{
		counter++;
		if(counter%8==0)
		{
			distance++;
		}
	}
	
	
}


void init_encoder()
{
	MCUCR|=(1<<ISC00)|(1<<ISC01);
	GICR|=(1<<INT0);
	DDRD&=~((1<<PIND5)|(1<<PIND2));
	PORTD|=(1<<PIND5)|(1<<PIND2);
}
void init_timer()
{
	TCCR0|=(1<<CS02)|(1<<CS00);
	TIMSK|=(1<<TOIE0);
	TIFR |= (1<<TOV0);
}

ISR(TIMER0_OVF_vect)
{
	//diffdist=counter-last_counter;
	diffdist=counter-last_counter;
	speed=diffdist;
	last_counter=counter;
	last_dist=distance;
	displayflag++;
	
}

void init_timer2()
{
	TCCR2|=(1<<WGM21)|(1<<CS22)|(1<<CS20)|(1<<CS21);
	TIMSK|=(1<<OCIE2);
	OCR2=77;	
}


ISR(TIMER2_COMP_vect)
{
	ending_flag++;
}

#endif /* ENCODER_H_ */
