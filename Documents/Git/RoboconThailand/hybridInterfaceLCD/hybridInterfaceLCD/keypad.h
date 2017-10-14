/*
 * keypad.h
 *
 * Created: 7/19/2016 11:05:06 PM
 *  Author: Niraj
 */ 


#ifndef KEYPAD_H_
#define KEYPAD_H_

#define Pressed 1
#define Release 0
#include <avr/io.h>
#define select_s1 PORTB&=~(1<<PB1)
#define select_s2 PORTB&=~(1<<PB0)
#define select_s3 PORTD&=~(1<<PD2)
#define select_s4 PORTD&=~(1<<PD3)
#define unselect_spin1 PORTB|=(1<<PB1)
#define unselect_spin2 PORTB|=(1<<PB0)
#define unselect_spin3 PORTD|=(1<<PD2)
#define unselect_spin4 PORTD|=(1<<PD3)
enum buttons_t {NO_BUTTON,BUTTON_0,BUTTON_1,BUTTON_2,BUTTON_3,BUTTON_4,BUTTON_5,BUTTON_6,BUTTON_7,BUTTON_8,BUTTON_9,BUTTON_HASH,BUTTON_AST,BUTTON_A,BUTTON_B,BUTTON_C,BUTTON_D};
unsigned int last_button=NO_BUTTON;

void keypad_init()
{
	DDRB|=((1<<PINB1)|(1<<PB0));
	DDRD|=((1<<PD2)|(1<<PD3));
	DDRD&=~((1<<PD4)|(1<<PD5)|(1<<PD6)|(1<<PD7));
	PORTB|=((1<<PB1)|(1<<PB0));
	PORTD|=((1<<PD2)|(1<<PD3)|(1<<PD4)|(1<<PD5)|(1<<PD6)|(1<<PD7));
}

unsigned int keypad_check()
{
	select_s1;
	unselect_spin2;
	unselect_spin3;
	unselect_spin4;
	
	_delay_ms(15);
	if (bit_is_clear(PIND,PIND4) && bit_is_clear(PINB,1))
	{
		return BUTTON_1;
	}
	else if(bit_is_clear(PIND,PIND5) && bit_is_clear(PINB,1))
	{
		return BUTTON_2;
	}
	else if(bit_is_clear(PIND,PD6) && bit_is_clear(PINB,1))
	{
		return BUTTON_3;
	}
	else if(bit_is_clear(PIND,PD7) && bit_is_clear(PINB,1))
	{
		return BUTTON_A;
	}
	unselect_spin1;
	_delay_ms(5);

	select_s2;
	_delay_ms(15);
	if(bit_is_clear(PIND,PIND4) && bit_is_clear(PINB,0))
	{
		return BUTTON_4;
	}
	else if(bit_is_clear(PIND,5) && bit_is_clear(PINB,0))
	{
		return BUTTON_5;
	}
	else if(bit_is_clear(PIND,PD6) && bit_is_clear(PINB,0))
	{
		return BUTTON_6;
	}
	else if(bit_is_clear(PIND,PD7) && bit_is_clear(PINB,0))
	{
		return BUTTON_B;
	}
	unselect_spin2;
	_delay_ms(5);
	select_s3;
	_delay_ms(15);
	if(bit_is_clear(PIND,PIND4) && bit_is_clear(PIND,2))
	{
		return BUTTON_7;
	}
	else if(bit_is_clear(PIND,PIND5) && bit_is_clear(PIND,2))
	{
		return BUTTON_8;
	}
	else if(bit_is_clear(PIND,PIND6) && bit_is_clear(PIND,2))
	{
		return BUTTON_9;
	}
	else if(bit_is_clear(PIND,PIND7) && bit_is_clear(PIND,2))
	{
		return BUTTON_C;
	}
	unselect_spin3;
	_delay_ms(5);
	select_s4;
	_delay_ms(15);
	if(bit_is_clear(PIND,PIND4) && bit_is_clear(PIND,3))
	{
		return BUTTON_AST;
	}
	else if(bit_is_clear(PIND,PIND5) && bit_is_clear(PIND,3))
	{
		return BUTTON_0;
	}
	else if(bit_is_clear(PIND,PIND6) && bit_is_clear(PIND,3))
	{
		return BUTTON_HASH;
	}
	else if(bit_is_clear(PIND,PIND7) && bit_is_clear(PIND,3))
	{
		return BUTTON_D;
	}
	unselect_spin4;
	_delay_ms(5);
	last_button=NO_BUTTON;
	return NO_BUTTON;
}
unsigned int get_status()
{
	select_s1;
	select_s2;
	select_s3;
	select_s4;
	if(bit_is_clear(PIND,PIND4))
	return Pressed;
	else if(bit_is_clear(PIND,PIND5))
	return Pressed;
	else if(bit_is_clear(PIND,PIND6))
	return Pressed;
	else if(bit_is_clear(PIND,PIND7))
	return Pressed;
	else 
	return Release;
}
unsigned int get_key()
{
	if((get_status()==Pressed) && (last_button!=NO_BUTTON))
	{
		return last_button;
	}
	else 
	{
		last_button=keypad_check();
		return last_button;
	}
}
#endif /* KEYPAD_H_ */