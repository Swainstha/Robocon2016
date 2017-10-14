/*
 * colorsensor.h
 *
 * Created: 2/8/2016 9:35:54 PM
 *  Author: Swain
 */ 


#ifndef COLORSENSOR_H_
#define COLORSENSOR_H_

#include <avr/io.h>
#include <math.h>
#include <stdlib.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <util/delay.h>
#include <avr/interrupt.h>

//volatile uint16_t red,green,blue;

//#define s0 PINA0
//#define s1 PINA1
#define s2 PINA1
#define s3 PINA0
//#define led PINA4
#define DDR_C DDRA
#define PORT_C PORTA

volatile uint16_t count2=0;
uint16_t red,green,blue,alpha;
float rg,rb,bg,br,gr,gb;
int col=0;
int last_col=0;
float r=0,g=0,bl=0;

void init_extinterrupt()
{
	sei();
	MCUCR|=(1<<ISC10)|(1<<ISC11);
	GICR|=(1<<INT1);
	DDRD&=~(1<<PIND3);
	PORTD|=(1<<PIND3);
}

ISR(INT1_vect)
{
	count2++;
}
void init_color_sensor()
{
	init_extinterrupt();
	DDR_C|=(1<<s2)|(1<<s3);
	//PORT_C|=(1<<s0)|(1<<s1)|(1<<led);
	//PORTC&=~(1<<OE);
}

void calc_color()
{
	//PORT_C|=(1<<led);
	//PORT_C|=((1<<s0)|(1<<s1));
	
	count2 =0;
	PORT_C|=(1<<s2)|(1<<s3);//green
	_delay_ms(1);
	green=count2;
	
	count2=0;
	PORT_C&=~((1<<s2)|(1<<s3));//red
	_delay_ms(1);
	red=count2;
	
	count2=0;
	PORT_C&=~(1<<s2);//blue
	PORT_C|=(1<<s3);
	_delay_ms(1);
	blue=count2;
	
	/*count2=0;
	PORT_C&=~(1<<s3);//alpha
	PORT_C|=(1<<s2);
	_delay_ms(2);
	alpha=count2;*/
	
	
	//PORT_C&=~((1<<s0)|(1<<s1)|(1<<led));
}
int check_color()
{
	calc_color();
	r=(red)/sqrt(red*red+green*green+blue*blue);
	bl=(blue)/sqrt(red*red+green*green+blue*blue);
	g=(green)/sqrt(red*red+green*green+blue*blue);
	rg=(float)r/g;
	rb=(float)r/bl;
	gb=(float)g/bl;
	br=(float)bl/r;
	bg=(float)bl/g;
	gr=(float)g/r;
	
	if(rg>1.6 && rb>1.8)
	col=1;
	else if(br>1.9 && bg>1.4 && bg<1.7)
	col=2;
	else if( bg<1.2 && gr>1.45 && gr<1.8)  //gr>1.8  gr<2.25
	col=3;
	//else if(gb>1.2 && gr<1)
	//col=4;
	/*else if(rg<1.5 && rb<1.3)
	{
		if(last_col==4)
			col=5;
		else
			col=last_col;
	}*/
	else col=0;
	if(col==0)
	{
		col=last_col;
	}
	last_col=col;
	return col;
}


#endif /* COLORSENSOR_H_ */
