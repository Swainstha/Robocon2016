/*
 * ductfan.h
 *
 * Created: 5/7/2016 12:06:20 PM
 *  Author: 
 */ 


#ifndef DUCTFAN_H_
#define DUCTFAN_H_

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>

#define Duct_DDR DDRB     //OC1B
#define Duct_PORT PORTB
#define Duct_Pin PINB6

class Ductfan
{
	private:
		unsigned int minvalue,maxvalue;
		
	public:
		void Initialize();
		void SetRange_Us(unsigned int min,unsigned int max);
		void calibrate();
		void run(float dutycycle);
};


#endif /* DUCTFAN_H_ */