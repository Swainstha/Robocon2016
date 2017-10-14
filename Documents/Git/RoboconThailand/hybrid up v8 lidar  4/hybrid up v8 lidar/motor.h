/*
 * motor.h
 *
 * Created: 5/9/2016 8:53:24 AM
 *  Author:
 */ 


#ifndef MOTOR_H_
#define MOTOR_H_

#include <avr/io.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define Motor_A 0   /// gripper
#define Motor_B 1   /// arm
#define Wiper 2  

#define Forward 1
#define Reverse 0

#define Motor_A_DDR					DDRE   //OC3C   gripper
#define Motor_A_PORT				PORTE
#define Motor_A_PIN					PINE5
#define Motor_A_Direction_A_DDR		DDRA
#define Motor_A_Direction_A_PORT	PORTA
#define Motor_A_Direction_A_PIN		PINA5
#define Motor_A_Direction_B_DDR		DDRA
#define Motor_A_Direction_B_PORT	PORTA
#define Motor_A_Direction_B_PIN		PINA7

#define Motor_B_DDR					DDRE   //OC3A    arm
#define Motor_B_PORT				PORTE
#define Motor_B_PIN					PINE3
#define Motor_B_Direction_A_DDR		DDRA
#define Motor_B_Direction_A_PORT	PORTA
#define Motor_B_Direction_A_PIN		PINA1
#define Motor_B_Direction_B_DDR		DDRA
#define Motor_B_Direction_B_PORT	PORTA
#define Motor_B_Direction_B_PIN		PINA3

#define Wiper_DDR					DDRE  //OC3B   WIPER
#define Wiper_PORT					PORTE
#define Wiper_PIN					PINE4
#define Wiper_Direction_A_DDR		DDRC
#define Wiper_Direction_A_PORT		PORTC
#define Wiper_Direction_A_PIN		PINC4
#define Wiper_Direction_B_DDR		DDRC
#define Wiper_Direction_B_PORT		PORTC
#define Wiper_Direction_B_PIN		PINC6

class Motor
{
	private:
	
	public:
		void Initialize();
		void Set_Motor_Direction(unsigned char motor,unsigned char direction);
		void Run(unsigned int motor,unsigned int value);
};


#endif /* MOTOR_H_ */