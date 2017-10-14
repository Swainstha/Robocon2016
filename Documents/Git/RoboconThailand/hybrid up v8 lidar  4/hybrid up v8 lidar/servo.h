/*
 * servo.h
 *
 * Created: 5/7/2016 12:28:22 PM
 *  Author: 
 */ 


#ifndef SERVO_H_
#define SERVO_H_

#include <avr/io.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define DuctFan_Servo_DDR DDRB
#define DuctFan_Servo_PORT PORTB
#define DuctFan_Servo_Pin PINB7

#define Gripper_Servo_DDR DDRB
#define Gripper_Servo_PORT PORTB
#define Gripper_Servo_Pin PINB5

class Servo
{
	private:
		unsigned int Max_Duct_Servo,Min_Duct_Servo,Max_Gripper_Servo,Min_Gripper_Servo;
		
	public:
		void Initialize();
		void Set_Range_Duct_Servo(unsigned int min, unsigned int max);
		void Set_Range_Gripper_Servo(unsigned int min, unsigned int max);
		void Rotate_Duct_Servo(float angle);
		void Rotate_Gripper_Servo(float angle);
		void Deactivate_Duct_Servo();
		void Deactivate_Gripper_Servo();
};

#endif /* SERVO_H_ */