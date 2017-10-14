/*
 * servo.cpp
 *
 * Created: 5/9/2016 11:41:48 PM
 *  Author: 
 */ 

#include "servo.h"

void Servo::Initialize()
{
	Min_Duct_Servo = 400;
	Max_Duct_Servo = 2400;
	Min_Gripper_Servo = 400;
	Max_Gripper_Servo = 2400;
	DuctFan_Servo_DDR |= (1<<DuctFan_Servo_Pin);
	Gripper_Servo_DDR |= (1<<Gripper_Servo_Pin);
	TCCR1A |= ((1<<COM1B1)|(1<<COM1C1)|(1<<WGM11)|(1<<COM1A1));
	TCCR1B |= ((1<<WGM13)|(1<<WGM12)|(1<<CS11));
	ICR1 = 39999;
}

void Servo::Set_Range_Duct_Servo(unsigned int min, unsigned int max)
{
	Min_Duct_Servo = min;
	Max_Duct_Servo = max;
}

void Servo::Set_Range_Gripper_Servo(unsigned int min, unsigned int max)
{
	Min_Gripper_Servo = min;
	Max_Gripper_Servo = max;
}

void Servo::Rotate_Duct_Servo(float angle)
{
	float var;
	var = Min_Duct_Servo+angle*(float)(Max_Duct_Servo-Min_Duct_Servo)/180.0;
	OCR1C=(unsigned int)var*2;
}

void Servo::Rotate_Gripper_Servo(float angle)
{
	float var;
	var = Min_Gripper_Servo+angle*(float)(Max_Gripper_Servo-Min_Gripper_Servo)/180.0;
	OCR1A=(unsigned int)var*2;
}
