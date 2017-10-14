/*
 * motor.cpp
 *
 * Created: 5/9/2016 8:53:09 AM
 *  Author:
 */ 

#include "motor.h"

void Motor::Initialize()
{
	Motor_A_DDR |= (1<<Motor_A_PIN);
	Motor_A_Direction_A_DDR |= (1<<Motor_A_Direction_A_PIN);
	Motor_A_Direction_B_DDR |= (1<<Motor_A_Direction_B_PIN);
	Motor_A_PORT &=~ (1<<Motor_A_PIN);
	
	Motor_B_DDR |= (1<<Motor_B_PIN);
	Motor_B_Direction_A_DDR |= (1<<Motor_B_Direction_A_PIN);
	Motor_B_Direction_B_DDR |= (1<<Motor_B_Direction_B_PIN);
	Motor_B_PORT &=~ (1<<Motor_B_PIN);
	
	Wiper_DDR |= (1<<Wiper_PIN);
	Wiper_Direction_B_DDR |= (1<<Wiper_Direction_B_PIN);
	Wiper_Direction_A_DDR |= (1<<Wiper_Direction_A_PIN);
	Wiper_PORT &=~ (1<<Wiper_PIN);
	
	TCCR3A |= ((1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1)|(1<<WGM31));
	TCCR3B |= ((1<<CS30)|(1<<WGM33)|(1<<WGM32));
	ICR3 = 2000;
}

void Motor::Set_Motor_Direction(unsigned char motor,unsigned char direction)
{
	if (motor==Motor_A)
	{
		if (direction==Forward)
		{
			Motor_A_Direction_A_PORT |= (1<<Motor_A_Direction_A_PIN);
			Motor_A_Direction_B_PORT &=~ (1<<Motor_A_Direction_B_PIN);
		} 
		else
		{
			Motor_A_Direction_A_PORT &=~ (1<<Motor_A_Direction_A_PIN);
			Motor_A_Direction_B_PORT |= (1<<Motor_A_Direction_B_PIN);
		}
	}
	
	if(motor==Motor_B)
	{
		if (direction==Forward)
		{
			Motor_B_Direction_A_PORT |= (1<<Motor_B_Direction_A_PIN);
			Motor_B_Direction_B_PORT &=~ (1<<Motor_B_Direction_B_PIN);
		}
		else
		{
			Motor_B_Direction_A_PORT &=~ (1<<Motor_B_Direction_A_PIN);
			Motor_B_Direction_B_PORT |= (1<<Motor_B_Direction_B_PIN);
		}
	}
	
	if(motor==Wiper)
	{
		if (direction==Forward)
		{
			Wiper_Direction_A_PORT |= (1<<Wiper_Direction_A_PIN);
			Wiper_Direction_B_PORT &=~ (1<<Wiper_Direction_B_PIN);
		}
		else
		{
			Wiper_Direction_A_PORT &=~ (1<<Wiper_Direction_A_PIN);
			Wiper_Direction_B_PORT |= (1<<Wiper_Direction_B_PIN);
		}
	}
}

void Motor::Run(unsigned int motor,unsigned int value)
{
	if (motor==Motor_A)
	{
		OCR3C=value;
	}
	if (motor==Motor_B)
	{
		OCR3A=value;
	}
	if (motor==Wiper)
	{
		OCR3B=value;
	}
}