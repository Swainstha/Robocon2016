/*
 * dynamixel.h
 *
 * Created: 5/3/2016 6:34:14 PM
 *  Author: Swain
 */ 


#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

#define F_CPU 16000000UL
#define ID 0x01
#define Write_instruction 0x03
#define Read_instruction 0x02
#define TXRXControlDDR DDRD
#define TXRXControl PORTD
//#define TXCONTROLPin PD0
#define TXCONTROLPin PD1


#define TxOnRxOff TXRXControl&=~(1<<TXCONTROLPin) //|(1<<RXCONTROLPin))
#define TxOffRxOn TXRXControl|=(1<<TXCONTROLPin) //|(1<<RXCONTROLPin))
#define TxOff TXRXControlDDR&=~(1<<TXCONTROLPin)
#define TxOn TXRXControlDDR|=(1<<TXCONTROLPin)
#define RxOff
#define baud 1000000UL
#define baud_pre ((F_CPU/(baud*16))-1)

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>


int rei=0;
int buffer[20];
char buff[20];
uint8_t data[10];
int angle=0;
uint16_t angle_data=0;
uint8_t parameter[10];
uint8_t received[30];
uint8_t Goal_Pos=0x1e;
uint8_t Moving_Speed=0x20;

void uart_init();
void uart_write(uint8_t);
void transmit(uint8_t,uint8_t,uint8_t);
void set_Id();
void goal_position(uint8_t,uint8_t);
void speed_moving(uint8_t,uint8_t);

void uart_init()
{
	UBRRH=(baud_pre>>8);
	UBRRL=baud_pre;
	UCSRB |=(1<<TXEN);
	UCSRC |= (1<<URSEL) | (1<<UCSZ0) |(1<<UCSZ1);
}

void uart_write(uint8_t data)
{
	UDR = data;
	while(!(UCSRA & (1<<UDRE)));
}

uint8_t uart_read()
{
	while(!(UCSRA & (1<<RXC)));
	return UDR;
}

void transmit(uint8_t id,uint8_t instruction,uint8_t no_par)
{
	
	//TxOnRxOff;
	TxOn;
	UCSRB &=~(1<<RXEN);
	UCSRB|=(1<<TXEN);
	uint8_t len = no_par+2;
	uint8_t check_sum = (id+len+instruction);
	uart_write(0xff);
	uart_write(0xff);
	uart_write(id);
	uart_write(len);
	uart_write(instruction);
	for(int i=0 ;i<no_par;i++)
	{
		uart_write(parameter[i]);
		check_sum+=parameter[i];
	}
	uart_write(~(check_sum ));
	TxOff;
	UCSRB |=(1<<RXEN);
	UCSRB&=~(1<<TXEN);
	//while(bit_is_clear(UCSRA,TXC));
	//_delay_us(160);
//TxOffRxOn;
		//TXRXControl|=(1<<TXCONTROLPin);
}
void transmit1(uint8_t id,uint8_t instruction,uint8_t no_par)
{
	
	//TxOnRxOff;
	TxOn;
	UCSRB &=~(1<<RXEN);
	UCSRB|=(1<<TXEN);
	uint8_t len = 0x04;
	uint8_t check_sum = (id+len+instruction);
	uart_write(0xff);
	uart_write(0xff);
	uart_write(id);
	uart_write(len);
	uart_write(instruction);
	for(int i=0 ;i<no_par;i++)
	{
		uart_write(parameter[i]);
		check_sum+=parameter[i];
	}
	uart_write(~(check_sum ));
	TxOff;
	UCSRB |=(1<<RXEN);
	UCSRB&=~(1<<TXEN);
	//while(bit_is_clear(UCSRA,TXC));
	//_delay_us(160);
	//TxOffRxOn;
	//TXRXControl|=(1<<TXCONTROLPin);
}

void set_Id()
{
	parameter[0]=0x03;
	parameter[1]=0x01;
	transmit(0xFE,Write_instruction,0x02);
}


void led(uint8_t state)
{
	parameter[0]=0x19;
	parameter[1]=state;
	transmit(ID,Write_instruction,0x02);
}

void enable_torque()
{
	parameter[0]=0x18;
	parameter[1]=0x01;
	transmit(ID,Write_instruction,0x02);
}


void goal_position(uint16_t angle)
{
	//uint16_t byte=((float)(angle*4000)/360);
	uint16_t byte=angle*3.41;
	uint8_t upper_byte=((byte>>8) & 0xff);
	uint8_t lower_byte=(byte & 0xff);
	parameter[0]=Goal_Pos;
	parameter[1]=lower_byte;//0xff
	parameter[2]=upper_byte;//0x03
	transmit(ID,Write_instruction,0x03);
}

void speed_moving(uint16_t t)
{
	parameter[0]=Moving_Speed;
	parameter[1]=t & 0xff;//0xff
	parameter[2]=(t>>8)&0xff;//0x03
	transmit(ID,Write_instruction,0x03);
}


void run()
{
	parameter[0]=0x1e;
	parameter[1]=0xf0;
	parameter[2]=0x00;
	transmit(ID,Write_instruction,0x03);
}
void CW()
{
	parameter[0]=0x06;
	parameter[1]=0x00;
	parameter[2]=0x00;
	transmit(ID, Write_instruction,0x03);
}

void CCW()
{
	parameter[0]=0x08;
	parameter[1]=0xff;
	parameter[2]=0x03;
	transmit(ID, Write_instruction,0x03);
}

void maxtorque()
{
	parameter[0]=0x0e;
	parameter[1]=0xff;
	parameter[2]=0x03;
	transmit(ID,Write_instruction,0x03);
}
void reset()
{
	transmit(0xFE,0x06,0x00);
}

void dynamixel_init()
{
	uart_init();
	//reset();
	//CW();
	//CCW();
	set_Id();
	enable_torque();
	maxtorque();
	speed_moving(1023);
}
void dynamixel_position()
{
	
	parameter[0]=0x24;
	parameter[1]=0x02;
	transmit1(0x01,Read_instruction,0x02);
}
void angle_read()
{
	dynamixel_position();
	//_delay_us(100);
	UDR=0;
	for(int i=0;i<9;i++)
	{
		data[i]=uart_read();
	}
	angle_data=data[6]|(data[7]<<8);
	angle=angle_data/3.41;
	//return (int)angle;
}



#endif /* DYNAMIXEL_H_ */
