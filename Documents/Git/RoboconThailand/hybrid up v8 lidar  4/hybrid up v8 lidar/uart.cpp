/*
 * uart.c
 *
 * Created: 5/4/2016 3:33:27 AM
 *  Author: 
 */ 

#include "uart.h"

char timeOut=0;

char buffer=0,buffer1=0,buffer2=0;

void initUART0(void)
{
	//set baud rate
	UBRR0H=(MYUBRR0)>>8;
	UBRR0L=MYUBRR0;

	//Enable Receiver and Transmitter
	UCSR0B = ((1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0));
	
	//set frame format
	UCSR0C=((1<<UCSZ00)|(1<<UCSZ01));
}

void initUART1(void)
{
	//set baud rate
	UBRR1H=(MYUBRR1)>>8;
	UBRR1L=MYUBRR1;

	//Enable Receiver and Transmitter
	UCSR1B = ((1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1));
	
	//set frame format
	UCSR1C= ((1<<UCSZ10)|(1<<UCSZ11));
}

void initUART2(void)
{
	//set baud rate
	UBRR2H=(MYUBRR2)>>8;
	UBRR2L=MYUBRR2;

	//Enable Receiver and Transmitter
	UCSR2B = ((1<<RXEN2)|(1<<TXEN2)|(1<<RXCIE2));
	
	//set frame format
	UCSR2C= ((1<<UCSZ20)|(1<<UCSZ21));
}

void UART0Transmit(unsigned char data)
{
	//wait for transmit buffer to be empty
	while(!((UCSR0A)&(1<<UDRE0)));
	
	//send data
	UDR0=data;
}


void UART1Transmit(unsigned char data)
{
	//wait for transmit buffer to be empty
	while(!((UCSR1A)&(1<<UDRE1)));
	
	//send data
	UDR1=data;
}


void UART2Transmit(unsigned char data)
{
	//wait for transmit buffer to be empty
	while(!((UCSR2A)&(1<<UDRE2)));
	
	//send data
	UDR2=data;
}



unsigned char UART0Receive(void)
{
	char data=buffer;
	buffer=0;
	return data;
}

unsigned char UART1Receive(void)
{
	char data=buffer1;
	buffer1=0;
	return data;
}


unsigned char UART2Receive(void)
{
	char data=buffer2;
	buffer2=0;
	return data;
}

void UART0TransmitData(int data)
{
	char buffer[20];
	char *i=0;
	itoa(data,buffer,10);
	i=buffer;
	while(*i)
		UART0Transmit(*i++);
}

void UART1TransmitData(int data)
{
	char buffer[20];
	char *i=0;
	itoa(data,buffer,10);
	i=buffer;
	while(*i)
	UART1Transmit(*i++);
}

void UART2TransmitData(int data)
{
	char buffer[20];
	char *i=0;
	itoa(data,buffer,10);
	i=buffer;
	while(*i)
	UART2Transmit(*i++);
}


void UART0TransmitString(const char *s)
{
	while(*s)
		UART0Transmit(*s++);
}

void UART1TransmitString(const char *s)
{
	while(*s)
	UART1Transmit(*s++);
}


void UART2TransmitString(const char *s)
{
	while(*s)
	UART2Transmit(*s++);
}

ISR(USART0_RX_vect)
{
	buffer=UDR0;
	
}

ISR(USART1_RX_vect)
{
	buffer1=UDR1;
	
}

ISR(USART2_RX_vect)
{
	buffer2=UDR2;
	
}
