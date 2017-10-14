/*
 * uart.h
 *
 * Created: 5/4/2016 3:33:44 AM
 *  Author: 
 */ 


#ifndef UART_H_
#define UART_H_

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define BAUD_RATE2 9600
#define MYUBRR2 F_CPU/16/BAUD_RATE2-1

#define BAUD_RATE1 9600
#define MYUBRR1 F_CPU/16/BAUD_RATE1-1

#define BAUD_RATE0 9600
#define MYUBRR0 F_CPU/16/BAUD_RATE0-1

#define TIMEOUT 10

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>

void initUART0(void);
void UART0Transmit(unsigned char data);
unsigned char UART0Receive(void);
void UART0TransmitData(int data);
void UART0TransmitString(const char *s);

void initUART1(void);
void UART1Transmit(unsigned char data);
unsigned char UART1Receive(void);
void UART1TransmitData(int data);
void UART1TransmitString(const char *s);


void initUART2(void);
void UART2Transmit(unsigned char data);
unsigned char UART2Receive(void);
void UART2TransmitData(int data);
void UART2TransmitString(const char *s);



#endif /* UART_H_ */