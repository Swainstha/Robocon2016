/*
 * TWI.h
 *
 * Created: 1/30/2016 9:43:30 AM
 *  Author: Swain
 */ 


#ifndef TWI_H_
#define TWI_H_
#define F_CPU 16000000UL

#include <util/twi.h>


#define SCL_CLOCK  100000L
#define I2C_WRITE   0
#define I2C_READ    1

void i2c_init(void);
void i2c_stop(void);
unsigned char i2c_start(unsigned char addr);
unsigned char i2c_rep_start(unsigned char addr);
unsigned char i2c_write(unsigned char data);
unsigned char i2c_readAck(void);
unsigned char i2c_readNak(void);



#endif /* TWI_H_ */
