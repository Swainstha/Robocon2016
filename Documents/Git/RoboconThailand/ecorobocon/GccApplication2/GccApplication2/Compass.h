/*
 * Compass.h
 *
 * Created: 6/13/2016 9:33:31 PM
 *  Author: Swain
 */ 


#ifndef COMPASS_H_
#define COMPASS_H_
#include <avr/io.h>
#include "TWI.h"

#define HMC5883L_WRITE 0x3C // write address  //7-bit address=0x1E
#define HMC5883L_READ 0x3D // read address

int16_t raw_x = 0;
int16_t raw_y = 0;
int16_t raw_z = 0;
int headingDegrees=0;
int heading_proxy=0;
int compass_offset=0;
float headingDegrees1=0;
float headingDegrees2=0;
float x_scale=0, y_scale=0,z_scale=0;
int16_t x_max=0;
int16_t y_max=0;
int16_t z_max=0;
int16_t x_min=0;
int16_t y_min=0;
int16_t z_min=0;
//int16_t x_offset=-2;//14 -2
//int16_t y_offset=22;//56 -58
int16_t x_offset=-2;//14 -2
int16_t y_offset=22;//56 -58
int16_t z_offset=0;

void init_HMC5883L(void){

	i2c_init();
	i2c_start(HMC5883L_WRITE);
	i2c_write(0x00); // set pointer to CRA 0x10
	i2c_write(0x70); // write 0x70 to CRA  Number of samples per output=8  ,data output rate=15
	i2c_stop();

	i2c_start(HMC5883L_WRITE);
	i2c_write(0x01); // set pointer to CRB  0x20
	i2c_write(0xA0); //gain=390  resolution(mg/LSB)=2.56  ,output= -2048-2047
	i2c_stop();

	i2c_start(HMC5883L_WRITE);
	i2c_write(0x02); // set pointer to measurement mode
	i2c_write(0x00); // continous measurement
	i2c_stop();
}

void read_Compass(void){

	i2c_start(HMC5883L_WRITE);
	i2c_write(0x03); //set pointer to X-axis MSB
	i2c_stop();

	i2c_rep_start(HMC5883L_READ);

	raw_x = ((uint8_t)i2c_readAck())<<8;
	raw_x |= i2c_readAck();
	
	raw_z = ((uint8_t)i2c_readAck())<<8;
	raw_z |= i2c_readAck();
	
	
	raw_y = ((uint8_t)i2c_readAck())<<8;
	raw_y |= i2c_readNak();
	
	i2c_stop();
}
void calibrate_compass()
{ 

	float x_avg=0,y_avg=0,z_avg=0;
	float all_avg=0;

	for(int i=0;i<1000;i++)
	{
		read_Compass();
		if(raw_x>x_max)
		x_max=raw_x;
		else if(raw_x<x_min)
		x_min=raw_x;
		if(raw_y>y_max)
		y_max=raw_y;
		else if(raw_y<y_min)
		y_min=raw_y;
		if(raw_z>z_max)
		z_max=raw_z;
		else if(raw_z<z_min)
		z_min=raw_z;
	_delay_ms(10);
	
	}
	x_offset=((x_max+x_min)/2);
	y_offset=((y_min +y_max)/2);
	z_offset=((z_min +z_max)/2);
//	x_raw-=(x_min + x_max)/2;
//	y_raw-=(y_min + y_max)/2;
//	z_raw-=(z_min + z_max)/2;
/*	
	x_max=x_max-((x_min+x_max)/2);
	y_max=y_max-((y_min+y_max)/2);
	z_max=z_max-((z_min+z_max)/2);	
	
	x_min=x_min-((x_min+x_max)/2);
	y_min=y_min-((y_min+y_max)/2);
	z_min=z_min-((z_min+z_max)/2);
	
	x_avg=(x_max-(x_min))/2;
	y_avg=(y_max-(y_min))/2;
	z_avg=(z_max-(z_min))/2;
	
	all_avg=(x_avg+y_avg+z_avg)/3;
	
	x_scale=all_avg/x_avg;
	y_scale=all_avg/y_avg;
	z_scale=all_avg/z_avg;
	*/
	
}
float getHeading()
{
	read_Compass();
	raw_x=(raw_x-x_offset);
	raw_y=(raw_y-y_offset);//*y_scale;
	raw_z=(raw_z-z_offset);//*z_scal
	// convert the raw data into a heading in degrees
	headingDegrees1 = atan2((double)raw_y,(double)raw_x)* 180 / 3.14159265 + 180;
	headingDegrees2=headingDegrees1-compass_offset;
	if(headingDegrees2<0)
		headingDegrees2=360+headingDegrees2;
	else if(headingDegrees2>360)
		headingDegrees2=headingDegrees2-360;
	return headingDegrees2;
}
void calculate_compass_offset()
{
	heading_proxy=getHeading();
	compass_offset=heading_proxy-180;
}



#endif /* COMPASS_H_ */
