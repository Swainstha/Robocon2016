/*
 * GccApplication2.cpp
 *
 * Created: 6/14/2016 11:29:04 PM
 *  Author: Swain
 */ 

#define F_CPU 16000000UL
#define pi 3.1415
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include "dynamixel.h"
#include "lcd.h"
#include "colorsensor.h"
#include "encoder.h"
#include "Compass.h"
#include <avr/eeprom.h>

#define GAME_FIELD_D DDRC
#define CHECK_POINT_3_D DDRD
#define GAME_FIELD_P PINC
#define CHECK_POINT_3_P PIND
#define GAME_FIELD 2
#define CHECK_POINT_3 7
#define NINETY_COUNT_LOOP_D DDRA
#define NINETY_COUNT_LOOP_P PINA
#define NINETY_COUNT_LOOP 3
#define NINETY_COUNT_D DDRC
#define NINETY_COUNT_P PINC
#define NINETY_COUNT 6

int red_field = -1;
//FOR ENCODER
int count=0;
float error1=0;
float error2=0;
float last_error1=0;
int check1=0;
int check2=0;
int check3=0;
int check4=0;

//FOR PID
float pid=0;
float errSum=0;
int error,lasterror=0;
int ITerm=0,dErr=0;
int dist_check=0;
float kp=0.005,ki=0.001,kd=30;
float kv=5;
float dist_power=2;
float Prop_power=2.5;
int power=0;
int kpp=0,kii=0,kdd=0,kvv=0;
float pidout=0;
int setPoint=45;

//int error1=0;
//colorsensor
int color=0;
int lastcolor=0;
int transition_count=-1;
int color_flag=1;

//FOR SERVO
int straight=122;
int true_straight=straight;
volatile float output=straight;
int output1=0;
float last_output=straight;
float lastoutput=straight;
int left_turn=0;
int right_turn=0;
int upper_limit=straight+90;
int lower_limit=straight-90;
int running=1;
int run1=0;
int speed1=0;
int speed2=0;
int turn=1;
//uint16_t angle_data=0;
//int angle=0;
float ka=0.04;
float angle_check=0;
//uint8_t data[10];
int encoder_flag=0;
int velocity_time=0;
float velocity=0;
float x=-23.5;
float y=0;
float y_abs=0;
float x_abs=0;
bool d_first=true;
//FOR LINETRACKER
uint8_t linestate=0;
int position=0;
double sum=0;
double sumvalue=0;
int b[8];
int weight[8]={80,70,60,50,40,30,20,10};

//FOR COMPASS
int compass_head_diff=0;
int river_compass_angle=0;
int last_reading=0;
int axis=69;
int compass_reading=0;
int last_diff=0;
int ninety_compass_angle=0;
int diff_ninety=0;

//Increment decrement
int press=0;

//FLAGS
//for downhill
int down_auto=0;
int down_normal_pid=0;
int down_turn=0;
int first_ninety_counter=1;

//river
int river_entry=0;//for river navigation
int river_enable=1;//for only one time river 
int river_loop=0;
int river_flag=0;

//for ninety turn
int ninety_counter=0;// ninety_color_count_flag
int first_junc=1; //90 degree first junction
int ninety_enable=1;//for one time 90 degree turn
int ninety_flag=0;
int ninety_flag1=0;
int ninety_y_abs_setpoint=30;
float ninety_y_abs_derivative_error=0;
float last_y_abs=0;
//for compass calibration
int compass_calibrate_flag=0;

//for game field
int track=0;


int downhill_flag=0; 

//FUNCTIONS
void init_all()
{
	init_encoder();
	init_timer();
	init_timer2();
	dynamixel_init();
	sei();
	lcd_init();
	init_color_sensor();
	init_HMC5883L();
	GAME_FIELD_D&=~(1<<GAME_FIELD);
	CHECK_POINT_3_D&=~(1<<CHECK_POINT_3);
	NINETY_COUNT_D&=~(1<<NINETY_COUNT);
	NINETY_COUNT_LOOP_D&=~(1<<NINETY_COUNT_LOOP);
}
void check_for_field_and_check_point()
{
	calculate_compass_offset();
	headingDegrees=getHeading();
	if (bit_is_clear(GAME_FIELD_P,GAME_FIELD))//for RIght gamefield
	{
		axis=headingDegrees-90;
		track=0;
		red_field=-1;
		river_compass_angle=251-compass_offset;
		if(river_compass_angle>360)
			river_compass_angle=river_compass_angle-360;
		else if(river_compass_angle<0)
			river_compass_angle=360+river_compass_angle;
	}
	else if(bit_is_set(GAME_FIELD_P,GAME_FIELD))//for left gamefield
	{
		axis=headingDegrees-90;
		track=1;
		red_field=1;
		river_compass_angle=71-compass_offset;
		if(river_compass_angle>360)
			river_compass_angle=river_compass_angle-360;
		else if(river_compass_angle<0)
			river_compass_angle=360+river_compass_angle;
	}
	lcd_clear();
	Printf("STARTING TRACK=%d",track);
	lcd_gotoxy(0,1);
	Printf("C=%d N=%d",headingDegrees,axis);
	_delay_ms(1000);
	if(bit_is_clear(CHECK_POINT_3_P,CHECK_POINT_3))
	{
		headingDegrees=getHeading();
		if(track==0)
			axis=headingDegrees-180;
		else if(track==1)
			axis=headingDegrees;
		lcd_clear();
		Printf("After ninety=%d",axis);
		_delay_ms(1000);
	}
	while(bit_is_clear(NINETY_COUNT_LOOP_P,NINETY_COUNT_LOOP))
	{
		if(bit_is_clear(NINETY_COUNT_P,NINETY_COUNT) && press==0)
		{
			ninety_counter+=1;
			press=1;
		}
		else if(bit_is_set(NINETY_COUNT_P,NINETY_COUNT))
		{
			press=0;
		}
		lcd_clear();
		Printf("%d",ninety_counter);
		_delay_ms(10);
	}
	if(bit_is_clear(NINETY_COUNT_P,NINETY_COUNT) && bit_is_set(NINETY_COUNT_LOOP_P,NINETY_COUNT_LOOP))//FOR COMPASS CALIBRATION
	{
		goal_position(upper_limit-30);
		_delay_ms(700);
		lcd_clear();
		Printf("Calibrating");
		calibrate_compass();
		compass_calibrate_flag=1;
	}
	while(compass_calibrate_flag==1)
	{
		headingDegrees=getHeading();
		lcd_clear();
		Printf("%d %d %d %d",headingDegrees,x_max,x_min,y_max);
		lcd_gotoxy(0,1);
		Printf("x=%d y=%d %d",x_offset,y_offset,y_min);
		_delay_ms(10);
		if(bit_is_clear(NINETY_COUNT_P,NINETY_COUNT))
		{
			compass_calibrate_flag=0;
		}	
	}
}
void receive_servo_angle()
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

}
void check_for_color_conditions()
{
	if(color_flag==1)
	{
		color=check_color();
	}
	if(color==1 && lastcolor==2)
	{
		counter=0;
		last_counter=0;
		down_auto=1;
		down_normal_pid=1;
		down_turn=1;
		first_junc=0;
		river_enable=0;
		ninety_enable=0;
	}
	else if (color==2 && lastcolor==1)
	{
		distance=0;
		counter=0;
		color_flag=0;
		river_entry=1;//start river navigation
	}
	
	else if (color==1 && lastcolor==3)
	{
		if(counter>1600 || first_ninety_counter==1)
			{	
			ninety_counter++;
			first_ninety_counter=0;
			}
		if(ninety_counter==3)//start ninety navigation
		{
			counter=0;
			last_counter=0;
		}
		counter=0;
	}
}
void check_for_downhill_conditions()
{
	if(down_auto==1 && down_normal_pid==1 && down_turn==1)
	{
		if(abs(counter-last_counter)<=2 && counter>400 && counter<1050)
		{
			kp=1.5;
			Prop_power=1;
			ki=0.003;
			kd=30;
		}
		else
		{
			kp=0.005;
			Prop_power=2.5;
			ki=0.004;
			kd=30;
			if(counter>1050)
				down_auto=0;
		}
	}
	else if( (down_auto==0 && down_normal_pid==1 && down_turn==1) && counter>5000)
	{
		running=1;
		kp=0.005;
		Prop_power=2.5;
		ki=0.002;
		kd=30;
		first_junc=0;
		if(track==0)
		{
			lower_limit=true_straight-35;
			upper_limit=true_straight;
		}
		else if(track==1)
		{
			lower_limit=true_straight-35;
			upper_limit=true_straight;
		}
		down_auto=0;
		down_normal_pid=0;
		down_turn=1;
	}
	else if(down_auto==0 && down_normal_pid==0 && down_turn==1 && counter>4200 && counter<=last_counter )
	{
		running=0;
		//goal_position(167);
		_delay_ms(800);
		while(1)
		{	
			if(track==0)
			{
				output=straight-100;
				goal_position(output);
			
			}
			else if(track==1)
			{
				output=straight+100;
				goal_position(output);
			}
			_delay_ms(50);
			lcd_clear();
			Printf("F %d %d",counter,check4);
			lcd_gotoxy(0,1);
			Printf("%d %d %d %d %d",(int)(ninety_y_abs_derivative_error*100),check1,check2,check3);
		}
	}
	
}
void ninety_turn_navigation()
{
	if(ninety_counter==3 && ninety_enable==1)
	{
		float Kp_compass=0.5;
		float Kp_line =1;
		float tspeed=0;
		float tempCount=0;
		running=0;
		x=0;
		y=-23.5*sin(75*pi/180)-8-10;
		x_abs=0;
		y_abs=0;
		counter=0;
		last_counter=0;
		ninety_flag1=1;
		while(1)
		{
			if(speed!=0)
			{
				tspeed+=speed;
				tempCount++;
				speed=0;
			}
			if(tempCount>=4)
			{
			 headingDegrees=getHeading();
			 x+=(((float)tspeed)/(8.18))*cos(((headingDegrees-axis)*pi)/180);
			 y+=(((float)tspeed)/(8.18))*sin(((headingDegrees-axis)*pi)/180);
			 tspeed=0;
			 y_abs = y+23.5*sin(((headingDegrees-axis)*pi)/180);
			 x_abs = x+23.5*cos(((headingDegrees-axis)*pi)/180);
			 tempCount=0;
			 if(ninety_y_abs_derivative_error<(y_abs-last_y_abs))
				ninety_y_abs_derivative_error=y_abs-last_y_abs;
			 last_y_abs=y_abs;
			}
			
			error1=(ninety_y_abs_setpoint-y_abs);
			if(error1 >ninety_y_abs_setpoint)
			error1=ninety_y_abs_setpoint;
			
			if(track==0)
				error2=(axis+180)-headingDegrees;
			else if(track==1)
				error2=headingDegrees-axis;
			
			
			Kp_compass = 0.55+(30-abs(error1))*(0.45)/(30);
			Kp_line = 1+(30-abs(error1))*(2.5-1)/30;
			//error2=240-headingDegrees;
			output = straight-(red_field)*error1*Kp_line+error2*(red_field)*Kp_compass+(red_field)*ninety_y_abs_derivative_error*0;
			
			if(track==0)
			{
				if(output>upper_limit-65)
					output=upper_limit-65;
				else if(output<lower_limit+45)
					output=lower_limit+45;
			}
			else if(track==1)
			{
				if(output>upper_limit-55)
					output=upper_limit-55;
				else if(output<lower_limit+65)
					output=lower_limit+65;
			}
			goal_position(output);
			lcd_clear();
			Printf("%d %d %d",(int)x_abs,(int)y_abs,headingDegrees);
			lcd_gotoxy(0,1);
			Printf("%d %d %d",(int)error1,(int)error2,(int)(Kp_compass*100));
			if(track==0)
			{
				//if((headingDegrees>((axis+180)-30) && headingDegrees<((axis+180)+30)) || ((PINB>2 && PINB<64) && ninety_flag1==1))
				if(y_abs>15 && (PINB>=2 && PINB<=64))
				{
					running=1;
					ninety_counter=4;
					ninety_enable=0;
					position=45;
					first_junc=0;
					setPoint=35;
					ninety_flag1==0;
					break;
					
				}
				else if(y_abs<25 && abs(x_abs)>40 && ninety_flag1==1)
				{
					ninety_y_abs_setpoint=ninety_y_abs_setpoint+5;
					ninety_flag1=0;
				}
			}
			else if(track==1)
			{
				if(y_abs>15 && (PINB>=2 && PINB<=64))
				{
					running=1;
					ninety_counter=4;
					ninety_enable=0;
					position=45;
					first_junc=0;
					ninety_flag1==0;
					running=1;
					setPoint=55;
					break;
				}
				else if(y_abs<25 && abs(x_abs)>40 && ninety_flag1==1)
				{
					ninety_y_abs_setpoint=ninety_y_abs_setpoint+5;
					ninety_flag1=0;
				}
			}
		}
	}
}
void calculate_river_distance()
{
	if(track==1)
	{
		dist_check=350-(int)(speed*kv);
		if(dist_check>=330)
		dist_check=330;
		else if(dist_check<250)
		dist_check=190;
		else if(dist_check<260)
		dist_check=205;
		else if(dist_check<270)
		dist_check=215;
		else if(dist_check<280)
		dist_check=230;
		else if(dist_check<290)
		dist_check=275;
		else if (dist_check<300)
		dist_check=285;
		else if(dist_check<310)
		dist_check=300;
		else if(dist_check<320)
		dist_check=310;
		else if(dist_check<330)
		dist_check=320;
	}
	else if(track==0)
	{
		/*dist_check=350-(int)(speed*kv);
		if(dist_check>=330)
		dist_check=330;
		else if(dist_check<250)
		dist_check=190;
		else if(dist_check<270)
		dist_check=205;
		else if(dist_check<280)
		dist_check=230;
		else if(dist_check<290)
		dist_check=275;
		else if (dist_check<300)
		dist_check=285;
		else if(dist_check<310)
		dist_check=300;
		else if(dist_check<320)
		dist_check=310;
		else if(dist_check<330)
		dist_check=320;*/
		dist_check=330-(int)(speed*kv);
		if(dist_check>=320)
		dist_check=320;
		else if(dist_check<240)
		dist_check=180;
		else if(dist_check<250)
		dist_check=195;
		else if(dist_check<260)
		dist_check=205;
		else if(dist_check<270)
		dist_check=220;
		else if(dist_check<280)
		dist_check=255;
		else if (dist_check<290)
		dist_check=275;
		else if(dist_check<300)
		dist_check=290;
		else if(dist_check<310)
		dist_check=300;
		else if(dist_check<320)
		dist_check=310;
	}
	
	
}
void ninety_optional_1()
{
	if(first_junc==1)
	{
		switch(linestate)
		{
			case 0b11111111:
			case 0b01111111:
			case 0b00111111:
			case 0b00011111:
			case 0b11111110:
			case 0b11111100:
			case 0b11111000:
			if(run1==0 && color==1 && ninety_enable==1)
			{
				run1=1;
				running=0;
				if(track==0)
				output=lower_limit+40;
				else if(track==1)
				output=upper_limit-40;
				goal_position(output);
				ninety_enable=0;
				
			}
			downhill_flag=1;
			break;
			case 0b00100000:
			case 0b00110000:
			case 0b00010000:
			case 0b00011000:
			case 0b00001000:
			case 0b00001100:
			case 0b00001110:
			case 0b00111100:
			case 0b00111000:
			case 0b00000110:
			case 0b01100000:
			case 0b01110000:
			case 0b00000010:
			if(run1==1 && color==1)
			{
				running=3;
				run1=0;
				first_junc=0;
			}
			break;
		}
	}
}
void ninety_turn_optional_2()
{
	if(ninety_counter==3 && counter>80 && ninety_enable==1)
	{
		running=0;
		while(1)
		{
			if(displayflag>2)
			{
				lcd_clear();
				Printf("C%d A%d D%d",headingDegrees,axis,diff_ninety);
				lcd_gotoxy(0,1);
				Printf("E=%d O=%d",counter,output);
				displayflag=0;
			}
			if(track==0)
			{
				if(!PINB)
				{
					ninety_flag=1;
				}
				else if (ninety_flag==1 && PINB<32 && PINB>0)
				{
					break;
				}
				if(compass_flag2>=1){
					headingDegrees=getHeading();
					compass_flag2=0;
				}
				diff_ninety=headingDegrees-(axis+180);
				output=straight+diff_ninety*1.2;
				if(output>(straight+30))
					output=straight+30;
				else if (output<straight-50)
					output=straight-50;
				goal_position(output);
			}
			else if(track==1)
			{
				if(!PINB)
				{
					ninety_flag=1;
				}
				else if(ninety_flag==1 && PINB>4)
				{
					break;
				}
				if(compass_flag2>=1)
				{
					headingDegrees=getHeading();
					compass_flag2=0;
				}
				diff_ninety=headingDegrees-axis;
				output=straight+diff_ninety*1.2;
				if(output<straight-30)
					output=straight-30;
				else if (output>straight+50)
					output=straight+50;
				goal_position(output);	
			}
			if(counter>=250 && (!PINB))
			{
				if(track==0)
					output=straight-35;
				else if(track==1)
					output=straight+35;
				last_output=output;
				position=45;
				goal_position(output);
				break;
			}
		}
	}
	running=1;
	ninety_counter=4;
	position=45;
	first_junc=0;
	ninety_enable=0;
}
float calc_PID()
{
	
	ITerm+=ki*error;
	if(ITerm>30)
	ITerm=30;
	else if(ITerm<-30)
	ITerm=-30;
	float dErr = (error - lasterror);
	//Compute PID Output
	if(error<0)
	{
		pidout = -kp * (pow(-error,Prop_power)) + ITerm + kd * dErr;
	}
	else
	{
		pidout = kp * pow(error,Prop_power) + ITerm + kd * dErr;
	}
	lasterror = error;
	return pidout;
}
void check_and_calculate_pid()
{
	if(running==1)
	{
		if(position>=10 && position<=80)
		{
			error=setPoint-position;
			pid=calc_PID();
			output=straight+pid;
			if(output<lower_limit)
			output=lower_limit;
			else if(output>upper_limit)
			output=upper_limit;
			goal_position(output);
			
		}
		else
		{
			output=last_output;
			goal_position(output);
		}
	}
		else if(running==4)
		{
			if(position>=10 && position<=80)
			{
				receive_servo_angle();
				error=setPoint-position;
				pid=calc_PID();
				output=angle+pid;
				if(output<lower_limit)
				output=lower_limit;
				else if(output>upper_limit)
				output=upper_limit;
				goal_position(output);
				
			}
			else
			{
				output=last_output;
				goal_position(output);
			}
		}

	else if(running==3)
	{
		if(position>=10 && position<=80)
		{
			error=setPoint-position;
			pid=calc_PID();
			if(track==0)
				output=straight-25+pid;
			else if (track==1)
				output=straight+25+pid;
			if(output<lower_limit)
				output=lower_limit;
			else if(output>upper_limit)
				output=upper_limit;
			goal_position(output);
			
		}
		else
		{
			output=last_output;
			goal_position(output);
		}
		if(track==1)
		{
			if(output>=straight && output<=straight+25)
			{
				running=1;
			}
		}
		else if (track==0)
		{
			if(output>=(straight-25) && output<=straight)
			{
				running=1;
			}
		}
	}
}
void river_navigation()
{
	int max_error=0;
	if(river_entry==1 && river_enable==1)
	{
		float river_setpoint = 28.0;
		float river_compassSetPoint = 0;
		float Kp_line=1.0;
		float Kd_line=150;
		float Kp_compass=1.0;
		float output_error=0;
		float tspeed=0;
		float tempCount=0;
		running=0;
		//counter=0;
		//last_counter=0;
		//lastoutput=output;
		x=-(red_field)*23.5;
		y=0;
		x_abs=0;
		speed=0;
		y_abs=0;
		counter=0;
		last_counter=0;
		while(abs(x_abs)<140)
		{
			if(speed!=0)
			{
				tspeed+=speed;
				tempCount++;
				speed=0;
				error1=river_setpoint-y_abs;
			if(error1>30)
			error1=30;
			if(d_first==true)
			{
				last_error1=error1;
				d_first=false;
			}
			//error2=(headingDegrees-axis);
			//error2=(headingDegrees-(axis+180));
			output_error=output-last_output;
			if(track==0)
			error2=(axis+180-river_compassSetPoint)-headingDegrees;
			else if(track==1)
			error2=headingDegrees-axis;
	
			/*
			if(abs(error1)<12)
			Kp_line = 1.5+ (12-abs(error1))*(2.5-1.5)/12;	
			else
			Kp_line = 1 + (30-abs(error1))*(1.5-1)/18;
			*/
			Kp_line = 2.5;
			Kd_line = 5;
			Kp_compass = 1;
			output=167-(red_field)*error1*Kp_line+(red_field)*(last_error1-error1)*Kd_line+(red_field)*error2*Kp_compass+(red_field)*output_error*0;
			if(max_error<(last_error1-error1))
				max_error=last_error1-error1;
			last_error1=error1;
			
		
		    if(output>(upper_limit-60))
			output=upper_limit-60;
			else if(output<(lower_limit+60))
			output=lower_limit+60;
			}
			if(tempCount>=4)
			{
			  headingDegrees=getHeading();
			  x+=(((float)tspeed)/(8.18))*cos(((headingDegrees-axis)*pi)/180);
			  y+=(((float)tspeed)/(8.18))*sin(((headingDegrees-axis)*pi)/180);
			  tspeed=0;
			  y_abs = y+23.5*sin(((headingDegrees-axis)*pi)/180);
			  x_abs = x+23.5*cos(((headingDegrees-axis)*pi)/180);
			  tempCount=0;
			}
			goal_position(output);
			last_output=output;
			lcd_clear();
			Printf("%d %d %d",(int)(x_abs*10),(int)(y_abs*10),headingDegrees);
			lcd_gotoxy(0,1);
			Printf("%d %d %d %d",(int)output,(int)(error1),(int)(error2),max_error);
		}
		river_entry=0;
		running=1;
		lcd_clear();
		position=0;
		river_enable=0;
		first_junc=0;
		output=straight;
		last_output=straight;
		river_flag=1;
		counter=0;
		last_counter=0;
		Printf("outside loop");
		while(1)
		{
			if(track==1)
			{
				if(PINB<64 && PINB>0)
				{
					break;
				}
			}
			if(track==0)
			{
				setPoint=65;
				lower_limit=true_straight+10;
				upper_limit=true_straight+90;
				straight=true_straight;
				//if(PINB>4)
				{
					break;
				}
			}
			
		}
	}
}
void navigation_check()
{
		float tspeed = 0;
		int tempCount=0;
		x=0;
		y=0;
		x_abs=0;
		speed=0;
		y_abs=0;
		counter=0;
		last_counter=0;
		while(1)
		{
			if(speed!=0)
			{
				tspeed+=speed;
			  	tempCount++;
				speed=0;
			}
			if(tempCount>=5)
			{
			headingDegrees=getHeading();
			x+=(((float)tspeed)/(8.18))*cos(((headingDegrees-axis)*pi)/180);
			y+=(((float)tspeed)/(8.18))*sin(((headingDegrees-axis)*pi)/180);
			tspeed=0;
			y_abs = y+23.5*sin(((headingDegrees-axis)*pi)/180);
			x_abs = x+23.5*cos(((headingDegrees-axis)*pi)/180);
			lcd_clear();
			Printf("%d %d %d",(int)(x_abs),(int)(y_abs),headingDegrees);
			lcd_gotoxy(0,1);
			Printf("%d %d %d %d",(int)(x),(int)y, axis, counter);
			tempCount=0;
			}
		}
}

void river_navigation_optional()
{
	if (river_entry==1 && counter >=dist_check && encoder_flag==0)
		 {
			 check3=speed;
			 check1=dist_check;
			 running=0;
			 encoder_flag++;
			 river_loop=1;
			 river_entry=0;
			 headingDegrees=getHeading();
			 check2=counter;
			 compass_flag=0;
		 }
		 while(river_loop)
		 {
			 //if(compass_flag>=1)
			 //{
				 headingDegrees=getHeading();
 				 //compass_flag=0;
			 //}
			encoder_flag=2;
			if(track==0)
				compass_head_diff=headingDegrees-river_compass_angle;
				//compass_head_diff=headingDegrees-(axis+180);
			else if(track==1)
				compass_head_diff=headingDegrees-river_compass_angle;
				//compass_head_diff=headingDegrees-(axis);
			output=(int)(straight+(compass_head_diff)*0.8);
			 if(output>(straight+60))
				output=(straight+60);
			else if (output<(straight-60))
				output=(straight-60);
			 goal_position(output);
			 if(counter>=(1150) && encoder_flag==2)//700
			 {
				running=1;
				lcd_clear();
				position=45;
				river_enable=0;
				river_entry=0;
				river_loop=0;
				first_junc=0;
				output=straight;
				last_output=straight;
				river_flag=1;
				Printf("outside loop");
				encoder_flag=3;
				while(1)
				{
					if(track==1)
					{
						setPoint=65;
						check4=counter;
						upper_limit=true_straight;
						lower_limit=true_straight-90;
						straight=true_straight;
						counter=0;
						last_counter=0;
						color_flag=1;
						break;
					}
					if(track==0)
					{
						setPoint=25;
						upper_limit=true_straight+90;
						lower_limit=true_straight;
						straight=true_straight;
						//if(PINB>4)
						//{
							check4=counter;
							counter=0;
							last_counter=0;
							color_flag=1;
							break;
						//}
					}
					
				}
				 
			 }
			 if(displayflag>=2)
			 {
				 lcd_clear();
				 Printf("E=%d C%d D%d",counter,(int)headingDegrees,compass_head_diff);
				 lcd_gotoxy(0,1);
				 Printf("%d %d %d",check1,check2,check3);
				 displayflag=0;
			 }
		}
}


int main(void)
{
	init_all();
	check_for_field_and_check_point();
	goal_position(straight);
	lcd_clear();
	Printf("STARTING...");
	lcd_gotoxy(0,1);
	Printf("TRACK=%d %d %d %d",track,headingDegrees,axis,ninety_counter);
	while(counter==last_counter);
	//navigation_check();
    while(1)
    {	
		check_for_color_conditions();	
		check_for_downhill_conditions();
		ninety_turn_navigation();
		if(river_flag==1 && counter>350)
		{
			river_flag=0;
			setPoint=45;
			straight=true_straight;;
			lower_limit=straight-90;
			upper_limit=straight+90;
		}
		else if(river_flag==1 && counter<300)
		{
			setPoint = 45 + (300-counter)*20/300;
		}
		if(ninety_counter==4 && counter>=500)
		{
			setPoint=45;
			ninety_counter=5;
		}
		//ninety_turn_optional_2();		
		calculate_river_distance();
		river_navigation_optional();
		//river_navigation();
		sum=0;
		sumvalue=0;
		linestate=0;
		for(int i=0;i<=7;i++)
		{
			if(bit_is_set(PINB,i))
			{
				b[i]=1;
				linestate|=(1<<i); 
			}
			else
				b[i]=0;
			sum+=(weight[i]*b[i]);
			sumvalue+=b[i];
		}
		position=sum/sumvalue;
		ninety_optional_1();
		check_and_calculate_pid();
		kpp=kp*10000;
		kii=ki*10000;
		kdd=kd;
		kvv=kv*10;
		power=Prop_power*100;
		if(displayflag>=1)
		{	
			lcd_clear();
			Printf("T=%d %d %d %d",track,color,heading_proxy,river_compass_angle);
			lcd_gotoxy(0,1);
			//Printf("%d %d %d %d %d %d %d",ninety_counter,first_junc,river_entry,down_auto,down_normal_pid,down_turn,river_flag);
			Printf("%d %d %d %d %d",(int)(ninety_y_abs_derivative_error*100),check1,check2,check3,check4);
			Printf("%d",counter);
			displayflag=0;
		}
		last_output=output;
		lastcolor=color;
	}
}
