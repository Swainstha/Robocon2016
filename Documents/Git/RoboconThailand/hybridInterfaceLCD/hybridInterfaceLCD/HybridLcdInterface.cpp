/*
 * HybridLcdInterface.cpp
 *
 * Created: 7/19/2016 10:38:21 PM
 *  Author: Niraj
 */ 

#define F_CPU 16000000UL
#define BAUD_RATE 9600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include "lcd.h"
#include "keypad.h"

#define RED 2
#define BLUE 4

#define INCREMENT 2 
#define DECREMENT 1
#define NOCHANGE 0

# define USART_BAUDRATE 9600
# define BAUD_PRESCALE ((( F_CPU / ( USART_BAUDRATE * 16UL))) - 1)

enum menu_states_t {HOME_SCREEN,CALIBRATE,CALIBRATE_MODE,MODE_CHO0SE,HYBRID_BASE_DISP,HYBRID_TOP_DISP,RETRY_MODE,HYBRID_BASE_SPEED,ENCODER_CHECK_DISP,LINE_PID_DISP,MOTOR_PID_DISP,MOTOR_CHECK_DISP,PROXIMITY_CHECK,JUNCTION_CHECK,
	                HYBRID_TOP_SEQUENCE,LIDAR_DISP,WIPERMOTOR_PID_DISP,GRIPPER_SERVO_DISP,DUCT_SERVO_DISP,DUCTFAN_DISP,POLOLU_ARM_DISP,POLOLU_GRIPPER_DISP,POLOLU_WIPER_DISP,TOP_MOTOR_CHECK_DISP};
enum states_t {HOME,BASE_DISP_LIST1,BASE_DISP_LIST2,BASE_DISP_LIST3,
	           ACT_A,ACT_B,ACT_C,ACT_D,ACT_E,ACT_F,ACT_G,ACT_H,ACT_I,ACT_J,ACT_K,ACT_L,ACT_M};
menu_states_t  present_menu;
menu_states_t  last_menu;
unsigned int present_state=HOME;
unsigned int  last_state=HOME;
menu_states_t list_states[15];
unsigned int list_index=0;

char buffer[15];
char buffer1[10];
char buffer2[10];
char buffer3[10];
unsigned char linereadenable=0;

volatile unsigned char displaycounter=0;
volatile unsigned char displayflag=0;
volatile unsigned char connection_checkflag=0;
unsigned char connection_status=0;
unsigned char connection_check_enable=1;
unsigned char duct_calibrate_complete=0;
unsigned char retry_mode=0;
unsigned char teamselect=0;
volatile unsigned int timeout=0;
volatile unsigned int datatimeout=0;
void init_timer0()
{
	TCCR0A|=(1<<WGM01);
	TCCR0B|=(1<<CS02)|(1<<CS00);
	TIMSK0|=(1<<OCIE0A);
	TIFR0|=(1<<OCF0A);
	OCR0A=154;
}
ISR(TIMER0_COMPA_vect)
{
	timeout++;
	displaycounter++;
	datatimeout++;
	if(displaycounter>=8)
	{
		connection_checkflag=1;
		displayflag=1;
		displaycounter=0;
	}
}
void transmitdata(unsigned char transmitdata)
{
	while( !( UCSR0A &(1<<UDRE0) ));
	UDR0=transmitdata;
}
unsigned char receivedata()
{
    unsigned char tempdata;
	timeout=0;
	while ( !(UCSR0A & (1<<RXC0)) )
	{
		if(timeout>3)
		return 0;
	}
	
	tempdata=UDR0;
	return tempdata;
}
unsigned char transceivedata(unsigned char transmitdata)
{
	unsigned char tempdata;
	while( !( UCSR0A &(1<<UDRE0) ));
	UDR0=transmitdata;
	timeout=0;
    TCNT0=0;
	while ( !(UCSR0A & (1<<RXC0)))
	{
		if(timeout>8)
		return 0;
	}
	tempdata=UDR0;
	return tempdata;
}
char* getUART_data(unsigned char transmit,char key )
{
	strcpy(buffer,"");
	unsigned int dataindex=0;
	char tempdata;
	while( !( UCSR0A &(1<<UDRE0) ));
	UDR0=transmit;
	datatimeout=0;
	while(1)
	{
		timeout=0;
		while (!(UCSR0A & (1<<RXC0)) )
		{
		if(timeout>8)
		{
			break;
		}	
		}
		tempdata=UDR0;
		if( tempdata=='#')
		{
			buffer[dataindex]='\0';
			break;
		}
		else if(datatimeout>=25)
		{
			strcpy(buffer,"");
			datatimeout=0;
			break;
		}
		buffer[dataindex]=tempdata;
		dataindex++;
	}
return buffer;
}
void clearbuffer()
{
	strcpy(buffer1,"");
	strcpy(buffer2,"");
	strcpy(buffer3,"");
}
void init_myuart( void )
{
	DDRD|=(1<<PIND1);   //tx as output
	DDRD&=~(1<<PIND0);  //rx as input
    UCSR0B=(1<<RXEN0)|(1<<TXEN0);
	UCSR0C=(1<<UCSZ01)|(1<<UCSZ00);
	UBRR0H=(BAUD_PRESCALE>>8);
	UBRR0L=BAUD_PRESCALE;
}
void displayhomescreen();
void calibratescreen();
void choosemode_screen();
void hybrid_base_screen();
void hybrid_top_screen();
void retry_mode_screen();
void linepid_screen();
void motorpid_screen();
void pole_check_screen();
void encoder_check_screen();
void basemotor_check_screen();
void proximity_check_screen();
void junction_check_screen();
void hybridbasespeed_screen();
void lidar_screen();
void wipermotor_pid_screen();
void pololu_arm_screen();
void pololu_gripper_screen();
void pololu_wiper_screen();
void ductfan_screen();
void ductservo_screen();
void gripper_screen();
void hybrid_top_sequence_screen();
void top_motor_check_screen();
int main(void)
{
	lcd_init();
	keypad_init();
	init_timer0();
	init_myuart();
	sei();
	list_states[list_index]=HOME_SCREEN;
	present_menu=HOME_SCREEN;
	lcd_clear();
	Printf(2,"hello");
    while(1)
    {
	    switch(get_key())
		{
			case BUTTON_0:present_menu=HOME_SCREEN;
			              list_index=0;
						  transmitdata('b');
						  _delay_ms(5);
						  transmitdata('1');
						  _delay_ms(5);
						  teamselect=0;
						  connection_check_enable=1;
						   _delay_ms(250);
						  break;	
		}
		switch(present_menu)
		{
			case HOME_SCREEN:     displayhomescreen();
			                      break;
			case CALIBRATE_MODE:  calibratescreen();
			                      break;
			case MODE_CHO0SE:     choosemode_screen();
			                      break;
			case HYBRID_BASE_DISP:hybrid_base_screen();
			                      break;
			case HYBRID_TOP_DISP: hybrid_top_screen();
			                      break;
			case RETRY_MODE: retry_mode_screen();
			                      break;				  
			case HYBRID_BASE_SPEED:hybridbasespeed_screen();
			                        break;
			case LINE_PID_DISP:   linepid_screen();
			                      break;
			case MOTOR_PID_DISP:  motorpid_screen();
			                      break;
			case ENCODER_CHECK_DISP:encoder_check_screen();
			                        break;
			case MOTOR_CHECK_DISP:basemotor_check_screen();           
			                      break;
			case PROXIMITY_CHECK:proximity_check_screen();
			                      break;
			case JUNCTION_CHECK:junction_check_screen();
			                    break;			
			case HYBRID_TOP_SEQUENCE:hybrid_top_sequence_screen();
			                         break;
			case POLOLU_ARM_DISP:pololu_arm_screen();
			                     break;
			case POLOLU_GRIPPER_DISP:pololu_gripper_screen();
			                     break;
			case POLOLU_WIPER_DISP:pololu_wiper_screen();
			                     break;
		    case WIPERMOTOR_PID_DISP:wipermotor_pid_screen();
			                     break;
			case LIDAR_DISP:lidar_screen();
			                     break;
			case DUCTFAN_DISP:ductfan_screen();
			                     break;
            case DUCT_SERVO_DISP:ductservo_screen();
			                     break;
			case GRIPPER_SERVO_DISP:gripper_screen();
			                     break;
	        case TOP_MOTOR_CHECK_DISP:top_motor_check_screen();
	        break;	
		}
		_delay_ms(10);
    }
}

void displayhomescreen()
{
	if(displayflag)
	{
		lcd_clear();
		if(teamselect)
		{
			if(duct_calibrate_complete)
			Printf(1,"Calibration Complete");
			else 
			Printf(1,"B to Calibrate Duct");
			Printf(2,"C to Calibrate Line");
			Printf(3,"  Press D to Start  ");
		}
		else
		{
			if(connection_status)
			Printf(2,"  STATUS:CONNECTED  ");
			else
			Printf(2,"STATUS:DISCONNECTED");
			
			Printf(1,"**HYBRID ROBO 2016**");
			Printf(3," RED-4        BLUE-6");
		}	
		if(teamselect==RED)
		   Printf(4,"  RED         MENU-#");
		else if(teamselect==BLUE)
		   Printf(4,"  BLUE       MENU-#");
	    else
		   Printf(4,"Calibrate-C   MENU-#");
		displayflag=0;
	}
	if(connection_checkflag && connection_check_enable)
	{
		if(transceivedata('5')!='6')
		{
	     connection_status=0;
		}
		else
		 connection_status=1;
		connection_checkflag=0;
	}
	switch(get_key())
	{
		case BUTTON_C:transmitdata('2');
		              present_menu=CALIBRATE_MODE;
					  clearbuffer();
					  list_states[++list_index]=present_menu;
					  _delay_ms(200);
					  break;
		case BUTTON_D:  if(teamselect==RED)    //send start command to hybrid base circuit
							{
								while(transceivedata('Z')!='#')
								{
									_delay_ms(2);
								}
								while(transceivedata('f')!='#')
								{
									_delay_ms(2);
								}
								connection_check_enable=0;
							}
						else if(teamselect==BLUE)
						{
							while(transceivedata('X')!='#')
							{
								_delay_ms(2);
							}
							while(transceivedata('f')!='#')
							{
								_delay_ms(2);
							}
							connection_check_enable=0;
						}
					    break;
						
		case BUTTON_B: Printf(1," Calibrating.....    ");
		               transmitdata('4');
		               transmitdata('4');
		               _delay_ms(5);
					   transmitdata('t');
					   _delay_ms(100);
					   datatimeout=0;
					   while(receivedata()!='c')
					   {
						   if(datatimeout>600)
						   break;
					   }
					   transmitdata('3');
					   transmitdata('3');
					   duct_calibrate_complete=1;	               
		               _delay_ms(100);
					   break;
	    case BUTTON_HASH:
		                 present_menu=MODE_CHO0SE;
						 list_states[++list_index]=present_menu;
						 _delay_ms(200);
						 break;
		case BUTTON_4:   //select red team
					  while(transceivedata('Z')!='#')
					  {
						  _delay_ms(5);
					  }
		              teamselect=RED;
					  connection_check_enable=0;
		              _delay_ms(200);
					  break;
		case BUTTON_6:   //select blue team
					  while(transceivedata('X')!='#')
					  {
						 _delay_ms(5);
					  }
					  teamselect=BLUE;
					  connection_check_enable=0;
					 _delay_ms(200);
		              break;
	}
}
void calibratescreen()
{	
	unsigned char displaycalibrating=0;
	switch(get_key())
	{
		case BUTTON_A:
		               present_menu=list_states[--list_index];
					  _delay_ms(200);
		              break;
		case BUTTON_5://send calibrate command to hybrid base
		               transmitdata('C');
					   displaycalibrating=1;
					   linereadenable=0;
					   displayflag=1;
					   _delay_ms(200);
		               break;
		case BUTTON_AST://send read command to base circuit to read line position
					   transmitdata('V');
					   displaycalibrating=0;
					    linereadenable=1;
						clearbuffer();
						strcpy(buffer1,getUART_data('A','#'));
						_delay_ms(5);
						strcpy(buffer2,getUART_data('B','#'));
						_delay_ms(5);
						strcpy(buffer3,getUART_data('N','#'));
						displayflag=1;				   
						_delay_ms(200);
		                break;
	}
	if(displayflag)
	{
		lcd_clear();
		Printf(1,"#Calib Mode#  ");
		if(displaycalibrating)
		{
		   Printf(2,"   Calibrating....");
		   _delay_ms(2000);
		}		   
		else
		{
		   Printf(2,"   Calibrate-5    ");
		}		   
		Printf(3,"LS1=%s  LS2=%s",buffer1,buffer2);
		Printf(4,"Line=%s    Read-*",buffer3);	
		displayflag=0;
	}
}
void choosemode_screen()
{
	unsigned int testdata;
	if(displayflag)
	{
		lcd_clear();
		Printf(1,"#Circuit Mode#  ");
		Printf(2,"1.Hybrid Base  ");
		Printf(3,"2.Hybrid Top");
		Printf(4,"3.Retry Mode  Back-A");
		displayflag=0;
	}
	testdata=get_key();
	switch(testdata)
	{
		case BUTTON_1:
		              present_menu=HYBRID_BASE_DISP;
					  transmitdata('3');
					  list_states[++list_index]=present_menu;
					  present_state=BASE_DISP_LIST1;
					  _delay_ms(200);
					  break;
		case BUTTON_2:
		              present_menu=HYBRID_TOP_DISP;
					  present_state=BASE_DISP_LIST1;
					  transmitdata('4');
					  _delay_ms(2);
					  transmitdata('Z');
					  _delay_ms(2);
					  list_states[++list_index]=present_menu;
					  _delay_ms(200);
		              break;
		case BUTTON_3:
		present_menu=RETRY_MODE;
		transmitdata('3');
		list_states[++list_index]=present_menu;
		_delay_ms(500);
		break;
		case BUTTON_A:present_menu=list_states[--list_index];
		              clearbuffer();
		              _delay_ms(200);
		              break;			  			  
	}
}
void retry_mode_screen()
{
	switch(get_key())
	{
		case BUTTON_1:
		retry_mode=1;
		_delay_ms(200);
		
		break;
		case BUTTON_2:
		retry_mode=2;
		transmitdata('4');
		transmitdata('4');
		_delay_ms(2);
		transmitdata('X');
		_delay_ms(200);
		break;
		
		case BUTTON_3:
		retry_mode=3;
		transmitdata('4');
		transmitdata('4');
		_delay_ms(2);
		transmitdata('X');
		_delay_ms(200);
		break;
		
		case BUTTON_A:present_menu=list_states[--list_index];
		clearbuffer();
		_delay_ms(200);
		break;
		case BUTTON_D:
		             if(retry_mode==1)
					 {
					   transmitdata('3');
					   transmitdata('3');
					   transmitdata('p');	 
					 }
					 
					 if(retry_mode==2)
					 {
						 transmitdata('3');
						 transmitdata('3');
						 transmitdata('q');
					 }
					 if(retry_mode==3)
					 {
						 transmitdata('3');
						 transmitdata('3');
						 transmitdata('r'); 
					 }
					
					 _delay_ms(500);
	}
	if(displayflag)
	{
		lcd_clear();
		if(!retry_mode)
		{
			Printf(1,"#Circuit Mode#");
			Printf(2,"1.Retry Mode 1");
			Printf(3,"2.Retry Mode 2");
			Printf(4,"3.Retry Mode 3");
		}
		else if(retry_mode==1)
		{
			Printf(1,"Press D to Start");
			Printf(3,"  Retry Mode 1  ");
		}
		else if(retry_mode==2)
		{
			Printf(1,"Press D to Start");
			Printf(3,"  Retry Mode 2  ");
		}
		else if(retry_mode==3)
		{
			Printf(1,"Press D to Start");
			Printf(3,"  Retry Mode 3  ");
		}
		displayflag=0;
	}
}
void hybrid_base_screen()
{
	if(displayflag)
	{
		lcd_clear();
		
		if(present_state==BASE_DISP_LIST1)
		{
		Printf(1,"#Hybrid Base# Back-A");
		Printf(2,"1.Hybrid basespeed");
		Printf(3,"2.Motor PID");
		Printf(4,"3.Line PID    Next-#");	
		}
		else if(present_state==BASE_DISP_LIST2)
		{
		Printf(1,"4.Enc Check   BACK-A");
		Printf(2,"5.Proximity Check");
		Printf(3,"6.Pole Check");	
		Printf(4,"7.Junction Check");
		}
		displayflag=0;
	}
	switch(get_key())
	{
		case BUTTON_1:transmitdata('1');
					present_menu=HYBRID_BASE_SPEED;
					clearbuffer();
					list_states[++list_index]=present_menu;
					_delay_ms(200);
					break;
		case BUTTON_2:transmitdata('1');
		              present_menu=MOTOR_PID_DISP;
					  clearbuffer();
					  list_states[++list_index]=present_menu;
					  _delay_ms(200);
		              break;
		case BUTTON_3:transmitdata('1');
		              present_menu=LINE_PID_DISP;
					  clearbuffer();
					  list_states[++list_index]=present_menu;
					  _delay_ms(200);
		              break;
		
		case BUTTON_4:if(present_state==BASE_DISP_LIST2)
		                {
		                transmitdata('2');
						clearbuffer();
						present_menu=ENCODER_CHECK_DISP;
						list_states[++list_index]=present_menu;
						}
						_delay_ms(200);
						break;
		case BUTTON_5:if(present_state==BASE_DISP_LIST2)
						{
							transmitdata('2');
							clearbuffer();
							present_menu=PROXIMITY_CHECK;
							list_states[++list_index]=present_menu;
						}
						_delay_ms(200);
						break;
		
		case BUTTON_6:if(present_state==BASE_DISP_LIST2)
		              {
					  transmitdata('3');
					  _delay_ms(5);
					  transmitdata('J');
					  clearbuffer();
		              //present_menu=MOTOR_CHECK_DISP;
					  //present_menu=POLE_CHECK_DISP;
					  //list_states[++list_index]=present_menu;
					  }
					  _delay_ms(200);
					  break;	
		case BUTTON_7:if(present_state==BASE_DISP_LIST2)
						{
							transmitdata('2');
							clearbuffer();
							present_menu=JUNCTION_CHECK;
							list_states[++list_index]=present_menu;
						}
						_delay_ms(200);
						break;
		  
		case BUTTON_HASH:last_state=present_state;
		                 present_state=BASE_DISP_LIST2;
						 _delay_ms(200);
		                 break;			  
		case BUTTON_A:
		              if(present_state==BASE_DISP_LIST2)
		              present_state=last_state;
					  else 
		              present_menu=list_states[--list_index];
					  clearbuffer();
					  _delay_ms(200);
		              break;
	}
}
void hybridbasespeed_screen()
{
	unsigned int test=get_key();
		switch(test)
		{
			case BUTTON_A:present_menu=list_states[--list_index];
			              clearbuffer();
			             _delay_ms(200);
			              break;
			case BUTTON_1:
			              strcpy(buffer1,getUART_data('t','#'));
						  _delay_ms(200);
			              displayflag=1;
			              break;
			case BUTTON_7:
			              strcpy(buffer1,getUART_data('g','#'));
						  _delay_ms(200);
			              displayflag=1;
			              break;
			case BUTTON_3:strcpy(buffer2,getUART_data('u','#'));
			              _delay_ms(200);
			              displayflag=1;
			              break;
			case BUTTON_9:strcpy(buffer2,getUART_data('j','#'));
			              _delay_ms(200);
			              displayflag=1;
			              break;
		}
		
		if(displayflag)
		{
			lcd_clear();
			Printf(1,"#Motor PID#  Back-A");
			Printf(2,"Norm = %s",buffer1);
			Printf(3,"Max = %s",buffer2);
			_delay_ms(200);
			displayflag=0;
		}
}
void encoder_check_screen()
{
	unsigned char readenable=0;
	
	switch(get_key())
	{
		case BUTTON_A:present_menu=list_states[--list_index];
		              _delay_ms(200);
	                  break;
		case BUTTON_AST:readenable=1;
		                displayflag=1;
						_delay_ms(200);
			            break;
	}
	if(displayflag)
	{
		lcd_clear();
		Printf(1,"#Encoder Data#  Back-A");
		if(readenable)
		{
			Printf(2,"DLt=%s   DRt=%s",getUART_data('D','#'),getUART_data('E','#'));
			_delay_ms(10);
			Printf(3,"VLt=%s   VRt=%s",getUART_data('F','#'),getUART_data('G','#'));
			_delay_ms(300);
			readenable=0;
		}
		Printf(4,"Read-*  ");
		displayflag=0;
	}
}

void linepid_screen()
{
	switch(get_key())
	{
		case BUTTON_A:present_menu=list_states[--list_index];
		              clearbuffer();
		              _delay_ms(200);
		break;
		case BUTTON_1:strcpy(buffer1,getUART_data('q','#'));
		              _delay_ms(200);
		              displayflag=1;
		              break;
		case BUTTON_7:strcpy(buffer1,getUART_data('a','#'));
		              _delay_ms(200);
		              displayflag=1;
		              break;
		case BUTTON_2:strcpy(buffer2,getUART_data('y','#'));
		              _delay_ms(200);
		              displayflag=1;
		              break;
		case BUTTON_8:strcpy(buffer2,getUART_data('h','#'));
		              _delay_ms(200);
		              displayflag=1;
		              break;
		case BUTTON_3:strcpy(buffer3,getUART_data('p','#'));
		              _delay_ms(200);
					  displayflag=1;
					  break;
		case BUTTON_9:strcpy(buffer3,getUART_data('l','#'));
		              _delay_ms(200);
		              displayflag=1;
		              break;
	}
	
	if(displayflag)
	{
		lcd_clear();
		Printf(1,"#Line PID#  Back-A");
		Printf(2,"Kp = %s",buffer1);
		Printf(3,"Ki = %s",buffer2);
		Printf(4,"Kd = %s",buffer3);
		displayflag=0;
	}
}
void motorpid_screen()
{
	switch(get_key())
	{
		case BUTTON_A:present_menu=list_states[--list_index];
		              clearbuffer();
					  _delay_ms(200);
		break;
		case BUTTON_1:strcpy(buffer1,getUART_data('Q','#'));
		_delay_ms(200);
		displayflag=1;
		break;
		case BUTTON_7:strcpy(buffer1,getUART_data('A','#'));
		_delay_ms(200);
		displayflag=1;
		break;
	
		case BUTTON_3:strcpy(buffer3,getUART_data('P','#'));
		_delay_ms(200);
		displayflag=1;
		break;
		case BUTTON_9:strcpy(buffer3,getUART_data('L','#'));
		_delay_ms(200);
		displayflag=1;
		break;
	}
	if(displayflag)
	{
		lcd_clear();
		Printf(1,"#Line PID#  Back-A");
		Printf(2,"Kp = %s",buffer1);
		Printf(3,"Kd = %s",buffer3);
		displayflag=0;
	}
}
void basemotor_check_screen()
{
	if(displayflag)
	{
		lcd_clear();
		Printf(1,"#Base Motor#  Back-A");
		Printf(2,"Front-2   Back-8");
		Printf(4,"Stop-*");
		displayflag=0;
	}
	switch(get_key())
	{
		case BUTTON_A:present_menu=list_states[--list_index];
		              clearbuffer();
					  _delay_ms(200);
		              break;
		case BUTTON_2:transmitdata('4');
		              _delay_ms(200);
		               break;
		case BUTTON_8:transmitdata('5');
		              _delay_ms(200);
		               break;
	    case BUTTON_AST:transmitdata('6');
		                _delay_ms(200);
						break;
	}
}
void proximity_check_screen()
{
	if(displayflag)
	{
		lcd_clear();
		Printf(1,"#PROXIMITY#   BACK-A");
		Printf(2,"2.Px Down = %s",getUART_data('K','#'));
		Printf(3,"3.Px Climb = %s",getUART_data('J','#'));
		displayflag=0;
	}
	switch(get_key())
	{
		case BUTTON_A:present_menu=list_states[--list_index];
		              clearbuffer();
					  _delay_ms(200);
		               break;
	}
}
void junction_check_screen()
{
	if(displayflag)
	{
		lcd_clear();
		Printf(1,"1.J LEFT = %s",getUART_data('H','#'));
		Printf(2,"2.J RIGHT = %s",getUART_data('I','#'));
		Printf(3,"3.JPULSELEFT = %s",getUART_data('L','#'));
		Printf(4,"4.JPULSERIGHT = %s=",getUART_data('M','#'));
		displayflag=0;
	}
	switch(get_key())
	{
		case BUTTON_A:present_menu=list_states[--list_index];
		              clearbuffer();
					  _delay_ms(200);
		break;
	}
}


void lidar_screen()
{
	switch(get_key())
	{
		case BUTTON_A:present_menu=list_states[--list_index];
		              clearbuffer();
		             _delay_ms(200);
		              break;
		
		case BUTTON_1:strcpy(buffer1,getUART_data('m','#'));
						_delay_ms(200);
						displayflag=1;
						break;
			
		case BUTTON_2:strcpy(buffer2,getUART_data('n','#'));
			_delay_ms(200);
			displayflag=1;
			break;
	}
	if(displayflag)
	{
		lcd_clear();
		Printf(1,"#LIDAR DATA#  BACK-A");
		Printf(2,"1.Lidar up = %s",buffer1);
		Printf(3,"2.Lidar Down = %s",buffer2);
		displayflag=0;
	}
	
}
void wipermotor_pid_screen()
{
	switch(get_key())
	{
		case BUTTON_A:present_menu=list_states[--list_index];
		clearbuffer();
		_delay_ms(200);
		break;
		case BUTTON_1:strcpy(buffer1,getUART_data('u','#'));
		_delay_ms(200);
		displayflag=1;
		break;
		case BUTTON_7:strcpy(buffer1,getUART_data('v','#'));
		_delay_ms(200);
		displayflag=1;
		break;
		case BUTTON_2:strcpy(buffer2,getUART_data('w','#'));
		_delay_ms(200);
		displayflag=1;
		break;
		case BUTTON_8:strcpy(buffer2,getUART_data('x','#'));
		_delay_ms(200);
		displayflag=1;
		break;
		case BUTTON_3:strcpy(buffer3,getUART_data('y','#'));
		_delay_ms(200);
		displayflag=1;
		break;
		case BUTTON_9:strcpy(buffer3,getUART_data('z','#'));
		_delay_ms(200);
		displayflag=1;
		break;
	}
	
	if(displayflag)
	{
		lcd_clear();
		Printf(1,"#Wiper PID#  Back-A");
		Printf(2,"Kp = %s",buffer1);
		Printf(3,"Ki = %s",buffer2);
		Printf(4,"Kd = %s",buffer3);
		displayflag=0;
	}
}
void pololu_arm_screen()
{
	switch(get_key())
	{
		case BUTTON_A:present_menu=list_states[--list_index];
		clearbuffer();
		_delay_ms(200);
		break;
		
		case BUTTON_1:strcpy(buffer1,getUART_data('k','#'));
		_delay_ms(200);
		displayflag=1;
		break;
		
		case BUTTON_2:strcpy(buffer2,getUART_data('k','#'));
		_delay_ms(200);
		displayflag=1;
		break;
	}
	if(displayflag)
	{
		lcd_clear();
		Printf(1,"#POLOLU ARM#  BACK-A");
		Printf(2,"1.Sensor data  = %s",buffer1);
		Printf(3,"2.Threshold =  %s",buffer2);
		displayflag=0;
	}
		
}
void pololu_gripper_screen()
{
	switch(get_key())
	{
		case BUTTON_A:present_menu=list_states[--list_index];
		clearbuffer();
		_delay_ms(200);
		break;
		
		case BUTTON_1:strcpy(buffer1,getUART_data('j','#'));
		_delay_ms(200);
		displayflag=1;
		break;
		
		case BUTTON_2:strcpy(buffer2,getUART_data('j','#'));
		_delay_ms(200);
		displayflag=1;
		break;
	}
	if(displayflag)
	{
		lcd_clear();
		Printf(1,"#POLOLU GRIP# BACK-A");
		Printf(2,"1.Sensor data  = %s",buffer1);
		Printf(3,"2.Threshold =  %s",buffer2);
		displayflag=0;
	}
	
}
void pololu_wiper_screen()
{
	switch(get_key())
	{
		case BUTTON_A:present_menu=list_states[--list_index];
		clearbuffer();
		_delay_ms(200);
		break;
		
		case BUTTON_1:strcpy(buffer1,getUART_data('l','#'));
		_delay_ms(200);
		displayflag=1;
		break;
		
		case BUTTON_2:strcpy(buffer2,getUART_data('l','#'));
		_delay_ms(200);
		displayflag=1;
		break;
	}
	if(displayflag)
	{
		lcd_clear();
		Printf(1,"#POLOLU WIPER#  BACK-A");
		Printf(2,"1.Sensor data  = %s",buffer1);
		Printf(3,"2.Threshold =  %s",buffer2);
		displayflag=0;
	}
	
}
void ductfan_screen()
{
	switch(get_key())
	{
		case BUTTON_A:present_menu=list_states[--list_index];
		clearbuffer();
		_delay_ms(200);
		break;
		
		case BUTTON_1:strcpy(buffer1,getUART_data('h','#'));
		_delay_ms(200);
		displayflag=1;
		break;
		
		case BUTTON_2:strcpy(buffer2,getUART_data('i','#'));
		_delay_ms(200);
		displayflag=1;
		break;
	}
	if(displayflag)
	{
		lcd_clear();
		Printf(1,"#DUCT FAN#  BACK-A");
		Printf(2,"  Calibrate-C");
		Printf(3,"2.Increase Speed=%s",buffer1);
		Printf(4,"8.Decrease Speed=%s",buffer2);
		displayflag=0;
	}
	
}
void ductservo_screen()
{
	switch(get_key())
	{
		case BUTTON_A:present_menu=list_states[--list_index];
		clearbuffer();
		_delay_ms(200);
		break;
		
		case BUTTON_2:strcpy(buffer1,getUART_data('o','#'));
		_delay_ms(200);
		displayflag=1;
		break;
		
		case BUTTON_8:strcpy(buffer2,getUART_data('p','#'));
		_delay_ms(200);
		displayflag=1;
		break;
	}
	if(displayflag)
	{
		lcd_clear();
		Printf(1,"#DUCT SERVO#  BACK-A");
		Printf(2,"2.Increase Speed=%s",buffer1);
		Printf(3,"8.Decrease Speed=%s",buffer2);
		displayflag=0;
	}
	
}
void gripper_screen()
{
	switch(get_key())
	{
		case BUTTON_A:present_menu=list_states[--list_index];
		clearbuffer();
		_delay_ms(200);
		break;
		
		case BUTTON_2:transmitdata('q');
		_delay_ms(200);
		displayflag=1;
		break;
		
		case BUTTON_8:transmitdata('r');
		_delay_ms(200);
		displayflag=1;
		break;
	}
	if(displayflag)
	{
		lcd_clear();
		Printf(1,"#GRIP ANGLE#  BACK-A");
		Printf(2,"2.Increase angle=%s",buffer1);
		Printf(3,"8.Decrease angle=%s",buffer2);
		displayflag=0;
	}	
}

void hybrid_top_sequence_screen()
{
	
	switch(get_key())
	{
		case BUTTON_A:
		              if(present_state!=ACT_A)
					  {
						  present_state--;
					  }
					  else
					  present_menu=list_states[--list_index]; 
		             _delay_ms(300);
		              break;
		
		case BUTTON_HASH:present_state++;
		                 if(present_state>ACT_M)
						 present_state=ACT_M;
						 else if(present_state<ACT_A)
						 present_state=ACT_A;
						_delay_ms(300);
						displayflag=1;
						break;
		
		case BUTTON_AST:
		                if(present_state==ACT_A)
		                {
						transmitdata('a');	
						}
						else if(present_state==ACT_B)
						{
						transmitdata('B');	
						}
						else if(present_state==ACT_C)
						{
						transmitdata('C');	
						}
						else if(present_state==ACT_D)
						{
							transmitdata('D');
						}
						else if(present_state==ACT_E)
						{
						transmitdata('E');	
						}
					    else if(present_state==ACT_F)
						{
						transmitdata('F');	
						}
						else if(present_state==ACT_G)
					    {
							transmitdata('G');
						}
						else if(present_state==ACT_H)
						{
						transmitdata('H');
				
						}
						else if(present_state==ACT_I)
						{
						transmitdata('I');	
						}
						else if(present_state==ACT_J)
						{
						transmitdata('J');
						}
						
						else if(present_state==ACT_K)
						{
						transmitdata('K');
						}
						else if(present_state==ACT_L)
						{
						transmitdata('L');	
						}
						else if(present_state==ACT_M)
						{
						transmitdata('M');
						}
		                _delay_ms(200);
		                break;
	}
	switch(present_state)
	{
		case ACT_A:strcpy(buffer1,"TASK A");
		           break;
	    case ACT_B:strcpy(buffer1,"TASK B");
	    break;
		case ACT_C:strcpy(buffer1,"TASK C");
		break;
		case ACT_D:strcpy(buffer1,"TASK D");
		break;
		case ACT_E:strcpy(buffer1,"TASK E");
		break;
		case ACT_F:strcpy(buffer1,"TASK F");
		break;
		case ACT_G:strcpy(buffer1,"TASK G");
		break;
		case ACT_H:strcpy(buffer1,"TASK H");
		break;
		case ACT_I:strcpy(buffer1,"TASK I");
		break;
		case ACT_J:strcpy(buffer1,"TASK J");
		break;
		case ACT_K:strcpy(buffer1,"TASK K");
		break;
		case ACT_L:strcpy(buffer1,"TASK L");
		break;
		case ACT_M:strcpy(buffer1,"TASK M");
		break;
	}
	if(displayflag)
	{
		lcd_clear();
		Printf(1,"#TOP SEQUEN#  BACK-A");
		Printf(2," -%s- ",buffer1);
		Printf(4,"Start-*     Next-#");
		displayflag=0;
	}
}
void hybrid_top_screen()
{
	if(displayflag)
	{
		lcd_clear();
		
		if(present_state==BASE_DISP_LIST1)
		{
			Printf(1,"#Hybrid Top# Back-A");
			Printf(2,"1.Top Sequence");
			Printf(3,"2.Lidar data");
			Printf(4,"3.Duct Servo    Next-#");
		}
		else if(present_state==BASE_DISP_LIST2)
		{
			Printf(1,"4.Duct fan   BACK-A");
			Printf(2,"5.Wiper PID");
			Printf(3,"6.Pololu arm");
			Printf(4,"7.Pololu wiper");
		}
		else if(present_state==BASE_DISP_LIST3)
		{
			Printf(1,"8.Pololu Grip BACK-A");
			Printf(2,"9.Servo Grip");
			Printf(3,"*.Top Motor Check");
		}
		displayflag=0;
	}
	
	switch(get_key())
	{
		case BUTTON_1:
		present_menu=HYBRID_TOP_SEQUENCE;
		present_state=ACT_A;
		clearbuffer();
		strcpy(buffer1,"TASK A");
		list_states[++list_index]=present_menu;
		_delay_ms(200);
		break;
		case BUTTON_2:
		present_menu=LIDAR_DISP;
		clearbuffer();
		list_states[++list_index]=present_menu;
		_delay_ms(200);
		break;
		case BUTTON_3:
		present_menu=DUCT_SERVO_DISP;
		clearbuffer();
		list_states[++list_index]=present_menu;
		_delay_ms(200);
		break;
		
		case BUTTON_4:
		               if(present_state==BASE_DISP_LIST2)
                 		{
							clearbuffer();
							present_menu=DUCTFAN_DISP;
							list_states[++list_index]=present_menu;
						}
						_delay_ms(200);
						break;
		case BUTTON_5:
						if(present_state==BASE_DISP_LIST2)
						{
							clearbuffer();
							present_menu=WIPERMOTOR_PID_DISP;
							list_states[++list_index]=present_menu;
						}
						_delay_ms(200);
						break;
		
		case BUTTON_6:
		                 if(present_state==BASE_DISP_LIST2)
							{
								clearbuffer();
								present_menu=POLOLU_ARM_DISP;
								list_states[++list_index]=present_menu;
							}
							_delay_ms(200);
							break;
		case BUTTON_7:
		              if(present_state==BASE_DISP_LIST2)
							{
								clearbuffer();
								present_menu=POLOLU_GRIPPER_DISP;
								list_states[++list_index]=present_menu;
							}
							_delay_ms(200);
							break;
	   	case BUTTON_8:
						   if(present_state==BASE_DISP_LIST3)
	   					{
		   					clearbuffer();
		   					present_menu=POLOLU_GRIPPER_DISP;
		   					list_states[++list_index]=present_menu;
	   					}
	   					_delay_ms(200);
	   					break;
	   	case BUTTON_9:
					   if(present_state==BASE_DISP_LIST3)
	   				{
		   				clearbuffer();
		   				present_menu=GRIPPER_SERVO_DISP;
		   				list_states[++list_index]=present_menu;
	   				}
	   				_delay_ms(200);
	   				break;
		case BUTTON_AST:
		               if(present_state==BASE_DISP_LIST3)
		                {
							present_menu=TOP_MOTOR_CHECK_DISP;
							list_states[++list_index]=present_menu;
						}
						_delay_ms(200);
						break;
		   	
		case BUTTON_HASH:present_state++;
						_delay_ms(200);
						break;
		case BUTTON_A:
						if(present_state!=BASE_DISP_LIST1)
						present_state--;
						else
						{
						present_menu=list_states[--list_index];
						transmitdata('a');
						_delay_ms(2);
						}
						clearbuffer();
						_delay_ms(200);
						break;
	}
}
void top_motor_check_screen()
{
	switch(get_key())
	{
		case BUTTON_A:present_menu=list_states[--list_index];
		clearbuffer();
		_delay_ms(200);
		break;
		
		case BUTTON_1:transmitdata('b');
		_delay_ms(200);
		displayflag=1;
		break;
		
		case BUTTON_2:transmitdata('d');
		_delay_ms(200);
		displayflag=1;
		break;
		case BUTTON_3:transmitdata('f');
		_delay_ms(200);
		displayflag=1;
		break;
	}
	if(displayflag)
	{
		lcd_clear();
		Printf(1,"#MOTORS CHECK# BACK-A");
		Printf(2,"1.Arm motion");
		Printf(3,"2.Gripper motion");
		Printf(4,"3.Wiper motion");
		displayflag=0;
	}
	
}