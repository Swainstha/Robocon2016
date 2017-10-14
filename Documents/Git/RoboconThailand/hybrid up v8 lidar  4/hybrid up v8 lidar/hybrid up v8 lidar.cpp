/*
 * hybrid_v8.cpp
 *
 * Created: 6/22/2016 5:55:19 PM
 *  Author: grunze
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include "motor.h"
#include "servo.h"
#include "ductfan.h"
#include "uart.h"
#include "Lidar down.h"
#include "Lidar up.h"
#include "adc.h"
#include "PID.h"

//class defination
Motor Mot;
Servo Servoo;
Ductfan Duct;
Analog pololu;
Lidar_Down LidarDown;
Lidar_Up LidarUp;
PID LidarDownPid;

//functions
void Initilization_All_Value();
void Init_Limit_Switch();
void Reset_Position();	//servo initial point
void Wait_For_Start();	//wait for A value
void LidarDown_Height();
void LidarDown_PidCompute();
void Init_Timer4();
void Uart_Check1();	
void Uart_Check2();
void Arm_Extend_Height_Manage_Fun();
void River_Push_Fun();
void Reset_Height_To_Align_Eco_Fun();
void Gripping_Height_To_Max_Fun();
void Putting_Propeller_Fun();
void Run_Duct();
void Push_Eco();
void Wiper_Height_Reading();
void Gripper_Length();
void Starting_Checking();
void Lidar_Scanning();
void Gripper_Position();
void Arm_Manage_Hill();
void Pololu_Wiper_Height_Manage();
void Pololu_Wiper_Height_Manage_D();
void Pololu_Wiper_Height_Manage_D_Last();
void Lidar_Data_Calculate();
void Restart_Second_Fun();
void Restart_First_Fun();
void Pololu_Wiper(unsigned char Motor_Direction,unsigned char Wiper_Count,unsigned char Present_Value,unsigned char Previous_Value,int thershold,int difference,unsigned char Final_Count,unsigned char Direction);
void Pololu_Motor_A(unsigned char Motor_Direction,unsigned char Motor_A_Count,unsigned char Present_Value,unsigned char Previous_Value,int thershold,int difference,unsigned char Final_Count,unsigned char Direction);
void Pololu_Motor_B(unsigned char Motor_Direction,unsigned char Motor_B_Count,unsigned char Present_Value,unsigned char Previous_Value,int thershold,int difference,unsigned char Final_Count,unsigned char Direction);

//Init of activity flag
unsigned char Arm_Extend_Height_Manage=0;
unsigned char River_Push=0;
unsigned char Reset_Height_To_Align_Eco=0;
unsigned char Gripping_Height_To_Max=0;
unsigned char Putting_Propeller=0;


//pololu values
#define Motor_B_Thershold			650
#define Motor_B_Difference			75
#define Motor_A_Thershold			650
#define Motor_A_Difference			50
#define Wiper_Thershold				650
#define Wiper_Difference			75
#define Forward_Direction_Wiper		1
#define Forward_Direction_Motor_A	1
#define Forward_Direction_Motor_B	1
#define Downward_Direction_Wiper	2
#define Downward_Direction_Motor_A	2
#define Downward_Direction_Motor_B	2

//flag for interrupt
volatile unsigned char Timer_Start_Flag=0;
volatile unsigned char River_Timer_Count=0;
volatile unsigned char pidflag=0;
#define Start_River_Timer River_Timer_Count=0;Timer_Start_Flag=1;TCNT4=0;
#define Stop_River_Timer 	Timer_Start_Flag=0;River_Timer_Count=0;

//defination part
#define Max_Motor_Speed 1800
#define Min_Motor_Speed -1800

//Lidar hill transition variables
#define ONHILL 0
#define OFFHILL 1
unsigned int last_lidardowndata=0;
unsigned char lidar_hillstatus=0;
unsigned int hill_separation=0;
unsigned int Hillheight=0;
unsigned char first_run_init=1;

//PID VALUE
float KP=300;
float KD=0;
float KI=0;

//Ductfan 
unsigned char Rampflag=0;
unsigned char Ductrampstep=1;
unsigned int Ramp_Speed=70;
unsigned char Reverse_Flag=0;

//Servo max value and min value defination
int Servo_Duct_Min=400;
int Servo_Duct_Max=2400;
int Servo_Gripper_Min=300;
int Servo_Gripper_Max=2500;

//pololu variables for motor B
int Pololu_Present_Motor_B=0;
int Pololu_Previous_Motor_B=0;
int Junction_Motor_B=0;

//pololu variable for Motor A
int Pololu_Present_Motor_A=0;
int Pololu_Previous_Motor_A=0;
int Junction_Motor_A=0;
unsigned char Gripper_Length_Manage=0;

//pololu variable for Wiper
int Pololu_Present_Wiper=0;
int Pololu_Previous_Wiper=0;
int Junction_Wiper=0;
//other flag
unsigned char Last_Height_Down=0;
unsigned char River_Check=0;
unsigned char River_Push_Control=0;
int Vertical_Separation=38;
int Duct_Speed=0;
unsigned char Duct_Control_Flag=1;
unsigned char Duct_Constant_Flag=0;
unsigned char Duct_Stop_Flag=1;
float Lidar_Data=0;
int previous_angle=0;
int Gripper_Angle=100;
int Duct_Angle=176;
unsigned char Highland=0;

//lidar rotate flags
unsigned char Lidar_run=0;
float Lidar_Data1=0;
unsigned char Servo_Rotate_Angle=10;
unsigned char Lost_Eco=0;
float Last_Lidar_Data=0;

unsigned char Check_List=0;
unsigned char Main_Flag=0;

//lidar down flag
unsigned int LidarDown_Data=0;
unsigned char LidarDown_PidCompute_Flag=1;
int Pid_LidarDown=0;
unsigned char LidarDown_Flag=1;
int LidarDown_Last_Data=0;
unsigned char Gripper_Position_Manage=0;
unsigned char Last_Lidar_Data_Send=0;

unsigned char Restart_Flag_First=0;
unsigned char Restart_Flag_Second=0;
unsigned char Restart_Flag_Third=0;

//uart flags
unsigned char Case_A=1;
unsigned char Case_B=0;
unsigned char Case_C=0;
unsigned char Case_D=0;
unsigned char Case_E=0;
unsigned char Case_F=0;
unsigned char Case_G=0;
unsigned char Case_H=0;
unsigned char Case_I=0;
unsigned char Case_J=0;
unsigned char Case_K=0;
unsigned char Case_L=0;
unsigned char Case_M=0;
unsigned char Case_N=0;
unsigned char Case_W=1;
unsigned char Case_X=1;
unsigned char Case_Y=1;
unsigned char Case_Z=1;

unsigned char Last_Wiper_Present_Count=0;
unsigned char Last_Wiper_Prevoius_Count=0;
unsigned char Last_Motor_A_Present_Count=0;
unsigned char Last_Motor_A_Prevoius_Count=0;
unsigned char Last_Motor_B_Present_Count=0;
unsigned char Last_Motor_B_Prevoius_Count=0;
unsigned char Set_Value_Flag=0;
unsigned char wiper_up_Flag=0;

int main(void)
{
	Initilization_All_Value();
	sei();	
	Reset_Position();
	Duct.calibrate();
	LidarDown_Height();
	Wait_For_Start();		
			
   while(1)
   {
	   if (Main_Flag)
	   {
		   LidarDown_PidCompute();	   
		   if (Arm_Extend_Height_Manage )
		   {
			   Arm_Extend_Height_Manage_Fun();
		   }
		   if (River_Push)
		   {
			   River_Push_Fun();
		   }
		   if (Reset_Height_To_Align_Eco)
		   {
			   Reset_Height_To_Align_Eco_Fun();
		   }
		   if (Gripping_Height_To_Max)
		   {
			   Gripping_Height_To_Max_Fun();
		   }
		   if (Putting_Propeller)
		   {
			   Putting_Propeller_Fun();
		   }
		   if (!Duct_Stop_Flag)
		   {
			   Push_Eco();
		   }
	   
		   if (Gripper_Position_Manage)
		   {
			   Gripper_Position();
		   }
	   
		   if (Reverse_Flag)
		   {
				Arm_Manage_Hill();  
		   }
	   
		   if (Last_Height_Down)
		   {
				//Pololu_Wiper_Height_Manage_D_Last();   
		   }
		   if (Last_Lidar_Data_Send)
		   {
			   Lidar_Data_Calculate();
		   }
		   if (Restart_Flag_First)
		   {
			   Restart_First_Fun();
		   }
		   if (Restart_Flag_Second)
		   {
			   Restart_Second_Fun();
		   }
		   Uart_Check2();
	   }	
	   if (Check_List)
	   {
		   Uart_Check1();
	   }
	   	   
	   
   }
}

void Lidar_Data_Calculate()
{
	LidarDown_Data=LidarDown.Lidar_Down_Get_Data();
	if (!LidarDown_Data)
	{		
		if (LidarDown_Data>250)
		{
			UART2Transmit('L');
		}
	}
}

void Arm_Manage_Hill()
{
	Pololu_Motor_B(Reverse,Junction_Motor_B,Last_Motor_B_Present_Count,Last_Motor_B_Prevoius_Count,Motor_B_Thershold,Motor_B_Difference,5,Downward_Direction_Motor_B);
	 if (Junction_Motor_B==5)
	 {
		 Mot.Run(Motor_B,0);
		 Reverse_Flag=0;
	 }
}

void Lidar_Scanning()
{
	Lidar_Data1=LidarUp.Lidar_Up_Get_Data();
	
	if (Last_Lidar_Data==0 && Lidar_Data1==0 ) 
	{
		Lost_Eco++;
	}
	if (Lost_Eco>2)
	{
		Duct_Angle=Duct_Angle-Servo_Rotate_Angle-2;
		if (Duct_Angle<=91)
		{
			Duct_Angle=91;
		}
		Servoo.Rotate_Duct_Servo(Duct_Angle);
		Lost_Eco=0;
	}
	Last_Lidar_Data=Lidar_Data1;
	_delay_ms(10);
}

void Gripper_Position()
{
	Pololu_Motor_A(Reverse,Junction_Motor_A,Last_Motor_A_Present_Count,Last_Motor_A_Prevoius_Count,Motor_A_Thershold,Motor_A_Difference,4,Forward_Direction_Motor_A);
	if (Junction_Motor_A==4)
	{
		Mot.Run(Motor_A,0);
		Gripper_Position_Manage=0;
	}
}

void Gripper_Length()
{
	Pololu_Motor_A(Forward,Junction_Motor_A,Last_Motor_A_Present_Count,Last_Motor_A_Prevoius_Count,Motor_A_Thershold,Motor_A_Difference,2,Downward_Direction_Motor_A);
	if (Junction_Motor_A==2)
	{
		Mot.Run(Motor_A,0);
		Gripper_Length_Manage=0;
	}
	
}

void LidarDown_Height()
{
	LidarDownPid.Set_PID(KP,KI,KD);
	LidarDown_Flag=1;
	Vertical_Separation=38;
	
	while(1)
	{
		LidarDown_PidCompute();
		if((LidarDown_Data>37) && (LidarDown_Data<40))
		{
			Mot.Run(Wiper,0);
			LidarDown_PidCompute_Flag=0;
			break;
		}	
	}
		
}
//limit switch pcint enable 
void Init_Limit_Switch()
{
	DDRK &=~ ((1<<DDK6)|(1<<DDK7));
	PORTK |= ((1<<PK6)|(1<<PK7));
	PCICR |= (1<<PCIE2);
	PCMSK2 |= ((1<<PCINT22)|(1<<PCINT23));
}
//initilization of all function
void Initilization_All_Value()
{
	DDRH |= ((1<<PH3));
	PORTH &=~ ((1<<PH3));
	initUART1();
	initUART2();
	LidarDown.Init_Lidar_Down();
	LidarUp.Init_Lidar_Up();
	Init_Limit_Switch();
	Init_Timer4();
	pololu.ADC_Init();
	Mot.Initialize();
	Servoo.Initialize();
	Duct.Initialize();
	LidarDownPid.Initialize();
	LidarDownPid.Set_Range(Min_Motor_Speed,Max_Motor_Speed);
	LidarDownPid.Set_PID(KP,KI,KD);
	LidarDownPid.offset=0;
	Servoo.Set_Range_Duct_Servo(Servo_Duct_Min,Servo_Duct_Max);
	Servoo.Set_Range_Gripper_Servo(Servo_Gripper_Min,Servo_Gripper_Max);
}
//reset of duct and servo
void Reset_Position()
{
	Servoo.Rotate_Duct_Servo(Duct_Angle);
	PORTH &=~ ((1<<PH3));
	//Servoo.Rotate_Gripper_Servo(Gripper_Angle);
	PORTH |= ((1<<PH3));
}
//wait until the robot waits
void Wait_For_Start()
{
	unsigned char Run_Flag=1;
	unsigned char Uart2_Datta=0;
	
	while (Run_Flag)
	{
		Uart2_Datta=UART2Receive();	
		switch(Uart2_Datta)
		{
			case 'A':
			if (Case_A)
			{
				UART2Transmit('#');
				Run_Flag=0;
				Check_List=0;
				Main_Flag=1;
				Case_A=0;
				Case_B=1;			
			}
			break;
			
			case 'W':
			if (Case_W)
			{
				UART2Transmit('#');
				Restart_Flag_First=1;
				Set_Value_Flag=1;
				Main_Flag=1;
				Run_Flag=0;
				Check_List=0;
			}
			break;
			
			case 'X':
			if (Case_X)
			{
				UART2Transmit('#');
				Restart_Flag_Second=1;
				Set_Value_Flag=1;
				Main_Flag=1;
				Run_Flag=0;
				Check_List=0;
			}
			break;
			
			case 'Y':
			if (Case_Y)
			{
				UART2Transmit('#');
				Restart_Flag_Third=1;
				Set_Value_Flag=1;
				Main_Flag=1;
				Run_Flag=0;
				Check_List=0;
			}
			break;
			
			case 'Z':
			if (Case_Z)
			{
				UART2Transmit('#');
				Case_Z=0;
				Run_Flag=0;
				Check_List=1;
				Main_Flag=0;
			}
			break;
			
			
			
		}
	}
}
////Uart receive function
void LidarDown_PidCompute()
{
	if(LidarDown_PidCompute_Flag&&pidflag)
	{
		unsigned int LidarDown_TempData=0;
		LidarDown_Data=0;
		unsigned int com=0;
	
		for (int i=0;i<2;i++)
		{
			LidarDown_TempData=LidarDown.Lidar_Down_Get_Data();
			if (LidarDown_TempData>0)
			{
				LidarDown_Last_Data=LidarDown_TempData;
				LidarDown_Data=LidarDown_Data+LidarDown_TempData;
				com++;
		
			}
		}
		if (com)
		{
			LidarDown_Data=LidarDown_Data/com;
		}
			
		if (LidarDown_Data!=0)
		{			
			if(Highland)
			{
				if(first_run_init)
				{
					last_lidardowndata=Hillheight;
					first_run_init=0;
				}
				if(abs(LidarDown_Data-last_lidardowndata)>15)
				{
					lidar_hillstatus=!lidar_hillstatus;
				}
				if(lidar_hillstatus==ONHILL)
				{
					hill_separation=Vertical_Separation;
				}
				else if(lidar_hillstatus==OFFHILL)
				{
					hill_separation=Vertical_Separation+Hillheight;
				}
					
				last_lidardowndata=hill_separation;	
			}
			else
			{
				hill_separation=Vertical_Separation;
			}
			Pid_LidarDown = LidarDownPid.Compute_PID((float)LidarDown_Data,(float)hill_separation);
		
			if (Pid_LidarDown<0)
			{
				Pid_LidarDown=(0-Pid_LidarDown);
				Mot.Set_Motor_Direction(Wiper,Forward);
			}		
			else 
			{
				Mot.Set_Motor_Direction(Wiper,Reverse);
			}
		}
		
		if(LidarDown_Flag)
		{
			Mot.Run(Wiper,Pid_LidarDown);
		}
		else
		{
			Mot.Run(Wiper,0);
		}
			
	}
		pidflag=0;
}
void Uart_Check1()
{
	char UART2_Data_Check=0;
	UART2_Data_Check=UART2Receive();
	switch(UART2_Data_Check)
	{
		case 'a':
		Main_Flag=1;
		Check_List=0;
		Case_B=1;
		break;
		
		case 'b':	// move arm for 200ms forward
		Mot.Set_Motor_Direction(Motor_B,Forward);
		Mot.Run(Motor_B,2000);
		_delay_ms(200);
		Mot.Run(Motor_B,0);
		Mot.Set_Motor_Direction(Motor_B,Reverse);
		Mot.Run(Motor_B,2000);
		_delay_ms(200);
		Mot.Run(Motor_B,0);
		break;
		
		case 'c':	// move arm for 200ms reverse
		break;
		
		case 'd':	// move gripper for 200ms forward
		Mot.Set_Motor_Direction(Motor_A,Forward);
		Mot.Run(Motor_A,2000);
		_delay_ms(200);
		Mot.Run(Motor_A,0);
		Mot.Set_Motor_Direction(Motor_A,Forward);
		Mot.Run(Motor_A,2000);
		_delay_ms(200);
		Mot.Run(Motor_A,0);
		break;
		
		case 'e':	// move gripper for 200ms reverse
		break;
		
		case 'f':	// move wiper for 200ms forward
		Mot.Set_Motor_Direction(Wiper,Forward);
		Mot.Run(Wiper,2000);
		_delay_ms(200);
		Mot.Run(Wiper,0);
		Mot.Set_Motor_Direction(Wiper,Reverse);
		Mot.Run(Wiper,2000);
		_delay_ms(200);
		Mot.Run(Wiper,0);
		break;
		
		
		case 'g':	// move wiper for 200ms reverse
		break;
		
		case 'h':	//increase duct speed by 5% every one click
		Duct_Speed=Duct_Speed+5;
		Duct.run(Duct_Speed);
		UART2TransmitData(Duct_Speed);
		UART2Transmit('#');
		break;
		
		case 'i':	//decrease duct speed by 5% every one click
		Duct_Speed=Duct_Speed-5;
		Duct.run(Duct_Speed);
		UART2TransmitData(Duct_Speed);
		UART2Transmit('#');
		break;
		
		case 'j':	//gripper pololu data
		UART2TransmitData(pololu.Get_Junction_Sensor1(MOTOR_A));
		UART2Transmit('#');
		break;
		
		case 'k':	//arm pololu data
		UART2TransmitData(pololu.Get_Junction_Sensor1(MOTOR_B));
		UART2Transmit('#');
		break;
		
		case 'l':	//wiper pololu data
		UART2TransmitData(pololu.Get_Junction_Sensor1(WIPER));
		UART2Transmit('#');
		break;
		
		case 'm':	//lidar up data
		UART2TransmitData(LidarUp.Lidar_Up_Get_Data());
		UART2Transmit('#');
		break;
		
		case 'n':	//lidar down data
		UART2TransmitData(LidarDown.Lidar_Down_Get_Data());
		UART2Transmit('#');
		break;
		
		case 'o':	//duct servo angle increase by 5 degree
		Duct_Angle=Duct_Angle+5;
		Servoo.Rotate_Duct_Servo(Duct_Angle);
		UART2TransmitData(Duct_Angle);
		UART2Transmit('#');
		break;
		
		case 'p':	//duct servo angle decrease by 5 degree
		Duct_Angle= Duct_Angle-5;
		Servoo.Rotate_Duct_Servo(Duct_Angle);
		UART2TransmitData(Duct_Angle);
		UART2Transmit('#');
		break;
		
		case 'q':	//gripper servo angle increase by 5 degree
		Gripper_Angle= Gripper_Angle+5;
		Servoo.Rotate_Gripper_Servo(Gripper_Angle);
		UART2TransmitData(Gripper_Angle);
		UART2Transmit('#');
		break;
		
		case 'r':	//gripper servo angle decrease by 5 degree
		Gripper_Angle= Gripper_Angle-5;
		Servoo.Rotate_Gripper_Servo(Gripper_Angle);
		UART2TransmitData(Gripper_Angle);
		UART2Transmit('#');
		break;
		
		case 's':	//limit switch
		UART2TransmitData(LidarDown_Flag);
		UART2Transmit('#');
		break;
		
		case 't':
		break;
		
		case  'u':
		KP=KP+10;
		LidarDownPid.Set_PID(KP,KI,KD);
		UART2TransmitData(KP);
		UART2Transmit('#');
		break;
		
		case  'v':
		KP=KP-10;
		LidarDownPid.Set_PID(KP,KI,KD);
		UART2TransmitData(KP);
		UART2Transmit('#');
		break;
		
		case 'w':
		KI=KI+0.1;
		LidarDownPid.Set_PID(KP,KI,KD);
		UART2TransmitData((KI*10));
		UART2Transmit('#');
		break;
		
		case 'x':
		KI=KI-0.1;
		LidarDownPid.Set_PID(KP,KI,KD);
		UART2TransmitData((KI*10));
		UART2Transmit('#');
		break;
		
		case  'y':
		KD=KD+10;
		LidarDownPid.Set_PID(KP,KI,KD);
		UART2TransmitData(KD);
		UART2Transmit('#');
		break;
		
		case  'z':
		KD=KD+5;
		LidarDownPid.Set_PID(KP,KI,KD);
		UART2TransmitData(KD);
		UART2Transmit('#');
		break;
	}
	
	UART2_Data_Check=0;
}
void Uart_Check2()
{	
	switch(UART2Receive())
	{
		case 'B':				//before junction 1
		if (Case_B)
		{
			UART2Transmit('#');
			Arm_Extend_Height_Manage=1;
			Case_B=0;
			Case_C=1;
		}
		break;
		
		case 'C':				//after rotation in first junction
		if (Case_C)
		{
			UART2Transmit('#');
			Duct_Stop_Flag=0;
			Duct_Speed=10;
			Hillheight=19;
			Highland=1;
			Case_C=0;
			Case_D=1;
		}
		break;
		
		case 'D':				//junction 2
		if (Case_D)
		{
			UART2Transmit('#');
			LidarDown_PidCompute_Flag=1;
			LidarDownPid.Set_PID(400,0,0);
			Vertical_Separation=23;
			LidarDown_Flag=1;
			Hillheight=20;
			Case_D=0;
			Case_E=1;
		}
		break;
			
		case 'E':				//junction 3
		if (Case_E)
		{
			UART2Transmit('#');
			Vertical_Separation=30;
			Duct_Speed=60;
			Hillheight=20;
			Reverse_Flag=1;
			Set_Value_Flag=1;
			Case_E=0;
			Case_F=1;
		}
		break;
		
		case 'F':				//junction 4
		if (Case_F)
		{
			UART2Transmit('#');
			Duct_Speed=70;
			Hillheight=40;
			Case_F=0;
			Case_G=1;
		}
		break;
		
		case 'G':				//junction 5
		if (Case_G)
		{
			UART2Transmit('#');
			Duct_Speed=70;
			Duct_Angle=160;
			Servoo.Rotate_Duct_Servo(Duct_Angle);
			Vertical_Separation=30;
			Hillheight=40;
			Case_G=0;
			Case_H=1;
		}
		break;
		
		case 'H':				//junction 6
		if (Case_H)
		{
			UART2Transmit('#');
			Lidar_run=1;
			Duct_Angle=167;
			Servoo.Rotate_Duct_Servo(Duct_Angle);
			Vertical_Separation=25;
			Hillheight=60;
			Case_H=0;
			Case_I=1;
		}
		break;
		
		case 'I':				//junction 7
		if (Case_I)
		{
			UART2Transmit('#');
			Hillheight=60;
			Case_I=0;
			Case_J=1;
		}
		break;
		
		case 'J':		//junction 8
		if (Case_J)
		{
			UART2Transmit('#');
			Lidar_run=0;
			Hillheight=60;
			River_Push=1;
			Highland=0;
			Case_J=0;
			Case_K=1;
		}
		break;
		
		case 'K':				// after junction 8 before junction 9
		if (Case_K)
		{
			UART2Transmit('#');
			LidarDown_PidCompute_Flag=1;
			LidarDownPid.Set_PID(250,0,0);
			Reset_Height_To_Align_Eco=1;
			Case_K=0;
			Case_L=1;
		}
		break;
		
		case 'L':				// at pole
		if (Case_L)
		{
			UART2Transmit('#');
			Gripping_Height_To_Max=1;
			Case_L=0;
			Case_M=1;
		}
		break;	
		
		case 'M':				//while putting propler
		if (Case_M)
		{
			UART2Transmit('#');
			Putting_Propeller=1;
			Case_M=0;
		}
		break;
		
		case 'N':
		if (Case_N)
		{
			UART2Transmit('#');
			//Restart_First=1;
		}
		
	}
}
//run and stop duct
void Ramp_Duct(int targetspeed)
{
	int sign=0;
	if(targetspeed>Duct_Speed)
	sign=1;
	else
	sign=-1;
	if(Rampflag)
	{
			if(abs(targetspeed-Duct_Speed)>Ductrampstep)
			{
				Duct_Speed=Duct_Speed+Ductrampstep*sign;
				Rampflag=0;
			}
			else
			{
				Duct_Speed=targetspeed;
				Rampflag=0;
				Duct_Constant_Flag=1;
			}
	}
	Duct.run(Duct_Speed);
}
//push eco
void Push_Eco()
{
	
	if (!Duct_Constant_Flag)
	{
		Ramp_Duct(Ramp_Speed);
	}
	if (Duct_Constant_Flag)
	{
		Duct.run(Duct_Speed);
	}
		
}
//activity 3 case D
void River_Push_Fun()
{
	unsigned char Zone_One_Flag=0;
	float last_lidar_data=0;
	float last_reading=0;
	int sign=1;
	unsigned int empty_data_count=0;
	Duct_Speed=70;
	Duct.run(Duct_Speed);
	River_Check=1;
	Duct_Angle=91;
	previous_angle=Duct_Angle+8;
	Servoo.Rotate_Duct_Servo(Duct_Angle);
	Mot.Run(Wiper,0);
	LidarDown_Flag=0;
	Start_River_Timer;
	while(River_Push)
	{
		if(last_lidar_data>220)
		{
			Duct_Speed=85;
			Duct.run(Duct_Speed);
		}
		
		if (((last_lidar_data>150) && (last_lidar_data<220))||(River_Timer_Count>=50))
		{
			Zone_One_Flag=1;
			Duct_Speed=80;
			Duct.run(Duct_Speed);
		}
		
		last_reading=Lidar_Data;
		Lidar_Data=LidarUp.Lidar_Up_Get_Data();
		
		if(Lidar_Data>last_lidar_data)
		last_lidar_data=Lidar_Data;
		
		if(Lidar_Data==0 && last_reading==0)
		empty_data_count++;
		if(empty_data_count>2)
		{
			if((last_lidar_data>180)||(River_Timer_Count>=50))
			{
				while(Lidar_Data<=last_lidar_data)
				{
					if (River_Push_Control)
					{
						break;
					}
					Duct_Angle=Duct_Angle+sign;
					if(Duct_Angle>previous_angle)
					Duct_Angle=previous_angle;
					Servoo.Rotate_Duct_Servo(Duct_Angle);
					Lidar_Data=LidarUp.Lidar_Up_Get_Data();
					_delay_ms(20);
				}
			}
			empty_data_count=0;
		}
		if (((last_lidar_data>360 && last_lidar_data<430) && Zone_One_Flag )||River_Push_Control)
		{
			UART2Transmit('s');
			Duct_Constant_Flag=1;
			River_Push_Control=0;
			River_Push=0;
			Stop_River_Timer;
		}
		_delay_ms(20);
	}
	
}
//activity 4 case E
void Reset_Height_To_Align_Eco_Fun()
{ 
	unsigned char Down_Flag=0;
	if (LidarDown.Lidar_Down_Get_Data()>50)
	{
		Junction_Wiper=0;
		Last_Wiper_Present_Count=0;
		Last_Wiper_Prevoius_Count=0;
		Junction_Motor_A=0;
		Last_Motor_A_Present_Count=0;
		Last_Motor_A_Prevoius_Count=0;
		
		
		Duct_Angle=176;
		Servoo.Rotate_Duct_Servo(Duct_Angle);
		Duct_Speed=0;
		Duct.run(Duct_Speed);
		Duct_Stop_Flag=1;
		Duct_Constant_Flag=0;
		LidarDown_Flag=1;
		Set_Value_Flag=1;
		while (Reset_Height_To_Align_Eco)
		{
			Pololu_Wiper(Forward,Junction_Wiper,Last_Wiper_Present_Count,Last_Wiper_Prevoius_Count,Wiper_Thershold,Wiper_Difference,4,Forward_Direction_Wiper);
			if (Junction_Wiper==4)
			{
				Mot.Run(Wiper,0);							
				Down_Flag=1;
				Set_Value_Flag=1;
			}
			if (Down_Flag)
			{
				Pololu_Motor_B(Reverse,Junction_Motor_B,Last_Motor_B_Present_Count,Last_Motor_B_Prevoius_Count,Motor_B_Thershold,Motor_B_Difference,4,Downward_Direction_Motor_B);				
				if (Junction_Motor_B==4)
				{
					Mot.Run(Motor_B,0);
					Mot.Run(Wiper,0);
					Reset_Height_To_Align_Eco=0;
					Gripper_Angle=80;
					PORTH &=~ (1<<PH3);
					Servoo.Rotate_Gripper_Servo(Gripper_Angle);
					Gripper_Position_Manage=1;
					Set_Value_Flag=1;
					LidarDown_PidCompute_Flag=0;
				}
			}
			
		}
	}
}
//activity 5 case F
void Gripping_Height_To_Max_Fun()
{
	unsigned char Height_Flag=1;
	unsigned char Some_Flag=0;
	unsigned char Arm_Flag=0;
	PORTH &=~ ((1<<PH3));
	Gripper_Angle=51;
	Servoo.Rotate_Gripper_Servo(Gripper_Angle);
	UART2Transmit('Y');
	_delay_ms(300);
	PORTH |= ((1<<PH3));
	LidarDown_Flag=1;
	Set_Value_Flag=1;
	while (Gripping_Height_To_Max)
	{
		if (Height_Flag)
		{
			Pololu_Wiper(Reverse,Junction_Wiper,Last_Wiper_Present_Count,Last_Wiper_Prevoius_Count,Wiper_Thershold,Wiper_Difference,6,Forward_Direction_Wiper);
			if ((Junction_Wiper==6)||(LidarDown_Flag==0))
			{
				Height_Flag=0;
				Arm_Flag=1;
				Set_Value_Flag=1;
			}
		}
		
		if (Arm_Flag)
		{
			Pololu_Motor_B(Reverse,Junction_Motor_B,Last_Motor_B_Present_Count,Last_Motor_B_Prevoius_Count,Motor_B_Thershold,Motor_B_Difference,3,Downward_Direction_Motor_B);
			if (Junction_Motor_B==3)
			{
				Mot.Run(Motor_A,0);
				Mot.Run(Motor_B,0);
				Mot.Run(Wiper,0);
				LidarDown_PidCompute_Flag=0;
				Gripping_Height_To_Max=0;
				LidarDown_Flag=0;
				Arm_Flag=0;
				Last_Lidar_Data_Send=1;
			}
		}
		Lidar_Data_Calculate();
	}	
}
//activity case last
void Putting_Propeller_Fun()
{
	Mot.Run(Wiper,0);
	LidarDown_Flag=0;
	Set_Value_Flag=1;
	while(Putting_Propeller)
	{
		Pololu_Motor_B(Reverse,Junction_Motor_B,Last_Motor_B_Present_Count,Last_Motor_A_Prevoius_Count,Motor_B_Thershold,Motor_B_Difference,0,Downward_Direction_Motor_B);
		if (Junction_Motor_B==0)
		{
			Putting_Propeller=0;
			Mot.Run(Motor_B,0);
			PORTH &=~ (1<<PH3);
			Gripper_Angle=75;
			Servoo.Rotate_Gripper_Servo(Gripper_Angle);
			_delay_ms(500);
			Mot.Set_Motor_Direction(Motor_A,Forward);
			Mot.Run(Motor_A,2000);
			_delay_ms(500);
			Mot.Run(Motor_A,0);
			Mot.Set_Motor_Direction(Motor_B,Forward);
			Mot.Run(Motor_B,2000);
			_delay_ms(900);
			Mot.Run(Motor_B,0);
			Mot.Set_Motor_Direction(Wiper,Reverse);
			Mot.Run(Wiper,1800);
			_delay_ms(300);
			Mot.Run(Wiper,0);
			//Last_Height_Down=1;
			//Downward_Direction1=1;
		}
	}
}
//activity 1 case A
void Arm_Extend_Height_Manage_Fun()
{
	Set_Value_Flag=1;
	while(Arm_Extend_Height_Manage)
	{
		LidarDown_PidCompute();
		
		Pololu_Motor_B(Forward,Junction_Motor_B,Last_Motor_B_Present_Count,Last_Motor_B_Prevoius_Count,Motor_B_Thershold,Motor_B_Difference,7,Forward_Direction_Motor_B);		
		UART2TransmitData(Junction_Motor_B);
		UART2TransmitString("\t");
		UART2TransmitData(Pololu_Present_Motor_B);
		UART2TransmitString("\t");
		UART2TransmitData(Pololu_Previous_Motor_B);
		UART2TransmitString("\n\r");
		if (Junction_Motor_B==5)
		{
			LidarDown_PidCompute_Flag=1;
			Vertical_Separation=19;
		}
 		if (Junction_Motor_B==7)
 		{
			 Junction_Motor_B=8;    //remove it after
 	 		 Mot.Run(Motor_B,0);
 			 Arm_Extend_Height_Manage=0;
			 Vertical_Separation=19; 
 		}
		
	}
}

void Restart_First_Fun()
{
	if (wiper_up_Flag==0)
	{
		Pololu_Motor_B(Forward,Junction_Motor_B,Last_Motor_B_Present_Count,Last_Motor_B_Prevoius_Count,Motor_B_Thershold,Motor_B_Difference,4,Forward_Direction_Motor_B);
		if (Junction_Motor_B==4)
		{
			Mot.Run(Motor_B,0);
			wiper_up_Flag=1;
			Set_Value_Flag=1;
		}
	}
	if (wiper_up_Flag==1)
	{
		Pololu_Wiper(Reverse,Junction_Wiper,Last_Wiper_Present_Count,Last_Wiper_Prevoius_Count,Wiper_Thershold,Wiper_Difference,4,Forward_Direction_Wiper);
		if (Junction_Wiper==4)
		{
			Mot.Run(Wiper,0);
			Restart_Flag_First=0;			
			Case_K=1;
				
		}
	}
	
}

void Restart_Second_Fun()
{
	Pololu_Motor_B(Forward,Junction_Motor_B,Last_Motor_B_Present_Count,Last_Motor_B_Prevoius_Count,Motor_B_Thershold,Motor_B_Difference,3,Forward_Direction_Motor_B);
	if (Junction_Motor_B==3)
	{
		Mot.Run(Motor_B,0);
		wiper_up_Flag=1;
		Set_Value_Flag=1;
	}
	if (wiper_up_Flag)
	{
		Pololu_Wiper(Reverse,Junction_Wiper,Last_Wiper_Present_Count,Last_Wiper_Prevoius_Count,Wiper_Thershold,Wiper_Difference,2,Forward_Direction_Wiper);
		if (Junction_Wiper==2)
		{
			Mot.Run(Wiper,0);
			Case_M=1;
			Set_Value_Flag=1;
			Gripper_Position_Manage=1;
			Restart_Flag_Second=0;
		}
	}
}

void Pololu_Wiper(unsigned char Motor_Direction,unsigned char Wiper_Count,unsigned char Present_Value,unsigned char Previous_Value,int thershold,int difference,unsigned char Final_Count,unsigned char Direction)
{
	if (Set_Value_Flag)
	{
		Pololu_Previous_Wiper=Previous_Value;
		Pololu_Previous_Wiper=Present_Value;
		Junction_Wiper=Wiper_Count;
		Set_Value_Flag=0;
	}
	Pololu_Present_Wiper=pololu.Get_Junction_Sensor(WIPER,thershold,difference);
	if ((Direction==Forward_Direction_Wiper) && (Pololu_Present_Wiper!=Pololu_Previous_Wiper))
	{
		Junction_Wiper++;
		Pololu_Previous_Wiper=Pololu_Present_Wiper;
	}
	if ((Direction==Downward_Direction_Wiper) && (Pololu_Present_Wiper!=Pololu_Previous_Wiper))
	{
		Junction_Wiper--;
		Pololu_Previous_Wiper=Pololu_Present_Wiper;
	}
	
	if ((Junction_Wiper!=Final_Count))
	{
		Mot.Set_Motor_Direction(Wiper,Motor_Direction);
		Mot.Run(Wiper,1800);
	}
	else
	{
		Mot.Run(Wiper,0);
	}
	Last_Wiper_Present_Count=Pololu_Present_Wiper;
	Last_Wiper_Prevoius_Count=Pololu_Previous_Wiper;
}

void Pololu_Motor_A(unsigned char Motor_Direction,unsigned char Motor_A_Count,unsigned char Present_Value,unsigned char Previous_Value,int thershold,int difference,unsigned char Final_Count,unsigned char Direction)
{
	if (Set_Value_Flag)
	{
		Pololu_Previous_Motor_A=Previous_Value;
		Pololu_Previous_Motor_A=Present_Value;
		Junction_Motor_A=Motor_A_Count;
		Set_Value_Flag=0;
	}
	Pololu_Present_Motor_A=pololu.Get_Junction_Sensor(MOTOR_A,thershold,difference);
	if ((Direction==Forward_Direction_Motor_A) && (Pololu_Present_Motor_A!=Pololu_Previous_Motor_A))
	{
		Junction_Motor_A++;
		Pololu_Previous_Motor_A=Pololu_Present_Motor_A;
	}
	if ((Direction==Downward_Direction_Motor_A) && (Pololu_Present_Motor_A!=Pololu_Previous_Motor_A))
	{
		Junction_Motor_A--;
		Pololu_Previous_Motor_A=Pololu_Present_Motor_A;
	}
	
	if ((Junction_Motor_A!=Final_Count))
	{
		Mot.Set_Motor_Direction(Motor_A,Motor_Direction);
		Mot.Run(Motor_A,2000);
	}
	else
	{
		Mot.Run(Motor_A,0);
	}
	Last_Motor_A_Present_Count=Pololu_Present_Motor_A;
	Last_Motor_A_Prevoius_Count=Pololu_Previous_Motor_A;
}

void Pololu_Motor_B(unsigned char Motor_Direction,unsigned char Motor_B_Count,unsigned char Present_Value,unsigned char Previous_Value,int thershold,int difference,unsigned char Final_Count,unsigned char Direction)
{
	if (Set_Value_Flag)
	{
		Pololu_Previous_Motor_B=Previous_Value;
		Pololu_Previous_Motor_B=Present_Value;
		Junction_Motor_B=Motor_B_Count;
		Set_Value_Flag=0;
	}
	Pololu_Present_Motor_B=pololu.Get_Junction_Sensor(MOTOR_B,thershold,difference);
	if ((Direction==Forward_Direction_Motor_B) && (Pololu_Present_Motor_B!=Pololu_Previous_Motor_B))
	{
		Junction_Motor_B++;
		Pololu_Previous_Motor_B=Pololu_Present_Motor_B;
	}
	if ((Direction==Downward_Direction_Motor_B) && (Pololu_Present_Motor_B!=Pololu_Previous_Motor_B))
	{
		Junction_Motor_B--;
		Pololu_Previous_Motor_B=Pololu_Present_Motor_B;
	}
	
	if ((Junction_Motor_A!=Final_Count))
	{
		Mot.Set_Motor_Direction(Motor_B,Motor_Direction);
		Mot.Run(Motor_B,1800);
	}
	else
	{
		Mot.Run(Motor_B,0);
	}
	Last_Motor_B_Present_Count=Pololu_Present_Motor_B;
	Last_Motor_B_Prevoius_Count=Pololu_Previous_Motor_B;
}

void Init_Timer4()
{
	TCCR4B |= ((1<<WGM42)|(1<<CS42)|(1<<CS40));
	TIMSK4 |= (1<<OCIE4A);
	TIFR4 |= (1<<OCF4A);
	OCR4A = 388;
	TCNT4=0;
}
	
ISR (PCINT2_vect)
{
	if((PINK&(1<<PK6))&&(PINK&(1<<PK7)))
	{
		LidarDown_Flag=1;
	}
	else if((!(PINK&(1<<PK6)))||(!(PINK&(1<<PK7))))
	{
		LidarDown_Flag=0;
	}
}

ISR (TIMER4_COMPA_vect)
{
	pidflag=1;
	Rampflag=1;
	if (Timer_Start_Flag)
	{
		River_Timer_Count++;
	}
	if (Timer_Start_Flag && River_Check)
	{
		if(River_Timer_Count>=240)
		{
			River_Push_Control=1;
			River_Timer_Count=0;
		}
	}
}