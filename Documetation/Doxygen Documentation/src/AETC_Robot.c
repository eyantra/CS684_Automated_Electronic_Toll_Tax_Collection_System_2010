/**
 * @file AETC_Robot.c
 * Program for speed calculation, white line follower and obstacle detection.
 * @author Puskar Kothavade, Ashish Pardhi, Mugdha Nazare, IIT Bombay
 * @date 10/Oct/2010
 * @version 1.0
 *
 * @section LICENSE
  * Copyright (c) 2010. ERTS Lab IIT Bombay
   * All rights reserved.

  * Redistribution and use in source and binary forms, with or without
   *modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
    * notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
   * notice, this list of conditions and the following disclaimer in
   * the documentation and/or other materials provided with the distribution.

   * Neither the name of the copyright holders nor the names of
   * contributors may be used to endorse or promote products derived
   * from this software without specific prior written permission.

   * Source code can be used for academic purpose.
   * For commercial use permission form the author needs to be taken.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/signal.h>
#include <math.h> 
#include "AETC_lcd.c"

#define FCPU 11059200ul ///< defined here to make sure that program works properly
/**
* Functions prototype
*/
void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();
void timer1_init();
unsigned char ADC_Conversion(unsigned char); 
/**
* Global Variables
*/

unsigned char ADC_Value;
unsigned char flag1 = 0, flag2 = 0, f = 0; ///< stores flag values
unsigned char Left_white_line = 0;///< store left white line sensor value
unsigned char Center_white_line = 0;///< store center white line sensor value
unsigned char Right_white_line = 0;///<  store right white line sensor value
unsigned char Front_Sharp_Sensor = 0;///< store front sensor value
unsigned char Front_IR_Sensor = 0;
static unsigned int speed_cnt = 0;///< store speed count 
unsigned int vehicle_stop = 0;
static unsigned int timer_flag = 0; 
unsigned int LID_Transmit = 0;///< store transmitted ID value

unsigned char data;///< store data
float speed;///< store speed
unsigned char speed_int, speed_dec; 
unsigned int RegSpeed = 150; 
unsigned char StartTheBot = 0;

/*
* ISR Routine to Increment the speed counter
*/
ISR(TIMER1_OVF_vect)
{
	speed_cnt++;
}

/*
* Function to configure LCD port
*/
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

/*
* ADC pin configuration
*/
void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

/*
* Function to configure ports to enable robot's motion
*/
void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

/*
* Function to Initialize PORTS
*/
void port_init()
{
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();	
}

/*
* Timer 5 initialised in PWM mode for velocity control
* Prescale:64
* PWM 8bit fast, TOP=0x00FF
* Timer Frequency:674.988Hz
*/
void timer5_init()
{
	TCCR5B = 0x00;	///< Stop
	TCNT5H = 0xFF;	///< Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	///< Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	///< Output compare register high value for Left Motor
	OCR5AL = 0xFF;	///< Output compare register low value for Left Motor
	OCR5BH = 0x00;	///< Output compare register high value for Right Motor
	OCR5BL = 0xFF;	///< Output compare register low value for Right Motor
	OCR5CH = 0x00;	///< Output compare register high value for Motor C1
	OCR5CL = 0xFF;	///< Output compare register low value for Motor C1
	TCCR5A = 0xA9;	///< {COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}  For Overriding normal port functionalit to OCRnA outputs. {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode
	
	TCCR5B = 0x0B;	///< WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

/*
* Function For Timer1 Initialisation
*/

void timer1_init()
{
	TCCR1B = 0x00;  ///< STOP
	TCNT1H = 0x00;	///< Timer register higher 8 bit to zero
	TCNT1L = 0x00;	///< Timer register lower 8 bit to zero
	OCR1AH = 0xFF;	///< Output compare register higher 8 bits...not used in this case
	OCR1AL = 0xFF;	///< Output compare register lower 8 bits.....not used in this case
	OCR1BH = 0x00;	///< Output compare register higher 8 bits...not used in this case
	OCR1BL = 0x00;	///< Output compare register lower 8 bits.....not used in this case
	OCR1CH = 0x00;	///< Output compare register higher 8 bits...not used in this case
	OCR1CL = 0x00;	///< Output compare register lower 8 bits.....not used in this case
	TCCR1A = 0x00;	///< Using channel A only, but not using output compare mode. WGM: normal mode COM10:1=00
	TCCR1C = 0x00;	///< No FOC
	TIMSK1 = 0x01;	///< Timer overflow interrupt enabled

	SREG = SREG | 0x80; ///< Setting I flag of status register to one to globally enable all interrupts

}

/*
* Function For ADC Initialisation
*/
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		///< MUX5 = 0
	ADMUX = 0x20;		///< Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		///< ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

/*
* Function For ADC Conversion
*/
unsigned char ADC_Conversion(unsigned char Ch) 
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		///< Set start conversion bit
	while((ADCSRA&0x10)==0);	///< Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; ///< clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

/**
* Function To Print Sesor Values At Desired Row And Coloumn Location on LCD
*/

void print_sensor(char row, char coloumn,unsigned char channel)
{
	
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

/**
* Function for velocity control.
*/
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

/**
* Function used for setting motor's direction.
*/
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 		///<  removing upper nibbel for the protection
 PortARestore = PORTA; 		///<  reading the PORTA original status
 PortARestore &= 0xF0; 		///< making lower direction nibbel to 0
 PortARestore |= Direction; ///< adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore; 		///< executing the command
}

void forward (void) 
{
  motion_set (0x06);
}

void stop (void)
{
  motion_set (0x00);
}


/**
* Function To Initialize UART0 
* desired baud rate:9600
* actual baud rate:9600 (0.0%)
* char size: 8 bit
* parity: Disabled
*/
void uart0_init(void)
{
 UCSR0B = 0x00; ///< disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x47; ///< set baud rate lo
 UBRR0H = 0x00; ///< set baud rate hi
 UCSR0B = 0x98;
}

/*
* Function For Devices Initialisation
*/
void init_devices (void)
{
 	cli(); ///< Clears the global interrupts
	port_init(); ///< initialise the ports
	adc_init();
	uart0_init();
	timer5_init();
	timer1_init();
	sei();   //Enables the global interrupts
}

/**
* Usart receiver ISR
*/

SIGNAL(SIG_USART0_RECV)
{
	unsigned char data;
	data = UDR0; ///< making copy of data from UDR0 in data variable
	
	if(data == 'Z' && timer_flag==1)
	{
		StartTheBot=1;
	}

	if(data =='O')				///< To Regulate the speed
	{
		forward();
		RegSpeed = 150;
	}

	if(data =='I')				///< To transmit ID to Laptop
	{
		if (LID_Transmit == 0 && timer_flag==1)
		{
		    UDR0 = '2' ;	
      		LID_Transmit = 1;
		}
				
	}
   	if(data == 'X')					///< To transmit speed int value
	{
		if(timer_flag>=3)
		{
			UDR0 = speed_int;
		}
	}

	if(data == 'Y')					///< To transmit speed value after decimal point
	{
		if(timer_flag>=3)
		{
			UDR0 = speed_dec;
		}
	}

	if(data == 'V')                  ///< IF third black patch comes then transmit 1 otherwise 0.
	{
		if(timer_flag==4)
		{
			UDR0 = '1';
		}
	}
	if(data == 'D')                   ///< Image capture process over. Vehicle can go ahead now.
	{
		if(timer_flag==4)
		{
			vehicle_stop=1;
		}
	}
}

/**
* Main Function
*/
int main()
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	while(1)
	{

		Left_white_line = ADC_Conversion(3);	///< Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);///< Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	///< Getting data of Right WL Sensor
		Front_Sharp_Sensor = ADC_Conversion(11);///< Getting data of Front Sharp sensor
		Front_IR_Sensor = ADC_Conversion(6);    ///<  Getting data of Front IR sensor


		flag1=0;
		flag2=0;


		print_sensor(1,1,3);	///< Prints value of White Line Sensor1
		print_sensor(1,5,2);	///< Prints Value of White Line Sensor2
		print_sensor(1,9,1);	///< Prints Value of White Line Sensor3
		lcd_print(1, 13, timer_flag, 4);

		if((Center_white_line<0x28))
		{
			flag1=1;
			forward();
			velocity(RegSpeed,RegSpeed);
		}


		if((Left_white_line>0x28) && (flag1==0))
		{
			flag1=1;
			forward();
			velocity(100,50);
		}


		if((Right_white_line>0x28) && (flag1==0))
		{
			flag1=1;
			forward();
			velocity(50,100);
		}


		if((Center_white_line>0x28) && (Left_white_line>0x28) && (Right_white_line>0x28))  
		{

			forward();
			velocity(RegSpeed,RegSpeed);
			if (timer_flag == 0)                 ///< vehicle at first black patch
			{	
				timer_flag=1;                    
				while(StartTheBot==0)
				{
					stop();
				}

				if (StartTheBot == 1)
				{
					forward();
					velocity(RegSpeed,RegSpeed);
					TCCR1B = 0x01;   ///< Timer 1 start witrh  no prescaler
					while(Center_white_line>0x28 && Left_white_line>0x28 && Right_white_line>0x28)//wait till the time the entire red line is crossed.
					{
						Left_white_line = ADC_Conversion(3);	///< Getting data of Left WL Sensor
						Center_white_line = ADC_Conversion(2);	///< Getting data of Center WL Sensor
						Right_white_line = ADC_Conversion(1);	///< Getting data of Right WL Sensor
					}

					timer_flag=2;
				}
			}
			else if (timer_flag == 2)           ///< vehicle at second black patch
			{
				TCCR1B = 0x00;   ///< Timer 1 stop
				timer_flag=3;
				lcd_print(2, 1, TCNT1H, 4);
				lcd_print(2, 6, TCNT1L, 4);
				lcd_print(2, 11, speed_cnt, 5);
				speed = 27/ (((unsigned int) speed_cnt * 65536 * 90.9E-9) + ((unsigned int) TCNT1H * 256 * 90.9E-9) + ((unsigned int) TCNT1L * 90.9E-9));
				f=1;
				speed_int= speed;
				speed_dec=(speed-speed_int)*100;

			}

			else if (timer_flag ==3)         ///< vehicle at third black patch
			{
				timer_flag=4;

				while (vehicle_stop==0)     ///< Capturing photo in progress
				{
					stop();
				}

				if(vehicle_stop==1)         ///< photograph taken. Vehicle can go ahead now
				{				
					forward();
				}

			}
		}


		if((Front_Sharp_Sensor>0x80)|| (Front_IR_Sensor<0xF0)) ///< Obstacle detection
		{
			flag2=1;
			stop();
		}


	}
}
/**
* End of Program.
*/


