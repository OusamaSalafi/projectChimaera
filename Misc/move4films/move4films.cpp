#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../../Roboard/roboard.h"
#include "../../Roboard/pwm.h"
#include "../../Roboard/ad79x8.h"

#include "motor.h"

#define SLEEPTIME 2500

//#define DEBUG
//#define RELEASE
//int go = 0;
unsigned int accRaw[8];

int main(int argc, char **argv){ //we need argc and argv for the rosInit function

	char c[100],s[100],e[100];
	unsigned int speed;
	int counter = 100;
	int i = 0;

	if(argv[1] == "test")
	{
	while(1)
	{
		for(j = 2; j <7; j ++)
		{
	
			for(i = 0; i < 500; i ++)
			{
			
				updatePWM(RIGHT_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);
				updatePWM(LEFT_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);
				updatePWM(FRONT_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);
				updatePWM(BACK_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);
				updatePWM(TEST_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);
				
				updatePWM(j, 1600);
				usleep(SLEEPTIME);
				printf("Channel %d\n", j);	
				
			}
		}	
		
	}
	}
	else if(argv[1] == "go")
	{
	while(1)
	{
	
		readADC();
		while(go)
		{
		
			for(i = 0; i < 500; i ++)
			{
			
				updatePWM(RIGHT_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US - i);
				updatePWM(LEFT_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US + i);
				updatePWM(FRONT_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);
				updatePWM(BACK_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);
				updatePWM(TEST_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);
				
				printf("1 - Turning\n");	
				
			}
			for(i = 0; i < 50; i ++)
			{
			
				updatePWM(RIGHT_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);
				updatePWM(LEFT_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);
				updatePWM(FRONT_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US + i);
				updatePWM(BACK_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);
				updatePWM(TEST_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);	
				
				printf("2 - Down\n");
				
			}
			for(i = 0; i < 500; i ++)
			{
			
				updatePWM(RIGHT_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US + i);
				updatePWM(LEFT_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US - i);
				updatePWM(FRONT_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);
				updatePWM(BACK_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);
				updatePWM(TEST_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);
				
				printf("3 - Turning\n");	
				
			}		
		
		}
	
	
	}
	}
	else
	{
		printf("args are test or go\n");
	}
		
	updatePWM(RIGHT_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);
	updatePWM(LEFT_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);
	updatePWM(FRONT_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);
	updatePWM(BACK_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);
	//updatePWM(TEST_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);

	printf("Shutting Down\n");

	return 0;
}

/*************************************************
** Initialises the motors			**
*************************************************/

int initMotors(void){

	if(!pwm_Initialize(0xffff, PWMCLOCK_50MHZ, PWMIRQ_DISABLE)){
        	//ROS_ERROR("Unable to initialise PWM library - %s", roboio_GetErrMsg());
		return 0;
	}
	else{
		//ROS_INFO("Successfully logged in as root, pwm activated");
	}

        // Set the channels to produce a zero velocity PWM
        pwm_SetPulse( LEFT_MOTOR_CHANNEL, PWM_FREQUENCY_US, (ZERO_DUTY_CYCLE_US + LEFT_PWM_OFFSET));
        pwm_SetPulse( RIGHT_MOTOR_CHANNEL, PWM_FREQUENCY_US, ZERO_DUTY_CYCLE_US + RIGHT_PWM_OFFSET );
        pwm_SetPulse( FRONT_MOTOR_CHANNEL, PWM_FREQUENCY_US, ZERO_DUTY_CYCLE_US + FRONT_PWM_OFFSET );
        pwm_SetPulse( BACK_MOTOR_CHANNEL, PWM_FREQUENCY_US, ZERO_DUTY_CYCLE_US + BACK_PWM_OFFSET );
        //pwm_SetPulse( TEST_CHANNEL, PWM_FREQUENCY_US, ZERO_DUTY_CYCLE_US + BACK_PWM_OFFSET );

        pwm_SetCountingMode( LEFT_MOTOR_CHANNEL, PWM_CONTINUE_MODE );
        pwm_SetCountingMode( RIGHT_MOTOR_CHANNEL, PWM_CONTINUE_MODE );
        pwm_SetCountingMode( FRONT_MOTOR_CHANNEL, PWM_CONTINUE_MODE );
        pwm_SetCountingMode( BACK_MOTOR_CHANNEL, PWM_CONTINUE_MODE );
        //pwm_SetCountingMode( TEST_CHANNEL, PWM_CONTINUE_MODE );
        
        // Enable the pins
        pwm_EnablePin( LEFT_MOTOR_CHANNEL );
        pwm_EnablePin( FRONT_MOTOR_CHANNEL );
        pwm_EnablePin( RIGHT_MOTOR_CHANNEL );
        pwm_EnablePin( BACK_MOTOR_CHANNEL );
        pwm_EnablePin( TEST_MOTOR_CHANNEL );

	//ROS_INFO("Motors Initialised");

        pwm_EnableMultiPWM(0xffffffffL);

	//ROS_INFO("Motors Online");

	return 1;
}

/*************************************************
** updates the PWM rate				**
*************************************************/

void updatePWM(unsigned int channel, unsigned int rate){
	unsigned int tmpOffset=0;

	switch(channel){
		case	FRONT_MOTOR_CHANNEL:	tmpOffset = FRONT_PWM_OFFSET;	break;
		case	LEFT_MOTOR_CHANNEL:	tmpOffset = LEFT_PWM_OFFSET;	break;
		case	RIGHT_MOTOR_CHANNEL:	tmpOffset = RIGHT_PWM_OFFSET;	break;
		case	BACK_MOTOR_CHANNEL:	tmpOffset = BACK_PWM_OFFSET;	break;
		//case	TEST_CHANNEL:		tmpOffset = 0;			break;
		default: break;//ROS_ERROR("Dude this is not a valid PWM channel");	break;
	}

	if(rate > MAX_DUTY_CYCLE_US){
		rate = MAX_DUTY_CYCLE_US;
		//ROS_WARN("Attempting to over speed motor %u",channel);
	}
	if(rate < MIN_DUTY_CYCLE_US){
		rate = MIN_DUTY_CYCLE_US;
		//ROS_WARN("Attempting to over reverse motor %u",channel);
	}

	if(!pwm_SetPulse(channel, PWM_FREQUENCY_US, (rate + tmpOffset))){
		//ROS_ERROR("Failed to update PWM");
	}

	return;
}

void readADC(void){

	unsigned int val,i;
	float gotmp;

	if(spi_Init(SPICLK_21400KHZ))
	{
		//for(i=0;i<8;i++){
			accRaw[0] = adc_ReadChannel(0, ADCMODE_RANGE_2VREF,ADCMODE_UNSIGNEDCODING);
			//printf("Val at channel %u: is %u\n",i,accRaw[i]);
		//}
		spi_Close();
	}
	
	gotmp /= 1023.0;
	gotmp *= 5000.0;
	
	if(gotmp >= 4000.0)
	{
	
		go = 1;
		printf("Go Forth!\n");
	
	}
	
	return; 
}
