#include "ros/ros.h"
#include "std_msgs/UInt32.h"

#include "roboard.h"
#include "pwm.h"

#include "motor.h"

//#define DEBUG
#define RELEASE

int main(int argc, char **argv){ //we need argc and argv for the rosInit function

	unsigned char once = 1;
	ros::init(argc, argv, "motor");	//inits the driver

	#ifdef DEBUG
	ROS_WARN("Warning, motors are in debug mode");
	#endif

	if(!initMotors()){
		ROS_ERROR("Root lock failure");
		return 0;
	}

	/* Messages and services */

	ros::NodeHandle motorN;

	/* Publish */
	ros::Publisher motorDirectionMsg = motorN.advertise<std_msgs::UInt32("motorDirection", 100);
	
	std_msgs::UInt32 motorDir;

	/* Subscribe */

	ros::Subscriber sub1 = motorN.subscribe("pidRampDepthRight",	100, depthRightCallback);
	ros::Subscriber sub2 = motorN.subscribe("pidRampDepthLeft",		100, depthLeftCallback);
	ros::Subscriber sub3 = motorN.subscribe("pidRampYawRight",		100, yawRightCallback);
	ros::Subscriber sub4 = motorN.subscribe("pidRampYawLeft",		100, yawLeftCallback);
	ros::Subscriber sub7 = motorN.subscribe("pidRampPitch",			100, pitchCallback);
	
	ros::Subscriber sub5 = motorN.subscribe("pilotGo",	100, goCallback);
	ros::Subscriber sub6 = motorN.subscribe("alertFront",	100, alertCallback);

	ros::Rate loop_rate(10); //how many times a second (i.e. Hz) the code should run

	while (ros::ok()){
		if(once){
			while(go != 1){
				ros::spinOnce();
				ROS_WARN("Motors waiting for go");
				ros::Duration(1.0).sleep();
			}
			ROS_INFO("Motors given the go");
			loop_rate.sleep();
			once = 0;
		}
		
		//Add together the left and right values, divide them by stationary motors (1500 * 2), 
		//	if less than 1 it's going backwards, if more than 1 it's going forwards.
		//	Otherwise it's turning. 
		if( ((float(yawLeftPWM) + float(yawRightPWM)) / 3000.0) > 1.0 )	//Forwards
			motorDir.data = 1;
		else if( ((float(yawLeftPWM) + float(yawRightPWM)) / 3000.0) < 1.0 )	//Backwards
			motorDir.data = -1;	
		else									//Turning on the spot or stationary.
			motorDir.data = 0;

		motorDirectionMsg.publish(motorDir);
		ros::spin();
		
	}

	updatePWM(DEPTH_RIGHT_CHANNEL, 	ZERO_DUTY_CYCLE_US);
	updatePWM(DEPTH_LEFT_CHANNEL, 	ZERO_DUTY_CYCLE_US);
	updatePWM(YAW_RIGHT_CHANNEL, 	ZERO_DUTY_CYCLE_US);
	updatePWM(YAW_LEFT_CHANNEL, 	ZERO_DUTY_CYCLE_US);
	updatePWM(PITCH_CHANNEL, 		ZERO_DUTY_CYCLE_US);
	
	//updatePWM(TEST_MOTOR_CHANNEL, ZERO_DUTY_CYCLE_US);

	printf("Shutting Down\n");

	return 0;
}

/*************************************************
** Returns the go signal			**
*************************************************/

void goCallback(const std_msgs::UInt32::ConstPtr& pilotGo){
	go4 = go3 = go2 = go = pilotGo->data;
	return;
}

/*************************************************
** Returns the PWM rate for the alert motor	**
*************************************************/

void alertCallback(const std_msgs::UInt32::ConstPtr& alertFront){
	//frontPWM = alertFront->data;

	//ROS_DEBUG("Alert: %u",frontPWM);
	//updatePWM(FRONT_MOTOR_CHANNEL, frontPWM);
	return;
}

/*************************************************
** Returns the PWM rate for the depth (right) motor	**
*************************************************/

void depthRightCallback(const std_msgs::UInt32::ConstPtr& pidRampDepthRight){
	depthRightPWM = pidRampDepthRight->data;

	ROS_DEBUG("Depth (right): %u",depthRightPWM);
	if(go == 1){
		#ifdef DEBUG
		depthRightPWM = DEBUGSPEED;
		#endif
		updatePWM(DEPTH_RIGHT_CHANNEL, depthRightPWM);
		go = 0;
	}
	else{
		updatePWM(DEPTH_RIGHT_CHANNEL, ZERO_DUTY_CYCLE_US);
	}
	return;
}

/*************************************************
** Returns the PWM rate for the depth (left) motor	**
*************************************************/

void depthLeftCallback(const std_msgs::UInt32::ConstPtr& pidRampDepthLeft){
	depthLeftPWM = pidRampDepthLeft->data;

	ROS_DEBUG("Depth (left): %u",depthLeftPWM);
	
	if(go2 == 1){
		#ifdef DEBUG
		depthLeftPWM = DEBUGSPEED;
		#endif
		updatePWM(DEPTH_LEFT_CHANNEL, depthLeftPWM);
		go2 = 0;
	}
	else{
		updatePWM(DEPTH_LEFT_CHANNEL, ZERO_DUTY_CYCLE_US);
	}
	return;
}

/*************************************************
** Returns the PWM rate for the yaw (right) motor	**
*************************************************/

void yawRightCallback(const std_msgs::UInt32::ConstPtr& pidRampYawRight){
	yawRightPWM = pidRampYawRight->data;

	ROS_DEBUG("Yaw (right): %u",yawRightPWM);
	
	if(go3 == 1){
		#ifdef DEBUG
		yawRightPWM = DEBUGSPEED;
		#endif
		updatePWM(YAW_RIGHT_CHANNEL, yawRightPWM);
		go3 = 0;
	}
	else{
		updatePWM(YAW_RIGHT_CHANNEL, ZERO_DUTY_CYCLE_US);
	}
	return;
}

/*************************************************
** Returns the PWM rate for the yaw (left) motor	**
*************************************************/

void yawLeftCallback(const std_msgs::UInt32::ConstPtr& pidRampYawLeft){
	yawLeftPWM = pidRampYawLeft->data;

	ROS_DEBUG("yaw (left): %u",yawLeftPWM);
	
	if(go4 == 1){
		#ifdef DEBUG
		yawLeftPWM = DEBUGSPEED;
		#endif
		updatePWM(YAW_LEFT_CHANNEL, yawLeftPWM);
		go4 = 0;
	}
	else{
		updatePWM(YAW_LEFT_CHANNEL, ZERO_DUTY_CYCLE_US);
	}
	return;
}

/*************************************************
** Returns the PWM rate for the pitch motor	**
*************************************************/

void pitchCallback(const std_msgs::UInt32::ConstPtr& pidRampPitch){
	pitchPWM = pidRampPitch->data;

	ROS_DEBUG("Pitch: %u",pitchPWM);
	
	if(go4 == 1){
		#ifdef DEBUG
		pitchPWM = DEBUGSPEED;
		#endif
		updatePWM(PITCH_CHANNEL, pitchPWM);
		go4 = 0;
	}
	else{
		updatePWM(PITCH_CHANNEL, ZERO_DUTY_CYCLE_US);
	}
	return;
}


/*************************************************
** Initialises the motors			**
*************************************************/

int initMotors(void){

	if(!pwm_Initialize(0xffff, PWMCLOCK_50MHZ, PWMIRQ_DISABLE)){
        	ROS_ERROR("Unable to initialise PWM library - %s", roboio_GetErrMsg());
		return 0;
	}
	else{
		ROS_INFO("Successfully logged in as root, pwm activated");
	}

        // Set the channels to produce a zero velocity PWM
        pwm_SetPulse( DEPTH_RIGHT_CHANNEL, PWM_FREQUENCY_US, (ZERO_DUTY_CYCLE_US + DEPTH_RIGHT_PWM_OFFSET));
        pwm_SetPulse( DEPTH_LEFT_CHANNEL, PWM_FREQUENCY_US, ZERO_DUTY_CYCLE_US + DEPTH_LEFT_PWM_OFFSET );
        pwm_SetPulse( YAW_RIGHT_CHANNEL, PWM_FREQUENCY_US, ZERO_DUTY_CYCLE_US + YAW_RIGHT_PWM_OFFSET );
        pwm_SetPulse( YAW_LEFT_CHANNEL, PWM_FREQUENCY_US, ZERO_DUTY_CYCLE_US + YAW_LEFT_PWM_OFFSET );
        pwm_SetPulse( PITCH_CHANNEL, PWM_FREQUENCY_US, ZERO_DUTY_CYCLE_US + PITCH_PWM_OFFSET );

        pwm_SetCountingMode( DEPTH_RIGHT_CHANNEL, PWM_CONTINUE_MODE );
        pwm_SetCountingMode( DEPTH_LEFT_CHANNEL, PWM_CONTINUE_MODE );
        pwm_SetCountingMode( YAW_RIGHT_CHANNEL, PWM_CONTINUE_MODE );
        pwm_SetCountingMode( YAW_LEFT_CHANNEL, PWM_CONTINUE_MODE );
        pwm_SetCountingMode( PITCH_CHANNEL, PWM_CONTINUE_MODE );
        
        // Enable the pins
        pwm_EnablePin( DEPTH_RIGHT_CHANNEL );
        pwm_EnablePin( YAW_RIGHT_CHANNEL );
        pwm_EnablePin( DEPTH_LEFT_CHANNEL );
        pwm_EnablePin( YAW_LEFT_CHANNEL );
        pwm_EnablePin( PITCH_CHANNEL );

	ROS_INFO("Motors Initialised");

        pwm_EnableMultiPWM(0xffffffffL);

	ROS_INFO("Motors Online");

	return 1;
}

/*************************************************
** updates the PWM rate				**
*************************************************/

void updatePWM(unsigned int channel, unsigned int rate){
	unsigned int tmpOffset=0;

	switch(channel){
		case	DEPTH_RIGHT_CHANNEL:	tmpOffset = DEPTH_RIGHT_PWM_OFFSET;	break;
		case	DEPTH_LEFT_CHANNEL:		tmpOffset = DEPTH_LEFT_PWM_OFFSET;	break;
		case	YAW_RIGHT_CHANNEL:		tmpOffset = YAW_RIGHT_PWM_OFFSET;	break;
		case	YAW_LEFT_CHANNEL:		tmpOffset = YAW_LEFT_PWM_OFFSET;	break;
		case	PITCH_CHANNEL:			tmpOffset = PITCH_PWM_OFFSET;		break;
		default: ROS_ERROR("Dude this is not a valid PWM channel");	break;
	}

	if(rate > MAX_DUTY_CYCLE_US){
		rate = MAX_DUTY_CYCLE_US;
		ROS_WARN("Attempting to over speed motor %u",channel);
	}
	if(rate < MIN_DUTY_CYCLE_US){
		rate = MIN_DUTY_CYCLE_US;
		ROS_WARN("Attempting to over reverse motor %u",channel);
	}

	if(!pwm_SetPulse(channel, PWM_FREQUENCY_US, (rate + tmpOffset))){
		ROS_ERROR("Failed to update PWM");
	}

	return;
}
