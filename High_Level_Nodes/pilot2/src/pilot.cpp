#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"

#include "pilot.h"

int pipe_Y;
int pipe_X;
float pipe_angle;

std_msgs::Float32 pilotHeading;
std_msgs::Float32 pilotSpeed;

int main(int argc, char **argv){ //we need argc and argv for the rosInit function

	int counter;

	ros::init(argc, argv, "pilot");	//inits the driver

	/* Messages and services */

	ros::NodeHandle pilotN;

	/* Publish */
	//These are the desired values, and go to the PID 
	ros::Publisher pilotHeadingMsg = pilotN.advertise<std_msgs::Float32>("pilotHeading", 100);
	ros::Publisher pilotDepthMsg = pilotN.advertise<std_msgs::Float32>("pilotDepth", 100);
	ros::Publisher pilotPitchMsg = pilotN.advertise<std_msgs::Float32>("pilotPitch", 100);
	ros::Publisher pilotOkGoMsg = pilotN.advertise<std_msgs::UInt32>("pilotOkGo", 100);
	ros::Publisher pilotSpeedMsg = pilotN.advertise<std_msgs::Float32>("pilotSpeed", 100);
	ros::Publisher alertFrontMsg = pilotN.advertise<std_msgs::UInt32>("alertFront", 100);
	ros::Publisher fullScanMsg = pilotN.advertise<std_msgs::UInt32>("fullScan", 100);

	/*Sets up the message structures*/


	std_msgs::Float32 pilotDepth;
	std_msgs::Float32 pilotPitch;
	std_msgs::UInt32 pilotOkGo;
	std_msgs::UInt32 alertFront;
	std_msgs::UInt32 fullScan;


	/* Subscribe */
	//The actual values heading depth and pitch
	ros::Subscriber sub1 = pilotN.subscribe("compassHeading", 100, headingCallback);
	ros::Subscriber sub2 = pilotN.subscribe("svpDepth", 100, depthCallback);
	ros::Subscriber sub3 = pilotN.subscribe("compassPitch", 100, pitchCallback);
	//The state of the safety cut-off switch
	ros::Subscriber sub4 = pilotN.subscribe("adcGo", 	100, adcGoCallback);
	//sonar values
	ros::Subscriber sub5 = pilotN.subscribe("sonarFront", 	100, sonarFCallback);
	ros::Subscriber sub6 = pilotN.subscribe("sonarRight", 	100, sonarRCallback);
	ros::Subscriber sub7 = pilotN.subscribe("sonarDone", 	100, sonarDCallback);
	//Pipe values
	ros::Subscriber sub8 = pilotN.subscribe("pipe_X", 	100, pipeXCallback);
	ros::Subscriber sub9 = pilotN.subscribe("pipe_Y", 	100, pipeYCallback);
	ros::Subscriber sub10 = pilotN.subscribe("pipe_theta", 	100, pipethetaCallback);

	//ros::Rate loop_rate(10); //how many times a second (i.e. Hz) the code should run

	ROS_INFO("Pilot Is Online");


	ros::Rate loop_rate(1);

	counter = 10;

	while(ros::ok()){

		ros::spinOnce();

		if(latch && (go != 2)){
			go = 1;
		}
		else{
			go = 0;
		}

		if((counter > 0) && go){

			ROS_INFO("We go in %d seconds",counter);

			counter--;

			if(counter == 0){
				pilotOkGo.data = 1;
				pilotOkGoMsg.publish(pilotOkGo);
			}
			else{
				if(counter > 7){
					alertFront.data = 1750;
					alertFrontMsg.publish(alertFront);
				}
				else{
					alertFront.data = 1500;
					alertFrontMsg.publish(alertFront);
				}

				pilotOkGo.data = 0;
				pilotOkGoMsg.publish(pilotOkGo);
			}
		}
		else if(go){
			switch(switcher){
				case	0:	pilotHeading.data = FIRSTHEADING;
						pilotDepth.data = RUNDEPTH;
						pilotPitch.data = 0.0f;
						pilotSpeed.data = -10.0f;

						if((heading < (FIRSTHEADING + HACC)) && (heading > (FIRSTHEADING - HACC))){
							if(hcounter > HCOUNT){
								switcher++;
								tcounter=0;
								hcounter = 0;
							}
							else{
								hcounter++;
							}
						}
						break;

				case	1:	pilotHeading.data = FIRSTHEADING;
						pilotDepth.data = RUNDEPTH;
						pilotPitch.data = 0.0f;
						pilotSpeed.data = RUNSPEED;

						if(tcounter >= OUTTIME){
							switcher++;
							tcounter=0;
						}
						break;
	
				case	2:	pilotHeading.data = SECONDHEADING;
						pilotDepth.data = RUNDEPTH;
						pilotPitch.data = 0.0f;
						pilotSpeed.data = -10.0f;

						if((heading < (SECONDHEADING + HACC)) && (heading > (SECONDHEADING - HACC))){
							if(hcounter > HCOUNT){
								switcher++;
								tcounter=0;
								hcounter = 0;
							}
							else{
								hcounter++;
							}
						}
						break;

				case	3:	pilotHeading.data = SECONDHEADING;
						pilotDepth.data = RUNDEPTH;
						pilotPitch.data = 0.0f;
						pilotSpeed.data = RUNSPEED;

						if(fRange = -1.0f){
							pilotSpeed.data = STOPSPEED;
						}
						else if(fRange <= FTHRESH){
							switcher++;
							tcounter=0;
						}
						break;

				case 	4:	pilotHeading.data = THIRDHEADING;
						pilotDepth.data = RUNDEPTH;
						pilotPitch.data = 0.0f;
						pilotSpeed.data = STOPSPEED;
						if((heading < (THIRDHEADING + HACC)) && (heading > (THIRDHEADING - HACC))){
							if(hcounter > HCOUNT){
								switcher++;
								tcounter=0;
								hcounter = 0;
							}
							else{
								hcounter++;
							}
						}
						break;

				case	5:	pilotSpeed.data = RUNSPEED;
						pilotDepth.data = RUNDEPTH;
						pilotPitch.data = 0.0f;
						if(rRange = -1.0f){
							pilotSpeed.data = STOPSPEED;
						}
						else if(rRange > (WALLRANGE + WALLACC)){
							theading = pilotHeading.data;
							theading = theading + 1.0;
							pilotHeading.data = theading;
						}
						else if(rRange < (WALLRANGE - WALLACC)){
							theading = pilotHeading.data;
							theading = theading - 1.0;
							pilotHeading.data = theading;
						}
						if(fRange = -1.0f){
							pilotSpeed.data = STOPSPEED;
						}
						else if(fRange <= FTHRESH){
							switcher++;
							tcounter=0;
						}
						break;
				
				case	6:	pilotHeading.data = FOURTHHEADING;
						pilotDepth.data = RUNDEPTH;
						pilotPitch.data = 0.0f;
						pilotSpeed.data = STOPSPEED;
						if((heading < (FOURTHHEADING + HACC)) && (heading > (FOURTHHEADING - HACC))){
							if(hcounter > HCOUNT){
								switcher++;
								tcounter=0;
								hcounter = 0;
							}
							else{
								hcounter++;
							}
						}
						break;

				case	7:	fullScan.data = 1;
						fullScanMsg.publish(fullScan);
						if(done){
							tcounter = 0;
							switcher++;
						}
						else if(tcounter > 300){
							tcounter = 0;
							switcher++;
						}
						break;

				case	8:	pilotHeading.data = STOPHEADING;
						pilotDepth.data = STOPDEPTH;
						pilotPitch.data = 0.0f;
						pilotSpeed.data = STOPSPEED;
						if(tcounter >= 4){
							pilotOkGo.data = 0;
						}
						break;
				default:	ROS_ERROR("WRONG!");	break;
			}
			
			
			tcounter++;
			ROS_DEBUG("PH: %.3f PD: %.3f PP: %.3f GO: %u State: %u Count: %u H %u",pilotHeading.data,pilotDepth.data,pilotPitch.data,pilotOkGo.data, switcher,tcounter,hcounter);

			/*Below here we publish our readings*/
			pilotSpeedMsg.publish(pilotSpeed);
			pilotOkGoMsg.publish(pilotOkGo);
			pilotHeadingMsg.publish(pilotHeading);		
			pilotDepthMsg.publish(pilotDepth);		
			pilotPitchMsg.publish(pilotPitch);
		}
		loop_rate.sleep();
	}

	printf("Shutting Down\n");

	return 0;
}

int map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (int)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

/*************************************************
 ** Follows the pipe
 * Uses values published by the dwncam2 node
 * As the values are updated the section that
 * is updated changes
 * X and angle control the orientation
 * Y controls the speed
 ************************************************/
void pipe_follower()
{
	int speed = 50;
	float pipe_heading;
	static int average_heading[9] = {0};
	static int counter = 0;
	int avg = 0;
	if (pipe_X != 255 && pipe_Y != 255)
	{
		//float speed_multiplier;
		//Look at the Y value and act
		//Modifies the speed of the sub based on the Y position of the pipe centre
		speed = map(pipe_Y, -100, 100, 25, 75);
		//Look at the X value and act
		pipe_heading = map(pipe_X, -100, 100, (int)(heading - 45.0), (int)(heading + 45.0));
		//heading is current heading + pipe angle
		pipe_heading = heading + pipe_angle;
		//Keep an average of pipe headings so a new mad value
		//keep a counter
		if(counter == 10) counter = 0;
		//store the current value
		average_heading[counter] = pipe_heading;
		//find the average
		for(int i = 0; i < 10; i++)
		{
			avg += average_heading[i];
		}
		
		avg = avg / 10;
		//check to see if the new heading is within ten of the average
		if((pipe_heading > avg + 10) || (pipe_heading < avg - 10))
		{
				pipe_heading = avg;
		}
		//doesn't send us flying off into the void
		//assign the final values to pilotspeed && pilotheading
		pilotHeading.data = pipe_heading;
		pilotSpeed.data = speed;
		return;
	}
}

/*************************************************
** Returns the pipe X pos			**
*************************************************/

void pipeXCallback(const std_msgs::UInt32::ConstPtr& PipeX){
	pipe_X = PipeX->data;
	if(pipe_X != 255) pipe_X -= 100;
	return;
}

/*************************************************
** Returns the pipe Y pos			**
*************************************************/

void pipeYCallback(const std_msgs::UInt32::ConstPtr& PipeY){
	pipe_Y = PipeY->data;
	if(pipe_Y != 255) pipe_Y -= 100;
	return;
}

/*************************************************
** Returns the pipe angle			**
*************************************************/

void pipethetaCallback(const std_msgs::Float32::ConstPtr& PipeTheta){
	pipe_angle = PipeTheta->data;
	return;
}

/*************************************************
** Returns the compass heading			**
*************************************************/

void headingCallback(const std_msgs::Float32::ConstPtr& compassHeading){
	heading = compassHeading->data;
	return;
}


/*************************************************
** Returns the compass pitch			**
*************************************************/

void pitchCallback(const std_msgs::Float32::ConstPtr& compassPitch){
	pitch = compassPitch->data;
	return;
}


/*************************************************
** Returns the svp depth			**
*************************************************/

void depthCallback(const std_msgs::Float32::ConstPtr& svpDepth){
	depth = svpDepth->data;
	return;
}

/*************************************************
** Returns the ADC				**
*************************************************/

void adcGoCallback(const std_msgs::UInt32::ConstPtr& adcGo){
	go = adcGo->data;
	
	if(go){
		latch = 1;
	}

	return;
}

/*************************************************
** Returns the front range			**
*************************************************/

void sonarFCallback(const std_msgs::Float32::ConstPtr& sonarFront){
	fRange = sonarFront->data;
	return;
}

/*************************************************
** Returns the right range			**
*************************************************/

void sonarRCallback(const std_msgs::Float32::ConstPtr& sonarRight){
	rRange = sonarRight->data;
	return;
}

/*************************************************
** Returns the done				**
*************************************************/

void sonarDCallback(const std_msgs::UInt32::ConstPtr& sonarDone){
	done = sonarDone->data;
	return;
}
