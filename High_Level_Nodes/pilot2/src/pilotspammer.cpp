//Includes
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ros/ros.h>
#include "std_msgs/UInt32.h"

//Our desired heading, start off at zero.
int heading = 0;
//start pitch at 90 (horizontal), 0 would be pointing dead up, 180 straight down
int pitch = 90;
//180 would be level, 0/360 would be upside down
int roll = 180;
//starting depth is 0
int depth = 0;

ros::Publisher p_heading;
ros::Publisher p_pitch;
ros::Publisher p_roll;
ros::Publisher p_depth;

std_msgs::UInt32 msgHeading;
std_msgs::UInt32 msgPitch;
std_msgs::UInt32 msgRoll;
std_msgs::UInt32 msgDepth;

//callback functions for each slider

void heading_cb(int pos)
{
	msgHeading.data = pos;
	p_heading.publish(msgHeading);
}

void pitch_cb(int pos)
{
	msgPitch.data = pos;
	p_pitch.publish(msgPitch);
}
void roll_cb(int pos)
{
	msgRoll.data = pos;
	p_roll.publish(msgRoll);
}
void depth_cb(int pos)
{
	msgDepth.data = pos;
	p_depth.publish(msgDepth);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pilotspammer");	//Initialize the node
		
	//Our node handle
	ros::NodeHandle pilotspammer_nh;
	
	/*Advertises our various messages*/
	p_heading = pilotspammer_nh.advertise<std_msgs::UInt32>("newPilotHeading", 100);
	p_pitch = pilotspammer_nh.advertise<std_msgs::UInt32>("newPilotPitch", 100);
	p_roll = pilotspammer_nh.advertise<std_msgs::UInt32>("newPilotRoll", 100);
	p_depth = pilotspammer_nh.advertise<std_msgs::UInt32>("newPilotDepth", 100);
		
	//create the window
	cvNamedWindow("Pilot Spammer",0);
	
	//Create track Bars
	cvCreateTrackbar("Heading","Pilot Spammer", &heading,360,&heading_cb);
	cvCreateTrackbar("Pitch","Pilot Spammer", &pitch,180,&pitch_cb);
	cvCreateTrackbar("Roll","Pilot Spammer", &roll,360,&roll_cb);
	cvCreateTrackbar("Depth","Pilot Spammer", &depth,100,&depth_cb);

	while (ros::ok())
	{

		// Allow The HighGUI Windows To Stay Open
		cv::waitKey(3);
			
	}
}
