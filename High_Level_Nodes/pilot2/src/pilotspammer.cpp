//Includes
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ros/ros.h>
#include "std_msgs/UInt32.h"
#include "std_msgs/Float32.h"

//Our desired heading, start off at zero.
int heading = 0;
//start pitch at 90 (horizontal), 0 would be pointing dead up, 180 straight down
int pitch = 90;
//180 would be level, 0/360 would be upside down
int roll = 180;
//starting depth is 0
int depth = 0;
int go = 0;

ros::Publisher p_heading;
ros::Publisher p_pitch;
ros::Publisher p_roll;
ros::Publisher p_depth;
ros::Publisher p_go;

std_msgs::Float32 msgHeading;
std_msgs::Float32 msgPitch;
std_msgs::Float32 msgRoll;
std_msgs::Float32 msgDepth;
std_msgs::UInt32 msgGo;

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
void go_cb(int pos)
{
	msgGo.data = pos;
	p_go.publish(msgGo);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pilotspammer");	//Initialize the node
		
	//Our node handle
	ros::NodeHandle pilotspammer_nh;
	
	/*Advertises our various messages*/
	p_heading = pilotspammer_nh.advertise<std_msgs::Float32>("pilotHeading", 100);
	p_pitch = pilotspammer_nh.advertise<std_msgs::Float32>("pilotPitch", 100);
	p_roll = pilotspammer_nh.advertise<std_msgs::Float32>("pilotRoll", 100);
	p_depth = pilotspammer_nh.advertise<std_msgs::Float32>("pilotDepth", 100);
	
	p_go = pilotspammer_nh.advertise<std_msgs::UInt32>("pilotGo", 100);
		
	//create the window
	cvNamedWindow("Pilot Spammer",0);
	
	//Create track Bars
	cvCreateTrackbar("Heading","Pilot Spammer", &heading,360,&heading_cb);
	cvCreateTrackbar("Pitch","Pilot Spammer", &pitch,180,&pitch_cb);
	cvCreateTrackbar("Roll","Pilot Spammer", &roll,360,&roll_cb);
	cvCreateTrackbar("Depth","Pilot Spammer", &depth,100,&depth_cb);
	
	cvCreateTrackbar("Go","Pilot Spammer", &go,1,&go_cb);

	while (ros::ok())
	{

		// Allow The HighGUI Windows To Stay Open
		cv::waitKey(3);
			
	}
}
