//Includes
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ros/ros.h>
#include "std_msgs/UInt32.h"

//motor PWM values, 1500 is stop, 1000 full reverse, 2000 full forward
int depth_right = 500;
int depth_left = 500;
int yaw_left = 500;
int yaw_right = 500;
int pitch = 500;
int go = 0;

ros::Publisher p_depth_right;
ros::Publisher p_depth_left; 
ros::Publisher p_yaw_right;
ros::Publisher p_yaw_left;
ros::Publisher p_pitch;
ros::Publisher p_go;

//callback functions for each slider

void depth_right_cb(int pos)
{
	std_msgs::UInt32 msg;
	msg.data = pos+1000;
	p_depth_right.publish(msg);
}

void depth_left_cb(int pos)
{
	std_msgs::UInt32 msg;
	msg.data = pos+1000;
	p_depth_left.publish(msg);
}

void yaw_right_cb(int pos)
{
	std_msgs::UInt32 msg;
	msg.data = pos+1000;
	p_yaw_right.publish(msg);
}

void yaw_left_cb(int pos)
{
	std_msgs::UInt32 msg;
	msg.data = pos+1000;
	p_yaw_left.publish(msg);
}

void pitch_cb(int pos)
{
	std_msgs::UInt32 msg;
	msg.data = pos+1000;
	p_pitch.publish(msg);
}


void go_cb(int pos)
{
	std_msgs::UInt32 msg;
	msg.data = pos;
	p_pitch.publish(msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motorspammer");	//Initialize the node
		
	//Our node handle
	ros::NodeHandle motorspammer_nh;
	
	/*Advertises our various messages*/
	p_depth_right = motorspammer_nh.advertise<std_msgs::UInt32>("pidRampDepthRight", 100);
	p_depth_left = motorspammer_nh.advertise<std_msgs::UInt32>("pidRampDepthLeft", 100); 
	p_yaw_right = motorspammer_nh.advertise<std_msgs::UInt32>("pidRampYawLeft", 100);
	p_yaw_left = motorspammer_nh.advertise<std_msgs::UInt32>("pidRampYawRight", 100);
	p_pitch = motorspammer_nh.advertise<std_msgs::UInt32>("pidRampPitch", 100);
	
	p_go = motorspammer_nh.advertise<std_msgs::UInt32>("pilotGo", 100);	
	
	//create the window
	cvNamedWindow("MotorSpammer",0);
	
	//Create track Bars
	cvCreateTrackbar("Depth Right (CH1)","MotorSpammer", &depth_right,1000,&depth_right_cb);
	cvCreateTrackbar("Depth Left (CH2)_","MotorSpammer", &depth_left,1000,&depth_left_cb);
	cvCreateTrackbar("Yaw Left (CH3)___","MotorSpammer", &yaw_right,1000,&yaw_right_cb);	
	cvCreateTrackbar("Yaw Right (CH4)__","MotorSpammer", &yaw_left,1000,&yaw_left_cb);
	cvCreateTrackbar("PITCH (CH5)______","MotorSpammer", &pitch,1000,&pitch_cb);	
	
	cvCreateTrackbar("GO", "MotorSpammer", &go, 1, &go_cb);

	while (ros::ok())
	{

		// Allow The HighGUI Windows To Stay Open
		cv::waitKey(3);
	
	}
}
