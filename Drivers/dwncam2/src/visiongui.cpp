//Includes
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ros/ros.h>
#include "std_msgs/Char.h"

//Variables to store hue, saturation, and brightness in
int hue = 127;
int sat = 127;
int bri = 127;

int huem = 127;
int satm = 127;
int brim = 127;


//Callback function for trackbars
void changeColor(int pos)
{
}

int main(int argc, char **argv){ //we need argc and argv for the rosInit 

	ros::init(argc, argv, "visiongui");	//Initialize the node

	//Our node handle
	ros::NodeHandle visionguiN;	

	int lh = 127;
	int ls = 127;
	int lb = 127;

	int lhm = 127;
	int lsm = 127;
	int lbm = 127;

	/*Advertises our various messages*/
	ros::Publisher p_hue = visionguiN.advertise<std_msgs::Char>("vision_hue_min", 100);
	ros::Publisher p_sat = visionguiN.advertise<std_msgs::Char>("vision_saturation_min", 100); 
	ros::Publisher p_bri = visionguiN.advertise<std_msgs::Char>("vision_brightness_min", 100);
	
	ros::Publisher p_huem = visionguiN.advertise<std_msgs::Char>("vision_hue_max", 100);
	ros::Publisher p_satm = visionguiN.advertise<std_msgs::Char>("vision_saturation_max", 100); 
	ros::Publisher p_brim = visionguiN.advertise<std_msgs::Char>("vision_brightness_max", 100);
	
	//Create window
	cvNamedWindow("ColorSelector",0);

	//Create track Bars
	cvCreateTrackbar("MIN Hue","ColorSelector", &hue,179,&changeColor);
	cvCreateTrackbar("MIN Saturation","ColorSelector", &sat,255,&changeColor);
	cvCreateTrackbar("MIN Brightness","ColorSelector", &bri,255,&changeColor);

	cvCreateTrackbar("MAX Hue","ColorSelector", &huem,179,&changeColor);
	cvCreateTrackbar("MAX Saturation","ColorSelector", &satm,255,&changeColor);
	cvCreateTrackbar("MAX Brightness","ColorSelector", &brim,255,&changeColor);

	while (ros::ok()){

		// Allow The HighGUI Windows To Stay Open
		cv::waitKey(3);

		//See if there are any changes
		if (hue != lh)
		{
			lh = hue;
			
			std_msgs::Char h;
			
			h.data = lh;
			
			//Publish new data
			p_hue.publish(h);
		}

		if (sat != ls)
		{
			ls = sat;
			
			std_msgs::Char s;
			
			s.data = ls;
			
			//Publish new data
			p_sat.publish(s);
		}

		if (bri != lb)
		{
			lb = bri;
			
			std_msgs::Char b;
			
			b.data = lb;
			
			//Publish new data
			p_bri.publish(b);
		}

		if (huem != lhm)
		{
			lhm = huem;
			
			std_msgs::Char hm;
			
			hm.data = lhm;
			
			//Publish new data
			p_huem.publish(hm);
		}

		if (satm != lsm)
		{
			lsm = satm;
			
			std_msgs::Char sm;
			
			sm.data = lsm;
			
			//Publish new data
			p_satm.publish(sm);
		}

		if (brim != lbm)
		{
			lbm = brim;
			
			std_msgs::Char bm;
			
			bm.data = lbm;
			
			//Publish new data
			p_brim.publish(bm);
		}
	}

	return 0;
}

