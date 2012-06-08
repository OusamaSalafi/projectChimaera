//Includes
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ros/ros.h>
#include "std_msgs/Char.h"

//Variables to store hue, saturation, and brightness in
int hue = 127;
int sat = 127;
int bri = 127;


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

	/*Advertises our various messages*/
	ros::Publisher p_hue = visionguiN.advertise<std_msgs::Char>("vision_hue_min", 100);
	ros::Publisher p_sat = visionguiN.advertise<std_msgs::Char>("vision_saturation_min", 100); 
	ros::Publisher p_bri = visionguiN.advertise<std_msgs::Char>("vision_brightness_min", 100);
	
	//Create window
	cvNamedWindow("ColorSelector",0);

	//Create track Bars
	cvCreateTrackbar("Hue","ColorSelector", &hue,179,&changeColor);
	cvCreateTrackbar("Saturation","ColorSelector", &sat,255,&changeColor);
	cvCreateTrackbar("Brightness","ColorSelector", &bri,255,&changeColor);

	while (ros::ok()){

		// Allow The HighGUI Windows To Stay Open
		cv::waitKey(3);

		//See if there are any changes
		if ((hue != lh) || (sat != ls) || (bri != lb))
		{
			lh = hue;
			ls = sat;
			lb = bri;

			std_msgs::Char h;
			std_msgs::Char s;
			std_msgs::Char b;

			h.data = lh;
			s.data = ls;
			b.data = lb;

			//Publish new data
			p_hue.publish(h);
			p_sat.publish(s);
			p_bri.publish(b);
		}
	}

	return 0;
}

