//Includes
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ros/ros.h>
#include "std_msgs/Char.h"
#include "inifile.h"
#include <iostream>
#include "std_msgs/Char.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"

//Variables to store hue, saturation, and brightness in
int hue = 127;
int sat = 127;
int bri = 127;

int huem = 127;
int satm = 127;
int brim = 127;

//
char* ass = "dwncam2_config.ini";
char file_path[300];

//Callback function for trackbars
void changeColor(int pos)
{
}

int main(int argc, char **argv){ //we need argc and argv for the rosInit 

	ros::init(argc, argv, "visiongui");	//Initialize the node

	//Our node handle
	ros::NodeHandle visionguiN;	

	CIniFile ini;

	char nval[10];

	//Read in the environmental variable that stores the location of the config files
		char *configpath = getenv("SUB_CONFIG_PATH");
		if (configpath == NULL)
		{
		std::cout << "Problem getting SUB_CONFIG_PATH variable." << std::endl;
		exit(-1);
		}
		//filepath=configpath+filename
		sprintf(file_path, "%s%s", configpath, ass);

	/* Read Config File */
	ini.Load(file_path);
	
	//Read the values in
	hue = atoi(ini.GetKeyValue("hue" , "min_hue").c_str());
	huem = atoi(ini.GetKeyValue("hue" , "max_hue").c_str());			
	sat = atoi(ini.GetKeyValue("saturation" , "min_sat").c_str());
	satm = atoi(ini.GetKeyValue("saturation" , "max_sat").c_str());
	bri = atoi(ini.GetKeyValue("brightness" , "min_bri").c_str());
	brim = atoi(ini.GetKeyValue("brightness" , "max_bri").c_str());
	
	int lh = hue;
	int ls = sat;
	int lb = bri;

	int lhm = huem;
	int lsm = satm;
	int lbm = brim;

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
	cvCreateTrackbar("MAX Hue","ColorSelector", &huem,179,&changeColor);
	
	cvCreateTrackbar("MIN Saturation","ColorSelector", &sat,255,&changeColor);
	cvCreateTrackbar("MAX Saturation","ColorSelector", &satm,255,&changeColor);
	
	cvCreateTrackbar("MIN Brightness","ColorSelector", &bri,255,&changeColor);
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
			sprintf(nval, "%d", hue);
			ini.SetKeyValue( "hue", "min_hue", nval);
		}

		if (sat != ls)
		{
			ls = sat;
			
			std_msgs::Char s;
			
			s.data = ls;
			
			//Publish new data
			p_sat.publish(s);

			sprintf(nval, "%d", sat);
			ini.SetKeyValue( "saturation", "min_sat", nval);
		}

		if (bri != lb)
		{
			lb = bri;
			
			std_msgs::Char b;
			
			b.data = lb;
			
			//Publish new data
			p_bri.publish(b);
			sprintf(nval, "%d", bri);
			ini.SetKeyValue( "brightness", "min_bri", nval);
		}

		if (huem != lhm)
		{
			lhm = huem;
			
			std_msgs::Char hm;
			
			hm.data = lhm;
			
			//Publish new data
			p_huem.publish(hm);

			sprintf(nval, "%d", huem);
			ini.SetKeyValue( "hue", "max_hue", nval);
		}

		if (satm != lsm)
		{
			lsm = satm;
			
			std_msgs::Char sm;
			
			sm.data = lsm;
			
			//Publish new data
			p_satm.publish(sm);
			sprintf(nval, "%d", satm);
			ini.SetKeyValue( "saturation", "max_sat", nval);
		}

		if (brim != lbm)
		{
			lbm = brim;
			
			std_msgs::Char bm;
			
			bm.data = lbm;
			
			//Publish new data
			p_brim.publish(bm);
			sprintf(nval, "%d", brim);
			ini.SetKeyValue( "brightness", "max_bri", nval);
		}
	}
	std::cout << "Saving INI Changes..." << std::endl;
	ini.Save(file_path);
	std::cout << "Done. Closing!" << std::endl;
	return 0;
}

