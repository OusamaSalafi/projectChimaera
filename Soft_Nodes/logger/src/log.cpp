/* *******************************************************************
 Rule: 
Each team will produce a log file of the mission within around 10 minutes of the end of
the run. 
The format of the log file will be a comma separated ASCII file of the format:
Time, position, action, a comment between simple quotes.
(SSSSS,XXX.x,YYY.y,ZZZ.z,AA.aa). 

Logged data will be plotted by plotting routine
written by the organising committee. This will be used to score the log file. For acoustic
modem task the additional file of AUV heading and light on/off data and cooperative
AUV heading command will need to be provided. For the ASV tracking task the
additional file of range and bearing data from the AUV to the pinger will need to be
provided.

************************************************************************ */

#include <stdio.h>
#include <stdlib.h>

#include <string>
// writing on a text file
#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"



void xCallback(const std_msgs::Int32::ConstPtr& logX);
void yCallback(const std_msgs::Int32::ConstPtr& logY);
void zCallback(const std_msgs::Float32::ConstPtr& svpDepth);
void aCallback(const std_msgs::String::ConstPtr& logAction);

int x, y;
float z;
char aa[100];

using namespace std;

int main(int argc, char **argv)
{
	//Get start time, to take off from current time
	time_t startTime; 
	startTime=time(NULL);
	
	ros::init(argc, argv, "logger");
	
	ros::NodeHandle n;
	//Set up subscriptions
	ros::Subscriber sub1 = n.subscribe("logX", 100, xCallback);
	ros::Subscriber sub2 = n.subscribe("logY", 100, yCallback);
	ros::Subscriber sub3 = n.subscribe("svpDepth", 100, zCallback);
	ros::Subscriber sub4 = n.subscribe("logAction", 100, aCallback);
	
	char tmpName[100];
	
	char buff[20];
	time_t now = time(NULL);
	strftime(buff, 20, "%Y-%m-%d %H:%M:%S", localtime(&now));
	printf("%s\n", buff);
		
	ofstream logFile ("/home/alex/projectChimaera/Logs/log.txt");

	ros::spinOnce();

	//Set loop rate (in Hz);
	ros::Rate r(5);
	while (ros::ok())
	{

		//Check for new data:
		ros::spinOnce();

		//Print log file for debugging:
		printf("%ld,%f,%f,%f,%s\n", time(NULL) - startTime, x, y, z, aa);
		
		//Save log to file:
		if (logFile.is_open())
		{	
		logFile << (time(NULL) - startTime) << ",";
		logFile << x << ",";
		logFile << y << ",";
		logFile << z << ",";
		logFile << aa << "\n";
		//logFile << "This is a line.\n";
		}
		else 
		{
			printf("Unable to open file, logging stopped :(.\n");	
		}
		
		r.sleep();

	}
	
	logFile.close();
	
}

/*************************************************
** Returns the X pose
*************************************************/

void xCallback(const std_msgs::Int32::ConstPtr& logX)
{
	x = logX->data;
	return;
}

/*************************************************
** Returns the Y pose
*************************************************/

void yCallback(const std_msgs::Int32::ConstPtr& logY)
{
	y = logY->data;
	return;
}
/*************************************************
** Returns the depth (Z)
*************************************************/

void zCallback(const std_msgs::Float32::ConstPtr& svpDepth)
{
	//1 decibar = 1.019716 meters
	
	//convert from milibar to decibar and times by meters per decibar.
	z = (100 * svpDepth->data) * 1.019716;
		
	return;
}
/*************************************************
** Returns the sonar bearing **
*************************************************/

void aCallback(const std_msgs::String::ConstPtr& logAction)
{
	//aa = logAction->data;

	strcpy(aa, (logAction->data.c_str()));
	
	//strcpy(char * dest, const char * src);
	
	//logAction ->data;
	

	return;
}
