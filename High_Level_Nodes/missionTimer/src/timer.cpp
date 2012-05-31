#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// reading a text file
#include <iostream>
#include <fstream>
#include <string>

#include "ros/ros.h"
#include "std_msgs/Int32.h"

using namespace std;

int main(int argc, char **argv)
{
    
	//Set up all the usual ros stuff:
	ros::init(argc, argv, "timer");

	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<std_msgs::Int32>("timerFlag", 100);

	
	//Set up time variables:
	time_t startSeconds, currentSeconds;

	//Get start time.
	startSeconds = time (NULL);

	
	int missionLength;
	float tempRead;
	string line;

	//Get missionLength from file:
	ifstream timerFile ("/home/uwesub/projectChimaera/Config/missionTimer_missionLength.txt");
	if (timerFile.is_open())		//Open file (and check if it opened);
	{
		getline (timerFile,line);	//Read first line
		//cout << line << endl;		//print it
		
		tempRead = atof(line.c_str());	//convert to float (tempRead)

		timerFile.close();		//Close file
	}
	//If the file fails to read, complain and then set to a defaul time (10 mins)
	else 
	{
		printf("Unable to open file, resorting to defaul 10min run.\n");
		tempRead = 10.0;	
	}
	
	//missionLength needs to be in seconds, times the value in the file by 60, Bob's your uncle.
	missionLength = int(tempRead * 60);

	//Set ROS loop rate (Hz):
	ros::Rate r(5);
	
	//Main ros loop:
	while (ros::ok())
	{
		//Set up timerFlag publisher
		std_msgs::Int32 timerFlag;
		//Check current time.
		currentSeconds = time (NULL);
		//Debug
		//int start = startSeconds;
		//int current = currentSeconds;
		//printf("Read  :%f\nM Len : %d\nStart : %d\nCurnt : %d\n\n",tempRead,missionLength, start, current);
	
		//Check if mission is over time, if so send a 1, otherwise send a 0
		if(currentSeconds >= startSeconds + missionLength)
		{
			timerFlag.data = 1;
			printf("OUT OF TIME :-(\n");
		}
		else
			timerFlag.data = 0;
		

		//Publish array
		pub.publish(timerFlag);
		//Let the world know
		//ROS_INFO("I published something!");
		ros::spinOnce();
		r.sleep();

	}

}
