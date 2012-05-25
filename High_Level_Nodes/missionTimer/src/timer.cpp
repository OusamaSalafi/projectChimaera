

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// reading a text file
#include <iostream>
#include <fstream>
#include <string>
using namespace std;

#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"

int main(int argc, char **argv)
{
    

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
	ifstream timerFile ("/home/alex/projectChimaera/missionLength.txt");
	if (timerFile.is_open())		//Open file (and check if it opened);
	{
		getline (timerFile,line);	//Read first line
		//cout << line << endl;		//print it
		
		tempRead = atof(line.c_str());	//convert to float (tempRead)

		timerFile.close();		//Close file
	}
	else 
	{
		printf("Unable to open file, resorting to defaul 10min run.\n");
		tempRead = 10.0;	
	}
	
	missionLength = int(tempRead * 60);

	while (ros::ok())
	{
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
		//Do this.
		ros::spinOnce();
		//Added a delay so not to spam
		sleep(2);
	}

}
