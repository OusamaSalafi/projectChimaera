#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <vector>
#include <iostream>

#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "sensor_msgs/LaserScan.h"


#define WIDTH	600
#define HEIGHT 	600
#define DEPTH	255

#define PI 3.14159265

float bins;
float bearing;
int binsArr[90];

void bearingCallback(const std_msgs::Float32::ConstPtr& sonarBearing);
void binsCallback(const std_msgs::Float32::ConstPtr& sonarBins);
void binsArrCallback(const std_msgs::Int32MultiArray::ConstPtr& sonarBinsArr);

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "sonarConverter");
	
	
	
	/* Messages and services */

	ros::NodeHandle conN;	
	
	int i;
	
	ros::Subscriber sub1 = conN.subscribe("sonarBearing", 100, bearingCallback);
	ros::Subscriber sub2 = conN.subscribe("sonarBins", 100, binsCallback);
	ros::Subscriber sub3 = conN.subscribe("sonarBinsArr", 100, binsArrCallback);
	
	ros::Publisher sonarConverterMsg = conN.advertise<sensor_msgs::LaserScan>("sonarConverter", 100);
	sensor_msgs::LaserScan sonarConverter;


	//laser scanner config
	double sonar_frequency = 2000; //same as usleep???
	int num_sonar_readings = 6399; //http://answers.ros.org/question/12381/time-issue-when-publishing-laser-scan-message

	sonarConverter.header.frame_id = "/sonarConverter"; 

	sonarConverter.angle_min = -0.0000017126; //see SLAM wiki	
	sonarConverter.angle_max = 0.0000017126; //see SLAM wiki
	sonarConverter.angle_increment = -0.0000017126; //see SLAM wiki - I'm not sure about this one, or are the above the same to allow for a 360 min and max angle?

	sonarConverter.time_increment = (1 / sonar_frequency) / (num_sonar_readings); //see link on num_sonar_readings

	sonarConverter.range_min = 0.833333333;
	sonarConverter.range_max = 75;	

	ros::Time scan_time = ros::Time::now();
		
	while(ros::ok())
	{
		
		int j;
		for(j=0; j<90; j++)
		{
			sonarConverter.intensities.push_back(binsArr[j]); //chuck the bins into one line of the scan as intensity
			sonarConverter.ranges.push_back(binsArr[j]);
			
			

		}		
		sonarConverter.header.stamp = scan_time; //this seems to allow in index of the scans
		sonarConverterMsg.publish(sonarConverter); //publish the Sonar scan as a LaserScan Message
		ros::spinOnce(); 
		usleep(2000);
	}
		
	
	return 0;
	
}


/*************************************************
** Returns the sonar bearing **
*************************************************/

void bearingCallback(const std_msgs::Float32::ConstPtr& sonarBearing){
bearing = sonarBearing->data;
return;
}

/*************************************************
** Returns the sonar bearing **
*************************************************/

void binsCallback(const std_msgs::Float32::ConstPtr& sonarBins){
bins = sonarBins->data;
return;
}

/*************************************************
** Returns the sonar bearing **
*************************************************/

void binsArrCallback(const std_msgs::Int32MultiArray::ConstPtr& sonarBinsArr)
{
	
int i = 0;
  // print all the remaining numbers
  for(std::vector<int>::const_iterator it = sonarBinsArr->data.begin(); it != sonarBinsArr->data.end(); ++it)
  {
    binsArr[i] = *it;
    i++;
  }

return;
}

