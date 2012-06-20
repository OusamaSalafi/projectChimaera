//Useful shit I found
//http://ros.informatik.uni-freiburg.de/roswiki/navigation(2f)Tutorials(2f)RobotSetup(2f)Sensors.html


#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
//+++
#include "sensor_msgs/LaserScan.h"

int main(int argc, char **argv)
{
    
	float bearing = 0.0;
	//ham
	ros::init(argc, argv, "sonarSpammer");
	
	ros::NodeHandle n;
	
	ros::Publisher pub = n.advertise<std_msgs::Int32MultiArray>("sonarBinsArr", 100);
	
	ros::Publisher sonarBearingMsg = n.advertise<std_msgs::Float32>("sonarBearing", 100);
	
	std_msgs::Int32MultiArray sonarBinsArr;
	std_msgs::Float32 sonarBearing;

	//laser scan publisher and dec
	ros::Publisher sonarScanMsg = n.advertise<sensor_msgs::LaserScan>("sonarScan", 100);
	sensor_msgs::LaserScan sonarScan;


	//laser scanner config
	double sonar_frequency = 2000; //same as usleep???
	int num_sonar_readings = 6399; //http://answers.ros.org/question/12381/time-issue-when-publishing-laser-scan-message

	sonarScan.header.frame_id = "/sonarScan"; 

	sonarScan.angle_min = -0.0000017126; //see SLAM wiki	
	sonarScan.angle_max = 0.0000017126; //see SLAM wiki
	sonarScan.angle_increment = 0.0000017126; //see SLAM wiki - I'm not sure about this one, or are the above the same to allow for a 360 min and max angle?

	sonarScan.time_increment = (1 / sonar_frequency) / (num_sonar_readings); //see link on num_sonar_readings

	sonarScan.range_min = 0.833333333;
	sonarScan.range_max = 75;	

	ros::Time scan_time = ros::Time::now();
	// end laser scanner config

	//Would it be better to have the sonar scan data as a point cloud rather than a laser scan message?
	//laser_geometry contains functionality to account for the skew resulting from moving robots or tilting laser scanners. http://www.ros.org/wiki/laser_geometry
	
	while (ros::ok())
	{
		if(bearing == 6399.0)//6399.0)
		{
			bearing = 0.0;
			//printf("Done!\n");
			//return 0;
		}		
	
		sonarBinsArr.data.clear();
		for (int i = 0; i < 90; i++)
		{
			sonarBinsArr.data.push_back(rand() % 255);
			sonarScan.intensities.push_back(i); //chuck the bins into one line of the scan as intensity
			sonarScan.ranges.push_back(rand() % 255);
			//usleep(200);
		}

		sonarScan.header.stamp = scan_time; //this seems to allow in index of the scans 
		sonarBearing.data = bearing;

		pub.publish(sonarBinsArr);
		sonarBearingMsg.publish(sonarBearing);
		
		sonarScanMsg.publish(sonarScan); //publish the Sonar scan as a LaserScan Message

		//printf("%f \n", bearing); //leaving this is causes major lag
		usleep(20000);

		ros::spinOnce();
		
		bearing = bearing + 1.0;

	}
	
}
