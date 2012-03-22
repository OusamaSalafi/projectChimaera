#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <stdio.h>
#include <unistd.h>
#include "../include/vNavIMU/vectornav.h"
#include "../include/vNavIMU/vNavIMU.h"

using namespace std;
#include <iostream>

#define LOOP_RATE 10

//#include <sstream>
//
//For ease of integration this node publishes to the same topics as the old compass driver
//
const char* const COM_PORT = "//dev//ttyS1";
const int BAUD_RATE = 115200;
float heading, pitch, roll;		/*Floats for the returned values*/

int main(int argc, char **argv)
{
	
	Vn100 vn100; //custom handle for vn100 from provided vectornav library - SHOULD CHECK THESE OUT	
	VnYpr ypr; 
	int connect = 0; //
	ros::init(argc, argv, "vNavIMU"); //init the driver 
	ros::NodeHandle imu; //create a handle for the node - this does the init and cleans up the node on destruction

	/*Advertises our various messages*/
	ros::Publisher compassHeadingMsg = imu.advertise<std_msgs::Float32>("compassHeading", 100);
	ros::Publisher compassPitchMsg = imu.advertise<std_msgs::Float32>("compassPitch", 100);
	ros::Publisher compassRollMsg = imu.advertise<std_msgs::Float32>("compassRoll", 100);

	/*Sets up the message structures*/
	std_msgs::Float32 compassHeading;
	std_msgs::Float32 compassPitch;
	std_msgs::Float32 compassRoll;

	ros::Rate loop_rate(LOOP_RATE);

	//init the vectornav IMU
	connect = vn100_connect(&vn100, COM_PORT, BAUD_RATE);
	if(connect != 0){ //vn100 Functions return 0 if everything is ok
		vnError(connect);
		return 0; //Connection failed in some way, abort
	}

	
	ROS_INFO("vNavIMU - Driver Online");
  
	while (ros::ok())
	{
		
		vn100_getYawPitchRoll(&vn100, &ypr); //vectornav function
		printf("YPR: %+#7.2f %+#7.2f %+#7.2f\n", ypr.yaw, ypr.pitch, ypr.roll);
		ROS_DEBUG("H: %+#7.2f P: %+#7.2f R: %+#7.2f",ypr.yaw, ypr.pitch, ypr.roll);
		compassHeading.data = ypr.yaw;	
		compassPitch.data = ypr.pitch;
		compassRoll.data = ypr.roll;
		

		ros::spinOnce();

		loop_rate.sleep();

	}

	ROS_INFO("vNavIMU - Shutting Down");
	vn100_disconnect(&vn100);
	return 0;
}

int vnError(int con)
{

	if (con == 1){
			ROS_ERROR("vNavIMU - Unknown Error");
		}
		else if (con == 2){
			ROS_ERROR("vNavIMU - Not Implemented");
		}
		else if (con == 3){
			ROS_ERROR("vNavIMU - Timeout");
		}
		else if (con == 4){
			ROS_ERROR("vNavIMU - Invalid Value");
		}
		else if (con == 5){
			ROS_ERROR("vNavIMU - File Not Found");
		}
		else if (con == 6){
			ROS_ERROR("vNavIMU - Not Connected");
		}
		else {
			ROS_ERROR("vNavIMU - An attempt to connect returned the unknown error code - %d", con);
		}
		
		return 0;	//we failed to open the port so end


}
