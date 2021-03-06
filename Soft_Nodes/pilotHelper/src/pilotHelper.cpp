#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#define PI 3.14159265

    	geometry_msgs::PoseStamped odom_pose;
    	geometry_msgs::PoseStamped pilot_pose;
    	std_msgs::Int8MultiArray mapOccupancy[1000];
    	unsigned int width = 0, height = 0;
    	//char mapOccupancy

void odomCallBack (const nav_msgs::Odometry::ConstPtr& odomData){

    	odom_pose.header.frame_id = "odom";
    	odom_pose.header.stamp = odomData->header.stamp;
	odom_pose.pose.position = odomData->pose.pose.position;
	odom_pose.pose.orientation = odomData->pose.pose.orientation;
	return;
	}
	
void pilotCallBack (const geometry_msgs::PoseStamped::ConstPtr& pilotData){

    	pilot_pose.header.frame_id = "map";
    	pilot_pose.header.stamp = pilotData->header.stamp;
	pilot_pose.pose.position = pilotData->pose.position;
	pilot_pose.pose.orientation = pilotData->pose.orientation;
	return;
	}

void mapCallBack (const nav_msgs::OccupancyGrid::ConstPtr& mapData){

    	width = mapData->info.width;
    	height = mapData->info.height;
    	/*for(int it = mapData->data.begin(); it != mapData->data.end(); ++it)
  	{
   		mapOccupancy[i] = *it;
    		i++;
  	}	*/
	return;
	}

int main(int argc, char** argv){
  ros::init(argc, argv, "pilotHelper");

  ros::NodeHandle n;
  ros::Time current_time;
  
  ros::Subscriber sub1 = n.subscribe("odom", 0, odomCallBack);
  ros::Subscriber sub2 = n.subscribe("gotoXY", 100, pilotCallBack);
  ros::Subscriber sub3 = n.subscribe("map", 100, mapCallBack);
  ros::Rate r(1.0);
  
  double nowx = 0.0, nowy = 0.0, desiredx = 0.0, desiredy = 0.0, 
  Adj = 0.0, Opp = 0.0, Hyp = 0.0, Theta = 0.0, initHyp = 0.0, oldPilotx = 0.0, oldPiloty = 0.0, desiredRate = 0.0;
  bool changed = false;
  while(n.ok()){    
  	ros::spinOnce();
  	current_time = ros::Time::now();
  	nowx = odom_pose.pose.position.x;
  	nowy = odom_pose.pose.position.y;
  	desiredx = pilot_pose.pose.position.x;
  	desiredy = pilot_pose.pose.position.y;
  	if ((desiredx != oldPilotx) || (desiredy != oldPiloty)){
  		changed = true;
  	}
  	
  	
  	
  	Adj = desiredx - nowx;
  	  
  	Opp = desiredy - nowy;
  			
  	Hyp = sqrt((Adj*Adj) + (Opp*Opp));
  	
  	Theta = atan(Opp/Adj)*(180/PI);
  	
  	if ((Opp < 0) && (Adj > 0)){
  		Theta = (Theta *-1) + 90; //into heading type values.
  	} else   	if ((Opp < 0) && (Adj < 0)){
  		Theta = (Theta *-1) - 90;
  	}    r.sleep();
  	if (changed){
  		oldPilotx = desiredx;
  		    r.sleep();oldPiloty = desiredy;
  		initHyp = Hyp;
  		changed = false;
  	}
  	desiredRate = (initHyp / Hyp)*100;
  	if (desiredRate >= 75){
  	desiredRate = 75;
  	}
  	if (desiredRate <= 5){
  	desiredRate = 0;
  	}
  	printf("Adj: %f Opp: %f Hyp: %f DesHyp: %f Th: %f Desired Rate: %f\n" ,Adj, Opp, Hyp, initHyp, Theta, desiredRate);
  	r.sleep();
  }
  }
  	
  	
