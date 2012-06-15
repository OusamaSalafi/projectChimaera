#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;  
  
  geometry_msgs::Quaternion orient;
  geometry_msgs::Vector3 angVel;
  geometry_msgs::Vector3 linAcc;
void IMUCallBack (const sensor_msgs::Imu::ConstPtr& IMUData);

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber sub1 = n.subscribe("IMU", 1, IMUCallBack);
  //ros::Subscriber sub2 = n.subscribe("map", 100, MapCallBack);
  tf::TransformBroadcaster odom_broadcaster;
  
  //sensor_msgs::Imu sub1;

	

  
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
  
  double accx = 0.0;
  double accy = 0.0;
  double accth = 0.0;
  
  double prevVelx = 0.0;
  double prevVely = 0.0;
  double prevVelth = 0.0;
  
  double prevAccx = 0.0;
  double prevAccy = 0.0;

  double delta_x;
  double delta_y;
  double delta_th;
  double dt;
  
  double calAccx = 0.0;
  double calAccy = 0.0;
  double calAccth = 0.0;
  int calCount = 0.0;
  
  bool calibrated = false;


  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  printf("Calibrating...");
  ros::Rate r(100.0);
  while(n.ok()){
  	
    if ((calCount <= 100) && (calibrated == false)){
  	calAccx += accx;
  	calAccy += accy;
  	calCount += 1;
  	
  	} else {
  	calibrated = true;
  	calAccx = calAccx / 100;
  	calAccy = calAccy / 100;
  	}
  	
  	
  
  
    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
    
    if (calibrated == true){
    
    dt = (current_time - last_time).toSec(); 
   	
    accx = linAcc.x;
    accy = linAcc.y;
    accth = linAcc.z;
    
    if ((accx > -0.0) && (accx < 0.1)){
    //accx = 0;
    prevVelx = 0;
     
    }/*
    if (((prevVelx > 1) || (prevVelx < -1)) || ((prevVelx < 0.0001) || (prevVelx > -0.0001))){ //PLAY WITH THIS TILL IT WORKS!
    prevVelx = 0;
    }*/
    
    
    
        
    if (((accy > 0) && (accy < 0.5)) || ((accy < 0) && (accy > -0.5))){
    //accy = 0;
    prevVely = 0;
    }
    
    vx = prevVelx + (accx * dt);// Velocity in X
    vy = prevVely + (accy * dt);// Velocity in Y
    vth = angVel.z;
    
    printf("T = %f \n", dt);
    printf("X velocity = %+#7.3f \n", vx);   
    printf("Y velocity = %+#7.2f \n", vy);
    printf("angular velocity = %+#7.2f\n\n", vth);
    
    
    printf("X Accel = %+#7.2f \n", accx);
    printf("Y Accelleration = %+#7.2f \n", accy);
    printf("angular accel = %+#7.2f\n\n", accth);
    //compute odometry in a typical way given the velocities of the robot
    

    /*double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;*/
    
    
    
    //if (((vx > 0.1) || (vx < -0.1)) && ((accx >= 0.1) || (accx <= -0.1))){
	    delta_x = (vx*dt) - (0.5*(accx)*(dt*dt));    
	    x += delta_x;
    //}
    
    //if (((vy > 0.1) || (vy < -0.1)) && ((accy >= 0.1) || (accy <= -0.1))){
	    delta_y = (vy*dt) - (0.5*(accy)*(dt*dt));
	    y += delta_y;
    //}
    
   
	    delta_th = ((prevVelth + vth) / 2)*dt;
	    th += delta_th;
    


    

    
    printf("X = %+#7.3f \n", x);
    printf("Y = %+#7.2f \n", y);
    printf("Th = %+#7.2f\n\n", th);

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    //just use orient same data.

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = orient;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = orient;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
    

    //publish the message
    odom_pub.publish(odom);

    prevVelx = vx;
    //prevVely = vy;
    //prevVelth = vth;
    last_time = current_time;
    }
    r.sleep();
  }
}

void IMUCallBack (const sensor_msgs::Imu::ConstPtr& IMUData){
	linAcc = IMUData->linear_acceleration;
	angVel = IMUData->angular_velocity;
	orient = IMUData->orientation;
	return;
	}
	
/*void MapCallBack (const nav_msgs::OccupancyGrid::ConstPtr& MapData){
	linAcc = IMUData->linear_acceleration;
	angVel = IMUData->angular_velocity;
	orient = IMUData->orientation;
	return;
	}*/
	






