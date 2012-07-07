#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "inifile.h"
#include "std_msgs/Char.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
//NOW IN INI FILE
//#define ACCXTHRESH 0.01
//#define ACCYTHRESH 0.01
//#define VELOCITYSR 1000 //250
//#define	VELOCITYTHRESHY 0.001
//#define	VELOCITYTHRESHX 0.000005
//#define VELDEADZONE 0.000005
  
geometry_msgs::Quaternion orient;
geometry_msgs::Vector3 angVel;
geometry_msgs::Vector3 linAcc;

void IMUCallBack (const sensor_msgs::Imu::ConstPtr& IMUData);

char* odomIni = "odom_config.ini";

char file_path[300];

int main(int argc, char** argv){
ros::init(argc, argv, "odometry_publisher");
ros::NodeHandle n;
ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
ros::Subscriber sub1 = n.subscribe("IMU", 0, IMUCallBack);
tf::TransformBroadcaster odom_broadcaster;
CIniFile ini;
//Read in the environmental variable that stores the location of the config files
char *configpath = getenv("SUB_CONFIG_PATH");
if (configpath == NULL)
{
	std::cout << "Problem getting SUB_CONFIG_PATH variable." << std::endl;
	exit(-1);
	}
	//filepath=configpath+filename
sprintf(file_path, "%s%s", configpath, odomIni);
	/* Read Config File */
ini.Load(file_path);

double ACCXTHRESH = atof(ini.GetKeyValue("ACC" , "accx_thresh").c_str());
double ACCYTHRESH = atof(ini.GetKeyValue("ACC" , "accy_thresh").c_str());			
int VELOCITYSR = atoi(ini.GetKeyValue("VEL" , "velsr").c_str());
double VELOCITYTHRESHY = atof(ini.GetKeyValue("VEL" , "vely_thresh").c_str());
double VELOCITYTHRESHX = atof(ini.GetKeyValue("VEL" , "velx_thresh").c_str());
double VELDEADZONE = atof(ini.GetKeyValue("VEL" , "vel_deadzone").c_str());

  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;	

  double deltavx = 0.0;
  double deltavy = 0.0;
  double deltavxTemp = 0.0;
  double deltavyTemp = 0.0;
  
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
  
  double accx = 0.0;
  double accy = 0.0;
  double accth = 0.0;
  
  double prevVelx = 0.0;
  double prevVely = 0.0;
  double prevVelth = 0.0;

  double delta_x;
  double delta_y;
  double delta_th;
  double dt;

  int velCount = 0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1000.0);
  while(n.ok()){
  	
  	
  	
  
  
    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
    
    dt = (current_time - last_time).toSec(); //set time
   	
    accx = linAcc.x; //get acceleration from IMU
    accy = linAcc.y;
    accth = linAcc.z;
    
    if ((accx < ACCXTHRESH) && (accx > -ACCXTHRESH)){ //threshold acceleration coming in
    accx = 0;
    } else if (accx > ACCXTHRESH) {
    accx = accx -ACCXTHRESH;
    } else if (accx < -ACCXTHRESH) {
    accx = accx +ACCXTHRESH;
    }
    
    if ((accy < ACCYTHRESH) && (accy > -ACCYTHRESH)){ //same for y
    accy = 0;
    } else if (accy > ACCYTHRESH) {
    accy = accy -ACCYTHRESH;
    } else if (accy < -ACCYTHRESH) {
    accy = accy +ACCYTHRESH;
    }
    

    vx = prevVelx + (accx * dt);// Velocity in X
    vy = prevVely + (accy * dt);// Velocity in Y
    vth = angVel.z; //IMU does angular velocity
    
    deltavxTemp += vx; //do integral of velocity
    deltavyTemp += vy;
    
    if (velCount == VELOCITYSR){ //average results
    velCount = 0;
    deltavx = deltavxTemp / VELOCITYSR;
    deltavy = deltavyTemp / VELOCITYSR;
    deltavxTemp = 0;
    deltavyTemp = 0;
    } else {
    velCount += 1;
    }
    
    //fix for velocity not returning to 0
    if (((vx <= deltavx + VELOCITYTHRESHX) && (vx >= deltavx - VELOCITYTHRESHX)) || ((vx > -VELDEADZONE) && (vx < VELDEADZONE))){ 
    vx = 0;
    } else if (vx > VELDEADZONE){
    vx = vx - VELDEADZONE;
    } else if (vx < -VELDEADZONE){
    vx = vx + VELDEADZONE;
    }
    
    if (((deltavx < 0) && (vx > 0) && (vx < 0.1))  || ((deltavx > 0) && (vx < 0) && (vx > -0.1))){
    vx = 0;
    }
    
    if ((vy <= deltavy + VELOCITYTHRESHY) && (vy >= deltavy - VELOCITYTHRESHY)){ 
    vy = 0;
    } else if (vy > deltavy + VELOCITYTHRESHY){
    vy = vy - VELOCITYTHRESHY;
    } else if (vx < deltavx - VELOCITYTHRESHY){
    vy = vy + VELOCITYTHRESHY;
    }
    

    printf("T = %f \n", dt);
    printf("X velocity thresh = %f \n", VELOCITYTHRESHY);   
    printf("Y velocity = %+#7.2f \n", vy);
    printf("angular velocity = %+#7.2f\n\n", vth);
    
    
    printf("X Accel = %+#7.2f \n", accx);
    printf("Y Accelleration = %+#7.2f \n", accy);
    printf("angular accel = %+#7.2f\n\n", accth);
    //compute odometry in a typical way given the velocities of the robot
    //X
    delta_x = (vx*dt) - (0.5*(accx)*(dt*dt));    //PHYSICS!
    x += delta_x;
    //Y
    delta_y = (vy*dt) - (0.5*(accy)*(dt*dt));
    y += delta_y;
    //TH   
    delta_th = ((prevVelth + vth) / 2)*dt; //DIFFERENT PHYSICS FOR THETA!
    th += delta_th;
	    
    printf("X = %+#7.2f \n", x);
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
    prevVely = vy;

    last_time = current_time;
    }
    r.sleep();
  
}

void IMUCallBack (const sensor_msgs::Imu::ConstPtr& IMUData){
	linAcc = IMUData->linear_acceleration;
	angVel = IMUData->angular_velocity;
	orient = IMUData->orientation;
	return;
	}

	






