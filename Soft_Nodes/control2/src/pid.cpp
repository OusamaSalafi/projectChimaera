#include <math.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <sstream>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"

#include "INIReader.h"

#include "pid.h"

char file_path[300];
char *file_name = "control2_config.ini";


int main(int argc, char **argv){

	float tmp;
	unsigned char once = 1;
	
	//call ini_init here to get variables from config file
	ini_init();

	ros::init(argc, argv, "pid");

	/* Messages and Services */

	ros::NodeHandle pidN;
	

	
	/* Choose between heading, depth and pitch control */

	if(!strcmp(argv[1],"heading")){

		/* Publish */

		ros::Publisher leftRateMsg = pidN.advertise<std_msgs::Float32>("leftRate", 100);
		ros::Publisher rightRateMsg = pidN.advertise<std_msgs::Float32>("rightRate", 100);

		std_msgs::Float32 leftRate;
		std_msgs::Float32 rightRate;

		/* Subscribe */

		ros::Subscriber sub1 = pidN.subscribe("compassHeading", 100, headingCallback);
		ros::Subscriber sub2 = pidN.subscribe("pilotHeading", 100, targetHeadingCallback);
		ros::Subscriber sub3 = pidN.subscribe("pilotSpeed", 100, speedCallback);
		ros::Subscriber sub4 = pidN.subscribe("pilotGo", 100, goCallback);

		ros::Rate loop_rate(10);

		ROS_INFO("Heading PID Online");

		KP = KPH;
		KD = KDH;
		KI = KIH;

		PLUSBUFF = PLUSBUFFH;
		MINUSBUFF = MINUSBUFFH;
		deadZ = 1;
		while(ros::ok()){
			
			if(once){
				while(go != 1){
					ros::spinOnce();
					ROS_WARN("%s waiting for go",argv[1]);
					ros::Duration(1.0).sleep();
					rightRate.data = 0.0;
					leftRate.data = 0.0;
					leftRateMsg.publish(leftRate);
					rightRateMsg.publish(rightRate);
				}
				ROS_INFO("%s given the go",argv[1]);
				loop_rate.sleep();
				once = 0;
			}

			ros::spinOnce();

			tmp = p(heading,targetHeading);	//get PID value

			if(tmp > MAXSPEEDH){
				tmp = MAXSPEEDH;
			}
			else if(tmp < MINSPEEDH){
				tmp = MINSPEEDH;
			}

			right = tmp;			//left does opposite of right
			left = (tmp * 0.75);

			left += speed;	//bit hacks but ensures we keep moving forwards even when balanced
			right -= speed;
			
			leftRate.data = left;
			rightRate.data = right;

			leftRateMsg.publish(leftRate);
			rightRateMsg.publish(rightRate);

			ROS_DEBUG("Heading PID Left %.3f PID Right %.3f",leftRate.data,rightRate.data);

			loop_rate.sleep();

		}

	}
	else if(!strcmp(argv[1],"depth")){
		
		/* Publish */

		ros::Publisher pitchRateMsg = pidN.advertise<std_msgs::Float32>("pitchRate", 100);
		ros::Publisher depthDRateMsg = pidN.advertise<std_msgs::Float32>("depthDRate", 100);

		std_msgs::Float32 pitchRate;
		std_msgs::Float32 depthDRate;

		/* Subscribe */

		ros::Subscriber sub1 = pidN.subscribe("svpDepth", 100, depthCallback);
		ros::Subscriber sub2 = pidN.subscribe("pilotDepth", 100, targetDepthCallback);
		ros::Subscriber sub3 = pidN.subscribe("pilotGo", 100, goCallback);

		ros::Rate loop_rate(10);

		ROS_INFO("Depth PID Online");

		KP = KPD;
		KD = KDD;
		KI = KID;

		PLUSBUFF = PLUSBUFFD;
		MINUSBUFF = MINUSBUFFD;

		deadZ = 0;

		while(ros::ok()){
			
			if(once){
				while(go != 1){
					ros::spinOnce();
					ROS_WARN("%s waiting for go",argv[1]);
					ros::Duration(1.0).sleep();
					pitchRate.data = 0.0;
					depthDRate.data = 0.0;
					pitchRateMsg.publish(pitchRate);
					depthDRateMsg.publish(depthDRate);
				}
				ROS_INFO("%s given the go",argv[1]);
				loop_rate.sleep();
				once = 0;
			}

			ros::spinOnce();

			tmp = p(depth,targetDepth);

			if(depth > targetDepth){	//if we are too deep don't turn on the motors, use bouyancy
				tmp *= -0.5f;
			}
			else{
				tmp *= 1.0;
			}

			pitchRate.data = tmp;

			if((targetDepth - depth) > PITCHTHRESH){	//if we are really far off target
				depthDRate.data = tmp * -0.4;			//add rear motor power
				depthDRateMsg.publish(depthDRate);
			}
			else{
				depthDRate.data = 0.0;
			}
							

			pitchRateMsg.publish(pitchRate);

			ROS_DEBUG("Depth PID Pitch %.3f Depth Rear %.3f",pitchRate.data,depthDRate.data);

			loop_rate.sleep();

		}
	}
	else if(!strcmp(argv[1],"pitch")){

		/* Publish */

		ros::Publisher depthPRateMsg = pidN.advertise<std_msgs::Float32>("depthPRate", 100);
		std_msgs::Float32 depthPRate;

		/* Subscribe */

		ros::Subscriber sub1 = pidN.subscribe("compassPitch", 100, pitchCallback);
		ros::Subscriber sub2 = pidN.subscribe("pilotPitch", 100, targetPitchCallback);
		ros::Subscriber sub3 = pidN.subscribe("pilotGo", 100, goCallback);

		ros::Rate loop_rate(10);

		ROS_INFO("Pitch PID Online");

		KP = KPP;
		KD = KDP;
		KI = KIP;

		while(ros::ok()){

			if(once){
				while(go != 1){
					ros::spinOnce();
					ROS_WARN("%s waiting for go",argv[1]);
					ros::Duration(1.0).sleep();
					depthPRate.data = 0.0;
					depthPRateMsg.publish(depthPRate);
				}
				ROS_INFO("%s given the go",argv[1]);
				loop_rate.sleep();
				once = 0;
			}

			ros::spinOnce();

			tmp = pd(pitch,targetPitch);

			tmp *= -1.0f;

			depthPRate.data = tmp;

			if(depthPRate.data < 0.0){	//if the nose is high
				depthPRate.data = 0.0;	//use bouyancy
			}

			depthPRateMsg.publish(depthPRate);

			ROS_DEBUG("Pitch PID Back %.3f",depthPRate.data);

			loop_rate.sleep();

		}
	}
	/* not using this at the moment for roll control, we're using the angle and adding the value on the opposite motor in pidRamp
	*
	*
	* ************************************************************************ */
	else if(!strcmp(argv[1],"roll")){

		/* Publish */

		ros::Publisher depthRLRateMsg = pidN.advertise<std_msgs::Float32>("depthRLRate", 100);
		ros::Publisher depthRRRateMsg = pidN.advertise<std_msgs::Float32>("depthRRRate", 100);
		std_msgs::Float32 depthRLRate;
		std_msgs::Float32 depthRRRate;

		/* Subscribe */

		ros::Subscriber sub1 = pidN.subscribe("compassRoll", 100, rollCallback);
		ros::Subscriber sub2 = pidN.subscribe("pilotRoll", 100, targetRollCallback);
		ros::Subscriber sub3 = pidN.subscribe("pilotGo", 100, goCallback);

		ros::Rate loop_rate(10);

		ROS_INFO("Roll PID Online");

		KP = KPR;
		KD = KDR;
		KI = KIR;

		while(ros::ok()){

			if(once){
				while(go != 1){
					ros::spinOnce();
					ROS_WARN("%s waiting for go",argv[1]);
					ros::Duration(1.0).sleep();
					depthRLRate.data = 0.0;
					depthRRRate.data = 0.0;
					depthRLRateMsg.publish(depthRLRate);
					depthRRRateMsg.publish(depthRRRate);
				}
				ROS_INFO("%s given the go",argv[1]);
				loop_rate.sleep();
				once = 0;
			}

			ros::spinOnce();

			tmp = pd(roll,targetRoll);

			tmp *= -1.0f;

			depthRLRate.data = tmp;
			depthRRRate.data = tmp * 0.75;

			if(depthRLRate.data < 0.0){	//if the nose is high
				depthRLRate.data = 0.0;	//use bouyancy
			}
			if(depthRRRate.data < 0.0){	//if the nose is high
				depthRRRate.data = 0.0;	//use bouyancy
			}
			
			depthRLRateMsg.publish(depthRLRate);
			depthRRRateMsg.publish(depthRRRate);

			ROS_DEBUG("Roll PID Left %.3f",depthRLRate.data);
			ROS_DEBUG("Roll PID Right %.3f",depthRRRate.data);

			loop_rate.sleep();

		}
	}
	else{
		ROS_ERROR("Please Enter 'heading', 'depth' or 'pitch'. Alternatively program some software for %s.",argv[1]);
	}




	printf("Shutting Down %s\n",argv[1]);

	return 0;
}

/*************************************************
** Returns the go signal			**
*************************************************/

void goCallback(const std_msgs::UInt32::ConstPtr& pilotGo){
	go = pilotGo->data;
	return;
}

/*************************************************
** Returns the pilots speed			**
*************************************************/

void speedCallback(const std_msgs::Float32::ConstPtr& pilotSpeed){
	speed = pilotSpeed->data;
	return;
}

/*************************************************
** Returns the compass heading			**
*************************************************/

void headingCallback(const std_msgs::Float32::ConstPtr& compassHeading){
	heading = compassHeading->data;
	return;
}

/*************************************************
** Returns the target heading			**
*************************************************/

void targetHeadingCallback(const std_msgs::Float32::ConstPtr& pilotHeading){
	targetHeading = pilotHeading->data;
	return;
}

/*************************************************
** Returns the compass pitch			**
*************************************************/

void pitchCallback(const std_msgs::Float32::ConstPtr& compassPitch){
	pitch = compassPitch->data;
	return;
}

/*************************************************
** Returns the target pitch			**
*************************************************/

void targetPitchCallback(const std_msgs::Float32::ConstPtr& pilotPitch){
	targetPitch = pilotPitch->data;
	return;
}

/*************************************************
** Returns the svp depth			**
*************************************************/

void depthCallback(const std_msgs::Float32::ConstPtr& svpDepth){
	depth = svpDepth->data;
	return;
}

/*************************************************
** Returns the target depth			**
*************************************************/

void targetDepthCallback(const std_msgs::Float32::ConstPtr& pilotDepth){
	targetDepth = pilotDepth->data;
	return;
}

/*************************************************
** Returns the compass pitch			**
*************************************************/

void rollCallback(const std_msgs::Float32::ConstPtr& compassRoll){
	roll = compassRoll->data;
	return;
}

/*************************************************
** Returns the target pitch			**
*************************************************/

void targetRollCallback(const std_msgs::Float32::ConstPtr& pilotRoll){
	targetRoll = pilotRoll->data;
	return;
}

/*************************************************
** Performed to correct the error as without	**
** this we will be often turning 270 degrees to	**
** the right instead of 90 degrees to the left	**
*************************************************/

float correctError(float error){
	//ROS_INFO("Error %.3f P %.3f M %.3f",error,PLUSBUFF,MINUSBUFF);
	if(error > 180.0f){
		//ROS_INFO("Origninal Error %.3f",error);
		error /= 360.0f;
		error = 1.0f - error;
		error *= 360.0f;
		error *= -1.0f;
		//ROS_INFO("New Error %.3f",error);
	}
	else if(error < -180.0f){
		//ROS_INFO("Origninal Error %.3f",error);
		error /= 360.0f;
		error *= -1.0f;
		error = 1.0f - error;
		error *= 360.0f;
		//ROS_INFO("New Error %.3f",error);
	}

	if(deadZ){
		if((error < PLUSBUFF) && (error > MINUSBUFF)){
			//ROS_INFO("Warn");
			ROS_WARN("DEAD ZONE");
			buffCheck++;
			if(buffCheck < BUFFTHRESH){
				error *= -0.5;
				return error;
			}
			return 0.0;
		} 
		//ROS_INFO("Not warn");
	}

	buffCheck = 0;

	return error;
}

/*************************************************
** P only control				**
*************************************************/

float p(float value, float targetValue){
	float error;

	error = targetValue - value;	//error is target - actual

	ROS_DEBUG("Error: %.3f Target: %.3f Actual %.3f",error, targetValue, value);

	error = correctError(error);

	if(error == 0.0){
		return error;
	}

	error *= KP;				//multiply by constant

	return error;
}

/*************************************************
** P and D control				**
*************************************************/

float pd(float value, float targetValue){
	float error,derivative,lastError = 0.0f;

	error = targetValue - value;

	error = correctError(error);

	if(error == 0.0){
		return error;
	}
	
	derivative = lastError - error;

	lastError = error;

	error *= KP;

	derivative *= KD;

	error += derivative;
	
	return error;
}

/*************************************************
** P and I control				**
*************************************************/

float pi(float value, float targetValue){
	float error, lastOut = 0.0f, lastError = 0.0f,output,tmp1,tmp2;

	error = targetValue - value;

	error = correctError(error);

	if(error == 0.0){
		return error;
	}

	tmp1 = error - lastError;
	tmp1 *= KP;

	tmp2 = (error + lastError) / 2.0f;
	tmp2 *= KI;

	output = lastOut + tmp1 + tmp2;

	output = fmaxf(output,MAXVAL);
	output = fminf(output,MINVAL);

	lastOut = output;

	lastError = error;

	return error;
}

/*************************************************
** PID control					**
*************************************************/

float pid(float value, float targetValue){
	float error, lastOut = 0.0f, lastError = 0.0f, lastError2 = 0.0f,output,tmp1,tmp2,tmp3;

	error = targetValue - value;

	error = correctError(error);

	if(error == 0.0){
		return error;
	}

	tmp1 = error - lastError;
	tmp1 *= KP;

	tmp2 = (error + lastError) / 2.0f;
	tmp2 *= KI;

	tmp3 = error - (2.0f * lastError) + lastError2;
	tmp3 *= KD;

	output = lastOut + tmp1 + tmp2 + tmp3;

	lastOut = output;

	lastError2 = lastError;
	lastError = error;

	return output;
}
 
 void ini_init()
 {
	 char tmp[10];
	//Read in the environmental variable that stores the location of the config files
	char *configpath = getenv("SUB_CONFIG_PATH");
	if (configpath == NULL)
	{
	std::cout << "Problem getting SUB_CONFIG_PATH variable. Reverting to defaults." << std::endl;
	//exit(-1);
	}
	//filepath=configpath+filename
	sprintf(file_path, "%s%s", configpath,	file_name);
	// Read The Config File
	INIReader reader(file_path);
	
	// Check The File Can Be Opened
	if (reader.ParseError() < 0)
	{
		std::cout << "Failed To Load \"~/projectChimaera/Config/control2_config.ini\"\n";
		//setup the defaults
	}
	else
	{
	//	  if(from_string<float>(f, std::string("123.456"), std::dec))
  
		//read the values in
		KPH = (float)strtod(reader.Get("Heading", "GAIN_P", "2.0").c_str(), NULL);
		KDH = (float)strtod(reader.Get("Heading", "GAIN_D", "1.0").c_str(), NULL);
		KIH = (float)strtod(reader.Get("Heading", "GAIN_I", "0.1").c_str(), NULL);
		
		KPD = (float)strtod(reader.Get("Depth", "GAIN_P", "300.0").c_str(), NULL);
		KDD = (float)strtod(reader.Get("Depth", "GAIN_D", "0.1").c_str(), NULL);
		KID = (float)strtod(reader.Get("Depth", "GAIN_I", "0.1").c_str(), NULL);
		
		KPP = (float)strtod(reader.Get("Pitch", "GAIN_P", "10").c_str(), NULL);
		KDP = (float)strtod(reader.Get("Pitch", "GAIN_D", "1.5").c_str(), NULL);
		KIP = (float)strtod(reader.Get("Pitch", "GAIN_I", "0.1").c_str(), NULL);

	}
	
	return;
 }
