#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Float32.h"

#include "pidRamp.h"

int main(int argc, char **argv){

	ros::init(argc, argv, "pidRamp");

	/* Messages and Services */

	ros::NodeHandle pidRampN;

	/* Publish */

	ros::Publisher pitchMsg = pidRampN.advertise<std_msgs::UInt32>("pidRampPitch", 100);
	ros::Publisher yawLMsg = pidRampN.advertise<std_msgs::UInt32>("pidRampYawLeft", 100);
	ros::Publisher yawRMsg = pidRampN.advertise<std_msgs::UInt32>("pidRampYawRight", 100);
	ros::Publisher depthRMsg = pidRampN.advertise<std_msgs::UInt32>("pidRampDepthLeft", 100);
	ros::Publisher depthLMsg = pidRampN.advertise<std_msgs::UInt32>("pidRampDepthRight", 100);

	std_msgs::UInt32 pidRampPitch;
	std_msgs::UInt32 pidRampLeft;
	std_msgs::UInt32 pidRampRight;
	std_msgs::UInt32 pidRampDepthL;
	std_msgs::UInt32 pidRampDepthR;

	/* Subscribe */

	ros::Subscriber sub1 = pidRampN.subscribe("pitchRate", 100, pitchRateCallback);
	ros::Subscriber sub2 = pidRampN.subscribe("leftRate", 100, leftRateCallback);
	ros::Subscriber sub3 = pidRampN.subscribe("rightRate", 100, rightRateCallback);
	ros::Subscriber sub4 = pidRampN.subscribe("depthRRate", 100, depthRRateCallback);
	ros::Subscriber sub5 = pidRampN.subscribe("depthLRate", 100, depthLRateCallback);
	ros::Subscriber sub6 = pidRampN.subscribe("pilotSpeed", 100, speedCallback);
	ros::Subscriber sub7 = pidRampN.subscribe("compassRoll", 100, rollCallback);
	
	ros::Subscriber sub8 = pidRampN.subscribe("pilotGo", 100, goCallback);

	ros::Rate loop_rate(25);

	ROS_INFO("PID Rate Online");

	while(ros::ok()){

		ros::spinOnce();

		pidRampPitch.data = slewer(PITCH);
		pidRampLeft.data = slewer(LEFT);
		pidRampRight.data = slewer(RIGHT);
		pidRampDepthL.data = slewer(DEPTHR);
		pidRampDepthR.data = slewer(DEPTHL);

		//ROS_DEBUG("F: %u L: %u R: %u B: %u",pidRampFront.data,pidRampLeft.data,pidRampRight.data,pidRampBack.data);

		pitchMsg.publish(pidRampPitch);
		yawLMsg.publish(pidRampLeft);
		yawRMsg.publish(pidRampRight);
		depthRMsg.publish(pidRampDepthL);
		depthLMsg.publish(pidRampDepthR);

		loop_rate.sleep();

	}

	printf("Shutting Down\n");

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
** Get pitch pwm target value			**
*************************************************/

void pitchRateCallback(const std_msgs::Float32::ConstPtr& pitchRate){
	targetRate[PITCH] = (int)pitchRate->data;
	return;
}

/*************************************************
** Get left pwm target value			**
*************************************************/

void leftRateCallback(const std_msgs::Float32::ConstPtr& leftRate){
	targetRate[LEFT] = (int)leftRate->data;
	return;
}

/*************************************************
** Get right pwm target value			**
*************************************************/

void rightRateCallback(const std_msgs::Float32::ConstPtr& rightRate){
	targetRate[RIGHT] = (int)rightRate->data;
	return;
}

/*************************************************
** Get depth right pwm target value			**
*************************************************/

void depthRRateCallback(const std_msgs::Float32::ConstPtr& depthRRate){
	targetRate[DEPTHR] = (int)depthRRate->data;
	return;
}

/*************************************************
** Get depth left pwm target value			**
*************************************************/

void depthLRateCallback(const std_msgs::Float32::ConstPtr& depthLRate){
	targetRate[DEPTHL] = (int)depthLRate->data;
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
** Returns the compass pitch			**
*************************************************/

void rollCallback(const std_msgs::Float32::ConstPtr& compassRoll){
	roll = compassRoll->data;
	return;
}
/*************************************************
** Slides the current PWM value until it is	**
** matching the target PWM value. This should	**
** prevent the snapping motion seen before 	**
** which damaged one of the motor pins		**
*************************************************/

unsigned int slewer(unsigned int pos){
	
	if(first){
		currentRate[pos] = 0;//targetRate[pos]; //could be why the pwm wasn't starting (i.e. ramp too fast)
		first = 0;
	}
	if(targetRate[pos] == 0){
		currentRate[pos] = 0;
	}
	else{
		if(targetRate[pos] > currentRate[pos]){
			if(pos == PITCH){
				if((currentRate[pos] + 10) > targetRate[pos]){
					currentRate[pos] += 1;
				}
				else{
					currentRate[pos] += CATCHRATEF;
				}	
			}
			else{
				if((currentRate[pos] + 10) > targetRate[pos]){
					currentRate[pos] += 1;
				}
				else{
					currentRate[pos] += CATCHRATE;
				}
			}
		}
		else if(targetRate[pos] < currentRate[pos]){
			if(pos == PITCH){
				/*if((currentRate[pos] - 10) < targetRate[pos]){
					currentRate[pos] -= 1;
				}
				else{
					currentRate[pos] -= CATCHRATEF;
				}*/
				currentRate[pos] = targetRate[pos];
			}
			else{
				if((currentRate[pos] - 10) < targetRate[pos]){
					currentRate[pos] -= 1;
				}
				else{
					currentRate[pos] -= CATCHRATE;
				}
			}
		}
	}

	if((pos == LEFT) || (pos == RIGHT)){

		if(speed == 0.0){
			if(currentRate[pos] > MAXSPEED){
				currentRate[pos] = MAXSPEED;
			}
	
			if(currentRate[pos] < MINSPEED){
				currentRate[pos] = MINSPEED;
			}
		}
		else{
			if(currentRate[pos] > (MAXSPEED + speed)){
				currentRate[pos] = (MAXSPEED + speed);
			}
	
			if(currentRate[pos] < MINSPEED){
				currentRate[pos] = MINSPEED;
			}

			if(currentRate[pos] > 100.0){
				currentRate[pos] = 100.0;
			}
		}
	}
	else if(pos == DEPTHL)
	{
		if(roll < 0 && go == 1)
			currentRate[pos] = currentRate[pos] + ((roll * -1) * ROLLBONUS);	

	
		if(currentRate[pos] > MAXSPEEDD){
			currentRate[pos] = MAXSPEEDD;
		}

		if(currentRate[pos] < MINSPEEDD){
			currentRate[pos] = MINSPEEDD;
		}	
	
	}
	else if(pos == DEPTHR)
	{
	
		if(roll > 0 && go == 1)
			currentRate[pos] = currentRate[pos] + (roll * ROLLBONUS);
	
		if(currentRate[pos] > MAXSPEEDD){
			currentRate[pos] = MAXSPEEDD;
		}

		if(currentRate[pos] < MINSPEEDD){
			currentRate[pos] = MINSPEEDD;
		}	
	
	}	
	else{
		if(currentRate[pos] > MAXSPEEDD){
			currentRate[pos] = MAXSPEEDD;
		}

		if(currentRate[pos] < MINSPEEDD){
			currentRate[pos] = MINSPEEDD;
		}
	}		

	outRate[pos] = currentRate[pos];
	outRate[pos] *= SCALAR;
	outRate[pos] = outRate[pos] + ZERO_DUTY_CYCLE_US;

	ROS_DEBUG("Chan %u Speed %d Target %d Out %d",pos,currentRate[pos],targetRate[pos],outRate[pos]);

	return (unsigned int)outRate[pos];
}
