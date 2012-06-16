#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"

#include "roboard.h"
#include "ad79x8.h"


#include "adc.h"


int main(int argc, char **argv){ //we need argc and argv for the rosInit function

	ros::init(argc, argv, "adc");	//inits the driver

	/* Messages and services */

	ros::NodeHandle adcN;

	/* Publish */

	ros::Publisher adcGoMsg = adcN.advertise<std_msgs::UInt32>("adcGo", 100);
	std_msgs::UInt32 adcGo;

	//initADC();

	ROS_INFO("ADC Online");

	ros::Rate loop_rate(10); //how many times a second (i.e. Hz) the code should run

	while (ros::ok()){
		//ros::spin();
		readADC();

		adcGo.data = checkGo();

		if(latch == 1)
		{
			adcGo.data = 1;
		}
		if(latch == 0)
		{
			adcGo.data = 0;
		}
		adcGoMsg.publish(adcGo);

		loop_rate.sleep();
	}

	printf("Shutting Down\n");

	return 0;
}

unsigned int checkGo(void){

	goTmp = accRaw;

	if(goTmp <= ADCTHRESH && latch == 0)
	{
		latch = 1;
		ROS_DEBUG("GO!!");
		printf("Running.\n");
		return 1;
	}
	else if(goTmp <= ADCTHRESH && latch == 1)
	{
		latch = 0;
		ROS_DEBUG("STOP!!");
		printf("Stopped.\n");
		return 0;
	}
	ROS_DEBUG("STOP!!");
	return 0;

}

void readADC(void){

	unsigned int val,i;

	if(spi_Init(SPICLK_21400KHZ))
	{
		//for(i=0;i<8;i++){
			accRaw = adc_ReadChannel(GO, ADCMODE_RANGE_2VREF,ADCMODE_UNSIGNEDCODING);
			//printf("Val at channel %u: is %u\n",GO,accRaw);
		//}
		spi_Close();
	}
	return; 
}


