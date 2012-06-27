#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"

#include "roboard.h"
#include "ad79x8.h"



#include "adc.h"

int go = 0;
//int counter = 0;

int main(int argc, char **argv){ //we need argc and argv for the rosInit function

	ros::init(argc, argv, "adc");	//inits the driver

	/* Messages and services */

	ros::NodeHandle adcN;

	/* Publish */

	ros::Publisher adcGoMsg = adcN.advertise<std_msgs::UInt32>("adcGo", 100);
	std_msgs::UInt32 adcGo;

//	initADC();

	ROS_INFO("ADC Online");

	ros::Rate loop_rate(10); //how many times a second (i.e. Hz) the code should run

	while (ros::ok()){
		//ros::spin();
		readADC();

		adcGo.data = checkGo();
		
		if((latch == 1) && (adcGo.data == 1)){
			if(counter > 100){
				adcGo.data = 2;
				latch = 2;
				counter = 0;
			}
		}

		if(latch == 1){
			adcGo.data = 1;
		}
		if(latch == 2){
			adcGo.data = 2;
			if(counter > 100){
				latch = 0;
			}
		}
		printf("Latch: %u, counter: %u",latch,counter);
		counter++;
		adcGoMsg.publish(adcGo);

		loop_rate.sleep();
	}

	printf("Shutting Down\n");

	return 0;
}

unsigned int checkGo(void){
	goTmp = (float)accRaw[GO];
	goTmp /= ADCRES;
	goTmp *= VREFH;
	
	if(goTmp >= VON){
		ROS_DEBUG("GO!!");
		if(latch == 0){
			counter = 0;
		}
		if(latch != 2){
			latch = 1;
		}
		return 1; //if we have an appropriate voltage return go
	}
	printf("STOP!!");
	return 0;	//else return stop
}	

void readADC(void){

        unsigned int val,i;
        float gotmp;

        if(spi_Init(SPICLK_21400KHZ))
        {
                //for(i=0;i<8;i++){
                        accRaw[0] = adc_ReadChannel(0, ADCMODE_RANGE_2VREF,ADCMODE_UNSIGNEDCODING);
                        //printf("Val at channel %u: is %u\n",i,accRaw[i]);
                //}
                spi_Close();
        }
        gotmp = accRaw[0];
        gotmp /= 1023.0;
        gotmp *= 5000.0;

        if(gotmp <= 200.0)
        {
        
                go = 1;
                printf("Go Forth!\n");

        }

        return; 
}



