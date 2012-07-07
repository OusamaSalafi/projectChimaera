#include <stdio.h>   
#include <unistd.h>  
#include <fcntl.h>   
#include <errno.h>   
#include <termios.h> 
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Char.h"

#include <sstream>

int led_test = 0;

int fd; 	      /* File descriptor for the port */
char fifoBuffer[255]; /* First in first out buffer where data is stored */
int writePos = 0;     /*Write position in fifoBuffer */

/* Globals - For storing received data */
std_msgs::Char reed_status;
std_msgs::Char bt_shutdown;
std_msgs::Float32 battery_voltage1;
std_msgs::Float32 battery_voltage2;
std_msgs::Float32 battery_voltage3;
std_msgs::Float32 battery_voltage4;
std_msgs::Float32 ambient_temperature;
std_msgs::Float32 motor1_temperature;
std_msgs::Float32 motor2_temperature;
std_msgs::Float32 fitpc_temperature;
std_msgs::Float32 router_temperature;
std_msgs::Float32 roboard_temperature;

std_msgs::Float32 battery1_publish_data;
std_msgs::Float32 battery2_publish_data;

/*********************************
** Generates an NMEA checksum	**
*********************************/
char *GenerateChecksum(char *message)
{
    int i = 0;
    unsigned int checksum = 0;
    char foundDollar = 0;
    char hex[3];

    // Generate a checksum for all characters between the $ and the *
    for (i = 0; i < strlen(message); i++)
    {
        // Asterix denotes the end of the message.
        if (message[i] == '*') break;

        // XOR the character with the checksum
        if ((foundDollar))
        {
            checksum ^= (unsigned int)message[i];
        }

        // $ Indicates the start of a message
        if (message[i] == '$') foundDollar = 1;
    }

    // Convert the checksum to a 2-digit hexadecimal and return.
    sprintf(hex, "%02X", checksum);
    return hex;
}


/*****************************************************************
** Searches for the last NMEA 0183 message within the buffer	**
*****************************************************************/
void msgSearch()
{
    int readPos = writePos;
    int count = 0;
    int i;

    // See if the last char received was a LF character
    if (fifoBuffer[readPos] == (char)10)
    {

        if (--readPos < 0) readPos = 255;

        //Work back until we find a $
        while (readPos != writePos)
        {
            count++;

            if (fifoBuffer[readPos] == '$')
            {
                //Extract the message
                char message[count];
                char * crc;
                for (i = 0; i < count; i++)
                {
                    message[i] = fifoBuffer[readPos];
                    readPos = (readPos + 1) % 255;

                    //Ensure the checksum is included
                    if ((i > 3) && (message[i-2] == '*'))
                    {
                        i++;
                        break;
                    }
                }

                message[i] = 0; //Null char for strlen

                // Verify the crc matches
                crc = GenerateChecksum(message);

                if ((crc[0] == message[strlen(message)-2]) &&
                    (crc[1] == message[strlen(message)-1]))
                {
		    char source[3];
		    char messageID[4];
		    char content[73];

                    //Raise the message received flag.
                    for (i = 0; i < strlen(message) - 2; i++)
                    {
                        if (i < 3)
                            source[i] = message[i+1];
                        else if (i < 6)
                            messageID[i-3] = message[i];
                        else if ((i > 6))
                        {
                            if (message[i] == '*')
                            {
                                content[i-7] = 0;
                               break;
                            }
                            content[i-7] = message[i];
                        }
                    }

                    //Clean up, and raise the message received flag
                    source[2] = 0;
                    messageID[3] = 0;

		    //ROS_INFO("MSG SRC: %s, ID: %s, CONTENTS: %s", source, messageID, content);

		    std::istringstream oss(std::string((char *)content));
		    std::string word;

		    int index = 0;
		    
		    while(getline(oss, word, ',')) {
			if (index == 0)
				reed_status.data = (char)atoi(word.c_str());
			else if (index == 1)
				battery_voltage1.data = (float)strtod(word.c_str(), NULL);
			else if (index == 2)
				battery_voltage2.data = (float)strtod(word.c_str(), NULL);
			else if (index == 3)
				battery_voltage3.data = (float)strtod(word.c_str(), NULL);
			else if (index == 4)
				battery_voltage4.data = (float)strtod(word.c_str(), NULL);
			else if (index == 5)
				ambient_temperature.data = (float)strtod(word.c_str(), NULL);
			else if (index == 6)
				motor1_temperature.data = (float)strtod(word.c_str(), NULL);
			else if (index == 7)
				motor2_temperature.data = (float)strtod(word.c_str(), NULL);
			else if (index == 8)
				fitpc_temperature.data = (float)strtod(word.c_str(), NULL);
			else if (index == 9)
				router_temperature.data = (float)strtod(word.c_str(), NULL);
			else if (index == 10)
				roboard_temperature.data = (float)strtod(word.c_str(), NULL);
			else if (index == 11)
				bt_shutdown.data = (char)atoi(word.c_str());
	
			index++;
		    }	
		}
                else
                {
                    // The checksum failed. Ignore the message.
                    ROS_ERROR("CRC Failed");
                }

                //Clear the buffer
                for (i = 0; i < 255; i++)
                    fifoBuffer[i] = 0;

                break;

            }
            if (--readPos < 0) readPos = 255;
        }
    }
}


/*************************************************
** Reads the data, stores it in a buffer	**
*************************************************/
char read_port(void){
	char returnBuffer[255]; 		/*Buffer which stores read data*/
	int retval = read(fd,returnBuffer,sizeof(returnBuffer));
	char *bufPos;
	char msgFound = 0;

	bufPos = &returnBuffer[0];
	for (int i = 0; i < retval; i++)
	{
		/* Write to our fifo buffer */
		fifoBuffer[writePos] = bufPos[i];
		//printf("%c",bufPos[i]);
		/* Search for messages */
		if (fifoBuffer[writePos] == 10)
		{
			msgSearch();
			msgFound = 1;
		}
		/* Get the next write pos */
		writePos = (writePos + 1) % 255;
	}

	return (msgFound);
}

/*********************************
** Opens serial port USB0	**
*********************************/
int open_port(void){	

	// Open the USB port
	fd = open("/dev/ttyUSB1", O_RDWR | O_NDELAY | O_NOCTTY);
	if (fd == -1){
		ROS_ERROR("Could not open port");
		return 0;
	}
	else{
		fcntl(fd, F_SETFL, 0);
		ROS_INFO("Port opened with a descriptor of %d",fd);
	}
	return (fd);
}

/*********************************
** Configures the serial port	**
*********************************/
void config_port(void){
	struct termios options;

	//Get the parameters associated with the terminal
	tcgetattr(fd, &options);

	// Use 4800 baud for both input and output
	cfsetispeed(&options, B4800);
	cfsetospeed(&options, B4800);
		
	// Ignore Modem Status Lines, Enable Receiver, 
	// 8 Bit Character Size, 1 Stop Bit, No Parity	
	options.c_cflag |= (CLOCAL | CREAD | CS8);

	options.c_iflag = IGNPAR; 	//Ignore Chacters with Parity Errors
	options.c_oflag = 0;		//No Output Options
	options.c_lflag = 0;		//No Terminal Options
	options.c_cc[VTIME] = 10;	
	options.c_cc[VMIN] = 0;
	tcflush(fd, TCIFLUSH);

	tcsetattr(fd, TCSANOW, &options); //Update attributes now

	return;
}

int write_port(unsigned char sendBuffer[], unsigned int sendSize)
{

	int n;

	n = write(fd,sendBuffer, sendSize);

	if (n < 0){	
		ROS_ERROR("Failed to write to port");
		return 0;
	}
	else{
		//printf("We Transmitted %d\n",n);
	}
	usleep(10000);
	//sleep(1);
	return (n);
}

int main(int argc, char **argv){ //we need argc and argv for the rosInit function

	ros::init(argc, argv, "picsubnet");	//inits the driver
	ros::NodeHandle picsubnetN;		//this is what ROS uses to connect to a node

	/*Advertises our various messages*/
	ros::Publisher pubReedSwitch = picsubnetN.advertise<std_msgs::Char>("reed_switch", 100);
	ros::Publisher pubBTShutdown = picsubnetN.advertise<std_msgs::Char>("bt_shutdown", 100);
	ros::Publisher pubBatteryVoltage1 = picsubnetN.advertise<std_msgs::Float32>("battery_voltage_1", 100); 
	ros::Publisher pubBatteryVoltage2 = picsubnetN.advertise<std_msgs::Float32>("battery_voltage_2", 100);
	ros::Publisher pubAmbient = picsubnetN.advertise<std_msgs::Float32>("ambient_temperature", 100); 
	ros::Publisher pubMotor1 = picsubnetN.advertise<std_msgs::Float32>("motor_temperature_1", 100); 
	ros::Publisher pubMotor2 = picsubnetN.advertise<std_msgs::Float32>("motor_temperature_2", 100); 
	ros::Publisher pubFitPC = picsubnetN.advertise<std_msgs::Float32>("fitpc_temperature", 100); 
	ros::Publisher pubRouter = picsubnetN.advertise<std_msgs::Float32>("router_temperature", 100); 
	ros::Publisher pubRoboard = picsubnetN.advertise<std_msgs::Float32>("roboard_temperature", 100);

	ros::Rate loop_rate(15); //how many times a second (i.e. Hz) the code should run

	if(!open_port()){
		ROS_ERROR("FAILED TO CONNECT");
		return 0; //we failed to open the port so end
	}

	config_port();

	ROS_INFO("PIC SUBNET ONLINE");

	while (ros::ok()){


		//All on/All off
		led_test++;
		if (led_test == 1)
			write_port((unsigned char *)"$1N", 4);
		else if (led_test == 2)
			write_port((unsigned char *)"$2N", 4);
		else if (led_test == 3)
			write_port((unsigned char *)"$3N", 4);
		else if (led_test == 4)
			write_port((unsigned char *)"$4N", 4);
		else if (led_test == 5)
			write_port((unsigned char *)"$5N", 4);
		else if (led_test == 6)
			write_port((unsigned char *)"$6N", 4);
		else if (led_test == 7)
			write_port((unsigned char *)"$7N", 4);
		else if (led_test == 8)
			write_port((unsigned char *)"$1F", 4);
		else if (led_test == 9)
			write_port((unsigned char *)"$2F", 4);
		else if (led_test == 10)
			write_port((unsigned char *)"$3F", 4);
		else if (led_test == 11)
			write_port((unsigned char *)"$4F", 4);
		else if (led_test == 12)
			write_port((unsigned char *)"$5F", 4);
		else if (led_test == 13)
			write_port((unsigned char *)"$6F", 4);
		else if (led_test == 14)
		{
			write_port((unsigned char *)"$7F", 4);
			led_test = 0;
		}

		// Read data from the port
		if (read_port() == 1)
		{

			//if (bt_shutdown.data == (uint8_t)1) ROS_ERROR("BLUETOOTH SHUTDOWN ACTIVATED");

			//if (reed_status.data == (uint8_t)-1) ROS_WARN("REED SWITCH - NO RESPONSE");

			//There should only ever be 2 batteries connected...
			int battery_counter = 0;

			if (battery_voltage1.data != (float)-1.0) 
			{
				battery1_publish_data.data = battery_voltage1.data;
				battery_counter++;
			}
			if (battery_voltage2.data != (float)-1.0) 
			{
				if (battery_counter == 0)
					battery1_publish_data.data = battery_voltage2.data;
				else
					battery2_publish_data.data = battery_voltage2.data;					
				battery_counter++;
			}
			if (battery_voltage3.data != (float)-1.0) 
			{
				if (battery_counter == 0)
					battery1_publish_data.data = battery_voltage3.data;
				else if (battery_counter == 1)
					battery2_publish_data.data = battery_voltage3.data;
				//else
					//ROS_WARN("More than 2 batteries detected...");
				battery_counter++;
			}
			if (battery_voltage4.data != (float)-1.0) 
			{
				if (battery_counter == 0)
					battery1_publish_data.data = battery_voltage4.data;
				else if (battery_counter == 1)
					battery2_publish_data.data = battery_voltage4.data;
				//else
					//ROS_WARN("More than 2 batteries detected...");					
				battery_counter++;
			}

			if (battery_counter < 2)
				;
				//ROS_WARN("Only %d Batteries found", battery_counter);
			else
			{
				pubBatteryVoltage1.publish(battery1_publish_data);
				pubBatteryVoltage2.publish(battery2_publish_data);
			}

			//Publish new data
			pubReedSwitch.publish(reed_status);
			pubBTShutdown.publish(bt_shutdown);
			pubAmbient.publish(ambient_temperature);
			pubMotor1.publish(motor1_temperature);
			pubMotor2.publish(motor2_temperature);
			pubFitPC.publish(fitpc_temperature);
			pubRouter.publish(router_temperature);
			pubRoboard.publish(roboard_temperature);
		}

		/*Have a snooze*/
		loop_rate.sleep();

	}

	close(fd);
	ROS_INFO("Shutting Down");

	return 0;
}

