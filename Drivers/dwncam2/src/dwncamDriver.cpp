/*
 *  OpenCV Demo for ROS
 *  Copyright (C) 2010, I Heart Robotics
 *  I Heart Robotics <iheartrobotics@gmail.com>
 *  http://www.iheartrobotics.com
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <math.h>
//For Reading Config Files
#include "ini.h"
#include "INIReader.h"
#include <iostream>

//Threshold values
int min_hue;
int max_hue;
int min_sat;
	
// ROS/OpenCV HSV Demo
// Based on http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

class Demo
{
	
typedef cv::Vec<uchar, 1> Vec1b;	
typedef cv::Vec<uchar, 3> Vec3b;

protected:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	sensor_msgs::CvBridge bridge_;
	cv::Mat img_in_;
	cv::Mat img_in_proc_;
	cv::Mat img_hsv_;
	cv::Mat img_hue_;
	cv::Mat img_sat_;
	cv::Mat img_bin_;
	cv::Mat img_out_;
	cv::Mat img_red_;
	cv::Mat img_green_;
	cv::Mat img_blue_;
	cv::Mat img_tmp_;
	IplImage *cv_input_;


public:

	Demo(ros::NodeHandle & nh):nh_ (nh), it_ (nh_)
	{
		ROS_INFO("Getting Cam");
		// Listen for image messages on a topic and setup callback
		image_sub_ = it_.subscribe ("/gscam/image_raw", 1, &Demo::imageCallback, this);
		// Open HighGUI Window
		ROS_INFO("Got Cam");
		cv::namedWindow ("input", 1);
		cv::namedWindow ("binary image", 1);
		cv::namedWindow ("segmented output", 1);
		cv::namedWindow ("preprocessed image", 1);
		ROS_INFO("Opened window?");
	}


	void findCentre(void)
	{
		int x, y, count, x_estimate, y_estimate, x_centre, y_centre;
		int x_draw1, x_draw2, y_draw1, y_draw2;
		CvScalar s;

		x_estimate = y_estimate = count = 0;

		IplImage ipl_img = img_bin_;
		IplImage ipl_seg = img_out_;
		//use the first order moments to find centre of black region
		for(x=0;x<ipl_img.height;x++)
		{
			for(y=0;y<ipl_img.width;y++)
			{
				s = cvGet2D(&ipl_img,x,y);
				if(s.val[0] == 255)
				{ //if we have found a white pixel
					count++; //increase counters
					x_estimate += x;
					y_estimate += y;
				}
			}
		}

		if(count < 3000)
		{ //arbritrary size threshold
			printf("No Target in sight saw only %d\n",count);
		}
		else
		{
			x_centre = y_estimate / count; //estimate center of x
			y_centre = x_estimate / count; //and y


			x_draw1 = x_centre + 100; //this is for drawing only
			x_draw2 = x_centre - 100;
			y_draw1 = y_centre + 100;
			y_draw2 = y_centre - 100;

			s.val[0] = 100;
			s.val[1] = 100;
			s.val[2] = 100;

			CvPoint pt1 = {x_draw1,y_centre};
			CvPoint pt2 = {x_draw2,y_centre};
			CvPoint pt3 = {x_centre,y_draw1};
			CvPoint pt4 = {x_centre,y_draw2};
			//Draw cross to threshold image
			cvLine(&ipl_img, pt1 , pt2, s, 1, 8,0);
			cvLine(&ipl_img, pt3 , pt4, s, 1, 8,0);

			s.val[0] = 0;
			s.val[1] = 0;
			s.val[2] = 255;
			//Draw cross to segmented image
			cvLine(&ipl_seg, pt1 , pt2, s, 1, 8,0);
			cvLine(&ipl_seg, pt3 , pt4, s, 1, 8,0);

			img_bin_ = cv::Mat (&ipl_img).clone ();
			img_out_ = cv::Mat (&ipl_seg).clone ();

			printf("X: %d Y: %d Count: %d Yeahhhhhhhhhhhhhhhhhhhhhh buoy!\n",x_centre,y_centre,count);
		}

		return;
	}


	void imageCallback(const sensor_msgs::ImageConstPtr & msg_ptr)
	{
		
		int i, j;
		cv::Mat img_v_;

		// Convert ROS Imput Image Message to IplImage
		try
		{
			cv_input_ = bridge_.imgMsgToCv (msg_ptr, "bgr8");
 		}
		catch (sensor_msgs::CvBridgeException error)
		{
			ROS_ERROR ("CvBridge Input Error");
 		}
 		
		// Convert IplImage to cv::Mat
		img_in_ = cv::Mat (cv_input_).clone ();
		cv::vector<cv::Mat> planes;
		//Apply histogram equlisation to input image 
		//1: convert input to HSV
		//2: separate V
		//3: equlise V 
		// 4: merge and convert back to BGR
		//This will have the effect of contrast stretching
		
		//initilise and image to store v in 
		img_v_ = cv::Mat::zeros(img_hsv_.rows, img_hsv_.cols, CV_8U);
		//convert to HSV
		cv::cvtColor (img_in_, img_hsv_, CV_BGR2HSV);
		//pull out V store in planes[2]
		cv::split(img_hsv_, planes);
		//equlise V
		cv::equalizeHist(planes[2], img_tmp_);
		planes[2] = img_tmp_;
		
		cv::merge(planes, img_hsv_);

        //convert back to a BGR image
		cv::cvtColor (img_hsv_, img_in_proc_, CV_HSV2BGR);
		// output = input
/*		img_out_ = img_in_.clone ();
		// Convert Input image from BGR to HSV
		cv::cvtColor (img_in_, img_hsv_, CV_BGR2HSV);
		// Zero Matrices
		img_hue_ = cv::Mat::zeros(img_hsv_.rows, img_hsv_.cols, CV_8U);
		img_sat_ = cv::Mat::zeros(img_hsv_.rows, img_hsv_.cols, CV_8U);
		img_bin_ = cv::Mat::zeros(img_hsv_.rows, img_hsv_.cols, CV_8U);
		// HSV Channel 0 -> img_hue_ & HSV Channel 1 -> img_sat_
		int from_to[] = { 0,0, 1,1};
		cv::Mat img_split[] = { img_hue_, img_sat_};
		cv::mixChannels(&img_hsv_, 3,img_split,2,from_to,2);

		for(i = 0; i < img_out_.rows; i++)
		{
			for(j = 0; j < img_out_.cols; j++)
			{
  				// The output pixel is white if the input pixel
				// hue is yellow and saturation is reasonable
												        	//original values | best found
				if(img_hue_.at<uchar>(i,j) > min_hue &&     //4               | 4
				img_hue_.at<uchar>(i,j) < max_hue &&        //30              | 34
				img_sat_.at<uchar>(i,j) > min_sat){         //126             | 50
					img_bin_.at<uchar>(i,j) = 255;
				} else {
					img_bin_.at<uchar>(i,j) = 0;
					// Clear pixel blue output channel
					img_out_.at<uchar>(i,j*3+0) = 0;
					// Clear pixel green output channel
					img_out_.at<uchar>(i,j*3+1) = 0;
					// Clear pixel red output channel
					img_out_.at<uchar>(i,j*3+2) = 0;
				}
			}
		}

		findCentre();
*/	
		// Display Input image
		cv::imshow ("input", img_in_);
		//Display preprocessed input image
		cv::imshow ("preprocessed image", img_in_proc_);
		// Display Binary Image
	//	cv::imshow ("binary image", img_bin_);
		// Display segmented image
	//	cv::imshow ("segmented output", img_out_);

		// Needed to  keep the HighGUI window open
		cv::waitKey (3);

	}


};



int main(int argc, char **argv)
{
 	// Initialize ROS Node
 	ros::init(argc, argv, "ihr_demo1");
	// Start node and create a Node Handle
	ros::NodeHandle nh;
 	// Instaniate Demo Object
	ROS_INFO("Online");
	Demo d(nh);
	
	//Read threshold values from file
	INIReader reader("config.ini");
	//check the file can be opened
    if (reader.ParseError() < 0) 
    {
        std::cout << "Can't load 'config.ini'\n";
        return 1;
    }
    /*std::cout << "Config loaded from 'config.ini': min_hue="
              << reader.GetInteger("hue", "min_hue", -1) << ", max_hue="
              << reader.GetInteger("hue", "max_hue", -1) << ", min_sat="
              << reader.GetInteger("saturation", "min_sat", -1) <<"\n";*/
	//read the values in, defaults to best values found for the test video
	min_hue = reader.GetInteger("hue", "min_hue", 4);
	std::cout << "min_hue = " << min_hue << "\n";
	max_hue = reader.GetInteger("hue", "max_hue", 34);
	std::cout << "min_hue = " << max_hue << "\n";
	min_sat = reader.GetInteger("saturation", "min_sat", 50);
	std::cout << "min_hue = " << min_sat << "\n";
	
  	// Spin ...



	/* Leave One Section Uncommented And The Others Commented Out */
	/* Replace Video/Image Filename If Necessary (Files Should Be Stored In The "dwncam2" Folder) */


	/****************** FOR CAMERA INPUT MODE ******************/
  	//ros::spin();
	/***********************************************************/


	/****************** FOR VIDEO INPUT MODE ******************/
	cv::Mat video_frame;
	IplImage video_frame2;
	sensor_msgs::CvBridge bridge2_;
	cv::VideoCapture cap("Test.avi");
    	if(!cap.isOpened()){
        	return -1;
	}
	while(ros::ok()){
		ros::spinOnce();
		cap >> video_frame;
		video_frame2 = video_frame;
		d.imageCallback(bridge2_.cvToImgMsg(&video_frame2, "passthrough"));
	}
	/**********************************************************/


	/****************** FOR IMAGE INPUT MODE ******************/
	/*cv::Mat video_frame;
	IplImage video_frame2;
	sensor_msgs::CvBridge bridge2_;
	while(ros::ok()){
		ros::spinOnce();
		video_frame = cvLoadImage("a.jpg", CV_LOAD_IMAGE_COLOR);
		video_frame2 = video_frame;
		d.imageCallback(bridge2_.cvToImgMsg(&video_frame2, "passthrough"));
	}*/
	/**********************************************************/



	// ... until done
	return 0;
}

