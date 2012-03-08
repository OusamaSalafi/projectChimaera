#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <math.h>


class Demo{

	protected:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	sensor_msgs::CvBridge bridge_;
	// Matrices For Image Data
	cv::Mat img_in_;
	cv::Mat img_hls_;
	cv::Mat img_hsv_;
	cv::Mat img_thresh_hsv_;
	cv::Mat img_thresh_hsv2_;
	cv::Mat img_grey_;
	cv::Mat img_out_;
	cv::Mat img_r_;
	cv::Mat img_g_;
	cv::Mat img_b_;
	cv::Mat img_rgb_;
	cv::Mat img_hsv2_;
	// For thresholding in HSV space we need to split the
	// HSV image into H, S and V componants to allow thresholding
	cv::Mat img_h_;
	cv::Mat img_h_thresh_;
	//cv::Mat img_s_;
	//cv::Mat img_v_;
	// IPL image for storing each frame of the input video
	IplImage* cv_input_;
	IplImage* ipl_r_;
	IplImage* ipl_g_;
	IplImage* ipl_b_;
	IplImage* ipl_rgb_;
	IplImage* ipl_hsv_;
	IplImage* ipl_h_;
	IplImage* ipl_h_thresh_;


	public:

	Demo (ros::NodeHandle & nh):nh_ (nh), it_ (nh_){
		// Listen for image messages on a topic and setup callback
		// callback runs everytime gscam updates
		image_sub_ = it_.subscribe ("/gscam/image_raw", 1, &Demo::imageCallback, this);
		// Open HighGUI Window
		cv::namedWindow("input", 1);
		//cv::namedWindow("hls", 1);
		//cv::namedWindow("hsv", 1);
		//cv::namedWindow("thresh hsv up", 1);
		//cv::namedWindow("thresh hsv down", 1);
		//cv::namedWindow("grey", 1);
		//cv::namedWindow("out", 1);
		//cv::namedWindow("r", 1);
		//cv::namedWindow("g", 1);
		//cv::namedWindow("b", 1);
		cv::namedWindow("rgb", 1);
		//cv::namedWindow("hsv2", 1);
		//cv::namedWindow("h", 1);
		cv::namedWindow("h thresh", 1);
	}


	void findCentre(void)
	{
		int x, y, count, x_estimate, y_estimate, x_centre, y_centre;
		int x_draw1, x_draw2, y_draw1, y_draw2;
		CvScalar s;

		x_estimate = y_estimate = count = 0;

		IplImage ipl_img = img_out_;
		IplImage ipl_hsv = img_thresh_hsv2_;
		//use the first order moments to find centre of black region
		for(x=0;x<ipl_img.height;x++)
		{
			for(y=0;y<ipl_img.width;y++)
			{
				s = cvGet2D(&ipl_img,x,y);
				if(s.val[0] == 0)
				{ //if we have found a black pixel
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

			s.val[0] = 255;


			CvPoint pt1 = {x_draw1,y_centre};
			CvPoint pt2 = {x_draw2,y_centre};
			CvPoint pt3 = {x_centre,y_draw1};
			CvPoint pt4 = {x_centre,y_draw2};

			cvLine(&ipl_img, pt1 , pt2, s, 1, 8,0);
			cvLine(&ipl_img, pt3 , pt4, s, 1, 8,0);

			s.val[0] = 0;
			s.val[1] = 255;
			s.val[2] = 0;

			cvLine(&ipl_hsv, pt1 , pt2, s, 1, 8,0);
			cvLine(&ipl_hsv, pt3 , pt4, s, 1, 8,0);

			img_out_ = cv::Mat (&ipl_img).clone ();
			img_thresh_hsv2_ = cv::Mat (&ipl_hsv).clone ();

			printf("X: %d Y: %d Count: %d Yeahhhhhhhhhhhhhhhhhhhhhh buoy!\n",x_centre,y_centre,count);
		}

		return;
	}


	void imageCallback (const sensor_msgs::ImageConstPtr & msg_ptr)
	{

		// Convert ROS Input Image Message To IplImage
		try
		{
			cv_input_ = bridge_.imgMsgToCv(msg_ptr, "bgr8");
		}
		catch(sensor_msgs::CvBridgeException error)
		{
			ROS_ERROR("CvBridge Input Error");
		}


		cv_input_ = cvLoadImage("/home/sam/projectChimaera/Test_Ground/camtesting/pipe.jpg", CV_LOAD_IMAGE_COLOR);

		// Convert IplImage To cv::Mat(OpenCV Matrix)
		img_in_ = cv::Mat(cv_input_).clone();
		
		// Convert Input Image From BGR (Blue Green Red) To HLS (Hue Lightness Saturation)
		cv::cvtColor(img_in_, img_hls_, CV_BGR2HLS);

		// Convert Input Image From BGR (Blue Green Red) To HSV (Hue Saturation Value)
		cv::cvtColor(img_in_, img_hsv_, CV_BGR2HSV);

		//cv::threshold(img_hls_, img_thresh_hls_, 120, 255, CV_THRESH_BINARY);
		//cv::threshold(img_hsv_, img_thresh_hsv_, 120, 255, CV_THRESH_BINARY);
		img_thresh_hsv_ = img_hsv_ > 80;
		img_thresh_hsv2_ = img_thresh_hsv_ < 70;

		// Convert From HSV To Greyscale
		cv::cvtColor(img_hsv_, img_grey_, CV_BGR2GRAY);

		img_out_ = img_hsv_; 
		//img_out_ = ~img_out_;

		//findCentre();
		
		
		// ALGORITHM:
		// -SPLIT TO RGB
		// -HISTOGRAM EQUALISATION ON EACH CHANNEL
		// -RECOMBINE INTO RGB IMAGE
		// -CONVERT TO HSV
		// -REMOVE S & V
		// -CONVERT TO GREYSCALE
		// -THRESHOLD (70?)
		
		ipl_r_ = cvCreateImage(cvGetSize(cv_input_), 8, 1);
		ipl_g_ = cvCreateImage(cvGetSize(cv_input_), 8, 1);
		ipl_b_ = cvCreateImage(cvGetSize(cv_input_), 8, 1);
		cvSplit(cv_input_, ipl_r_, ipl_g_, ipl_b_, NULL);
		cvEqualizeHist(ipl_r_, ipl_r_);
		cvEqualizeHist(ipl_g_, ipl_g_);
		cvEqualizeHist(ipl_b_, ipl_b_);
		ipl_rgb_ = cvCreateImage(cvGetSize(cv_input_), 8, 3);
		cvMerge(ipl_r_, ipl_g_, ipl_b_, NULL, ipl_rgb_);
		ipl_hsv_ = cvCreateImage(cvGetSize(cv_input_), 8, 3);
		cvCvtColor(ipl_rgb_, ipl_hsv_, CV_RGB2HSV);
		ipl_h_ = cvCreateImage(cvGetSize(cv_input_), 8, 1);
		cvSplit(ipl_hsv_, ipl_h_, NULL, NULL, NULL);
		ipl_h_thresh_ = cvCreateImage(cvGetSize(cv_input_), 8, 1);
		cvThreshold(ipl_h_, ipl_h_thresh_, 70, 255, CV_THRESH_TOZERO);
		cvThreshold(ipl_h_thresh_, ipl_h_thresh_, 105, 255, CV_THRESH_TOZERO_INV);
		cvThreshold(ipl_h_thresh_, ipl_h_thresh_, 0, 255, CV_THRESH_BINARY);
		img_r_ = cv::Mat(ipl_r_).clone();
		img_g_ = cv::Mat(ipl_g_).clone();
		img_b_ = cv::Mat(ipl_b_).clone();
		img_rgb_ = cv::Mat(ipl_rgb_).clone();
		img_hsv2_ = cv::Mat(ipl_hsv_).clone();
		img_h_ = cv::Mat(ipl_h_).clone();
		img_h_thresh_ = cv::Mat(ipl_h_thresh_).clone();
		

		// Display Input Image
		cv::imshow("input", img_in_);

		// Display HLS & HSV Images
		//cv::imshow("hls", img_hls_);
		//cv::imshow("hsv", img_hsv_);

		//cv::imshow("thresh hsv up", img_thresh_hsv_);
		//cv::imshow("thresh hsv down", img_thresh_hsv2_);

		// Display Greyscale Image
		//cv::imshow("grey", img_grey_);

		// Display Output Image
		//cv::imshow("out", img_out_);
		
		
		//cv::imshow("r", img_r_);
		//cv::imshow("g", img_g_);
		//cv::imshow("b", img_b_);
		cv::imshow("rgb", img_rgb_);
		//cv::imshow("hsv2", img_hsv2_);
		//cv::imshow("h", img_h_);
		cv::imshow("h thresh", img_h_thresh_);

		// Needed To Keep The HighGUI Window Open
		cv::waitKey(3);

	}

};


int main(int argc, char **argv){

	// Initialize ROS Node
	ros::init (argc, argv, "camtest");
	// Start Node & Create A Node Handle
	ros::NodeHandle nh;
	// Instaniate Demo Object
	ROS_INFO("Online");
	Demo d(nh);
	// Spin ...
	ros::spin();
	// ... until done
	return 0;

}
