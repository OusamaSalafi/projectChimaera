

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <math.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Char.h"
#include "std_msgs/Byte.h"
#include "inifile.h"

//#include "../../../Misc/INIReader/src/cpp/INIReader.h"

// Defines
// Input Modes: 0 = GSCAM
//              1 = VIDEO
//              2 = IMAGE
//              3 = IPCAM
#define INPUT_MODE		1
#define VIDEO_FILENAME	"Test.avi"
#define IMAGE_FILENAME	"Test.jpg"
#define PI 3.1459

//Threshold values
int min_hue_;
int max_hue_;
int min_sat_;
//other params
int erode_passes_;
int dilate_passes_;
int min_area_percentage_;
int screenshot_counter_;
int screenshot_trigger_;
int image_counter_;

// ROS/OpenCV HSV Demo
// Based on http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

cv::Mat img_in_;
cv::Mat img_hsv_;
cv::Mat img_hue_;
cv::Mat img_sat_;
cv::Mat img_thresh_;
cv::Mat img_bin_;
cv::Mat img_out_;
cv::Mat img_split[2];
cv::Mat_<uchar> img_count_;

/*Advertises X, Y and est distance*/
ros::Publisher b_X_;
ros::Publisher b_Y_; 
ros::Publisher b_Z_; 

std_msgs::Byte buoy_X_, buoy_Y_;

int centre_x, x;
int centre_y, y;

CvPoint buoy_centre_, point1_, point2_;

char savepath[300];
char *logpath;

class fwdcam{

	protected:
		ros::NodeHandle nh_;
		
		//For receiving new threshold values on the fly
		ros::Subscriber min_hue_sub_;
		ros::Subscriber max_hue_sub_;
		ros::Subscriber min_saturation_sub_;
		ros::Subscriber max_saturation_sub_;
		ros::Subscriber min_brightness_sub_;
		ros::Subscriber max_brightness_sub_;
		
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;
		sensor_msgs::CvBridge bridge_;
		cv::Mat img_in_;
		//cv::Mat img_hls_;
		cv::Mat img_hsv_;
		cv::Mat img_thresh_hls_;
		cv::Mat img_thresh_hls2_;
		cv::Mat img_grey_;
		cv::Mat img_out_;
		IplImage *cv_input_;
		
		

	public:

	fwdcam (ros::NodeHandle & nh):nh_ (nh), it_ (nh_){
		
		//setup publishers to realease pipe details
		b_X_ = nh_.advertise<std_msgs::Char>("buoy_X", 100);
		b_Y_ = nh_.advertise<std_msgs::Char>("buoy_Y", 100);
		b_Z_ = nh_.advertise<std_msgs::Char>("buoy_Z", 100);
		
		// Listen for image messages on a topic and setup callback
		image_sub_ = it_.subscribe ("/gscam/image_raw", 1, &fwdcam::imageCallback, this);
		
		//Listen for new values
		min_hue_sub_ = nh_.subscribe("fwdcam_hue_min", 1000, &fwdcam::vision_hue_minCallback, this);
		max_hue_sub_ = nh_.subscribe("fwdcam_hue_max", 1000, &fwdcam::vision_hue_maxCallback, this);
		min_saturation_sub_ = nh_.subscribe("fwdcam_saturation_min", 1000, &fwdcam::vision_saturation_minCallback, this);
		max_saturation_sub_ = nh_.subscribe("fwdcam_saturation_max", 1000, &fwdcam::vision_saturation_maxCallback, this);
		min_brightness_sub_ = nh_.subscribe("fwdcam_brightness_min", 1000, &fwdcam::vision_brightness_minCallback, this);
		max_brightness_sub_ = nh_.subscribe("fwdcam_brightness_max", 1000, &fwdcam::vision_brightness_maxCallback, this);
		
		// Open HighGUI Window
		cv::namedWindow ("input", 1);
		cv::namedWindow ("thresh", 1);
		cv::namedWindow ("binary", 1);
		cv::namedWindow ("out", 1);
		//cv::moveWindow("input",200 ,200);
		//cv::namedWindow ("thresh hsv", 1);
	}
	
	//Message callback functions
	void vision_hue_minCallback(const std_msgs::Char::ConstPtr& msg)
	{
		uchar data;
		data = msg->data;
		printf("New Hue Value = %u \n", data);
		min_hue_ = (int)data;
		return;
	}
	
	void vision_hue_maxCallback(const std_msgs::Char::ConstPtr& msg)
	{
		uchar data;
		data = msg->data;
		printf("New Brightness Value = %u \n", data);
		max_hue_ = (int)data;
		return;
	}
	
	void vision_saturation_minCallback(const std_msgs::Char::ConstPtr& msg)
	{
		uchar data;
		data = msg->data;
		printf("New Saturation Value = %u \n", data);
		min_sat_ = (int)data;
		return;
	}
	
	void vision_saturation_maxCallback(const std_msgs::Char::ConstPtr& msg)
	{
		uchar data;
		data = msg->data;
		printf("New Brightness Value = %u \n", data);
		return;
	}	
	
	void vision_brightness_minCallback(const std_msgs::Char::ConstPtr& msg)
	{
		uchar data;
		data = msg->data;
		printf("New Brightness Value = %u \n", data);
		return;
	}
	
	void vision_brightness_maxCallback(const std_msgs::Char::ConstPtr& msg)
	{
		uchar data;
		data = msg->data;
		printf("New Brightness Value = %u \n", data);
		return;
	}

	// Dilate Image Function
	void dilate_image(const cv::Mat* img_src_, cv::Mat* img_dst_, int passes)
	{
		int i, j;
		cv::Mat_<uchar> img_dilate_;
		cv::Mat_ <uchar> img_inv_;
		cv::Mat_ <float> distance_map_;
		//if passes == 0 don't try and execute the loop, return gracefully
		if(passes == 0){return;}
		//copy the input image so we don't accidently doitwise_not something silly to it
		img_dilate_ = img_src_->clone ();
		//As the distance map contains the distance for each pixel to the closest black pixel, to dilate
		//we will need to know the distance to the closest white pixel, easiest way is probably
		//to invert the input image
		cv::bitwise_not(img_dilate_, img_inv_); 
		//create a distance map (calculates each pixels distance from the closest black pixel)
		cv::distanceTransform(img_inv_, distance_map_, CV_DIST_L2, 3);//CV_DIST_L2 and 3x3 apparently gives a fast, coarse estimate (should be good enough for what we need)
		//do the dirty work, this is basically a threshold of the distance map with the results 
		//applied to the input image
		for(i = 0; i < (img_dilate_.rows); i++)
		{
				for(j = 0; j < (img_dilate_.cols); j++)
				{
					//don't waste time trying to turn pixels that are already white
					if(((int)(distance_map_(i,j)) <= passes) && (distance_map_(i,j) != 0.0))
					{
						//if the distance from the current pixel to the closest black (background)
						//pixel is less or equal to the desired number of passes turn it black
						img_dilate_(i,j) = 255;
					}
				}
			}
			//send the new image on its way and release the image memory
			*img_dst_ = img_dilate_.clone();
			img_dilate_.release();
			img_inv_.release();
			
			return;
	}



	// Erode Image Function
	void erode_image(const cv::Mat* img_src_, cv::Mat* img_dst_, float passes)
	{
		int i, j;
		cv::Mat_<uchar> img_erode_;
		cv::Mat_ <float> distance_map_;
		//if passes == 0 don't try and execute the loop, return gracefully
		if(passes == 0){return;}
		//copy the input image so we don't accidently do something silly to it
		img_erode_ = img_src_->clone ();
		//create a distance map (calculates each pixels distance from the closest black pixel)
		cv::distanceTransform(img_erode_, distance_map_, CV_DIST_L2, 3);//CV_DIST_L2 and a mask of 3x3 apparently gives a fast coarse estimate
		//do the dirty work, this is basically a threshold of the distance map with the results 
		//applied to the input image
		for(i = 0; i < (img_erode_.rows); i++)
		{
				for(j = 0; j < (img_erode_.cols); j++)
				{
					//don't waste time trying to turn pixels that are already black to black
					if(((int)(distance_map_(i,j)) <= passes) && (distance_map_(i,j) != 0.0))
					{
						//if the distance from the current pixel to the closest black (background)
						//pixel is less or equal to the desired number of passes turn it black
						img_erode_(i,j) = 0;
					}
				}
			}
			//send the new image on its way and release the image memory
			*img_dst_ = img_erode_.clone();
			img_erode_.release();
			
			return;	
	}

	/**************** Find Centre Function****************/
	/*Returns (-1,-1) if the area of the blob is less than
	 *the minimum specified
	 */
	/****************************************************/
	CvPoint find_centre(cv::Mat *img_input, int min_area)
	{
		int row, col, area = 0, mx = 0, my = 0;
		CvPoint output;
		for (row = 0; row < img_input->rows; row++){
			for (col = 0; col < img_input->cols; col++){
				if (img_input->at<uchar>(row, col) == 255){
					area += 1;
					mx += (row + 1);
					my += (col + 1);
				}
			}
		}
		if (area < min_area || area == 0){
			output.x = -1;
			output.y = -1;
		} else{
			output.x = (int)(my / area);
			output.y = (int)(mx / area);
		}
		return output;
	}

	int map(long x, long in_min, long in_max, long out_min, long out_max)
	{
		return (int)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
	}

	// Image Callback Function
	void imageCallback(const sensor_msgs::ImageConstPtr & msg_ptr)
	{

		// Declare Variables
		const int from_to[] = {0, 0, 1, 1};
		int i_, j_;

		// Convert ROS Input Image Message To IplImage
		try{
			cv_input_ = bridge_.imgMsgToCv(msg_ptr, "bgr8");
 		} catch (sensor_msgs::CvBridgeException error){
			ROS_ERROR("CvBridge Input Error");
 		}
 
		// Convert IplImage To cv::Mat
		img_in_ = cv::Mat(cv_input_).clone();

		// Convert The Input Image From BGR To HSV
		cv::cvtColor (img_in_, img_hsv_, CV_BGR2HSV);

		// Initialise The Hue, Sat & Thresh Images
		img_hue_ = cv::Mat::zeros(img_hsv_.rows, img_hsv_.cols, CV_8U);
		img_sat_ = cv::Mat::zeros(img_hsv_.rows, img_hsv_.cols, CV_8U);
		img_thresh_ = cv::Mat::zeros(img_hsv_.rows, img_hsv_.cols, CV_8U);
		
		// Get The Hue & Sat Channels From The HSV Image
		img_split[0] = img_hue_;
		img_split[1] = img_sat_;
		cv::mixChannels(&img_hsv_, 3, img_split, 2, from_to, 2);

		// Threshold The Image
		for (i_ = 0; i_ < img_hue_.rows; i_++){
			for (j_ = 0; j_ < img_hue_.cols; j_++){
				if (img_hue_.at<uchar>(i_, j_) >= min_hue_ &&
				img_hue_.at<uchar>(i_, j_) <= max_hue_ &&
				img_sat_.at<uchar>(i_, j_) >= min_sat_){
					img_thresh_.at<uchar>(i_, j_) = 255;
				}
			}
		}

		/* After thresholding the image can still be very 'messy' 
		 * with large amounts of noise. This can cause the centre to be 
		 * found incorrectly and cause trouble 
		 */

		// Create A Copy Of The Thresholded Image
		img_bin_ = img_thresh_.clone();

		// Close The Image Using Erosion & Dilation
		erode_image(&img_bin_, &img_bin_, erode_passes_);
		dilate_image(&img_bin_, &img_bin_, dilate_passes_);
		
		// Perform A Median Blue On The Image - Not Convinced That This Helps Much
		//cv::medianBlur(img_bin_, img_bin_, 9);
		
		// Create The Segmented Output Image
		img_out_ = img_in_.clone();
		for (i_ = 0; i_ < img_out_.rows; i_++){
			for (j_ = 0; j_ < img_out_.cols; j_++){
				if (img_bin_.at<uchar>(i_, j_) == 0){
					img_out_.at<uchar>(i_, (j_ * 3)) = 0;
					img_out_.at<uchar>(i_, (j_ * 3) + 1) = 0;
					img_out_.at<uchar>(i_, (j_ * 3) + 2) = 0;
				}
			}
		}

		// Find The Centre Of The blob
		buoy_centre_ = find_centre(&img_bin_, (int)((double)((double)(img_bin_.rows) *
		(double)(img_bin_.cols) * (double)(min_area_percentage_) / 100.0)));

		//if the blob is big enough draw a cross on its centre
		if(buoy_centre_.x >= 0)
		{
				point1_.x = buoy_centre_.x - 100;
				point1_.y = buoy_centre_.y;
				point2_.x = buoy_centre_.x + 100;
				point2_.y = buoy_centre_.y;
				line(img_in_, point1_, point2_, cvScalar(255, 0, 255, 0), 1, 8, 0);
				point1_.x = buoy_centre_.x;
				point1_.y = buoy_centre_.y - 100;
				point2_.x = buoy_centre_.x;
				point2_.y = buoy_centre_.y + 100;
				line(img_in_, point1_, point2_, cvScalar(255, 0, 255, 0), 1, 8, 0);
	
				//normalise coordinates from image size to -100 -> +100
			    buoy_centre_.x = map(buoy_centre_.x, 0, img_in_.cols, 0, 200);
				buoy_centre_.y = map(buoy_centre_.y, 0, img_in_.rows, 0, 200);
				
				//save a copy of the image to the log file if triggered
				//save image when pipe first found and every X frames following
				if((screenshot_counter_ == 0)||(screenshot_counter_ == screenshot_trigger_))
				{
					//create the filepath from the image counter and the image name
					sprintf(savepath, "%simg_in_%i.jpg",logpath, image_counter_);
					//save
					cv::imwrite(savepath, img_in_);
					image_counter_++;
					screenshot_counter_ = 0;
				}
				//counter will only be 0 once, so we will always getr an image saved of the
				//first contact with the buoy
				screenshot_counter_++;	
		}
		else
		{
				//if there is no buoy in sight publish 255 
				buoy_centre_.x= 255;
				buoy_centre_.y= 255;
		}

		//publish central coordinates
		// Publish The Centre X, Centre Y, Z Distance & Angle To ROS
		//X 
		buoy_X_.data = (char)buoy_centre_.x; 
		b_X_.publish(buoy_X_);
		//Y
		buoy_Y_.data = (char)buoy_centre_.y; 
		b_Y_.publish(buoy_Y_);
		//printf("X:%d \n Y:%d \n",buoy_X_.data, buoy_Y_.data);
		// Display Input image
		cv::imshow ("input", img_in_);
		cv::imshow ("thresh", img_thresh_);
		cv::imshow ("binary", img_bin_);
		cv::imshow ("out", img_out_);

		// Needed to  keep the HighGUI window open
		cv::waitKey (3);
	}

};


int main(int argc, char **argv){
	// Declare Variables
	cv::Mat video_frame;
	IplImage video_frame2;
	sensor_msgs::CvBridge bridge2_;
	char file_path[300];
	char *file_name_ = "fwdcam_config.ini";

 	// Initialize ROS Node
 	ros::init(argc, argv, "fwdcam");
 
	// Start Node & Create A Node Handle
	ros::NodeHandle nh;

 	// Instantiate Demo Object
	ROS_INFO("Online");
	fwdcam d(nh);
	
	//Read in the environmental variable that stores the location of the config files
	char *configpath = getenv("SUB_CONFIG_PATH");
	if (configpath == NULL)
	{
	std::cout << "Problem getting SUB_CONFIG_PATH variable." << std::endl;
	exit(-1);
	}
	
	//filepath=configpath+filename
	sprintf(file_path, "%s%s", configpath, file_name_);

	/* New Reading of the Config File here...This class doesn't take default values */
	CIniFile ini;
	ini.Load(file_path);
		
	//Read the values in
	min_hue_ = atoi(ini.GetKeyValue("hue" , "min_hue").c_str());
	max_hue_ = atoi(ini.GetKeyValue("hue" , "max_hue").c_str());			
	min_sat_ = atoi(ini.GetKeyValue("saturation" , "min_sat").c_str());
	erode_passes_ = strtod(ini.GetKeyValue("erode" , "erode_passes").c_str(), NULL);		
	dilate_passes_ = atoi(ini.GetKeyValue("dilate" , "dilate_passes").c_str());			
	min_area_percentage_ = atoi(ini.GetKeyValue("pipe" , "min_area_percentage").c_str());
	screenshot_trigger_ = atoi(ini.GetKeyValue("counters" , "screenshot_trigger").c_str());		
			
	//Assume that if both min and max hue = 0, then file reading failed. Setup default vals
	if ((min_hue_ == 0) && (max_hue_ == 0))
	{
		std::cout << "Failed To Load \"~/projectChimaera/Config/dwncam2_config.ini\"\n";
		min_hue_ = 4;
		max_hue_ = 34;
		min_sat_ = 50;
		erode_passes_ = 10;
		dilate_passes_ = 10;
		min_area_percentage_ = 20;
		screenshot_trigger_ = 500;
	}
		
	//Print the values out.
	std::cout << "min_hue = " << min_hue_ << "\n";
	std::cout << "max_hue = " << max_hue_ << "\n";
	std::cout << "min_saturation = " << min_sat_ << "\n";
	std::cout << "erode_passes = " << erode_passes_ << "\n";
	std::cout << "dilation_passes = " << dilate_passes_ << "\n";
	
	logpath = getenv("SUB_LOG_PATH");
	if (logpath == NULL)
	{
		std::cout << "Problem getting SUB_LOG_PATH variable." << std::endl;
		exit(-1);
	}

	sprintf(savepath, "%s%s", logpath, "/fwdcam_images");

  	// Spin & Provide An Input If Necessary
	if (INPUT_MODE == 0){
		ros::spin();
	} else if (INPUT_MODE == 1){
		cv::VideoCapture cap(VIDEO_FILENAME);
		if(!cap.isOpened()){
			return -1;
		}
		while(ros::ok()){
			ros::spinOnce();
			if (cap.grab()){
				cap.retrieve(video_frame);
				video_frame2 = video_frame;
				d.imageCallback(bridge2_.cvToImgMsg(&video_frame2, "passthrough"));
			} else{
				cap.release();
				cap.open(VIDEO_FILENAME);
				if(!cap.isOpened()){
					return -1;
				}
			}
		}
	} else if (INPUT_MODE == 2){
		while(ros::ok()){
			ros::spinOnce();
			video_frame = cvLoadImage(IMAGE_FILENAME, CV_LOAD_IMAGE_COLOR);
			video_frame2 = video_frame;
			d.imageCallback(bridge2_.cvToImgMsg(&video_frame2, "passthrough"));
		}
	} else if (INPUT_MODE == 3){
		cv::VideoCapture vcap;
		//http://192.168.2.30/mjpg/video.mjpg
		const std::string videoStreamAddress = "rtsp://192.168.2.30:554/ipcam.sdp";
		/* it may be an address of an mjpeg stream, 
		e.g. "http://user:pass@cam_address:8081/cgi/mjpg/mjpg.cgi?.mjpg" */
		//open the video stream and make sure it's opened
		if (!vcap.open(videoStreamAddress)){
			std::cout << "Error opening video stream" << std::endl;
			return -1;
		}
		while (ros::ok()){
			ros::spinOnce();
			while (!vcap.read(video_frame)){
				std::cout << "No frame" << std::endl;
				vcap.release();
				vcap.open(videoStreamAddress);
			}
			video_frame2 = video_frame;
			d.imageCallback(bridge2_.cvToImgMsg(&video_frame2, "passthrough"));
		}
	} else{
		ros::spin();

	}

	// Return
	return 0;

}
