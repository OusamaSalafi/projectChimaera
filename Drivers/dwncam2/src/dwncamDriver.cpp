// Downcam Driver
// Authors: P-LAN, Wurbledood, alexsleat, everto151

//Includes
#include <iostream>
#include <math.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ros/ros.h>
#include "dwncamDriver.h"
#include "image_transport/image_transport.h"
#include "ini.h"
#include "INIReader.h"
#include "cv_bridge/CvBridge.h"
#include "sensor_msgs/Image.h"


// Downcam Class
class Downcam
{

protected:

	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	sensor_msgs::CvBridge bridge_;
	IplImage *cv_input_;
	cv::Mat img_in_;
	cv::Mat img_hsv_;
	cv::Mat img_hue_;
	cv::Mat img_sat_;
	cv::Mat img_thresh_;
	cv::Mat img_bin_;
	cv::Mat img_out_;
	cv::Mat img_split[2];
	int min_hue_, max_hue_, min_sat_, dilate_passes_, min_area_percentage_, edges_allocated_,
	edges1_index_, edges1_length_, edges1_marker_, edges2_index_, edges2_length_, edges2_marker_,
	moving_average_size_, moving_length_, moving_index_, i_, j_, n_, n2_;
	int *edges1_, *edges2_, *blanket1_, *blanket2_;
	float erode_passes_;
	double sx_, sy_, sxx_, sxy_, alpha_, beta_, sx2_,
	sy2_, sxx2_, sxy2_, alpha2_, beta2_, alpha3_, beta3_, a1_, a2_, a3_;
	double *moving_alpha1_, *moving_beta1_, *moving_alpha2_, *moving_beta2_;
	CvPoint pipe_centre_, point1_, point2_;
	char angle_text_[50];


public:

	Downcam(ros::NodeHandle & nh):nh_ (nh), it_ (nh_)
	{

		// Listen For Image Messages On A Topic And Setup Callback
		ROS_INFO("Getting Cam");
		image_sub_ = it_.subscribe("/gscam/image_raw", 1, &Downcam::imageCallback, this);
		ROS_INFO("Got Cam");

		// Open HighGUI Windows
		cv::namedWindow ("input", 1);
		cv::namedWindow ("thresholded image", 1);
		cv::namedWindow ("binary image", 1);
		cv::namedWindow ("segmented output", 1);
		ROS_INFO("Opened Windows");

		// Read The Downcam Config File
		INIReader reader("../../Config/dwncam2_config.ini");
		// Check The File Can Be Opened
		if (reader.ParseError() < 0){
			std::cout << "Failed To Load \"../../Config/dwncam2_config.ini\"\n";
			min_hue_ = 4;
			max_hue_ = 34;
			min_sat_ = 50;
			erode_passes_ = 10;
			dilate_passes_ = 10;
			min_area_percentage_ = 20;
			moving_average_size_ = 10;
		} else{
			// Read The Values In With Defaults Set To The Best Values Found For The Test Video
			min_hue_ = reader.GetInteger("hue", "min_hue", 4);
			std::cout << "min_hue = " << min_hue_ << "\n";
			max_hue_ = reader.GetInteger("hue", "max_hue", 34);
			std::cout << "min_hue = " << max_hue_ << "\n";
			min_sat_ = reader.GetInteger("saturation", "min_sat", 50);
			std::cout << "min_hue = " << min_sat_ << "\n";
			erode_passes_ = (float)reader.GetInteger("erode", "erode_passes", 10);
			std::cout << "erode_passes = " << erode_passes_ << "\n";
			dilate_passes_ = reader.GetInteger("dilate", "dilate_passes", 10);
			std::cout << "dilation_passes = " << dilate_passes_ << "\n";
			min_area_percentage_ = reader.GetInteger("pipe", "min_area_percentage", 20);
			std::cout << "min_area_percentage = " << min_area_percentage_ << "\n";
			moving_average_size_ = reader.GetInteger("angle", "moving_average_size", 10);
			std::cout << "moving_average_size = " << moving_average_size_ << "\n";
		}
		
		// Make Sure The Moving Average Size Is Greater Than Zero
		if (moving_average_size_ < 1){
			moving_average_size_ = 1;
		}
		
		// Set The Initial Value For Dynamically Allocating The Edge Arrays
		edges_allocated_ = 0;

		// Allocate Memory For The Moving Average Arrays
		moving_alpha1_ = (double*)(malloc(moving_average_size_ * sizeof(double)));
		moving_beta1_ = (double*)(malloc(moving_average_size_ * sizeof(double)));
		moving_alpha2_ = (double*)(malloc(moving_average_size_ * sizeof(double)));
		moving_beta2_ = (double*)(malloc(moving_average_size_ * sizeof(double)));
		
		// Set The Initial Used Length & Index Of The Moving Average Arrays
		moving_length_ = 0;
		moving_index_ = 0;

	}



	// Find Centre Function
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



	// Blanket Function
	void blanket(int *input, int *output, int index, int length, int side)
	{
		int i, j, k, bestbefore, bestafter, pixel;
		double currentslope;
		for (i = index; i < index + length; i++){
			output[i] = -1;
			if (input[i] != -1){
				bestbefore = i;
				bestafter = i;
				k = 0;
				for (j = 0; j < 20; j++){
					pixel = i;
					currentslope = 0.0;
					if (bestafter > bestbefore){
						currentslope = (double)((double)(input[bestafter] - input[bestbefore]) / (double)(bestafter - bestbefore));
					}
					if (k == 0){
						pixel = i - (int)(j / 2) - 1;
						if (pixel >= index){
							if (input[pixel] != -1){
								if (side == 0){
									if ((double)((double)(input[bestafter] - input[pixel]) / (double)(bestafter - pixel)) > currentslope){
										bestbefore = pixel;
									}
								} else{
									if ((double)((double)(input[bestafter] - input[pixel]) / (double)(bestafter - pixel)) < currentslope){
										bestbefore = pixel;
									}
								}
							}
						}
					} else{
						pixel = i + (int)((j + 1) / 2);
						if (pixel < index + length){
							if (input[pixel] != -1){
								if (side == 0){
									if ((double)((double)(input[pixel] - input[bestbefore]) / (double)(pixel - bestbefore)) < currentslope){
										bestafter = pixel;
									}
								} else{
									if ((double)((double)(input[pixel] - input[bestbefore]) / (double)(pixel - bestbefore)) > currentslope){
										bestafter = pixel;
									}
								}
							}
						}
					}
					k = 1 - k;
				}
				if (input[bestbefore] != input[bestafter]){
					output[i] = input[bestbefore] + (int)((double)((double)(i - bestbefore) * (double)(input[bestafter] - input[bestbefore]) / (double)(bestafter - bestbefore)));
				} else{
					output[i] = input[bestbefore];
				}
			}
		}
		for (i = index; i < index + length; i++){
			if (output[i] == -1){
				bestbefore = -1;
				bestafter = -1;
				for (j = i - 1; j >= index; j--){
					if (output[j] != -1){
						bestbefore = j;
						break;
					}
				}
				for (j = i + 1; j < index + length; j++){
					if (output[j] != -1){
						bestafter = j;
						break;
					}
				}
				if (bestbefore == -1 && bestafter == -1){
					output[i] = 0;
				} else if (bestbefore == -1){
					output[i] = output[bestafter];
				} else if (bestafter == -1){
					output[i] = output[bestbefore];
				} else{
					output[i] = output[bestbefore] + (int)((double)((double)(i - bestbefore) * (double)(output[bestafter] - output[bestbefore]) / (double)(bestafter - bestbefore)));
				}
			}
		}
		for (i = index; i < index + length; i++){
			pixel = 0;
			bestbefore = 0;
			for (j = i - 5; j <= i + 5; j++){
				if (j >= index && j < index + length){
					pixel += output[j];
					bestbefore += 1;
				}
			}
			output[i] = (int)(pixel / bestbefore);
		}
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



	// Image Callback Function
	void imageCallback(const sensor_msgs::ImageConstPtr & msg_ptr)
	{

		// Declare Variables
		const int from_to[] = {0, 0, 1, 1};

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
				if (img_hue_.at<uchar>(i_, j_) > min_hue_ &&
				img_hue_.at<uchar>(i_, j_) < max_hue_ &&
				img_sat_.at<uchar>(i_, j_) > min_sat_){
					img_thresh_.at<uchar>(i_, j_) = 255;
				}
			}
		}

		/* After thresholding the image can still be very 'messy' 
		 * with large amounts of noise. This can cause the centre to be 
		 * found incorrectly and cause trouble when we try to find
		 * the pipes angle and estimated size/distance, so it needs to 
		 * be cleaned up.
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

		// Find The Centre Of The Pipe
		pipe_centre_ = find_centre(&img_bin_, (int)((double)((double)(img_bin_.rows) *
		(double)(img_bin_.cols) * (double)(min_area_percentage_) / 100.0)));

		// If We Have Found A Pipe
		if (pipe_centre_.x >= 0){

			// Dynamically Allocate Memory For The Edge Arrays if Required
			if (edges_allocated_ != img_bin_.rows){
				edges1_ = (int*)(malloc(img_bin_.rows * sizeof(int)));
				edges2_ = (int*)(malloc(img_bin_.rows * sizeof(int)));
				blanket1_ = (int*)(malloc(img_bin_.rows * sizeof(int)));
				blanket2_ = (int*)(malloc(img_bin_.rows * sizeof(int)));
				edges_allocated_ = img_bin_.rows;
			}

			// Get The Edges
			edges1_index_ = -1;
			edges1_length_ = -1;
			edges1_marker_ = -1;
			edges2_index_ = -1;
			edges2_length_ = -1;
			edges2_marker_ = -1;
			for (i_ = 0; i_ < img_bin_.rows; i_++){
				edges1_[i_] = -1;
				for (j_ = 0; j_ < img_bin_.cols; j_++){
					if (img_bin_.at<uchar>(i_, j_) == 255){
						edges1_[i_] = j_;
						if (edges1_marker_ == -1){
							edges1_marker_ = i_;
						}
						break;
					}
				}
				if (edges1_[i_] == -1 || i_ == img_bin_.rows - 1){
					if (edges1_marker_ != -1){
						if (edges1_[i_] == -1){
							if (i_ - edges1_marker_ > edges1_length_){
								edges1_index_ = edges1_marker_;
								edges1_length_ = i_ - edges1_marker_;
							}
						} else{
							if (i_ + 1 - edges1_marker_ > edges1_length_){
								edges1_index_ = edges1_marker_;
								edges1_length_ = i_ + 1 - edges1_marker_;
							}
						}
						edges1_marker_ = -1;
					}
				}
				edges2_[i_] = -1;
				for (j_ = img_bin_.cols - 1; j_ >= 0; j_--){
					if (img_bin_.at<uchar>(i_, j_) == 255){
						edges2_[i_] = j_;
						if (edges2_marker_ == -1){
							edges2_marker_ = i_;
						}
						break;
					}
				}
				if (edges2_[i_] == -1 || i_ == img_bin_.rows - 1){
					if (edges2_marker_ != -1){
						if (edges2_[i_] == -1){
							if (i_ - edges2_marker_ > edges2_length_){
								edges2_index_ = edges2_marker_;
								edges2_length_ = i_ - edges2_marker_;
							}
						} else{
							if (i_ + 1 - edges2_marker_ > edges2_length_){
								edges2_index_ = edges2_marker_;
								edges2_length_ = i_ + 1 - edges2_marker_;
							}
						}
						edges2_marker_ = -1;
					}
				}
			}

			// If We Have Edges
			if (edges1_index_ >= 0 && edges2_index_ >= 0){

				// Blanket The Left & Right Edges
				blanket(edges1_, blanket1_, edges1_index_, edges1_length_, 0);
				blanket(edges2_, blanket2_, edges2_index_, edges2_length_, 1);
				
				// TEMPORARY SECTION ########### FOR DEBUGGING
				img_thresh_ = cv::Mat::zeros(img_thresh_.rows, img_thresh_.cols, CV_8UC3);
				for (i_ = 0; i_ < edges1_length_; i_++){
					img_thresh_.at<uchar>(edges1_index_ + i_, 3 * edges1_[edges1_index_ + i_]) = 255;
				}
				for (i_ = 0; i_ < edges2_length_; i_++){
					img_thresh_.at<uchar>(edges2_index_ + i_, (3 * edges2_[edges2_index_ + i_]) + 1) = 255;
				}
				for (i_ = 0; i_ < edges1_length_; i_++){
					img_thresh_.at<uchar>(edges1_index_ + i_, (3 * blanket1_[edges1_index_ + i_]) + 2) = 255;
				}
				for (i_ = 0; i_ < edges2_length_; i_++){
					img_thresh_.at<uchar>(edges2_index_ + i_, (3 * blanket2_[edges2_index_ + i_]) + 1) = 255;
					img_thresh_.at<uchar>(edges2_index_ + i_, (3 * blanket2_[edges2_index_ + i_]) + 2) = 255;
				}

				// Calculate The Variables Required For Simple Linear Regression On Each Side Of The Pipe
				n_ = edges1_length_;
				sx_ = 0.0;
				sy_ = 0.0;
				sxx_ = 0.0;
				sxy_ = 0.0;
				for (i_ = 0; i_ < edges1_length_; i_++){
					/*sx_ += (double)(edges1_[edges1_index_ + i_]);
					sy_ += (double)(edges1_index_ + i_);
					sxx_ += (double)(pow(edges1_[edges1_index_ + i_], 2.0));
					sxy_ += (double)((double)(edges1_index_ + i_) * (double)(edges1_[edges1_index_ + i_]));*/
					sx_ += (double)(blanket1_[edges1_index_ + i_]);
					sy_ += (double)(edges1_index_ + i_);
					sxx_ += (double)(pow(blanket1_[edges1_index_ + i_], 2.0));
					sxy_ += (double)((double)(edges1_index_ + i_) * (double)(blanket1_[edges1_index_ + i_]));
				}
				n2_ = edges2_length_;
				sx2_ = 0.0;
				sy2_ = 0.0;
				sxx2_ = 0.0;
				sxy2_ = 0.0;
				for (i_ = 0; i_ < edges2_length_; i_++){
					/*sx2_ += (double)(edges2_[edges2_index_ + i_]);
					sy2_ += (double)(edges2_index_ + i_);
					sxx2_ += (double)(pow(edges2_[edges2_index_ + i_], 2.0));
					sxy2_ += (double)((double)(edges2_index_ + i_) * (double)(edges2_[edges2_index_ + i_]));*/
					sx2_ += (double)(blanket2_[edges2_index_ + i_]);
					sy2_ += (double)(edges2_index_ + i_);
					sxx2_ += (double)(pow(blanket2_[edges2_index_ + i_], 2.0));
					sxy2_ += (double)((double)(edges2_index_ + i_) * (double)(blanket2_[edges2_index_ + i_]));
				}

				// Get The Left Line
				beta_ = (double)(((double)(n_ * sxy_) - (double)(sx_ * sy_)) / ((double)(n_ * sxx_) - (double)(pow(sx_, 2))));
				alpha_ = (double)((double)(sy_ / (double)(n_)) - (double)(beta_ * sx_ / (double)(n_)));
				moving_alpha1_[moving_index_] = alpha_;
				moving_beta1_[moving_index_] = beta_;

				// Get The Right Line
				beta2_ = (double)(((double)(n2_ * sxy2_) - (double)(sx2_ * sy2_)) / ((double)(n2_ * sxx2_) - (double)(pow(sx2_, 2))));
				alpha2_ = (double)((double)(sy2_ / (double)(n2_)) - (double)(beta2_ * sx2_ / (double)(n2_)));
				moving_alpha2_[moving_index_] = alpha2_;
				moving_beta2_[moving_index_] = beta2_;

				// Update The Moving Average Index & Length
				moving_index_ += 1;
				if (moving_index_ > moving_length_){
					moving_length_ = moving_index_;
				}
				if (moving_index_ >= moving_average_size_){
					moving_index_ = 0;
				}

				// Get The Moving Average Lines
				alpha_ = 0.0;
				beta_ = 0.0;
				alpha2_ = 0.0;
				beta2_ = 0.0;
				for (i_ = 0; i_ < moving_length_; i_++){
					alpha_ += moving_alpha1_[i_];
					beta_ += moving_beta1_[i_];
					alpha2_ += moving_alpha2_[i_];
					beta2_ += moving_beta2_[i_];
				}
				alpha_ /= moving_length_;
				beta_ /= moving_length_;
				alpha2_ /= moving_length_;
				beta2_ /= moving_length_;

				// Plot The Left Line
				point1_.x = (int)(-alpha_ / beta_);
				point1_.y = 0;
				point2_.x = (int)((double)(img_bin_.rows - 1.0 - alpha_) / beta_);
				point2_.y = img_bin_.rows - 1;
				line(img_bin_, point1_, point2_, cvScalar(128, 128, 128, 0), 1, 8, 0);
				line(img_in_, point1_, point2_, cvScalar(0, 0, 255, 0), 1, 8, 0);

				// Plot The Right Line
				point1_.x = (int)(-alpha2_ / beta2_);
				point1_.y = 0;
				point2_.x = (int)((double)(img_bin_.rows - 1.0 - alpha2_) / beta2_);
				point2_.y = img_bin_.rows - 1;
				line(img_bin_, point1_, point2_, cvScalar(128, 128, 128, 0), 1, 8, 0);
				line(img_in_, point1_, point2_, cvScalar(0, 0, 255, 0), 1, 8, 0);

				// Get The Central Line
				a1_ = (double)(atan(fabs(beta_)));
				a2_ = (double)(atan(fabs(beta2_)));
				if (a1_ < 0.0){
					a1_ += CV_PI;
				}
				if (a2_ < 0.0){
					a2_ += CV_PI;
				}
				if (beta_ >= 0.0 && beta2_ >= 0.0){
					a3_ = (double)((double)(a1_ + a2_) / 2.0);
					beta3_ = tan(a3_);
				} else if (beta_ < 0.0 && beta2_ < 0.0){
					a3_ = (double)((double)(a1_ + a2_) / 2.0);
					beta3_ = -tan(a3_);
				} else{
					a3_ = a1_ + (double)((double)(CV_PI - a1_ - a2_) / 2.0);
					if (a3_ < (double)(CV_PI / 2.0)){
						beta3_ = -tan(a3_);
					} else{
						a3_ = CV_PI - a3_;
						beta3_ = tan(a3_);
					}
				}
				if (beta_ == beta2_){
					point1_.x = 0;
					point1_.y = (double)((double)(alpha_ + alpha2_) / 2.0);
				} else{
					point1_.x = (double)((double)(alpha2_ - alpha_) / (double)(beta_ - beta2_));
					point1_.y = (double)(alpha_ + (double)(beta_ * point1_.x));
				}
				alpha3_ = (double)(point1_.y) - (double)(beta3_ * point1_.x);
				point1_.x = (int)(-alpha3_ / beta3_);
				point1_.y = 0;
				point2_.x = (int)((double)(img_bin_.rows - 1.0 - alpha3_) / beta3_);
				point2_.y = img_bin_.rows - 1;
				line(img_in_, point1_, point2_, cvScalar(255, 0, 0, 0), 1, 8, 0);

				// Get The Angle
				a1_ = (double)((double)(atan(fabs(beta3_))) * 180.0 / (double)(CV_PI));
				if (a1_ < 0.0){
					a1_ += 180.0;
				}
				if (beta3_ >= 0.0){
					a1_ -= 90.0;
				} else{
					a1_ = 90.0 - a1_;
				}
				sprintf(angle_text_, "Pipe Angle Detected: %.2f", a1_);
				putText(img_in_, angle_text_, cvPoint(50, 75), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0, 0, 0), 1, CV_AA);

				// Display The Pipe Centre Point On The Input Image
				point1_.x = pipe_centre_.x - 100;
				point1_.y = pipe_centre_.y;
				point2_.x = pipe_centre_.x + 100;
				point2_.y = pipe_centre_.y;
				line(img_in_, point1_, point2_, cvScalar(255, 0, 255, 0), 1, 8, 0);
				point1_.x = pipe_centre_.x;
				point1_.y = pipe_centre_.y - 100;
				point2_.x = pipe_centre_.x;
				point2_.y = pipe_centre_.y + 100;
				line(img_in_, point1_, point2_, cvScalar(255, 0, 255, 0), 1, 8, 0);

				// Publish The Centre X, Centre Y, Angle & Estimated Distance To ROS
				
			}

		} else{

			// Reset The Used Length & Index Of The Moving Average Arrays
			moving_length_ = 0;
			moving_index_ = 0;

		}

		// Display The Input image
		cv::imshow ("input", img_in_);
		// Display The Thresholded Image
		cv::imshow ("thresholded image", img_thresh_);
		// Display The Binary Image
		cv::imshow ("binary image", img_bin_);
		// Display The Segmented Image
		cv::imshow ("segmented output", img_out_);
		
		// Allow The HighGUI Windows To Stay Open
		cv::waitKey(3);

	}
	
};



int main(int argc, char **argv)
{
 	// Initialize ROS Node
 	ros::init(argc, argv, "ihr_demo1");
	// Start Node & Create A Node Handle
	ros::NodeHandle nh;
 	// Instaniate Demo Object
	ROS_INFO("Online");
	Downcam d(nh);
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



	// ... Until Done
	return 0;
}
