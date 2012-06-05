#ifndef __DWNCAMDRIVER_H__
#define __DWNCAMDRIVER_H__

//Function prototypes
void dilate_image(const cv::Mat* src, cv::Mat* dst, int passes);
void erode_image(const cv::Mat* src, cv::Mat* dst, int passes);

#endif // __DWNCAMDRIVER_H__
