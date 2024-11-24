#pragma once

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"

struct ImageProcessSettings {
	bool blur = false;
	bool erode = false;
	bool dilate = false;
	bool sobel = false;
	int sobelThresh = 100;
	double threshold = 100.0;
	double thresholdMax = 255.0;
	int erosion = 3;
	int dilation = 3;
	cv::Size kernelSize{ 3, 3 };
};