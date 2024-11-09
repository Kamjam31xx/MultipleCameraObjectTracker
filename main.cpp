//#include "stdafx.h"

/*

parimeter walk flood fill
	find wall, walk wall, record xMax and yMax in array for dynamic programming algorithm. use touples for polarity/direction of travel.

minkowski shape sum center of mass and intersection etc thing



to-do.....
generate vector of all detected blobs temporally
generate temporal blob trees
score trees

to-do....
implement splines
use splines for continuation prediction (quadratic continuation)


*/

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <array>
#include <iostream>
#include <string>
#include <vector>
#include <tuple>
#include <limits>
#include <fstream>
#include <math.h>
#include <cstdlib>
#include <conio.h>
#include <utility>
#include <bitset>

#include "ImgProcTypes.h"
#include "DeltaTime.h"
#include "FrameRate.h"
#include "ImgProc.h"
#include "Keyboard.h"

DeltaTime Timer;
DeltaTime VelocityTimer;
FrameRate FPS = FrameRate(10);

cv::VideoCapture cap(0);

CameraSettings camera = CameraSettings();
ProcessSettings process = ProcessSettings();


int main()
{
	/*
		add blob to blob manhattan distance -> to -> nearest neighbor mass & nearest neighbor center of mass -> check if that matches temporal past blobs
		- aka a blob gets split into 2 blobs, detect if adding 2 blobs brings back an abstraction resembling that blobby-boi
	*/

	if (cap.isOpened() != true)
	{
		std::cout << "cap is not opened" << std::endl;
		return -1;
	}

	cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0);
	cap.set(cv::CAP_PROP_EXPOSURE, camera.exposure);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, camera.height);
	cap.set(cv::CAP_PROP_FRAME_WIDTH, camera.width);

	int front = 0;
	std::vector<cv::Mat> frame = { cv::Mat(), cv::Mat() };
	std::vector<cv::Mat> detect = { cv::Mat(), cv::Mat() };
	std::vector<cv::Mat> blurredFrame = { cv::Mat(), cv::Mat() };
	std::vector<cv::Mat> lightMaskFrame = { cv::Mat(), cv::Mat() };
	cv::Mat erosionElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * process.erosion + 1, 2 * process.erosion + 1));
	cv::Mat dilationElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * process.dilation + 1, 2 * process.dilation + 1));

	std::array<cv::Mat, 2> fillSource = { cv::Mat(), cv::Mat() };
	std::array<cv::Mat, 2> fillResult = { cv::Mat(), cv::Mat() };
	std::array<cv::Mat, 2> inFill = { cv::Mat(), cv::Mat() };
	std::array<cv::Mat, 2> upScaled = { cv::Mat(), cv::Mat() };
	std::array<cv::Mat, 2> leftCamera = { cv::Mat(), cv::Mat() };
	std::array<cv::Mat, 2> rightCamera = { cv::Mat(), cv::Mat() };
	std::array<cv::Mat, 2> leftResult = { cv::Mat(), cv::Mat() };
	std::array<cv::Mat, 2> rightResult = { cv::Mat(), cv::Mat() };

	Timer.tick();
	VelocityTimer.tick();

	BlobFrame lastLeft = BlobFrame{};
	BlobFrame lastRight = BlobFrame{};

	cv::waitKey(100);

	for (bool play = true; play;)
	{

		HandleKeyboard(process, camera, Timer, cap);

		if (process.wait)
		{
			if (process.advanceFrame)
			{
				cap >> frame[front];
				process.frameAdvanced = true;
				process.advanceFrame = false;
			}
		}
		else
		{
			cap >> frame[front];
		}


		if (frame.empty())
		{
			break;
		}
		else
		{
			// IMPLEMENT FUNCTION POINTERS TO ELIMINATE BRANCHING
			cv::cvtColor(frame[front], detect[front], cv::COLOR_BGR2GRAY);
			if (process.blur) {
				cv::GaussianBlur(frame[front], frame[front], process.kernelSize, 0);
			}
			cv::Mat derivativeMap;
			if (process.sobel)
			{
				cv::Mat dx;
				cv::Mat dy;

				cv::Sobel(detect[front], dx, CV_8UC3, 1, 0);
				cv::Sobel(detect[front], dy, CV_8UC3, 0, 1);

				// Convert the derivative maps to absolute values
				cv::Mat dx_abs;
				cv::Mat dy_abs;
				cv::convertScaleAbs(dx, dx_abs);
				cv::convertScaleAbs(dy, dy_abs);

				cv::add(dx_abs, dy_abs, derivativeMap);
			}

			cv::threshold(detect[front], detect[front], process.threshold, process.thresholdMax, cv::THRESH_BINARY);

			if (process.sobel)
			{
				cv::subtract(255, derivativeMap, derivativeMap);
				cv::threshold(derivativeMap, derivativeMap, process.sobelThresh, process.thresholdMax, cv::THRESH_BINARY);
				derivativeMap.copyTo(detect[front]);
			}
			if (process.erode) {
				cv::erode(detect[front], detect[front], erosionElement);
			}
			if (process.dilate) {
				cv::dilate(detect[front], detect[front], dilationElement);
			}
			cv::cvtColor(detect[front], fillResult[front], cv::COLOR_GRAY2BGR);

			int width = fillResult[front].cols / 2;
			int height = fillResult[front].rows;
			leftCamera[front] = cv::Mat(fillResult[front], cv::Rect(0, 0, width, height));
			rightCamera[front] = cv::Mat(fillResult[front], cv::Rect(width, 0, width, height));

			leftCamera[front].copyTo(leftResult[front]);
			rightCamera[front].copyTo(rightResult[front]);

			BlobFrame leftNow = BlobFrame{
				std::vector<Blob>(),
				std::vector<FillNodeIndex>(),
				std::vector<std::vector<FillNode>>(leftCamera[front].cols, std::vector<FillNode>())
			};

			BlobFrame rightNow = BlobFrame{
				std::vector<Blob>(),
				std::vector<FillNodeIndex>(),
				std::vector<std::vector<FillNode>>(rightCamera[front].cols, std::vector<FillNode>())
			};

			GetBlobs(&leftCamera[front], &leftResult[front], 255, process.areaMinSize, leftNow);
			GetBlobs(&rightCamera[front], &rightResult[front], 255, process.areaMinSize, rightNow);

			cv::waitKey(2);

			VelocityTimer.tick();
			float deltaTime = VelocityTimer.getDeltaMilliSeconds();
			if (process.color)
			{
				ColorTrack_Test(&leftResult[front], lastLeft, leftNow, deltaTime, process);
				ColorTrack_Test(&rightResult[front], lastRight, rightNow, deltaTime, process);
			}

			cv::subtract(255, leftResult[front], leftResult[front]);
			cv::subtract(255, rightResult[front], rightResult[front]);

			cv::waitKey(2);

			cv::imshow("_____< left >_____", leftResult[front]);
			cv::imshow("_____< right >_____", rightResult[front]);

			lastLeft = leftNow;
			lastRight = rightNow;
		}



		char c = (char)cv::waitKey(process.cvwait + (process.lag * process.cvwait * 128));




		Timer.tick();
		FPS.pushFrameTime(Timer.getDeltaMilliSeconds());
		process.logTime += float(Timer.getDeltaMilliSeconds() / 1000.0);


		if (process.logTime >= process.logRate)
		{
			std::cout << "t=" + std::to_string(Timer.getDeltaMilliSeconds()) + " r=" + std::to_string(FPS.getRate());
			if (process.frameAdvanced)
			{
				std::cout << "    -    advanced frame";
				process.frameAdvanced = false;
			}
			else if (process.wait)
			{
				std::cout << "    -    waiting...";
			}
			std::cout << std::endl;
			process.logTime = 0.0;
		}


	}

	cap.release();

	cv::destroyAllWindows();



	return 0;

}
