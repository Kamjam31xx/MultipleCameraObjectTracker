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

#include "ImgProcTypes.h"
#include "DeltaTime.h"
#include "FrameRate.h"
#include "ImgProc.h"

DeltaTime Timer;
DeltaTime VelocityTimer;
FrameRate FPS = FrameRate(10);

cv::VideoCapture cap(1);
//cv::VideoCapture cap("test_0.mp4");

double exposure = -8.50;
//exposure = -0.1;
int resolutionScale = 2;
int streamWidth = 2560 / resolutionScale;
int streamHeight = 960 / resolutionScale;
//int streamWidth = 640 * 4;
//int streamHeight = 240 * 4;
//int streamWidth = 3840 / resolutionScale;
//int streamHeight = 1440 / resolutionScale;

double threshold = 100.0;
double thresholdMax = 255.0;
int erosion = 3;
int dilation = 3;
cv::Size kernelSize(3, 3);
float logRate = 1.0;
float logTime = 0.0;
// add areaMin, areaMinPaddingFunction to keep tracking blobs that flip back n forth
float areaMinSize = 150.0;
int temporalSteps = 5;
int temporalFrames = 10;

float distScale = 1.0;
float distMod = 0.1;
float areaMod = 0.05;
float rectMod = 0.06;
float rectAreaRatioMod = 1.1;
float postModPosAreaRect = 1.0;
float discardThreshold = 0.15;
float acceptThreshold = 2.4;

bool wait = false;
bool advanceFrame = false;
bool frameAdvanced = false;
bool blur = false;
bool erode = false;
bool dilate = false;
bool color = false;
int cvwait = 1;
float cvwaitSlider = 1.0;
bool lag = false;
bool sobel = false;
int sobelThresh = 100;

void HandleKeyboard() {


	if (_kbhit())
	{
		char key = _getch();
		if (key == 'q')
		{
			exposure += float(Timer.getDeltaMilliSeconds() / 100.0);
			cap.set(cv::CAP_PROP_EXPOSURE, exposure);
			log("  < exposure >  :  " + std::to_string(exposure));
		}
		else if (key == 'a')
		{
			exposure -= float(Timer.getDeltaMilliSeconds() / 100.0);
			cap.set(cv::CAP_PROP_EXPOSURE, exposure);
			log("  < exposure >  :  " + std::to_string(exposure));
		}
		else if (key == 'w')
		{
			threshold += float(Timer.getDeltaMilliSeconds() / 50.0);
			log("  < threshold >  :  " + std::to_string(threshold));
		}
		else if (key == 's')
		{
			threshold -= float(Timer.getDeltaMilliSeconds() / 50.0);
			log("  < threshold >  :  " + std::to_string(threshold));
		}

		else if (key == 'e')
		{
			areaMinSize += float(Timer.getDeltaMilliSeconds() / 10.0);
			log("  < min size >  :  " + std::to_string(areaMinSize));
		}
		else if (key == 'd')
		{
			areaMinSize -= float(Timer.getDeltaMilliSeconds() / 10.0);
			log("  < min size >  :  " + std::to_string(areaMinSize));
		}

		else if (key == 'r')
		{
			distMod += float(Timer.getDeltaMilliSeconds() / 10000.0);
			log("  < distMod >  :  " + std::to_string(distMod));
		}
		else if (key == 'f')
		{
			distMod -= float(Timer.getDeltaMilliSeconds() / 10000.0);
			log("  < distMod >  :  " + std::to_string(distMod));
		}
		else if (key == 't')
		{
			areaMod += float(Timer.getDeltaMilliSeconds() / 10000.0);
			log("  < areaMod >  :  " + std::to_string(areaMod));
		}
		else if (key == 'g')
		{
			areaMod -= float(Timer.getDeltaMilliSeconds() / 10000.0);
			log("  < areaMod >  :  " + std::to_string(areaMod));
		}

		else if (key == 'y')
		{
			rectMod += float(Timer.getDeltaMilliSeconds() / 10000.0);
			log("  < rectMod >  :  " + std::to_string(rectMod));
		}
		else if (key == 'h')
		{
			rectMod -= float(Timer.getDeltaMilliSeconds() / 10000.0);
			log("  < rectMod >  :  " + std::to_string(rectMod));
		}
		else if (key == 'u')
		{
			rectAreaRatioMod += float(Timer.getDeltaMilliSeconds() / 10000.0);

			log("  < rect : area mod >  :  " + std::to_string(rectAreaRatioMod));
		}
		else if (key == 'j')
		{
			rectAreaRatioMod -= float(Timer.getDeltaMilliSeconds() / 10000.0);
			log("  < rect : area mod >  :  " + std::to_string(rectAreaRatioMod));
		}
		else if (key == 'i')
		{
			discardThreshold += float(Timer.getDeltaMilliSeconds() / 10000.0);
			log("  < discard threshold >  :  " + std::to_string(discardThreshold));
		}
		else if (key == 'k')
		{
			discardThreshold -= float(Timer.getDeltaMilliSeconds() / 10000.0);
			log("  < discard threshold >  :  " + std::to_string(discardThreshold));
		}
		else if (key == 'o')
		{
			acceptThreshold += float(Timer.getDeltaMilliSeconds() / 10000.0);
			log("  < accept threshold >  :  " + std::to_string(acceptThreshold));
		}
		else if (key == 'l')
		{
			acceptThreshold -= float(Timer.getDeltaMilliSeconds() / 10000.0);
			log("  < accept threshold >  :  " + std::to_string(acceptThreshold));
		}
		else if (key == 'p')
		{
			distScale += float(Timer.getDeltaMilliSeconds() / 10000.0);
			log("  < dist scale >  :  " + std::to_string(distScale));
		}
		else if (key == ';')
		{
			distScale -= float(Timer.getDeltaMilliSeconds() / 10000.0);
			log("  < dist scale >  :  " + std::to_string(distScale));
		}
		else if (key == '[')
		{
			postModPosAreaRect += float(Timer.getDeltaMilliSeconds() / 10000.0);
			log("  < post modPAR >  :  " + std::to_string(postModPosAreaRect));
		}
		else if (key == '\'')
		{
			postModPosAreaRect -= float(Timer.getDeltaMilliSeconds() / 10000.0);
			log("  < poast modPAR >  :  " + std::to_string(postModPosAreaRect));
		}

		else if (key == 'b')
		{
			wait = !wait;
			log("  < wait >");
		}
		else if (key == 'n')
		{
			advanceFrame = true;
			log("  < advance frame >");
		}
		else if (key == 'z')
		{
			erode = !erode;
			log("  < erode >");
		}
		else if (key == 'x')
		{
			dilate = !dilate;
			log("  < dilate >");
		}
		else if (key == 'c')
		{
			blur = !blur;
			log("  < blur >");
		}
		else if (key == 'v')
		{
			color = !color;
			log("  < color >");
		}
		else if (key == '<')
		{
			cvwaitSlider += float(Timer.getDeltaMilliSeconds() / 500.0);
			cvwait = 1 + abs(floor(cvwaitSlider));
			log("  < cv wait >  :  " + std::to_string(cvwait));
		}
		else if (key == '>')
		{
			cvwaitSlider -= float(Timer.getDeltaMilliSeconds() / 500.0);
			cvwait = 1 + abs(floor(cvwaitSlider));
			log("  < cv wait >  :  " + std::to_string(cvwait));
		}
		else if (key == 'm')
		{
			lag = !lag;
			log("  < lag >  :  " + std::to_string(lag));
		}
		else if (key == '/')
		{
			sobel = !sobel;
			if (sobel)
			{
				log("  < pre-process | sobel >  :  true");
			}
			else
			{
				log("  < pre-process | thresh >  :  false");
			}
		}
		else if (key == ']')
		{
			log("  ");
			log("  __SETTINGS___________________________________________");
			log("  ");
			log("                exposure |  " + std::to_string(exposure));
			log("               threshold |  " + std::to_string(threshold));
			log("                          ");
			log("                min size |  " + std::to_string(areaMinSize));
			log("                          ");
			log("                 distMod |  " + std::to_string(distMod));
			log("                 areaMod |  " + std::to_string(areaMod));
			log("                 rectMod |  " + std::to_string(rectMod));
			log("         rect : area mod |  " + std::to_string(rectAreaRatioMod));
			log("            post mod PAR |  " + std::to_string(postModPosAreaRect));
			log("                          ");
			log("       discard threshold |  " + std::to_string(discardThreshold));
			log("        accept threshold |  " + std::to_string(acceptThreshold));
			log("                          ");
			log("              dist scale |  " + std::to_string(distScale));
			log("                          ");
			log("                    wait |  " + std::to_string(wait));
			log("           advance frame |  " + std::to_string(advanceFrame));
			log("                          ");
			log("                   erode |  " + std::to_string(erode));
			log("                  dilate |  " + std::to_string(dilate));
			log("                    blur |  " + std::to_string(blur));
			log("                   color |  " + std::to_string(color));
			log("                          ");
			log("                 cv wait |  " + std::to_string(cvwait));
			log("  ");
		}

		if (exposure < -15) { exposure = -15; }
		else if (exposure > 15) { exposure = 15; }
		if (threshold < 0) { threshold = 0; }
		else if (threshold > 255) { threshold = 255; }
		areaMinSize = areaMinSize < 0 ? 0 : areaMinSize;
		distMod = distMod < 0 ? 0 : distMod;
		areaMod = areaMod < 0 ? 0 : areaMod;
		rectMod = rectMod < 0 ? 0 : rectMod;
		rectAreaRatioMod = rectAreaRatioMod < 0 ? 0 : rectAreaRatioMod;
		discardThreshold = discardThreshold < 0 ? 0 : discardThreshold;
		acceptThreshold = acceptThreshold < 0 ? 0 : acceptThreshold;
	}
}

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
	cap.set(cv::CAP_PROP_EXPOSURE, exposure);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, streamHeight);
	cap.set(cv::CAP_PROP_FRAME_WIDTH, streamWidth);

	int front = 0;
	std::vector<cv::Mat> frame = { cv::Mat(), cv::Mat() };
	std::vector<cv::Mat> detect = { cv::Mat(), cv::Mat() };
	std::vector<cv::Mat> blurredFrame = { cv::Mat(), cv::Mat() };
	std::vector<cv::Mat> lightMaskFrame = { cv::Mat(), cv::Mat() };
	cv::Mat erosionElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * erosion + 1, 2 * erosion + 1));
	cv::Mat dilationElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * dilation + 1, 2 * dilation + 1));

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

		if (wait)
		{
			if (advanceFrame)
			{
				cap >> frame[front];
				frameAdvanced = true;
				advanceFrame = false;
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
			cv::cvtColor(frame[front], detect[front], cv::COLOR_BGR2GRAY);
			if (blur) {
				cv::GaussianBlur(frame[front], frame[front], kernelSize, 0);
			}
			cv::Mat derivativeMap;
			if (sobel)
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

			cv::threshold(detect[front], detect[front], threshold, thresholdMax, cv::THRESH_BINARY);

			if (sobel)
			{
				cv::subtract(255, derivativeMap, derivativeMap);
				cv::threshold(derivativeMap, derivativeMap, sobelThresh, thresholdMax, cv::THRESH_BINARY);
				derivativeMap.copyTo(detect[front]);
			}
			if (erode) {
				cv::erode(detect[front], detect[front], erosionElement);
			}
			if (dilate) {
				cv::dilate(detect[front], detect[front], dilationElement);
			}
			cv::cvtColor(detect[front], fillResult[front], cv::COLOR_GRAY2BGR);

			leftCamera[front] = cv::Mat(fillResult[front], cv::Rect(0, 0, fillResult[front].cols / 2, fillResult[front].rows));
			rightCamera[front] = cv::Mat(fillResult[front], cv::Rect(fillResult[front].cols / 2, 0, fillResult[front].cols / 2, fillResult[front].rows));

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

			GetBlobs(&leftCamera[front], &leftResult[front], 255, areaMinSize, leftNow);
			GetBlobs(&rightCamera[front], &rightResult[front], 255, areaMinSize, rightNow);

			cv::waitKey(2);

			VelocityTimer.tick();
			float deltaTime = VelocityTimer.getDeltaMilliSeconds();
			if (color)
			{
				ColorTrackParams params = ColorTrackParams{ 
					deltaTime, 
					(int)areaMinSize, 
					distScale, 
					distMod,
					areaMod, 
					rectMod, 
					postModPosAreaRect, 
					rectAreaRatioMod, 
					discardThreshold, 
					acceptThreshold 
				};

				ColorTrack_Test(&leftResult[front], lastLeft, leftNow, params);
				ColorTrack_Test(&rightResult[front], lastRight, rightNow, params);
			}

			cv::subtract(255, leftResult[front], leftResult[front]);
			cv::subtract(255, rightResult[front], rightResult[front]);

			cv::waitKey(2);

			cv::imshow("_____< left >_____", leftResult[front]);
			cv::imshow("_____< right >_____", rightResult[front]);

			lastLeft = leftNow;
			lastRight = rightNow;
		}



		char c = (char)cv::waitKey(cvwait + (lag * cvwait * 128));




		Timer.tick();
		FPS.pushFrameTime(Timer.getDeltaMilliSeconds());
		logTime += float(Timer.getDeltaMilliSeconds() / 1000.0);


		if (logTime >= logRate)
		{
			std::cout << "t=" + std::to_string(Timer.getDeltaMilliSeconds()) + " r=" + std::to_string(FPS.getRate());
			if (frameAdvanced)
			{
				std::cout << "    -    advanced frame";
				frameAdvanced = false;
			}
			else if (wait)
			{
				std::cout << "    -    waiting...";
			}
			std::cout << std::endl;
			logTime = 0.0;
		}


	}

	cap.release();

	cv::destroyAllWindows();



	return 0;

}
