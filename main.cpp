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


/*
	add blob to blob manhattan distance -> to -> nearest neighbor mass & nearest neighbor center of mass -> check if that matches temporal past blobs
	- aka a blob gets split into 2 blobs, detect if adding 2 blobs brings back an abstraction resembling that blobby-boi
*/


#include <glad/glad.h>
#include <GLFW/glfw3.h>
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

#include "Shader.h"

#include "CallbacksGLFW.h"

DeltaTime Timer;
DeltaTime VelocityTimer;
FrameRate FPS = FrameRate(10);

cv::VideoCapture cap(1);

AppSettings settings = AppSettings{};

ImageProcessSettings imgProcessing = ImageProcessSettings{};
TrackingSettings tracking = TrackingSettings{};
LogSettings logging = LogSettings{};
RenderSettings rendering = RenderSettings{};
CaptureSettings capturing = CaptureSettings{};
CalibrationSettings calibration = CalibrationSettings{};

CameraSettings frontCamera = CameraSettings{ -8.50f, 2, 2560 / 2, 960 / 2 };

void InitSettings() {
	settings = AppSettings{
		TRACKING_MODE,
		&imgProcessing,
		&tracking,
		&logging,
		&rendering,
		&capturing,
		&calibration,
		std::vector<CameraSettings*>{&frontCamera}
	};
}

int main()
{
	InitSettings();
	if (!glfwInit()) {
		return -1;
	}
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 4);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	GLFWwindow* window = glfwCreateWindow(1230, 480, "window", NULL, NULL);

	if (!window) {
		glfwTerminate();
		std::cout << "!window == true" << std::endl;
		return -2;
	}

	glfwMakeContextCurrent(window);
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		glfwTerminate();
		std::cout << "gladLoader failure" << std::endl;
		return -3;
	}
	glfwSetErrorCallback(error_callback);
	if (cap.isOpened() != true) {
		std::cout << "cap is not opened" << std::endl;
		return -4;
	}

	// double time = glfwGetTime(); // typically the most accurate time source on each platform
	int winWidth;
	int winHeight;
	glfwGetFramebufferSize(window, &winWidth, &winHeight);
	glViewport(0, 0, winWidth, winHeight);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSwapInterval(0); // set to 1 for vsync
	// glfwWaitEvents // see docs https://www.glfw.org/docs/latest/group__window.html#ga554e37d781f0a997656c26b2c56c835e

	Shader shader;
	shader.CreateFromFiles("/Standard.vert", "/standard.frag", nullptr);

	cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0);
	cap.set(cv::CAP_PROP_EXPOSURE, frontCamera.exposure);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, frontCamera.height);
	cap.set(cv::CAP_PROP_FRAME_WIDTH, frontCamera.width);

	int front = 0;
	std::vector<cv::Mat> frame = { cv::Mat(), cv::Mat() };
	std::vector<cv::Mat> detect = { cv::Mat(), cv::Mat() };
	std::vector<cv::Mat> blurredFrame = { cv::Mat(), cv::Mat() };
	std::vector<cv::Mat> lightMaskFrame = { cv::Mat(), cv::Mat() };
	cv::Mat erosionElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * imgProcessing.erosion + 1, 2 * imgProcessing.erosion + 1));
	cv::Mat dilationElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * imgProcessing.dilation + 1, 2 * imgProcessing.dilation + 1));

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

	while (!glfwWindowShouldClose(window)) {
		
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		glfwSwapBuffers(window);
		glfwPollEvents();

		HandleKeyboard(Timer, cap, settings);

		if (capturing.wait) {
			if (capturing.advanceFrame) {
				cap >> frame[front];
				capturing.frameAdvanced = true;
				capturing.advanceFrame = false;
			}
		}
		else {
			cap >> frame[front];
		}


		if (frame.empty()) {
			break;
		}
		else {
			// IMPLEMENT FUNCTION POINTERS TO ELIMINATE BRANCHING
			cv::cvtColor(frame[front], detect[front], cv::COLOR_BGR2GRAY);
			if (imgProcessing.blur) {
				cv::GaussianBlur(frame[front], frame[front], imgProcessing.kernelSize, 0);
			}
			cv::Mat derivativeMap;
			if (imgProcessing.sobel) {
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

			cv::threshold(detect[front], detect[front], imgProcessing.threshold, imgProcessing.thresholdMax, cv::THRESH_BINARY);

			if (imgProcessing.sobel)
			{
				cv::subtract(255, derivativeMap, derivativeMap);
				cv::threshold(derivativeMap, derivativeMap, imgProcessing.sobelThresh, imgProcessing.thresholdMax, cv::THRESH_BINARY);
				derivativeMap.copyTo(detect[front]);
			}
			if (imgProcessing.erode) {
				cv::erode(detect[front], detect[front], erosionElement);
			}
			if (imgProcessing.dilate) {
				cv::dilate(detect[front], detect[front], dilationElement);
			}
			cv::cvtColor(detect[front], fillResult[front], cv::COLOR_GRAY2BGR);

			int width = fillResult[front].cols / 2;
			int height = fillResult[front].rows;
			leftCamera[front] = cv::Mat(fillResult[front], cv::Rect(0, 0, width, height));
			rightCamera[front] = cv::Mat(fillResult[front], cv::Rect(width, 0, width, height));

			// replace with bounds per function to avoid copying and stuff
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

			GetBlobs(&leftCamera[front], &leftResult[front], 255, tracking.areaMinSize, leftNow);
			GetBlobs(&rightCamera[front], &rightResult[front], 255, tracking.areaMinSize, rightNow);

			VelocityTimer.tick();
			float deltaTime = VelocityTimer.getDeltaMilliSeconds();
			if (rendering.color)
			{
				ColorTrack_Test(&leftResult[front], lastLeft, leftNow, deltaTime, tracking);
				ColorTrack_Test(&rightResult[front], lastRight, rightNow, deltaTime, tracking);
			}

			cv::subtract(255, leftResult[front], leftResult[front]);
			cv::subtract(255, rightResult[front], rightResult[front]);

			cv::waitKey(1);

			cv::imshow("_____< left >_____", leftResult[front]);
			cv::imshow("_____< right >_____", rightResult[front]);

			lastLeft = leftNow;
			lastRight = rightNow;

			glfwSwapBuffers(window);
		}



		char c = (char)cv::waitKey(capturing.cvwait + (capturing.lag * capturing.cvwait * 128));




		Timer.tick();
		FPS.pushFrameTime(Timer.getDeltaMilliSeconds());
		logging.logTime += float(Timer.getDeltaMilliSeconds() / 1000.0);


		if (logging.logTime >= logging.logRate)
		{
			std::cout << "t=" + std::to_string(Timer.getDeltaMilliSeconds()) + " r=" + std::to_string(FPS.getRate());
			if (capturing.frameAdvanced)
			{
				std::cout << "    -    advanced frame";
				capturing.frameAdvanced = false;
			}
			else if (capturing.wait)
			{
				std::cout << "    -    waiting...";
			}
			std::cout << std::endl;
			logging.logTime = 0.0;
		}


	}

	glfwTerminate();

	cap.release();

	cv::destroyAllWindows();



	return 0;

}
