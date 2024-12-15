#pragma once

enum ThreshType {
	BASIC_THRESH,
	OTSU_THRESH,
	ADAPTIVE_MEAN_THRESH,
	ADAPTIVE_GAUSS_THRESH
};

struct ImageProcessSettings {
	int downScaling = 0;
	bool blur = false;
	bool erode = false;
	bool dilate = false;
	bool sobel = false;
	int sobelThresh = 100;
	float threshold = 100.0;
	float thresholdMax = 255.0;
	int erosion = 3;
	int dilation = 3;
	cv::Size kernelSize{ 3, 3 };
	int threshType = ThreshType(BASIC_THRESH);
	int adaptiveKernelSize = 5;
	float adaptiveConstant = 2.0f;
};

struct TrackingSettings {
	float areaMinSize = 150.0;
	float areaMaxSize = 600.0;
	float distScale = 1.0;
	float distMod = 0.1;
	float areaMod = 0.05;
	float rectMod = 0.06;
	float rectAreaRatioMod = 1.1;
	float postMod = 1.0;
	float discardThreshold = 0.15;
	float acceptThreshold = 2.4;
	int temporalSteps = 5;
	int temporalFrames = 10;
};

struct LogSettings {
	float logRate = 1.0;
	float logTime = 0.0;
};

struct RenderSettings {
	bool color = false;
	bool invert = true;
	int blending = 128;
};

struct CaptureSettings {
	bool hault = false;
	bool advanceFrame = false;
	bool frameAdvanced = false;
	int wait = 1;
};
struct CalibrationSettings {
	Inches gridSize = 1.0;
	Inches gridDistance = 1.0;
	int cellsPerDimension = 10;
	int linesPerDimension = 11;

};

struct CameraSettings {
	float exposure = -8.50;
	int resolutionScale = 0;
	int width = 2560;
	int height = 960;
};

enum AppMode {
	
	TRACKING_MODE,
	CALIBRATION_MODE,

};

struct AppSettings {

	AppMode mode = TRACKING_MODE;

	ImageProcessSettings* imgProcessing;
	TrackingSettings* tracking;
	LogSettings* logging;
	RenderSettings* rendering;
	CaptureSettings* capturing;
	CalibrationSettings* calibration;
	std::vector<CameraSettings*> cameras = std::vector<CameraSettings*>(0);

};