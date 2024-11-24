#pragma once

#include "ImageProcessSettings.h"
#include "TrackingSettings.h"
#include "LogSettings.h"
#include "RenderSettings.h"
#include "CaptureSettings.h"
#include "CameraSettings.h"
#include "CalibrationSettings.h"

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