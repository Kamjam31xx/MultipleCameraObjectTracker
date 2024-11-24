#pragma once

#include <string>

#include "ImgProcTypes.h"

struct OV9750 {
	std::string company = "Omnivision";
	std::string name = "OV9750";
	std::string type = "sensor";
	std::string format = "1/3in";
	Inches diagonal = 6.0f * MILLIMETERS_TO_INCHES;
	Inches width = 4.8f * MILLIMETERS_TO_INCHES;
	Inches height = 3.6f * MILLIMETERS_TO_INCHES;
	IntVec2 imageSize = IntVec2{ 1280, 960 };
	float xFovDegrees = 100.0f;
};