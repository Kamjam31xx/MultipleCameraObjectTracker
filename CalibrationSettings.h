#pragma once

#include "ImgProcTypes.h"

struct CalibrationSettings {
	Inches gridSize = 1.0;
	Inches gridDistance = 1.0;
	int cellsPerDimension = 10;
	int linesPerDimension = 11;

};