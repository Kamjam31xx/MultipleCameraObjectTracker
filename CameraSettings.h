#pragma once

struct CameraSettings {
	double exposure = -8.50;
	int resolutionScale = 2;
	int width = 2560 / resolutionScale;
	int height = 960 / resolutionScale;
};