#pragma once

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