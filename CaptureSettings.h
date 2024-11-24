#pragma once

struct CaptureSettings {
	bool wait = false;
	bool advanceFrame = false;
	bool frameAdvanced = false;
	int cvwait = 1;
	float cvwaitSlider = 1.0;
	bool lag = false;
};