#pragma once

#include <queue>
#include <chrono>

class FrameRate
{

public:

	FrameRate(int bufferLength);

	float getRate();

	void pushFrameTime(float frameTime);

private:

	float frameRate;
	int length;
	std::vector<float> frameTimes;
};

