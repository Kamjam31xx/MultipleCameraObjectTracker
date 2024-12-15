#pragma once

#include <queue>
#include <chrono>

class FrameRate
{

public:

	FrameRate(int bufferLength);

	float getRate();

	void pushFrameTime(float frameTime);

	int length;
	float frameRate;
	std::vector<float> frameTimes;

private:


};

