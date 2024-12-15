#include "FrameRate.h"


FrameRate::FrameRate(int bufferLength)
{
	length = bufferLength;
}

float FrameRate::getRate()
{
	float total = 0;
	int frames = 0;

	for (float frameTime : frameTimes)
	{
		frames += 1;
		total += frameTime;
	}
	
	return 1000 / (total / frames);
}

void FrameRate::pushFrameTime(float frameTime)
{
	frameTimes.push_back(frameTime);
	while (frameTimes.size() > length && frameTimes.size()) {
		frameTimes.erase(frameTimes.begin());
	}
}
