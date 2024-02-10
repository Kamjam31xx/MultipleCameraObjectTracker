#pragma once

#include <chrono>

class DeltaTime
{

public:

	DeltaTime();

	void tick();

	std::chrono::duration<double> getDelta();

	float getDeltaNanoSeconds();
	float getDeltaMilliSeconds();
	float getDeltaSeconds();

private:

	std::chrono::steady_clock::time_point now;
	std::chrono::steady_clock::time_point last;
	std::chrono::duration<double> delta;

};