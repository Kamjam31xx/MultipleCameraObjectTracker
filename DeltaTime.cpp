#include "DeltaTime.h"

DeltaTime::DeltaTime()
{
	now = std::chrono::high_resolution_clock::now();
	last = std::chrono::high_resolution_clock::now();
	delta = std::chrono::duration<double>(0);
}

void DeltaTime::tick()
{
	now = std::chrono::high_resolution_clock::now();
	delta = now - last;
	last = now;
}

std::chrono::duration<double> DeltaTime::getDelta()
{
	return delta;
}

float DeltaTime::getDeltaNanoSeconds()
{
	return float((std::chrono::duration_cast<std::chrono::nanoseconds>(delta)).count());
}

float DeltaTime::getDeltaMilliSeconds()
{
	return float((std::chrono::duration_cast<std::chrono::milliseconds>(delta)).count());
}

float DeltaTime::getDeltaSeconds()
{
	return float((std::chrono::duration_cast<std::chrono::seconds>(delta)).count());
}