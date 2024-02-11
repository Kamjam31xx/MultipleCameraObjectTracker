#pragma once
#include <string>
#include <iostream>
#include "ImgProcTypes.h"

std::string str(int in)
{
	return std::to_string(in);
}

std::string toLog(PixelCoord in)
{
	return "px( " + str(in.x) + " , " + str(in.y) + " )";
}
std::string toLog(PerimeterPoint in)
{
	return "p( " + str(in) + " )";
}
std::string toLog(Rectangle in)
{
	return "rect( x:" + str(in.xMin) + " y:" + str(in.yMin) + " xMax:" + str(in.xMax) + " yMax:" + str(in.yMax) + " )";
}

void log(std::string in)
{
	std::cout << in << std::endl;
}


void log(std::string name, PixelCoord in)
{
	std::cout << "<" << name << "> " << toLog(in) << std::endl;
}
void log(std::string name, PerimeterPoint in)
{
	std::cout << "<" << name << "> " << toLog(in) << std::endl;
}
void log(std::string name, Rectangle in)
{
	std::cout << "<" << name << "> " << toLog(in) << std::endl;
}