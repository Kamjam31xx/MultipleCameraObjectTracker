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

void Logger(std::string in)
{
	std::cout << in << std::endl;
}


void Logger(std::string name, PixelCoord in)
{
	std::cout << "<" << name << "> " << toLog(in) << std::endl;
}
void Logger(std::string name, PerimeterPoint in)
{
	std::cout << "<" << name << "> " << toLog(in) << std::endl;
}
void Logger(std::string name, Rectangle in)
{
	std::cout << "<" << name << "> " << toLog(in) << std::endl;
}


void logExposure(float input) {
	Logger("  < exposure >  :  " + std::to_string(input));
}
void logThreshold(float input) {
	Logger("  < threshold >  :  " + std::to_string(input));
}
void logSetting(std::string name, double value) {
	Logger("  < " + name + " >  :  " + std::to_string(value));
}
void logSetting(std::string name, float value) {
	Logger("  < " + name + " >  :  " + std::to_string(value));
}
void logSetting(std::string name, bool value) {
	std::string valueStr = value ? "true" : "false";
	Logger("  < " + name + " > : " + valueStr);
}
void logSetting(std::string name, std::string value) {
	Logger("  < " + name + " > : " + value);
}
void logSetting(std::string name) {
	Logger("  < " + name + " >");
}