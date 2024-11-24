#pragma once

#include <vector>
#include <tuple>

#include "ImgProcTypes.h"
#include "Values.h"

// linear regression where the points are rotated to fit the line to them
Float64SlopeInterceptRotation LinearRegressionR2D1(std::vector<FloatVec2> points) {

	double rotationStepSize = 17.3;
	int rotationStep = 1;
	double thresh = 0.00000001;

	while (true) {

		double xSum = 0;
		double ySum = 0;
		double xySum = 0;
		double x2Sum = 0;

		double rotation = rotationStep * rotationStepSize * TO_RADIANS;

		for (int k = 0; k < points.size(); k++) {
			double x = (points[k].x * cos(rotation)) - (points[k].y * sin(rotation));
			double y = (points[k].y * cos(rotation)) + (points[k].x * sin(rotation));
			xSum += x;
			ySum += y;
			xySum += x * y;
			x2Sum += x * x;
		}

		double numerator = (points.size() * xySum) - (xSum * ySum);
		double denominator = (points.size() * x2Sum) - (xSum * xSum);

		bool denominatorTooSmall = denominator <= thresh && denominator >= -thresh;
		bool numeratorTooSmall = numerator <= thresh && numerator >= -thresh;
		if (denominatorTooSmall || numeratorTooSmall) {
			// has failed -> change rotation and reset
			rotationStep++;
			continue;
		}

		double slope = numerator / denominator;
		bool slopeTooSmall = slope <= thresh && slope >= -thresh;
		if (slopeTooSmall) {
			// has failed -> change rotation and reset
			rotationStep++;
			continue;
		}

		double intercept = (ySum - (slope * xSum)) / points.size();
		return Float64SlopeInterceptRotation{ slope, intercept, rotation * TO_DEGREES };
	}
}
// linear regression where the points are rotated to fit the line to them
std::tuple<Float64SlopeIntercept, bool> LinearRegressionR2D2(std::vector<IntVec2> points, int step, Degrees stepSize) {

	double thresh = 0.00000001;
	
	double xSum = 0;
	double ySum = 0;
	double xySum = 0;
	double x2Sum = 0;

	double rotation = step * stepSize;

	Float64SlopeIntercept failureResult = { 1,0 };
	double c = cos(rotation);
	double s = sin(rotation);
	for (int k = 0; k < points.size(); k++) {
		double x = (static_cast<double>(points[k].x) * c) - (static_cast<double>(points[k].y) * s);
		double y = (static_cast<double>(points[k].y) * c) + (static_cast<double>(points[k].x) * s);
		xSum += x;
		ySum += y;
		xySum += x * y;
		x2Sum += x * x;
	}

	double numerator = (points.size() * xySum) - (xSum * ySum);
	double denominator = (points.size() * x2Sum) - (xSum * xSum);

	bool denominatorTooSmall = denominator <= thresh && denominator >= -thresh;
	bool numeratorTooSmall = numerator <= thresh && numerator >= -thresh;
	if (denominatorTooSmall || numeratorTooSmall) {
		// has failed 
		return std::make_tuple(failureResult, true);
	}

	double slope = numerator / denominator;
	bool slopeTooSmall = slope <= thresh && slope >= -thresh;
	if (slopeTooSmall) {
		// has failed -> change rotation and reset
		return std::make_tuple(failureResult, true);
	}

	double intercept = (ySum - (slope * xSum)) / points.size();
	return std::make_tuple(Float64SlopeIntercept{ slope, intercept }, false);
}

std::tuple<std::vector<Float64SlopeIntercept>, double> LinearRegressionMultiR2D2(std::vector<std::vector<IntVec2>> pointSets, Degrees stepSize) {

	int step = 0;
	std::vector<Float64SlopeIntercept> results = std::vector<Float64SlopeIntercept>(0); // can presize

	while (true) {

		bool hasFailed = false;

		for (std::vector<IntVec2> points : pointSets) {
			std::tuple<Float64SlopeIntercept, bool> lineResult = LinearRegressionR2D2(points, step, stepSize);
			hasFailed = std::get<1>(lineResult);
			if (hasFailed) {
				step++;
				results = std::vector<Float64SlopeIntercept>(0);
				break;
			}
			else {
				results.push_back(std::get<0>(lineResult));
			}
		}

		if (hasFailed) {
			continue;
		}
		else {
			break;
		}
	}

	return std::make_tuple(results, step * stepSize);
}


Float64SlopeIntercept LinearRegression2D1(std::vector<FloatVec2> points, double failureSlope, double failureIntercept) {

	double thresh = 0.00000000001;
	double xSum = 0;
	double ySum = 0;
	double xySum = 0;
	double x2Sum = 0;

	for (int k = 0; k < points.size(); k++) {
		double x = points[k].x - points[k].y;
		double y = points[k].y + points[k].x;
		xSum += x;
		ySum += y;
		xySum += x * y;
		x2Sum += x * x;
	}

	double numerator = (points.size() * xySum) - (xSum * ySum);
	double denominator = (points.size() * x2Sum) - (xSum * xSum);

	bool denominatorTooSmall = denominator <= thresh && denominator >= -thresh;
	bool numeratorTooSmall = numerator <= thresh && numerator >= -thresh;
	if (denominatorTooSmall || numeratorTooSmall) {

		return Float64SlopeIntercept{ failureSlope, failureIntercept };
	}

	double slope = numerator / denominator;
	bool slopeTooSmall = slope <= thresh && slope >= -thresh;
	if (slopeTooSmall) {
		return Float64SlopeIntercept{ failureSlope, failureIntercept };
	}

	double intercept = (ySum - (slope * xSum)) / points.size();
	return Float64SlopeIntercept{ slope, intercept };
}