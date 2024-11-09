#pragma once

#include <vector>

#include "ImgProcTypes.h"

struct LinearRegressionParams {
	int maxIterations;
	double scoreThreshold;

	double slopeModifier;
	double interceptModifier;

	double slope;
	double intercept;
};

void LinearRegression(LinearRegressionParams params, std::vector<FloatVec2> points)
{

	double score = std::numeric_limits<double>::max(); // lower is better
	double currentSlope = params.slope;
	double currentIntercept = params.intercept;

	for (int i = 0; i < params.maxIterations; i++)
	{
		double residual = 0.0;
		for (FloatVec2 p : points)
		{
			double computed = (currentSlope * p.x) + currentIntercept;
			double difference = p.y - computed;

			double modDirection = // insert multiplicative identity thing

		}
	}
}
