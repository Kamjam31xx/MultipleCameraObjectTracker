#pragma once

#include <iostream>
#include <vector>
#include <array>
#include <iterator>
#include <math.h>
#include "ImgProcTypes.h"

struct Poly5 {
	double a0;
	double a1;
	double a2;
	double a3;
	double a4;
};
using Residuals = std::vector<double>;
using Poly5Jacobian = std::vector<std::array<double, 5>>;

std::vector<FloatVec2> originFromFirst(std::vector<FloatVec2> coords) {

	std::vector<FloatVec2> output;
	output.push_back(FloatVec2{ 0.0, 0.0 });

	double xMod = -coords[0].x;
	double yMod = -coords[0].y;

	for (int i = 1; i < coords.size(); i++) {

		output.push_back(FloatVec2{ coords[i].x + xMod , coords[i].y + yMod });

	}

	return output;
}

std::vector<FloatVec2> rotateToLowestSum(std::vector<FloatVec2> coords, int numPoints) {

	// finish

	return std::vector<FloatVec2>{};

}

Residuals calcResiduals(std::vector<FloatVec2> coords, Poly5 poly) {

	Residuals result;
	
	for (int i = 0; i < coords.size(); i++) {

		double x = coords[i].x;

		double term0 = poly.a0;
		double term1 = poly.a1 * x;
		double term2 = poly.a2 * pow(x, 2.0);
		double term3 = poly.a3 * pow(x, 3.0);
		double term4 = poly.a4 * pow(x, 4.0);

		result.push_back(term0 + term1 + term2 + term3 + term4 - coords[i].y);
	}

	return result;
}


Poly5Jacobian calcJacobian(std::vector<FloatVec2> coords, Poly5 poly) {

	Poly5Jacobian matrix = Poly5Jacobian(coords.size(), std::array<double, 5>{0.0, 0.0, 0.0, 0.0, 0.0});

	for (int i = 0; i < coords.size(); i++) {

		double x = coords[i].x;



	}
	
}



double sumSqaures(std::vector<double> residuals) {
	double sum = 0.0;
	for (int i = 0; i < residuals.size(); i++) {
		sum += residuals[i] * residuals[i];
	}
	return sum;
}

/*


beta[i] = a point
r[i] = a residual
tolerance == good enough stuff thing -> aka bing bong number the stuff is what?
maxiter == maximum bingbongs amounts of the what is hmm?

for the stuff is, make the bing bong but not too many bing bong, then when the stuff 
is bingbong af, do the stuff is. keep track of the bingbong amount to make is the what next?






*/

void GaussNewton(int count, double threshold, )