#pragma once

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <vector>

#include "ImgProcTypes.h"

// construct the jacobian for the given x, y, betas, and partial derivative functions
// unsafe - size your inputs right
void Jacobian(cv::Mat& out, std::vector<double>& xi, std::vector<double>& yi, cv::Mat& betas, std::function<double(double, double, int)> partials) {
	int m = betas.rows;
	for (int i = 0; i < xi.size(); i++) {
		for (int j = 0; j < m; j++) {
			int k = j + (i * m);
			out.at<double>(j, i) = partials(xi[k], yi[k], j);
		}
	}
}

// given the function for the residual using the betas of each xi yi, set the residuals result in the output column matrix
void CalcResiduals(cv::Mat& out, std::vector<double>& xi, std::vector<double>& yi, cv::Mat& betas, std::function<double(double, double, cv::Mat&)> residual) {
	for (int i = 0; i < xi.size(); i++) {
		out.at<double>(i, 0) = residual(xi[i], yi[i], betas);
	}
}

// for the function x - (hy + ky^2 + ... + uy^n)
double PolyNPartials(double x, double y, int function) {
	return x - (pow(-y, function + 1));
}

// for the function x - (hy + ky^2 + ... + uy^n)
double SquaredResiduals(cv::Mat residuals) {
	double sum = 0.0;
	for (int i = 0; i < residuals.rows; i++) {
		sum += pow(residuals.at<double>(i, 0), 2);
	}
	return sum;
}

// for the function x - (hy + ky^2 + ... + uy^n)
double PolyNResiduals(double x, double y, cv::Mat betas) {
	double yHat = 0;
	for (int i = 0; i < betas.rows; i++) {
		yHat += betas.at<double>(i, 0) * pow(y, i);
	}
	return x - yHat;
}

cv::Mat Betas(int n, bool descending, bool negative, int a, int x, int s) {

	cv::Mat betas = cv::Mat(n, 1, CV_64F);

	betas.at<double>(0, 0) = 1.0;
	for (int i = 1; i < n; i++) {

		double g = pow(a , pow(i,x));
		double b = descending ? s * i * (1.0 / g) : s * i * g;
		if (negative) {
			b = -b;
		}

		betas.at<double>(i, 0) = b;
	}

	return betas;
}

PolyN BetasToPolyN(cv::Mat& b) {
	switch (b.rows) {
	case 2: return PolyN{ b.rows, static_cast<float>(b.at<double>(0,0)), static_cast<float>(b.at<double>(1,0)), 0.0, 0.0, 0.0 };
	case 3: return PolyN{ b.rows, static_cast<float>(b.at<double>(0,0)), static_cast<float>(b.at<double>(1,0)), static_cast<float>(b.at<double>(2,0)), 0.0, 0.0 };
	case 4: return PolyN{ b.rows, static_cast<float>(b.at<double>(0,0)), static_cast<float>(b.at<double>(1,0)), static_cast<float>(b.at<double>(2,0)), static_cast<float>(b.at<double>(3,0)), 0.0 };
	case 5: return PolyN{ b.rows, static_cast<float>(b.at<double>(0,0)), static_cast<float>(b.at<double>(1,0)), static_cast<float>(b.at<double>(2,0)), static_cast<float>(b.at<double>(3,0)), static_cast<float>(b.at<double>(4,0)) };
	}
	return PolyN{ 0, 0.0, 0.0, 0.0, 0.0, 0.0 };

}

// unsafe, ensure same size for xi and yi
PolyN GaussNewtonPolyN(std::vector<double> xi, std::vector<double> yi, int n) {

	int numResiduals = xi.size();
	// note that m functions can be greater than n betas, as there may be more partial derivatives than betas
	int n = n; // number of betas
	int m = n; // number of functions
	cv::Mat betas = Betas(n, true, false, 10, 0, 1);

	// jacobian matrix of m functions and n residuals 
	cv::Mat jacobian = cv::Mat(numResiduals, m, CV_64F);
	cv::Mat residuals = cv::Mat(numResiduals, 1, CV_64F);
	cv::Mat psuedoInverse;

	// minimize the sum of squares of the residuals
	double sum = 0.0;
	while (true) {

		// construct the jacobian for the residuals of m function partial derivatives with respect to the betas
		Jacobian(jacobian, xi, yi, betas, PolyNPartials);
		cv::invert(jacobian, psuedoInverse); // note : check flags 
		CalcResiduals(residuals, xi, yi, betas, PolyNResiduals);

		// calculate the new betas
		betas = betas - (psuedoInverse * residuals);

		// calculate the sum of squares of the residuals
		sum = SquaredResiduals(residuals);
		if (sum < 1.0) {
			break;
		}
	}

	return BetasToPolyN(betas);
}

// unsafe, ensure same size for xi and yi
// does consecutive xi yi spans and uses their center of mass calculation to construct a square matrix
// for optimized GaussNewton inverse calculation.
PolyN GaussNewtonBriggsPolyN(std::vector<double> xi, std::vector<double> yi, int n) {

	std::vector<double> xj;
	std::vector<double> yj;
	int m = n;
	int stride = floorf(static_cast<float>(xj.size()) / static_cast<float>(m));
	for (int i = 0; i < m; i++) {
		int a = i * stride;
		int b = i == m - 1 ? xj.size() - 1 : ((i + 1) * stride) - 1;
		int numElements = (b - a) + 1;
		double sumX = 0.0;
		double sumY = 0.0;
		for (int j = a; j < b + 1; j++) {
			sumX += xi[j];
			sumY += yi[j];
		}
	}
	
	return GaussNewtonPolyN(xj, yj, n);
}

