#pragma once

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <array>
#include <iostream>
#include <string>
#include <vector>
#include <tuple>
#include <limits>
#include <fstream>
#include <math.h>
#include <cstdlib>
#include <conio.h>
#include <queue>
#include <deque>
#include <cmath>
#include <utility>
#include <algorithm>
#include <ranges>

#include "ImgProcTypes.h"
#include "DeltaTime.h"
#include "FrameRate.h"
#include "Utils.h"
#include "AppSettings.h"
#include "LinearRegression.h"
#include "ImgProcHelpers.h"

Poly2 ToPoly2(PolyN in) {
	return Poly2{ in.a, in.b };
}
Poly3 ToPoly3(PolyN in) {
	return Poly3{ in.a, in.b , in.c };
}
Poly4 ToPoly4(PolyN in) {
	return Poly4{ in.a, in.b, in.c, in.d };
}
Poly5 ToPoly5(PolyN in) {
	return Poly5{ in.a, in.b, in.c, in.d, in.e };
}
inline bool ContainsCoord(std::vector<PixelCoord>& pixels, PixelCoord val) {
	for (PixelCoord px : pixels) {
		if (px.x == val.x && px.y == val.y) {
			return true;
		}
	}
	return false;
}
template <typename T>
inline T Subtract(T a, T b) {
	return T{ a.x - b.x, a.y - b.y };
}
inline bool PixelInBounds(PixelCoord* p, int width, int height)
{
	if ((p->x >= 0) && (p->x < width) && (p->y >= 0) && (p->y < height))
	{
		return true;
	}
	else
	{
		return false;
	}
}
inline bool PixelInBounds(PixelCoord p, int width, int height)
{
	if ((p.x >= 0) && (p.x < width) && (p.y >= 0) && (p.y < height))
	{
		return true;
	}
	else
	{
		return false;
	}
}
inline bool Walkable(cv::Mat& mat, PixelCoord in, int threshold)
{
	return PixelInBounds(in, mat.rows, mat.cols) ? bool(mat.at<cv::Vec3b>(in.x, in.y)[0] > threshold) : false;
}
inline bool Blocked(cv::Mat& mat, PixelCoord in, int threshold)
{
	return !PixelInBounds(in, mat.rows, mat.cols) ? bool(mat.at<cv::Vec3b>(in.x, in.y)[0] > threshold) : false;
}
inline bool IsSame(PixelCoord a, PixelCoord b) {
	return a.x == b.x && a.y == b.y;
}
inline bool IsSame(IntVec2 a, IntVec2 b) {
	return a.x == b.x && a.y == b.y;
}
inline IntLine SwapAB(IntLine input) {
	return IntLine{ input.b, input.a };
}
inline PixelCoord Add(PixelCoord a, PixelCoord b) {
	return PixelCoord{ a.x + b.x, a.y + b.y };
}
bool PointIsInCornerPinRect(CornerPinRect* rect, int x, int y)
{
	if (x > rect->x && x < rect->x + rect->w && y > rect->y && y < rect->y + rect->h)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void GenerateLightMasksFrame(cv::Mat* color, cv::Mat* mask, cv::Mat* output)
{
	cv::Mat colorCpy;
	cv::Mat maskCpy;

	color->convertTo(colorCpy, CV_8UC3);
	mask->convertTo(maskCpy, CV_8UC3);

	cv::multiply(colorCpy, maskCpy, *output, 1.0 / 255.0);
}
inline void ColorPixel(cv::Mat* img, PixelCoord px, int red, int green, int blue)
{
	(*img).at<cv::Vec3b>(px.y, px.x)[0] = blue;
	(*img).at<cv::Vec3b>(px.y, px.x)[1] = green;
	(*img).at<cv::Vec3b>(px.y, px.x)[2] = red;
}
inline void ColorPixel(cv::Mat* img, PixelCoord px, ColorRGBi c)
{
	(*img).at<cv::Vec3b>(px.y, px.x)[0] = c.blue;
	(*img).at<cv::Vec3b>(px.y, px.x)[1] = c.green;
	(*img).at<cv::Vec3b>(px.y, px.x)[2] = c.red;
}
inline void AddColorPixel(cv::Mat* img, PixelCoord px, ColorRGBi c)
{
	(*img).at<cv::Vec3b>(px.y, px.x)[0] += c.blue;
	(*img).at<cv::Vec3b>(px.y, px.x)[1] += c.green;
	(*img).at<cv::Vec3b>(px.y, px.x)[2] += c.red;
}
inline void MulColorPixel(cv::Mat* img, PixelCoord px, ColorRGBi c)
{
	(*img).at<cv::Vec3b>(px.y, px.x)[0] *= c.blue;
	(*img).at<cv::Vec3b>(px.y, px.x)[1] *= c.green;
	(*img).at<cv::Vec3b>(px.y, px.x)[2] *= c.red;
}

bool aInB(std::vector<PixelCoord>* a, PixelCoord xIntercept)
{
	for (PixelCoord& px : *a)
	{
		if ((px.x == xIntercept.x) && (px.y == xIntercept.y))
		{
			return true;
		}
	}
	return false;
}

inline float Distance(FloatVec2 a, FloatVec2 b) {
	float term1 = powf(a.x - b.x, 2);
	float term2 = powf(a.y - b.y, 2);
	return std::sqrtf(term1 + term2);
}

inline int Squared(int a) {
	return a * a;
}

inline ColorRGBi RandomColor()
{

	return ColorRGBi{ rand() % 256 , rand() % 256 , rand() % 256 };

}

inline bool Traverseable(cv::Mat* mat, PixelCoord* in, int* threshold)
{
	return PixelInBounds(in, mat->rows, mat->cols) ? bool(mat->at<cv::Vec3b>(in->x, in->y)[0] > *threshold) : false;
}
inline bool Traverseable(cv::Mat* mat, PixelCoord in, int threshold)
{
	return PixelInBounds(in, mat->rows, mat->cols) ? bool(mat->at<cv::Vec3b>(in.x, in.y)[0] > threshold) : false;
}

inline bool NumInRangeMinMax(int n, int min, int max)
{
	return (min <= n) && (max >= n);
}
inline bool NumInRangeMinMax(int n, Range r)
{
	return (r.x1 <= n) && (r.x2 >= n);
}

inline bool RangesIntersect(PerimeterPoint a0, PerimeterPoint a1, PerimeterPoint b0, PerimeterPoint b1)
{
	return NumInRangeMinMax(a0, b0, b1) || NumInRangeMinMax(a1, b0, b1) || NumInRangeMinMax(b0, a0, a1) || NumInRangeMinMax(b1, a0, a1);
}
inline bool IsPerimeter(int* left, int* center, int* right)
{
	return *center && ((*left + *right) == 1);
}

inline void ColorBlobsThresholded(cv::Mat* out, BlobFrame& frame, const int areaThreshold)
{
	for (ShapeDataRLE& blob : frame.blobs)
	{
		if (blob.area >= areaThreshold)
		{
			for (FillNodeIndex& iNode : blob.indices)
			{
				FillNode& node = frame.nodes[iNode.y][iNode.i];
				for (int x = node.x1; x < node.x2 + 1; x++)
				{
					ColorPixel(out, PixelCoord{ x , node.index.y }, blob.color.red, blob.color.green, blob.color.blue);
				}
			}
		}
	}
}
inline void ConnectColorFrameToFrame(cv::Mat* out, BlobFrame& frame, BlobFrame& lastFrame, int distThreshold, int areaThreshold)
{

	std::vector<ShapeDataRLE> lastBlobs = lastFrame.blobs;

	for (ShapeDataRLE& blob : frame.blobs)
	{
		if (blob.area >= areaThreshold)
		{
			ColorRGBi color = ColorRGBi{ 0,0,0 };

			for (int k = 0; k < lastBlobs.size(); k++)
			{
				ShapeDataRLE prev = lastBlobs[k];

				float xDist = blob.centerOfMass.x - prev.centerOfMass.x;
				float yDist = blob.centerOfMass.y - prev.centerOfMass.y;
				float magnitude = std::sqrt((xDist * xDist) + (yDist * yDist));

				float areaDifference = (prev.area - blob.area) * (prev.area - blob.area) / (prev.area - blob.area);

				bool distCheck = magnitude < distThreshold;
				bool areaCheck = areaDifference < areaThreshold;

				if (areaCheck && distCheck)
				{
					color = prev.color;
					for (FillNodeIndex& i : blob.indices)
					{
						FillNode& node = frame.nodes[i.y][i.i];
						for (int x = node.x1; x < node.x2 + 1; x++)
						{
							ColorPixel(out, PixelCoord{ x , node.index.y }, color.red, color.green, color.blue);
						}
					}
					break;
				}

			}
		}
	}
}
inline float MinRatio(int a, int xIntercept)
{
	return (a > static_cast<float>(xIntercept)) ? (a / static_cast<float>(xIntercept)) : (xIntercept / static_cast<float>(a));
}
/*inline float Distance(FloatVec2 a, FloatVec2 xIntercept)
{
	float x = a.x - xIntercept.x;
	float y = a.y - xIntercept.y;

	return std::sqrt((x * x) + (y * y));
}*/
inline float Distance(int aX, int aY, int bX, int bY)
{
	float x = abs(aX) - abs(bX);
	float y = abs(aY) - abs(bY);
	float dist = std::sqrt((x * x) + (y * y));

	return dist;
}
inline float ManhattanNonNegative(int aX, int aY, int bX, int bY)
{
	return abs(aX - bX) + abs(bX - bY);
}
inline float NormalizedCosBell(float n)
{
	float pi = 3.14159265359;
	return 1 - cos(pi / (n + 2));
}
inline float QuadraticBell(float n)
{
	// maybe doesnt branch -> might not be as good of an approximation of normal dist
	return 1 / (1 + (n * n));
}
inline float Score1D(float max, float a, float xIntercept, float g)
{
	float x = a - xIntercept;
	x *= x * g;

	return max * NormalizedCosBell(g * x);
}
inline float Score2D(float max, FloatVec2 a, FloatVec2 xIntercept, FloatVec2 w, float g)
{
	float x = a.x - xIntercept.x;
	float y = a.y - xIntercept.y;
	x *= x * w.x;
	y *= y * w.y;

	return max * NormalizedCosBell(g * (x + y));
}
inline float Score3D(float max, FloatVec3 a, FloatVec3 xIntercept, FloatVec3 w, float g)
{
	float x = a.x - xIntercept.x;
	float y = a.y - xIntercept.y;
	float z = a.z - xIntercept.z;
	x *= x * w.x;
	y *= y * w.y;
	z *= z * w.z;

	return max * NormalizedCosBell(g * (x + y + z));
}
inline float QuadScore1D(float max, float a, float xIntercept, float g)
{
	float x = a - xIntercept;
	x *= x * g;

	return max * NormalizedCosBell(g * x);
}
inline float QuadScore2D(float max, FloatVec2 a, FloatVec2 xIntercept, FloatVec2 w, float g)
{
	float x = a.x - xIntercept.x;
	float y = a.y - xIntercept.y;
	x *= x * w.x;
	y *= y * w.y;

	return max * NormalizedCosBell(g * (x + y));
}
inline float QuadScore3D(float max, FloatVec3 a, FloatVec3 xIntercept, FloatVec3 w, float g)
{
	float x = a.x - xIntercept.x;
	float y = a.y - xIntercept.y;
	float z = a.z - xIntercept.z;
	x *= x * w.x;
	y *= y * w.y;
	z *= z * w.z;

	return max * NormalizedCosBell(g * (x + y + z));
}

template<typename T> Stats StatsFor(std::vector<T> values) {
	Stats out;

	if (values.size() < 2) {
		return out;
	}

	out.count = values.size();
	out.min = std::numeric_limits<float>::max();
	out.max = std::numeric_limits<float>::min();

	for (T item : values) {
		float val = static_cast<float>(item);
		out.sum += val;
		if (val < out.min) {
			out.min = val;
		}
		if (val > out.max) {
			out.max = val;
		}
	}

	out.range = abs(out.max - out.min);
	out.mean = out.sum / out.count;

	float sumDeviationSquares = 0.0;
	for (T item : values) {
		sumDeviationSquares += powf(static_cast<float>(item) - out.mean, 2);
	}
	out.sd = sumDeviationSquares / out.count;
	out.cv = out.mean == 0 ? 0 : out.sd / out.mean;

	std::sort(values.begin(), values.end());
	out.median = values[values.size() / 2];
	int endQ1 = (values.size() / 2) - 1;
	float sumQ1 = 0.0;
	int countQ1 = 0;
	for (int i = 0; i < endQ1; i++) {
		countQ1++;
		sumQ1 += static_cast<float>(values[i]);
	}
	out.q1 = sumQ1 / (float)countQ1;

	int startQ3 = values.size() % 2 ? values.size() / 2 : (values.size() / 2) + 1;
	float sumQ3 = 0.0;
	int countQ3 = 0;
	for (int i = startQ3; i < values.size(); i++) {
		countQ3++;
		sumQ3 += static_cast<float>(values[i]);
	}
	out.q3 = sumQ3 / (float)countQ3;

	out.iqr = out.q3 - out.q1;
	out.skewness = out.sd == 0 ? 0 : (out.mean - out.median) / out.sd;

	return out;
}
float SampleIntensityNearestRGBU8(cv::Mat* src, float scaling, IntVec2 coords) {
	coords = IntVec2{ static_cast<int>(round(scaling * coords.x)), static_cast<int>(round(scaling * coords.y)) };
	cv::Vec3b pixel = src->at<cv::Vec3b>(coords.x, coords.y);
	return static_cast<float>(pixel[0] + pixel[1] + pixel[2]) / 3.0;
}
bool CoordInShapeRLE(ShapeRLE* shape, IntVec2 coords) {
	if (shape->rowRanges[coords.y].size()) {
		for (Range r : shape->rowRanges[coords.y]) {
			if (r.x1 <= coords.x && coords.x <= r.x2) {
				return true;
			}
		}
	}
	return false;
}
/*
FloatVec3 ComparisonShapesIntensityRGBU8(cv::Mat* aSrcColor, cv::Mat* bSrcColor, cv::Mat* aSrcThresh, cv::Mat* bSrcThresh, ShapeRLE* a, ShapeRLE* b, int downScaling) {

	// a is the reference, and so b is fit to a
	cv::Mat aPixels(cv::Size(a->height, a->width), CV_8UC3, cv::Scalar(0,0,0));
	cv::Mat bPixels(cv::Size(b->height, b->width), CV_8UC3, cv::Scalar(0,0,0));

	// populate pixel data
	// set pixel data for aPixels
	for (int y = 0; y < a->rowRanges.size(); y++) {
		for (Range r : a->rowRanges[y]) {
			int end = r.x2 + 1;
			for (int x = r.x1; x < end; x++) {
				aPixels.at<cv::Vec3b>(x - a->bounds.xMin, y - a->bounds.yMin) = aSrcColor->at<cv::Vec3b>(x, y);
			}
		}
	}
	// set pixel data for bPixels
	for (int y = 0; y < b->rowRanges.size(); y++) {
		for (Range r : b->rowRanges[y]) {
			int end = r.x2 + 1;
			for (int x = r.x1; x < end; x++) {
				bPixels.at<cv::Vec3b>(x - b->bounds.xMin, y - b->bounds.yMin) = bSrcColor->at<cv::Vec3b>(x, y);
			}
		}
	}

	// resize b to match a with bi-linear interpolation
	FloatVec2 scaling = { (a->width / b->width) / downScaling, (a->height / b->height) / downScaling };
	cv::resize(bPixels, bPixels, cv::Size(), scaling.x, scaling.y, cv::INTER_LINEAR);
	
	// sum color values
	FloatVec3 sumA = { 0.0, 0.0, 0.0 };
	FloatVec3 sumB = { 0.0, 0.0, 0.0 };
	FloatVec3 sumAB = { 0.0, 0.0, 0.0 };

	for (int x = 0; x < aPixels.cols; x++) {
		for (int y = 0; y < aPixels.rows; y++) {
			cv::Vec3b pxA = aPixels.at<cv::Vec3b>(x, y);
			cv::Vec3b pxB = bPixels.at<cv::Vec3b>(x, y);

			sumA.x += 
		}
	}
	
	

	return 0.0;
}*/

int CenterRelativeIntersection(ShapeRLE* a, ShapeRLE* b) {

	// shift y indexing on b by relative difference in center of mass rounded down
	IntVec2 offsetB = { b->areaCenter.y - a->areaCenter.y, b->areaCenter.x - a->areaCenter.x };

	int intersectionArea = 0;

	for (int aY = 0; aY < a->rowRanges.size(); aY++) {

		int bY = aY + offsetB.y;
		
		std::vector<Range>& aRanges = a->rowRanges[aY];
		std::vector<Range>& bRanges = b->rowRanges[bY];

		// per row sum the LOGICAL_AND results per pixel between (a,b) from its compressed RLE format
		for (int ai = 0, bi = 0; (ai < aRanges.size()) && (bi < bRanges.size());) {

			Range aRange = aRanges[ai];
			Range bRange = bRanges[bi];
			bRange.x1 += offsetB.x;
			bRange.x2 += offsetB.x;

			Range intersection = { std::max(aRange.x1, bRange.x1), std::min(aRange.x2, bRange.x2) };
			bool intersects = intersection.x2 < intersection.x1;
			intersectionArea += intersects * (intersection.x2 - intersection.x1);

			bool stepAB = aRange.x2 == bRange.x2;
			bool stepA = bRange.x2 > aRange.x2;
			bool stepB = aRange.x2 > bRange.x2;

			ai += stepAB || stepA;
			bi += stepAB || stepB;
		}
	}

	return intersectionArea;
}
/*
float CenterRelativeIntersectionCosineBellWeighted(ShapeRLE* a, ShapeRLE* b) {

	// shift y indexing on b by relative difference in center of mass rounded down
	IntVec2 offsetB = { b->areaCenter.y - a->areaCenter.y, b->areaCenter.x - a->areaCenter.x };

	int intersectionArea = 0;

	for (int aY = 0; aY < a->rowRanges.size(); aY++) {

		int bY = aY + offsetB.y;

		std::vector<Range>& aRanges = a->rowRanges[aY];
		std::vector<Range>& bRanges = b->rowRanges[bY];

		// per row sum the LOGICAL_AND results per pixel between (a,b) from its compressed RLE format
		for (int ai = 0, int bi = 0; ai < aRanges.size() && bi < bRanges.size();) {

			Range aRange = aRanges[ai];
			Range bRange = bRanges[bi];
			bRange.x1 += offsetB.x;
			bRange.x2 += offsetB.x;

			float r = Magnitude(FloatVec2{bRan})

			Range intersection = { std::max(aRange.x1, bRange.x1), std::min(aRange.x2, bRange.x2) };
			bool intersects = intersection.x2 < intersection.x1;
			intersectionArea += intersects * (intersection.x2 - intersection.x1);

			bool stepAB = aRange.x2 == bRange.x2;
			bool stepA = bRange.x2 > aRange.x2;
			bool stepB = aRange.x2 > bRange.x2;

			ai += stepAB || stepA;
			bi += stepAB || stepB;
		}
	}

	return intersectionArea;
}*/
float CompareShapesColor() {
	return 0.0;
}
float CompareShapesDerivative() {
	return 0.0;
}

inline float Velocity2D(float x0, float y0, float x1, float y1, float time) {
	return time * abs(sqrtf(pow(x0 - x1, 2) + pow(y0 - y1, 2)));
}
template <typename T> 
inline float Magnitude(T v) {
	return sqrtf(powf(static_cast<float>(v.x), 2) + pow(static_cast<float>(v.y), 2));
}
inline FloatVec2 UnitVector(FloatVec2 v) {
	float magnitude = Magnitude(v);
	return FloatVec2{ v.x / magnitude, v.y / magnitude };
}
inline FloatVec2 Rotate(FloatVec2 v, float degrees) {
	float radians = degrees * TO_RADIANS;
	float x = (v.x * cosf(radians)) - (v.y * sinf(radians));
	float y = (v.x * sinf(radians)) + (v.y * cosf(radians));
	return FloatVec2{ x, y };
}
inline FloatVec2 DirectionUnitVector(Float32SlopeInterceptRotation line) {
	return Rotate(UnitVector(FloatVec2{ 1.0, line.m }), line.degrees);
}
inline float PointDistance(FloatVec2 a, FloatVec2 b) {
	return std::sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}
inline float DistancePointLine(FloatVec2 p, Float32SlopeInterceptRotation l) {

	FloatVec2 p0 = Rotate(p, l.degrees);

	// converting y = mx + b  ->  ax + bx +c = 0
	float a = 1;
	float b = l.m;
	float c = l.b;

	// distance = |ax + by + c| / root(a^2 + b+2)
	return abs(a * p.x + b * p.y + c) / sqrtf(1 + pow(b, 2));
}