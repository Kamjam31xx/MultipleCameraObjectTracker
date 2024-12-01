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

#include "ImgProcTypes.h"
#include "DeltaTime.h"
#include "FrameRate.h"
#include "Utils.h"
#include "AppSettings.h"

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
bool PointIsInCornerPinRect(CornerPinRect* bounds, int x, int y)
{
	if (x > bounds->x && x < bounds->x + bounds->w && y > bounds->y && y < bounds->y + bounds->h)
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
	(*img).at<cv::Vec3b>(px.x, px.y)[0] = blue;
	(*img).at<cv::Vec3b>(px.x, px.y)[1] = green;
	(*img).at<cv::Vec3b>(px.x, px.y)[2] = red;
}
inline void ColorPixel(cv::Mat* img, PixelCoord px, ColorRGBi c)
{
	(*img).at<cv::Vec3b>(px.x, px.y)[0] = c.blue;
	(*img).at<cv::Vec3b>(px.x, px.y)[1] = c.green;
	(*img).at<cv::Vec3b>(px.x, px.y)[2] = c.red;
}
inline void AddColorPixel(cv::Mat* img, PixelCoord px, ColorRGBi c)
{
	(*img).at<cv::Vec3b>(px.x, px.y)[0] += c.blue;
	(*img).at<cv::Vec3b>(px.x, px.y)[1] += c.green;
	(*img).at<cv::Vec3b>(px.x, px.y)[2] += c.red;
}
inline void MulColorPixel(cv::Mat* img, PixelCoord px, ColorRGBi c)
{
	(*img).at<cv::Vec3b>(px.x, px.y)[0] *= c.blue;
	(*img).at<cv::Vec3b>(px.x, px.y)[1] *= c.green;
	(*img).at<cv::Vec3b>(px.x, px.y)[2] *= c.red;
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
	return PixelInBounds(in, mat->rows, mat->cols) ? bool(mat->at<cv::Vec3b>(in->x, in->y)[0] > * threshold) : false;
}
inline bool Traverseable(cv::Mat* mat, PixelCoord in, int threshold)
{
	return PixelInBounds(in, mat->rows, mat->cols) ? bool(mat->at<cv::Vec3b>(in.x, in.y)[0] > threshold) : false;
}

inline bool NumInRangeMinMax(int n, int min, int max)
{
	return (min <= n) && (max >= n);
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
	return (a > xIntercept) ? (a / xIntercept) : (xIntercept / a);
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

Stats StatsFor(std::vector<float> values) {
	
	Stats out;

	out.count = values.size();
	out.min = std::numeric_limits<float>::max();
	out.max = std::numeric_limits<float>::min();
	out.median = values[values.size() / 2];

	for (float val : values) {
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
	for (float val : values) {
		sumDeviationSquares += pow(val - out.mean, 2);
	}
	out.sd = sumDeviationSquares / out.count;
	out.cv = out.sd / out.mean;

	std::sort(values.begin(), values.end());
	int endQ1 = (values.size() / 2) - 1;
	int sumQ1 = 0.0;
	int countQ1 = 0;
	for (int i = 0; i < endQ1; i++) {
		countQ1++;
		sumQ1 += values[i];
	}
	out.q1 = sumQ1 / countQ1;

	int startQ3 = values.size() % 2 ? values.size() / 2 : (values.size() / 2) + 1;
	int sumQ3 = 0.0;
	int countQ3 = 0;
	for (int i = startQ3; i < values.size(); i++) {
		countQ3++;
		sumQ3 += values[i];
	}
	out.q3 = sumQ3 / countQ3;
	
	out.iqr = out.q3 - out.q1;
	out.skewness = (out.mean - out.median) / out.sd;
}

Stats StatsFor(std::vector<int> values) {

	Stats out;

	out.count = values.size();
	out.min = std::numeric_limits<float>::max();
	out.max = std::numeric_limits<float>::min();
	out.median = values[values.size() / 2];

	for (int val : values) {
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
	for (int val : values) {
		sumDeviationSquares += pow(val - out.mean, 2);
	}
	out.sd = sumDeviationSquares / out.count;
	out.cv = out.sd / out.mean;

	std::sort(values.begin(), values.end());
	int endQ1 = (values.size() / 2) - 1;
	int sumQ1 = 0.0;
	int countQ1 = 0;
	for (int i = 0; i < endQ1; i++) {
		countQ1++;
		sumQ1 += values[i];
	}
	out.q1 = sumQ1 / countQ1;

	int startQ3 = values.size() % 2 ? values.size() / 2 : (values.size() / 2) + 1;
	int sumQ3 = 0.0;
	int countQ3 = 0;
	for (int i = startQ3; i < values.size(); i++) {
		countQ3++;
		sumQ3 += values[i];
	}
	out.q3 = sumQ3 / countQ3;

	out.iqr = out.q3 - out.q1;
	out.skewness = (out.mean - out.median) / out.sd;
}

inline void Track2D(cv::Mat* imgOut, std::vector<TrackedShapeRLE>* tracking, std::vector<std::vector<ShapeRLE>> previous, ShapeRLE current, float timeDelta) {

	// for each shape currently tracked
	for (TrackedShapeRLE tracked : *tracking) {

		// calculate future
		// fit line with linear regression
		StatsShapeRLE


	}

	// in    ->    previously tracked, past frames, current frame, timeDeltas, descriminators like minTrackedTime

	/* thonk 1
	// calculate futures of previous with successful tracking of frame count minTrackedTime
	

	// match current to calculated future


	// initialize tracking on leftovers 


	// 
	*/

	/* thonk 2
	// for each previously tracked blob, look for match in current frame -> possible hash insertie boi 
	//																		thonk reducer for blobie 
	//																		boi searchie thonkz

	// for each remaining tracked blob that wasnt tracked into the current frame -> use alternative match finding method

	// try simple tracking with some discrimination for acceptance 

	// for remaining -> try overlay blobie thonk matchinator boiz (potentially periodic differing from main stuffs)
	
	*/

	


}













// float deltaTime, int areaCheckThreshold, float distScale, float posMod, float areaMod, float rectMod, float rectRatioMod, float postModPAR, float discard, float accept
inline void ColorTrack_Test(cv::Mat* _out, BlobFrame& _last, BlobFrame& _now, float _dt, TrackingSettings _tracking)
{

	struct ScoreIdx
	{
		int j;
		float n;
	};

	std::vector<bool> initialized = std::vector<bool>(_now.blobs.size(), false);
	std::vector<std::vector<ScoreIdx>> scores = std::vector<std::vector<ScoreIdx>>(_now.blobs.size(), std::vector < ScoreIdx >());
	std::vector<float> velocities = std::vector<float>(_now.blobs.size(), 0.0);
	std::vector<FloatVec2> moves = std::vector<FloatVec2>(_now.blobs.size(), FloatVec2{ 0.0 , 0.0 });
	std::vector<float> variance = std::vector<float>(_now.blobs.size(), 0.0);

	// scoring connections i'th and j'th
	for (int i = 0; i < _now.blobs.size(); i++)
	{
		ShapeDataRLE& cur = _now.blobs[i];
		//  per blob  ->  compare against previous blobs
		if (cur.area >= _tracking.areaMinSize)
		{
			for (int j = 0; j < _last.blobs.size(); j++)
			{
				ShapeDataRLE& old = _last.blobs[j];

				if (old.area >= _tracking.areaMinSize)
				{
					FloatVec2& centerA = cur.centerOfArea;
					FloatVec2& centerB = old.centerOfArea;

					Rectangle& rectA = cur.rect;
					Rectangle& rectB = old.rect;

					FloatVec2 dimA = FloatVec2{ float(rectA.xMax - rectA.xMin), float(rectA.yMax - rectA.yMin) };
					FloatVec2 dimB = FloatVec2{ float(rectB.xMax - rectB.xMin), float(rectB.yMax - rectB.yMin) };

					float posWeight = 10.0;
					float areaWeight = 10.0;
					float rectWeight = 10.0;

					float wMod = 1.0; // wRatio* aSize.width + 1.0;
					float hMod = 1.0; // hRatio* aSize.height + 1.0; 

					float posScore = QuadScore2D(posWeight, centerA, centerB, FloatVec2{ wMod , hMod }, _tracking.distMod);
					float areaScore = QuadScore1D(areaWeight, cur.area, old.area, _tracking.areaMod);
					float rectScore = QuadScore2D(rectWeight, dimA, dimB, FloatVec2{ 1.0,1.0 }, _tracking.rectMod);

					float n = (_tracking.postMod * posScore * (areaScore + rectScore)) + (_tracking.distScale * posScore);

					// generate vector OR vectors or curve to represent the past path of the blob
					// v1 - v2 = vector between them
					// score this vector, scaling each and summing. 
					// get velocity and direction & add in delta time so its not funked.

					// velocity
					FloatVec2 v1 = FloatVec2{ old.centerOfArea.x - cur.centerOfArea.x, old.centerOfArea.y - cur.centerOfArea.y };
					float velocity2D = _dt * sqrt((v1.x * v1.x) + (v1.y * v1.y));

					if (n > _tracking.discardThreshold)
					{
						initialized[i] = true;
						scores[i].push_back(ScoreIdx{ j, n });
						velocities[i] = velocity2D;
						moves[i] = v1;
					}
					else
					{

					}
				}
			}
		}
	}

	for (int i = 0; i < initialized.size(); i++)
	{
		if (initialized[i])
		{
			int j = -1;
			int biggest = 0;
			FloatVec2 move = FloatVec2{ 0.0 , 0.0 };
			float velocity = 0.0;
			for (int k = 0; k < scores[i].size(); k++)
			{
				ScoreIdx& score = scores[i][k];
				if (score.n > biggest)
				{
					biggest = score.n;
					j = score.j;


					if (biggest > _tracking.acceptThreshold)
					{
						break;
					}
				}
			}
			if (j != -1)
			{
				ShapeDataRLE& blob = _now.blobs[i];
				blob.color = _last.blobs[j].color;

				if (blob.render)
				{
					for (FillNodeIndex vert : blob.indices)
					{
						FillNode node = _now.nodes[vert.y][vert.i];
						for (int x = node.x1; x < node.x2 + 1; x++)
						{
							ColorPixel(_out, PixelCoord{ x , node.index.y }, blob.color.red, blob.color.green, blob.color.blue);
						}
					}

					cv::Point centerOld = cv::Point{ (int)_last.blobs[j].centerOfArea.y , (int)_last.blobs[j].centerOfArea.x };
					cv::Point centerNow = cv::Point{ (int)_now.blobs[i].centerOfArea.y , (int)_now.blobs[i].centerOfArea.x };
					cv::line(
						*_out,
						centerOld,
						centerNow,
						cv::Scalar(70, 255, 5),
						4,
						cv::LINE_4
					);
				}
			}
		}
	}
}
inline void ConnectBlobsTemporal(cv::Mat* out, std::deque<BlobFrame>& in, int distThreshold, int areaThreshold, int rectRatioThreshold/*, int sizeThreshold, int rectOverlapThreshold*/)
{
	// simple shortest path with areaDifferenceThreshold && sizeDifferenceThreshold && positionThreshold && etc..
	// no connection neighbors from same frame to find a match yet. 

	BlobFrame& now = in.back();
	for (int i = in.size() - 1; i > 0; i--)
	{
		BlobFrame& past = in[i];

		// implement buffer for post processing - for now jsut do the dumb shit n use first found for testing
		for (ShapeDataRLE& p : past.blobs)
		{
			ColorRGBi& color = p.color;
			for (ShapeDataRLE& n : now.blobs)
			{
				float xDist = n.centerOfMass.x - p.centerOfMass.x;
				float yDist = n.centerOfMass.y - p.centerOfMass.y;
				float magnitude = std::sqrt((xDist * xDist) + (yDist * yDist));

				float areaDifference = abs(p.area - n.area);

				float ratioArea = p.area > n.area ? p.area / n.area : n.area / p.area;
				float ratioAreaThreshold = areaThreshold * (100.0 / areaThreshold);

				float scaledDistThreshold = ratioArea * distThreshold;

				bool distCheck = magnitude < scaledDistThreshold;
				bool areaCheck = ratioArea < ratioAreaThreshold;

				//int checks = distCheck + areaCheck + rectCheck;
				if (areaCheck && distCheck)
				{
					// color the blob
					n.color = color;
					for (FillNodeIndex& iNode : n.indices)
					{
						FillNode& node = now.nodes[iNode.y][iNode.i];
						for (int x = node.x1; x < node.x2 + 1; x++)
						{

							ColorPixel(out, PixelCoord{ x , node.index.y }, color.red, color.green, color.blue);
						}
					}
					break;
				}
			}
		}
		break;
	}

}


// walk 1 pixel to start
// inset from -x 
// -x || LEFT == black
// came from LEFT 
// came from BLACK
// -y guaranteed black

// possible whites = +x , +y
// clock wise -> check +x first, then +y 
// if +x & +y == black ----> is a single pixel

// SEARCH ORDER CLOCKWISE
// +x
//  -y
//   -x
//    +y

// check first pixel to set state
// always start look from -1 in the counter clockwise position

// walk perimeter  COUNTER CLOCKWISE  -by searching-   CLOCKWISE
// 16 possible cases total, only will see 14 in loop

std::vector<PixelCoord> PerimeterPixels(cv::Mat& img, PixelCoord start)
{
	// store outline pixels
	std::vector<PixelCoord> outlinePx = std::vector<PixelCoord>(0);

	// set possible walk & look directions counter-clockwise
	PixelCoord right = PixelCoord{ 1, 0 };
	PixelCoord down = PixelCoord{ 0, -1 };
	PixelCoord left = PixelCoord{ -1, 0 };
	PixelCoord up = PixelCoord{ 0, 1 };
	std::array<PixelCoord, 4> moves = { right, down, left, up };
	enum direction { RIGHT = 0, DOWN = 1, LEFT = 2, UP = 3 };

	// walk perimeter and store pixels
	PixelCoord on = start;
	direction from = LEFT;
	while (true) {

		direction adjacent = from;
		PixelCoord coord = {0,0};
		int turnsCCW = 0;
		int turnsCW = 0;
		
		// rotate look direction CCW until blocked
		for (int rotate = 1; rotate != 5; rotate++) {
			adjacent = direction((from + rotate) % 4);
			coord = Add(on, moves[adjacent]);
			if (Blocked(img, coord, 127)) {
				turnsCCW = rotate;
				break;
			} 
		}

		// rotate look direction CW until open
		for (int rotate = 1; rotate != 5; rotate++) {
			adjacent = direction((from + 4 - rotate) % 4);
			coord = Add(on, moves[adjacent]);
			if (Walkable(img, coord, 127)) {
				turnsCW = rotate;
				break;
			}
		}

		// check for single pixel
		if (turnsCW == 4) {
			outlinePx.push_back(coord);
			break;
		}

		// check for imporper start coordinate -> only insert from scan line method incrementing x
		if (turnsCCW == 4) {
			outlinePx.push_back(coord);
			break;
		}

		// typical cases
		bool noTurn = from == direction((adjacent + 2) % 4);
		bool rightTurn = from == direction((adjacent + 1) % 4);
		if (noTurn || rightTurn) {
			if (!ContainsCoord(outlinePx, on)) {
				outlinePx.push_back(on);
			}
		}

		// advance to next pixel
		on = coord;
		from = direction((adjacent + 2) % 4);
		bool returnedToHome = IsSame(on, start);
		if (returnedToHome) {
			break;
		}
	}

	return outlinePx;
}

// for each pixel, test left and right pixels color to determine if its a perimeter
// pixel for a shape with respect to the x axis.
std::vector<std::vector<PerimeterPoint>> PerimeterPointsAxisX(cv::Mat* in, int colorMax, int xMax, int yMax) {

	int colorStep = round(colorMax / 2) - 1;
	int xLim = in->rows - 1;
	int yLim = in->cols;
	std::vector<std::vector<PerimeterPoint>> points(in->cols, std::vector<PerimeterPoint>());

	//use for non branching code in the future, using resulting bool to multiple the index. index 0 will be junk to discard
	//std::vector<int> ySize = std::vector<int>(in->cols);

	// handle left edge at x = 0
	for (int y = 0; y < yLim; y++) {
		if ((in->at<cv::Vec3b>(0, y)[0] == colorMax) && (in->at<cv::Vec3b>(1, y)[0] == colorMax)) {
			points[y].push_back(PerimeterPoint(0));
		}
	}

	// handle inbetween left and right edge
	for (int x = 1; x < xLim; x++) {
		for (int y = 0; y < yLim; y++) {
			if ((in->at<cv::Vec3b>(x, y)[0] == colorMax) && ((!(in->at<cv::Vec3b>(x - 1, y)[0] == colorMax) + !(in->at<cv::Vec3b>(x + 1, y)[0] == colorMax)) == 1)) {
				points[y].push_back(PerimeterPoint{ x });
			}
		}
	}

	// handle right edge at x = xLim 
	for (int y = 0; y < yLim; y++) {
		if ((in->at<cv::Vec3b>(xLim, y)[0] == colorMax) && (in->at<cv::Vec3b>(xLim - 1, y)[0] == colorMax)) {
			points[y].push_back(PerimeterPoint(xLim));
		}
	}

	return points;
}
// for each pixel, test left and right pixels color to determine if its a perimeter
// pixel for a shape with respect to the x axis.
std::vector<std::vector<PerimeterPoint>> PerimeterPointsAxisX_Expirimental(cv::Mat* in, int colorMax, int xMax, int yMax) {

	int colorStep = round(colorMax / 2) - 1;
	int xLim = in->rows - 1;
	int yLim = in->cols;
	std::vector<std::vector<PerimeterPoint>> points(in->cols, std::vector<PerimeterPoint>(xMax / 2));
	std::vector<int> ySize = std::vector<int>(in->cols);
	for (int i = 0; i < in->cols; i++) {
		ySize[i] = 1;
		points[i][0] = 0;
	}

	// handle left edge at x = 0
	for (int y = 0; y < yLim; y++){
		bool record = (in->at<cv::Vec3b>(0, y)[0] == colorMax) && (in->at<cv::Vec3b>(1, y)[0] == colorMax);
		points[y][ySize[y] * record] = 0;
		ySize[y] += record;
	}

	// handle inbetween left and right edge
	for (int x = 1; x < xLim; x++){
		for (int y = 0; y < yLim; y++){
			bool record = (in->at<cv::Vec3b>(x, y)[0] == colorMax) && ((!(in->at<cv::Vec3b>(x - 1, y)[0] == colorMax) + !(in->at<cv::Vec3b>(x + 1, y)[0] == colorMax)) == 1);
			points[y][ySize[y] * record] = x;
			ySize[y] += record;
		}
	}

	// handle right edge at x = xLim 
	for (int y = 0; y < yLim; y++){
		bool record = (in->at<cv::Vec3b>(xLim, y)[0] == colorMax) && (in->at<cv::Vec3b>(xLim - 1, y)[0] == colorMax);
		points[y][ySize[y] * record] = xLim;
		ySize[y] += record;
	}

	// move to result while discarding junk data
	std::vector<std::vector<PerimeterPoint>> result(in->cols, std::vector<PerimeterPoint>(0));
	for (int y = 0; y < in->cols; y++) {
		std::vector<int>::const_iterator begin = points[y].begin() + 1;
		std::vector<int>::const_iterator end = points[y].begin() + ySize[y];
		result[y] = std::vector<PerimeterPoint>(begin, end);
	}

	return result;
}

inline std::vector<ShapeRLE> ExtractShapesRLE(cv::Mat* in) {

	// stores state
	BlobFrame frame = BlobFrame{};

	// max color value and threshold for fill
	int max = 255;
	int colorStep = round(max / 2) - 1;

	// padding to stop the kernel from trying to read indices at -1 and width or height which are out of bounds
	int xLim = in->rows - 1;
	int yLim = in->cols;


	std::vector<std::vector<PerimeterPoint>> points(in->cols, std::vector<PerimeterPoint>());
	for (int y = 0; y < yLim; y++)
	{

		if ((in->at<cv::Vec3b>(0, y)[0] == max) && (in->at<cv::Vec3b>(1, y)[0] == max))
		{
			points[y].push_back(PerimeterPoint(0));
		}
	}

	for (int x = 1; x < xLim; x++)
	{
		for (int y = 0; y < yLim; y++)
		{
			if ((in->at<cv::Vec3b>(x, y)[0] == max) && ((!(in->at<cv::Vec3b>(x - 1, y)[0] == max) + !(in->at<cv::Vec3b>(x + 1, y)[0] == max)) == 1))
			{
				points[y].push_back(PerimeterPoint{ x });
			}
		}
	}

	for (int y = 0; y < yLim; y++)
	{
		if ((in->at<cv::Vec3b>(xLim, y)[0] == max) && (in->at<cv::Vec3b>(xLim - 1, y)[0] == max))
		{
			points[y].push_back(PerimeterPoint(xLim));
		}
	}

	int id = 0;
	for (int y = 0; y < points.size(); y++)
	{
		for (int i = 0; i < points[y].size(); i += 2)
		{
			frame.nodes[y].push_back(FillNode{ false, false, id++, FillNodeIndex{ y, i / 2 }, points[y][i], points[y][i + 1], points[y][i + 1] - points[y][i], std::vector<int>(), std::vector<FillNodeIndex>() });
			frame.indices.push_back(FillNodeIndex{ y, i / 2 });
		}
	}

	for (int y = 0; y < frame.nodes.size() - 1; y++)
	{
		int y2 = y + 1;
		for (int i = 0; i < frame.nodes[y].size(); i++)
		{
			for (int k = 0; k < frame.nodes[y2].size(); k++)
			{
				if (RangesIntersect(frame.nodes[y][i].x1, frame.nodes[y][i].x2, frame.nodes[y2][k].x1, frame.nodes[y2][k].x2))
				{
					frame.nodes[y][i].connections.push_back(frame.nodes[y2][k].index);
					frame.nodes[y2][k].connections.push_back(frame.nodes[y][i].index);
				}
			}
		}
	}

	std::vector<FillNodeIndex> open;
	for (int s = 0; s < frame.indices.size(); s++)
	{
		ShapeDataRLE blob;
		open.push_back(frame.indices[s]);

		while (open.size())
		{
			FillNodeIndex index = open.back();
			open.pop_back();

			if (frame.nodes[index.y][index.i].walked == false)
			{
				blob.indices.push_back(index);
				frame.nodes[index.y][index.i].walked = true;

				for (int c = 0; c < frame.nodes[index.y][index.i].connections.size(); c++)
				{
					open.push_back(frame.nodes[index.y][index.i].connections[c]);
				}
			}
		}
		frame.blobs.push_back(blob);
	}

	for (int c = 0; c < frame.blobs.size(); c++)
	{
		ColorRGBi color = RandomColor();
		frame.blobs[c].color = color;
		std::vector<FillNodeIndex>& group = frame.blobs[c].indices;
		float xTotal = 0.0;
		float yTotal = 0.0;
		float coords = 0.0;

		int xMin = 31000;
		int yMin = 31000;
		int xMax = 0;
		int yMax = 0;
		for (int w = 0; w < group.size(); w++)
		{
			FillNode& node = frame.nodes[group[w].y][group[w].i];
			float nodeArea = (node.x2 - node.x1) + 1;
			frame.blobs[c].area += nodeArea;

			if (node.x1 < xMin)
			{
				xMin = node.x1;
			}
			if (node.x2 > xMax)
			{
				xMax = node.x2;
			}
			if (group[w].y < yMin)
			{
				yMin = group[w].y;
			}
			if (group[w].y > yMax)
			{
				yMax = group[w].y;
			}
		}

		frame.blobs[c].centerOfArea = FloatVec2{ xTotal / coords, yTotal / coords };
		frame.blobs[c].rect = Rectangle{ xMin, yMin, xMax, yMax };
		Rectangle& rect = frame.blobs[c].rect;
		frame.blobs[c].size = RectangleSize{ rect.xMax - rect.xMin , rect.yMax - rect.yMin };
	}

	std::vector<ShapeRLE> shapes = std::vector<ShapeRLE>(frame.blobs.size(), ShapeRLE{});

	for (ShapeDataRLE b : frame.blobs) {
		
		ShapeRLE shape = ShapeRLE{};
		shape.rowRanges = std::vector<std::vector<Range>>(b.size.height);
		shape.area = b.area;
		shape.areaCenter = b.centerOfArea;
		shape.bounds = b.rect;
		shape.width = b.size.width;
		shape.height = b.size.height;

		for (FillNodeIndex f : b.indices) {
			FillNode n = frame.nodes[f.y][f.i];
			Range r = { n.x1, n.x2 };
			shape.rowRanges[f.y - shape.bounds.yMin].push_back(r);
		}

		for (std::vector<Range>& row : shape.rowRanges) {
			std::vector<std::tuple<int, int>> sorting = std::vector<std::tuple<int, int>>(row.size());
			for (int i = 0; i < sorting.size(); i++) {
				sorting[i] = std::make_tuple(row[i].x1, i);
			}
			std::sort(sorting.begin(), sorting.end());
			std::vector<Range> sorted = std::vector<Range>(0);
			for (std::tuple<int, int> mapping : sorting) {
				int index = std::get<1>(mapping);
				Range r = row[index];
				sorted.push_back(r);
			}
			row = sorted;
		}

		shapes.push_back(shape);
	}

	return shapes;
}

inline void GetBlobs(cv::Mat* in, cv::Mat* out, int max, int minSize, BlobFrame& frame)
{
	int colorStep = round(max / 2) - 1;
	int xLim = in->rows - 1;
	int yLim = in->cols;

	std::vector<std::vector<PerimeterPoint>> points = PerimeterPointsAxisX(in, max, in->rows - 1, in->cols);

	// push the points into "nodes" and store them.
	int id = 0;
	for (int y = 0; y < points.size(); y++){
		for (int i = 0; i < points[y].size(); i += 2){
			frame.nodes[y].push_back(FillNode{ false, false, id++, FillNodeIndex{ y, i / 2 }, points[y][i], points[y][i + 1], points[y][i + 1] - points[y][i], std::vector<int>(), std::vector<FillNodeIndex>() });
			frame.indices.push_back(FillNodeIndex{ y, i / 2 });
		}
	}

	// relate the nodes if the ranges intersect for y adjacent ranges.
	// note : add x-most pixel from last span in the row to remove "walked" bool branching
	for (int y = 0; y < frame.nodes.size() - 1; y++){
		int y2 = y + 1;
		for (int i = 0; i < frame.nodes[y].size(); i++){
			for (int k = 0; k < frame.nodes[y2].size(); k++){
				if (RangesIntersect(frame.nodes[y][i].x1, frame.nodes[y][i].x2, frame.nodes[y2][k].x1, frame.nodes[y2][k].x2)){
					frame.nodes[y][i].connections.push_back(frame.nodes[y2][k].index);
					frame.nodes[y2][k].connections.push_back(frame.nodes[y][i].index);
				}
			}
		}
	}

	// dafuq
	std::vector<FillNodeIndex> open;
	for (int s = 0; s < frame.indices.size(); s++){
		ShapeDataRLE blob;
		open.push_back(frame.indices[s]);
		while (open.size()){
			FillNodeIndex index = open.back();
			open.pop_back();
			if (frame.nodes[index.y][index.i].walked == false){
				blob.indices.push_back(index);
				frame.nodes[index.y][index.i].walked = true;
				for (int c = 0; c < frame.nodes[index.y][index.i].connections.size(); c++){
					open.push_back(frame.nodes[index.y][index.i].connections[c]);
				}
			}
		}
		frame.blobs.push_back(blob);
	}

	// get bounding rects & other stuff --- not setting mass and density cuz dont have info here
	// color for funsies
	for (int c = 0; c < frame.blobs.size(); c++){
		ColorRGBi color = RandomColor();
		frame.blobs[c].color = color;
		std::vector<FillNodeIndex>& group = frame.blobs[c].indices;
		float xTotal = 0.0;
		float yTotal = 0.0;
		float coords = 0.0;

		int xMin = 31000;
		int yMin = 31000;
		int xMax = 0;
		int yMax = 0;
		for (int w = 0; w < group.size(); w++)
		{
			FillNode& node = frame.nodes[group[w].y][group[w].i];
			float nodeArea = (node.x2 - node.x1) + 1;
			frame.blobs[c].area += nodeArea;

			if (node.x1 < xMin)
			{
				xMin = node.x1;
			}
			if (node.x2 > xMax)
			{
				xMax = node.x2;
			}
			if (group[w].y < yMin)
			{
				yMin = group[w].y;
			}
			if (group[w].y > yMax)
			{
				yMax = group[w].y;
			}
			for (int x = node.x1; x < node.x2 + 1; x++)
			{
				ColorPixel(out, PixelCoord{ x , node.index.y }, 255, 255, 255);
				xTotal += x;
				yTotal += node.index.y;
				coords++;
			}
		}

		frame.blobs[c].centerOfArea = FloatVec2{ xTotal / coords, yTotal / coords };
		frame.blobs[c].rect = Rectangle{ xMin, yMin, xMax, yMax };
		Rectangle& rect = frame.blobs[c].rect;
		frame.blobs[c].size = RectangleSize{ rect.xMax - rect.xMin , rect.yMax - rect.yMin };

		if (frame.blobs[c].area > minSize)
		{
			frame.blobs[c].render = true;
			for (int x = rect.xMin; x < rect.xMax; x++)
			{
				ColorPixel(out, PixelCoord{ x	, rect.yMin }, 255, 0, 0);
				ColorPixel(out, PixelCoord{ x	, rect.yMax }, 255, 0, 0);
			}
			for (int y = rect.yMin; y < rect.yMax; y++)
			{
				ColorPixel(out, PixelCoord{ rect.xMin, y }, 255, 0, 0);
				ColorPixel(out, PixelCoord{ rect.xMax, y }, 255, 0, 0);
			}
		}
		else
		{
			frame.blobs[c].render = false;
		}
	}
}

FloatVec3 RelativeDirection()
{
	return FloatVec3{ 0.0, 0.0, 0.0 };
}

void ConnectStereoFrames(cv::Mat* _outLeft, cv::Mat* _outRight, BlobFrame _left, BlobFrame _right, float _fov, float _parallax, FloatVec2 _sensorSize, FloatVec3 _rotationLeft, FloatVec3 _rotationRight)
{

}







/*
void GetPerimeterBufferFromPixel(cv::Mat* _in, cv::Mat* _out, PixelCoord _start, int _threshold)
{

	cv::Mat imgOut;
	_in->copyTo(imgOut);

	// walk 1 pixel to start
	// inset from -x 
	// -x || LEFT == black
	// came from LEFT 
	// came from BLACK
	// -y guaranteed black

	// possible whites = +x , +y
	// clock wise -> check +x first, then +y 
	// if +x & +y == black ----> is a single pixel

	// SEARCH ORDER CLOCKWISE
	// +x
	//  -y
	//   -x
	//    +y

	// check first pixel to set state
	// always start look from -1 in the counter clockwise position

	direction look = NONE;

	PixelCoord positiveX = PixelCoord{ _start.x + 1 , _start.y };
	PixelCoord positiveY = PixelCoord{ _start.x     , _start.y + 1 };
	PixelCoord pixel = PixelCoord{ _start.x, _start.y };
	std::vector<std::vector<PerimeterPoint>> points(_in->cols, std::vector<PerimeterPoint>());

	if (Traverseable(_in, positiveX, _threshold))
	{
		log("x start");
		look = NEG_Y;
		pixel.x += 1;
	}
	else if (Traverseable(_in, positiveY, _threshold))
	{
		log("y start");
		look = POS_X;
		pixel.y += 1;
	}
	else // is single pixel
	{
		// push 2 counts & break
	}

	std::array<PixelCoord, 4> moves =
	{
		PixelCoord{ 1, 0 },
		PixelCoord{ 0, -1},
		PixelCoord{ -1, 0},
		PixelCoord{ 0, 1 },
	};

	// walk perimeter  COUNTER CLOCKWISE  -by searching-   CLOCKWISE
	// 16 possible cases total, only will see 14 in loop
	int count = 0;
	bool search = true;
	log("loop start");
	while (search && (count < 1000))
	{
		log("count = " + str(count));
		count++;
		int blocks = 0;
		int xBlocks = 0;
		for (int i = 0; i < 4; i++)
		{
			direction n = direction((look + i) % 4);

			PixelCoord neighbor = PixelCoord{ pixel.x + moves[n].x, pixel.y + moves[n].y };

			bool axisX = !(n % 2);

			if (Traverseable(_in, &neighbor, &_threshold))
			{
				for (int k = xBlocks; k != 0; k--)
				{
					points[pixel.y].push_back(pixel.x);
					ColorPixel(_out, pixel, 255, 255, 0);
				}
				look = direction((n + 3) % 4);
				if (true)
				{
					search = false;
				}
				pixel = neighbor;
				break;
			}
			else
			{
				blocks++;
				xBlocks += int(axisX);
			}
		}
		if (blocks == 4)
		{
			break;
		}
	}
}
*/
/*
void GetLightBlobs(cv::Mat* _mask, cv::Mat* _out, int _threshold)
{
	std::vector<CornerPinRect> boundingBoxes; // add size prediction based off past frames
	cv::Mat result;
	_mask->copyTo(result);


	for (int x = 1; x < (result.rows - 1); x++)
	{
		bool exitLoop = false;
		for (int y = 1; y < (result.cols - 1); y++)
		{

			if (_mask->at<cv::Vec3b>(x, y)[0] >= 10) //&& visited[x][y] == false)
			{
				GetPerimeterBufferFromPixel(_mask, &result, PixelCoord{ x,y }, _threshold);
				exitLoop = true;
				break;
			}
			else
			{
				ColorPixel(&result, PixelCoord{ x,y }, 50, 10, 10);
			}
		}
		if (exitLoop)
		{
			break;
		}
	}
	result.copyTo(*_out);
}
*/


inline void ConnectFillNodeAboveBelow(FillNode* _above, FillNode* _below)
{
	_below->connections.push_back(_above->index);
	_above->connections.push_back(_below->index);
}

// MUST BE GIVEN THRESHOLDED IMAGE
void SetSpanBufferPixels(cv::Mat* _in, cv::Mat* _out, int _threshold, int _max)
{

	int colorStep = round(_max / 2) - 1;
	int xLim = _in->rows - 1;
	int yLim = _in->cols;

	// check boundary pixels at edge of image
	for (int y = 0; y < yLim; y++)
	{
		int scalarMin = !(_in->at<cv::Vec3b>(1, y)[0] == _max) + 1;
		int scalarMax = !(_in->at<cv::Vec3b>(xLim - 1, y)[0] == _max) + 1;
		if (_in->at<cv::Vec3b>(0, y)[0] == _max)
		{
			ColorPixel(_out, PixelCoord{ 0,y }, scalarMin * colorStep, 0, 0);
		}
		if (_in->at<cv::Vec3b>(xLim, y)[0] == _max)
		{
			ColorPixel(_out, PixelCoord{ xLim,y }, scalarMax * colorStep, 0, 0);
		}
	}

	// check all other pixels 
	for (int x = 1; x < xLim; x++)
	{
		for (int y = 0; y < yLim; y++)
		{
			int scalar = !(_in->at<cv::Vec3b>(x - 1, y)[0] > _threshold) + !(_in->at<cv::Vec3b>(x + 1, y)[0] > _threshold);

			if (_in->at<cv::Vec3b>(x, y)[0] == _max)
			{
				ColorPixel(_out, PixelCoord{ x,y }, scalar * colorStep, 0, 0);
			}
		}
	}

	// store all the pixels that represent min && || max
	std::vector<std::vector<PerimeterPoint>> points(_in->cols, std::vector<PerimeterPoint>());
	for (int y = 0; y < yLim; y++)
	{
		bool onSpan = false;
		int spanSize = 0;
		for (int x = 0; x <= xLim; x++)
		{
			bool onPerimeter = _out->at<cv::Vec3b>(x, y)[2] == colorStep;

			if (onPerimeter)
			{
				points[y].push_back(PerimeterPoint(x));
				onSpan = !onSpan;
				spanSize += onPerimeter;
				if (onSpan == false)
				{

				}
			}
			else
			{
				spanSize += onSpan;
			}
			ColorPixel(_out, PixelCoord{ x,y }, onPerimeter * colorStep, onSpan * 255 * (!onPerimeter), 0);
		}
	}

	// move representations to nodes
	// store indexes of all nodes
	std::vector<FillNodeIndex> nodesIndexes;
	std::vector<std::vector<FillNode>> nodes(_in->cols, std::vector<FillNode>());
	int id = 0;
	for (int y = 0; y < points.size(); y++)
	{
		for (int i = 0; i < points[y].size(); i += 2)
		{
			nodes[y].push_back(FillNode{ false, false, id++, FillNodeIndex{ y, i / 2 }, points[y][i], points[y][i + 1], points[y][i + 1] - points[y][i], std::vector<int>(), std::vector<FillNodeIndex>() });
			nodesIndexes.push_back(FillNodeIndex{ y, i / 2 });
		}
	}

	for (int y = 0; y < nodes.size() - 1; y++)
	{
		int y2 = y + 1;
		for (int i = 0; i < nodes[y].size(); i++)
		{
			for (int k = 0; k < nodes[y2].size(); k++)
			{
				if (RangesIntersect(nodes[y][i].x1, nodes[y][i].x2, nodes[y2][k].x1, nodes[y2][k].x2))
				{
					nodes[y][i].connections.push_back(nodes[y2][k].index);
					nodes[y2][k].connections.push_back(nodes[y][i].index);
				}
			}
		}
	}

	std::vector<ShapeDataRLE> blobs;
	std::vector<FillNodeIndex> open;
	for (int s = 0; s < nodesIndexes.size(); s++)
	{
		ShapeDataRLE blob;
		open.push_back(nodesIndexes[s]);

		while (open.size())
		{
			FillNodeIndex index = open.back();
			open.pop_back();

			if (nodes[index.y][index.i].walked == false)
			{
				blob.indices.push_back(index);
				nodes[index.y][index.i].walked = true;

				for (int c = 0; c < nodes[index.y][index.i].connections.size(); c++)
				{
					open.push_back(nodes[index.y][index.i].connections[c]);
				}
			}
		}
		blobs.push_back(blob);
	}

	// color for funsies
	for (int c = 0; c < blobs.size(); c++)
	{
		std::vector<FillNodeIndex>& group = blobs[c].indices;
		ColorRGBi color = RandomColor();
		for (int w = 0; w < group.size(); w++)
		{
			FillNode& node = nodes[group[w].y][group[w].i];
			for (int f = 1; f < node.len; f++)
			{
				ColorPixel(_out, PixelCoord{ node.x1 + f , node.index.y }, color.red, color.green, color.blue);
			}
		}
	}
}


