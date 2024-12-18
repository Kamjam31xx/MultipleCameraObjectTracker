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
#include "CircularBuffer.h"



inline std::vector<TrackedShapeRLE> TrackSegments(CircularBuffer<float>* previousTimeDeltas, std::vector<TrackedShapeRLE>* tracking, CircularBuffer<std::vector<ShapeRLE>>* previous, std::vector<ShapeRLE>* current, float timeDelta, TrackingParameters params) {

	std::vector<TrackedShapeRLE> result = std::vector<TrackedShapeRLE>(0);

	// keep track of all unmatched shapes, removing them as matched and tracking into current frame
	std::vector<ShapeRLE*> unmatched = std::vector<ShapeRLE*>(); // note : use vector copy constructor
	for (ShapeRLE shape : (*current)) {
		unmatched.push_back(&shape);
	}

	// note : to-do -> remove indices from a list for the previous frame when found
	//				-> change indexing method and stuffs + abstract stuff away
	// 
	// for each previously tracked shape find using simple statistics stuffs
	/*std::vector<int> openTrackingIndices = std::vector<int>();
	for (int i = 0; i < (*tracking).size(); i++) {
		openTrackingIndices.push_back(i);
	}
	for (int i = 0; i < (*tracking).size(); i++) {


		TrackedShapeRLE tracked = (*tracking)[i];

		// if only tracked for 3
		if (tracked.shapePtrs.size() < 4) {
			continue;
		}

		// pack values from each frame the shape was tracked in
		std::vector<int> areas = std::vector<int>();
		std::vector<FloatVec2> areaCenters = std::vector<FloatVec2>();
		std::vector<int> widths = std::vector<int>();
		std::vector<int> heights = std::vector<int>();
		std::vector<Rectangle> rect = std::vector<Rectangle>();
		for (ShapeRLE* shapePtr : tracked.shapePtrs) {
			areas.push_back((*shapePtr).area);
			areaCenters.push_back((*shapePtr).areaCenter);
			widths.push_back((*shapePtr).width);
			heights.push_back((*shapePtr).height);
			rect.push_back((*shapePtr).rect);

		}

		// calc stats for variables
		Stats areaStats = StatsFor(areas);
		Stats widthStats = StatsFor(widths);
		Stats heightStats = StatsFor(heights);
		
		// fit line for direction to center of area
		Float32SlopeInterceptRotation areaCenterLine = LinearRegressionR2D1f(areaCenters);
		std::vector<float> positionDeviations = std::vector<float>();
		for (ShapeRLE* shapePtr : tracked.shapePtrs) {
			FloatVec2 relativePosition = (*shapePtr).areaCenter;
			float deviation = DistancePointLine(relativePosition, areaCenterLine);
		}
		Stats positionStats = StatsFor(positionDeviations);

		// calculate previous velocities and calc stats for velocity of the shape as it has been tracked
		std::vector<float> velocities = std::vector<float>(); 
		for (int j = 0; j + 1 < tracked.shapePtrs.size(); j++) {
			int x0 = (*tracked.shapePtrs[j]).areaCenter.x;
			int y0 = (*tracked.shapePtrs[j]).areaCenter.y;
			int x1 = (*tracked.shapePtrs[j + 1]).areaCenter.x;
			int y1 = (*tracked.shapePtrs[j + 1]).areaCenter.y;
			velocities.push_back(Velocity2D(x0, y0, x1, y1, (*previousTimeDeltas)[j + 1]));

		}
		Stats velocityStats = StatsFor(velocities);


		// for each shape in the current frame compare it to this shapes tracking history
		FloatVec2 direction = DirectionUnitVector(areaCenterLine);
		int x0 = (*tracked.shapePtrs[tracked.shapePtrs.size() - 1]).areaCenter.x;
		int y0 = (*tracked.shapePtrs[tracked.shapePtrs.size() - 1]).areaCenter.y;
		FloatVec2 futurePosition = { x0 + (direction.x * velocityStats.mean), y0 + (direction.y * velocityStats.mean) };
		for (int j = 0; j < unmatched.size(); j++) {

			ShapeRLE* open = unmatched[j];

			int x1 = (*open).areaCenter.x;
			int y1 = (*open).areaCenter.y;
			float velocity = Velocity2D(x0, y0, x1, y1, timeDelta);

			float positionDeviation = PointDistance(futurePosition, FloatVec2{ (float)x1, (float)y1 });
			float areaDeviation = abs(areaStats.mean - (*open).area);
			float widthDeviation = abs(widthStats.mean - (*open).width);
			float heightDeviation = abs(heightStats.mean - (*open).height);
			float velocityDeviation = abs(velocityStats.mean - velocity);

			float positionWeight = 1.0;
			float areaWeight = 0.6;
			float widthWeight = 0.25;
			float heightWeight = 0.25;
			float velocityWeight = 1.0;

			float positionScore = positionWeight * (positionDeviation - positionStats.sd);
			float areaScore = areaWeight * (areaDeviation - areaStats.sd);
			float widthScore = widthWeight * (widthDeviation - widthStats.sd);
			float heightScore = heightWeight * (heightDeviation - heightStats.sd);
			float velocityScore = velocityWeight * (velocityDeviation - velocityStats.sd);

			float score = positionScore + areaScore + widthScore + heightScore + velocityScore;

			float scoreThreshold = 1.0;
			if (score < scoreThreshold) {
				// hurray you found it
				(*tracking)[i].shapePtrs.push_back(unmatched[j]);
				unmatched.erase(unmatched.begin() + j);
				openTrackingIndices.erase(openTrackingIndices.begin() + i); // ? bad thonkz?
				break;

			}
		}
	}*/

	// for each remaining shape that was previously tracked and not matched to this frame
	std::vector<TrackedShapeRLE> newTracks = std::vector<TrackedShapeRLE>();
	int searchDepth = 1;
	for (int i = 0; i < searchDepth; i++) {

		std::vector<ShapeRLE>& lastShapes = (*previous)[previous->size() - 1];

		for (int j = 0; j < lastShapes.size(); j++) {

			bool shouldBreak = false;
			for (int k = 0; k < unmatched.size(); k++) {

				ShapeRLE& a = lastShapes[j];
				ShapeRLE& b = *unmatched[k];

				// for each shape from the last frame, compare against each shape in the current frame
				float intersection = CenterRelativeIntersection(&a, &b);
				bool intersectionGood = (intersection / lastShapes[j].area) <= params.maxAreaDifference;

				float distance = Magnitude(Subtract(a.areaCenter, b.areaCenter));
				bool distanceGood = distance <= params.maxDistance;

				if (intersectionGood && distanceGood) {
					newTracks.push_back(TrackedShapeRLE{ std::vector<ShapeRLE*>{&lastShapes[j]}, StatsShapeRLE{} });
					unmatched.erase(unmatched.begin() + k);
					shouldBreak = true;
					break;
				}
			}
			if (shouldBreak) {
				break;
			}
		}
	}

	for (TrackedShapeRLE t : newTracks) {
		tracking->push_back(t);
	}

}

inline cv::Vec3b ToCV_U8(ColorRGBi color) {
	return cv::Vec3b{ (uchar)color.blue, (uchar)color.green, (uchar)color.red };
}
inline void PaintShape(cv::Mat* image, const ShapeRLE& shape) {

	cv::Vec3b color = cv::Vec3b{ (uchar)shape.color.blue, (uchar)shape.color.green, (uchar)shape.color.red };

	for (int y = 0; y < shape.rowRanges.size(); y++) {
		for (const Range& range : shape.rowRanges[y]) {
			for (int x = range.x1; x < (range.x2 + 1); x++) {
				image->at<cv::Vec3b>(y, x) = color;
			}
		}
	}
}
inline void PaintShape(cv::Mat* image, const ShapeRLE& shape, ColorRGBi color) {

	cv::Vec3b cvColor = cv::Vec3b{ (uchar)color.blue, (uchar)color.green, (uchar)color.red };

	for (int y = 0; y < shape.rowRanges.size(); y++) {
		for (const Range& range : shape.rowRanges[y]) {
			for (int x = range.x1; x < (range.x2 + 1); x++) {
				image->at<cv::Vec3b>(y, x) = cvColor;
			}
		}
	}
}
inline void PaintShape(cv::Mat* image, const ShapeRLE& shape, IntVec2 projection) {

	cv::Vec3b color = cv::Vec3b{ (uchar)shape.color.blue, (uchar)shape.color.green, (uchar)shape.color.red };
	projection.y += shape.rect.yMin;

	for (int y = 0; y < shape.rowRanges.size(); y++) {
		for (const Range& range : shape.rowRanges[y]) {
			for (int x = range.x1; x < (range.x2 + 1); x++) {
				image->at<cv::Vec3b>(projection.y + y, projection.x + x) = color;
			}
		}
	}
}
inline void PaintShapeBlended(cv::Mat* image, const ShapeRLE& shape, IntVec2 projection, float amountOriginal, float amountNew) {

	cv::Vec3b color = cv::Vec3b{ (uchar)shape.color.blue, (uchar)shape.color.green, (uchar)shape.color.red };
	projection.y += shape.rect.yMin;

	for (int y = 0; y < shape.rowRanges.size(); y++) {
		for (const Range& range : shape.rowRanges[y]) {
			for (int x = range.x1; x < (range.x2 + 1); x++) {
				cv::Vec3b original = image->at<cv::Vec3b>(projection.y + y, projection.x + x);
				image->at<cv::Vec3b>(projection.y + y, projection.x + x) = cv::Vec3b{
					static_cast<uchar>((float(original[0]) * amountOriginal) + (float(color[0]) * amountNew)),
					static_cast<uchar>((float(original[1]) * amountOriginal) + (float(color[1]) * amountNew)),
					static_cast<uchar>((float(original[2]) * amountOriginal) + (float(color[2]) * amountNew))
				};
			}
		}
	}
}
inline void PaintShapes(cv::Mat* image, std::vector<ShapeRLE>* shapes) {

	for (const ShapeRLE& shape : *shapes) {
		PaintShape(image, shape);
	}
}
inline void PaintShapes(cv::Mat* image, std::vector<ShapeRLE>* shapes, IntVec2 projection) {

	for (const ShapeRLE& shape : *shapes) {
		PaintShape(image, shape, projection);
	}
}
inline void PaintShapesBlended(cv::Mat* image, std::vector<ShapeRLE>* shapes, IntVec2 projection, int blending) {

	float amountOriginal = (255.0f - blending) / 255.0f;
	float amountNew = (0.01f + blending) / 255.0f;
	for (const ShapeRLE& shape : *shapes) {
		PaintShapeBlended(image, shape, projection, amountOriginal, amountNew);
	}
}
inline void PaintShapes(cv::Mat* image, std::vector<ShapeRLE>* shapes, ColorRGBi color) {

	for (const ShapeRLE& shape : *shapes) {
		PaintShape(image, shape);
	}
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
			float biggest = 0;
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
//
// possible whites = +x , +y
// clock wise -> check +x first, then +y 
// if +x & +y == black ----> is a single pixel
//
// SEARCH ORDER CLOCKWISE
// +x
//  -y
//   -x
//    +y
//
// check first pixel to set state
// always start look from -1 in the counter clockwise position
//
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
std::vector<std::vector<PerimeterPoint>> PerimeterPointsThresholdedAxisX(cv::Mat* in) {

	int xLim = in->cols - 1;
	int yLim = in->rows;
	std::vector<std::vector<PerimeterPoint>> points(in->rows, std::vector<PerimeterPoint>());

	// handle left edge at x = 0
	for (int y = 0; y < yLim; y++) {
		if (in->at<uchar>(y, 0) && in->at<uchar>(y, 1)) {
			points[y].push_back(PerimeterPoint(0));
		}
	}

	// handle inbetween left and right edge
	for (int x = 1; x < xLim; x++) {
		for (int y = 0; y < yLim; y++) {
			if (in->at<uchar>(y, x) && ((!(in->at<uchar>(y, x-1)) + !(in->at<uchar>(y, x+1))) == 1)) {
				points[y].push_back(PerimeterPoint{ x });
			}
		}
	}

	// handle right edge at x = xLim 
	for (int y = 0; y < yLim; y++) {
		if (in->at<uchar>(y, xLim) && in->at<uchar>(y, xLim - 1)) {
			points[y].push_back(PerimeterPoint(xLim));
		}
	}

	return points;
}
// for each pixel, test left and right pixels color to determine if its a perimeter
// pixel for a shape with respect to the x axis.
std::vector<std::vector<PerimeterPoint>> PerimeterPointsAxisX(cv::Mat* in, int colorMax) {

	int colorStep = round(colorMax / 2) - 1;
	int xLim = in->cols - 1;
	int yLim = in->rows;
	std::vector<std::vector<PerimeterPoint>> points(in->rows, std::vector<PerimeterPoint>());

	// handle left edge at x = 0
	for (int y = 0; y < yLim; y++) {
		if ((in->at<cv::Vec3b>(y, 0)[0] == colorMax) && (in->at<cv::Vec3b>(y, 1)[0] == colorMax)) {
			points[y].push_back(PerimeterPoint(0));
		}
	}

	// handle inbetween left and right edge
	for (int x = 1; x < xLim; x++) {
		for (int y = 0; y < yLim; y++) {
			if ((in->at<cv::Vec3b>(y, x)[0] == colorMax) && ((!(in->at<cv::Vec3b>(y, x-1)[0] == colorMax) + !(in->at<cv::Vec3b>(y, x+1)[0] == colorMax)) == 1)) {
				points[y].push_back(PerimeterPoint{ x });
			}
		}
	}

	// handle right edge at x = xLim 
	for (int y = 0; y < yLim; y++) {
		if ((in->at<cv::Vec3b>(y, xLim)[0] == colorMax) && (in->at<cv::Vec3b>(y , xLim - 1)[0] == colorMax)) {
			points[y].push_back(PerimeterPoint(xLim));
		}
	}

	return points;
}
/*
std::vector<std::vector<PerimeterPoint>> PerimeterPointsAxisX(cv::Mat* in, int colorMax, int xMax, int yMax) {

	int colorStep = round(colorMax / 2) - 1;
	int xLim = in->rows - 1;
	int yLim = in->cols;
	std::vector<std::vector<PerimeterPoint>> points(in->cols, std::vector<PerimeterPoint>());

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
*/
// for each pixel, test left and right pixels color to determine if its a perimeter
// pixel for a shape with respect to the x axis.
/*std::vector<std::vector<PerimeterPoint>> PerimeterPointsAxisX_Expirimental(cv::Mat* in, int colorMax, int xMax, int yMax) {

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
}*/

inline std::vector<ShapeRLE> SegmentBinaryImage(cv::Mat* in) {

	std::vector<std::vector<PerimeterPoint>> points = PerimeterPointsThresholdedAxisX(in); // sorted (y, i)
	
	std::vector<std::vector<FillNode>> nodes = std::vector<std::vector<FillNode>>(in->rows);
	std::vector<FillNodeIndex> indices = std::vector<FillNodeIndex>(0); // fill node index contains the y coord of the range of pixels as y, and the index into the y ranges vector as i

	int id = 0;
	for (int y = 0; y < points.size(); y++) {
		for (int i = 0; i < points[y].size(); i += 2) {
			nodes[y].push_back(FillNode{ false, false, id++, FillNodeIndex{ y, i / 2 }, points[y][i], points[y][(size_t)i + 1], points[y][(size_t)i + 1] - points[y][i], std::vector<int>(0), std::vector<FillNodeIndex>(0) });
			indices.push_back(FillNodeIndex{ y, i / 2 });
		}
	}

	// for each node test if each range in the above nodes intersects its range.
	// if it intersects, push back a FillNodeIndex into its connections, to reference it. 
	// thus generating a tree that collides with itself.
	for (int y = 0; y < nodes.size() - 1; y++){
		int y2 = y + 1; // row above y
		for (int i = 0; i < nodes[y].size(); i++){
			for (int k = 0; k < nodes[y2].size(); k++){
				if (RangesIntersect(nodes[y][i].x1, nodes[y][i].x2, nodes[y2][k].x1, nodes[y2][k].x2)){
					nodes[y][i].connections.push_back(nodes[y2][k].index);
					nodes[y2][k].connections.push_back(nodes[y][i].index);
				}
			}
		}
	}

	// walk shape indices until tree is constructed of fill node indices,
	// which represent the shapes pixel data.
	std::vector<FillNodeIndex> open;
	std::vector<std::vector<FillNodeIndex>> shapesIndices;
	for (int s = 0, k = 0; s < indices.size(); s++){
		// working on shape at k index
		shapesIndices.push_back(std::vector<FillNodeIndex>());
		open.push_back(indices[s]);
		
		// remove first from open, push back all connections, until there are no connections.
		// store indices of connections in the k index for the shapes range indices data.
		while (open.size()){
			FillNodeIndex index = open.back();
			open.pop_back();

			if (nodes[index.y][index.i].walked == false){
				shapesIndices[k].push_back(index);
				nodes[index.y][index.i].walked = true;

				// for each connection to the FillNode push it back into open.
				for (int c = 0; c < nodes[index.y][index.i].connections.size(); c++){
					open.push_back(nodes[index.y][index.i].connections[c]);
				}
			}
		}
		k++;
	}

	// for each set of indices for a given shape
	std::vector<ShapeRLE> shapes = std::vector<ShapeRLE>();
	std::vector<std::vector<FillNodeIndex>*> shapeIndicesPtrs = std::vector<std::vector<FillNodeIndex>*>();
	for (int c = 0; c < shapesIndices.size(); c++)
	{
		ShapeRLE shape = ShapeRLE{};
		std::vector<FillNodeIndex>* shapeIndicesPtr = &shapesIndices[c];
		float xTotal = 0.0;
		float yTotal = 0.0;
		float coords = 0.0;

		int xMin = std::numeric_limits<int>::max();
		int yMin = std::numeric_limits<int>::max();
		int xMax = 0;
		int yMax = 0;
		for (int w = 0; w < shapeIndicesPtr->size(); w++)
		{
			FillNode& node = nodes[(*shapeIndicesPtr)[w].y][(*shapeIndicesPtr)[w].i];
			float nodeArea = (node.x2 - node.x1) + 1;
			shape.area += nodeArea;

			if (node.x1 < xMin)
			{
				xMin = node.x1;
			}
			if (node.x2 > xMax)
			{
				xMax = node.x2;
			}
			if ((*shapeIndicesPtr)[w].y < yMin)
			{
				yMin = (*shapeIndicesPtr)[w].y;
			}
			if ((*shapeIndicesPtr)[w].y > yMax)
			{
				yMax = (*shapeIndicesPtr)[w].y;
			}
		}

		shape.areaCenter = FloatVec2{ xTotal / coords, yTotal / coords };
		shape.rect = Rectangle{ xMin, yMin, xMax, yMax }; 
		Rectangle& rect = shape.rect;
		shape.height = yMax - yMin;
		shape.width = xMax - xMin;
		shapes.push_back(shape);
		shapeIndicesPtrs.push_back(shapeIndicesPtr);
	}

	// sort ranges and push into shape 
	int index = 0;
	for (ShapeRLE& shape : shapes) {

		std::vector<FillNodeIndex>* shapeNodeIndices = shapeIndicesPtrs[index];

		shape.rowRanges = std::vector<std::vector<Range>>(shapeNodeIndices->size());

		for (FillNodeIndex& f : *shapeNodeIndices) {
			FillNode n = nodes[f.y][f.i];
			Range r = { n.x1, n.x2 };
			shape.rowRanges[f.y - shape.rect.yMin].push_back(r);
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

		shape.color = RandomColor();

		index++;
	}

	return shapes;
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
		shape.rect = b.rect;
		shape.width = b.size.width;
		shape.height = b.size.height;

		for (FillNodeIndex f : b.indices) {
			FillNode n = frame.nodes[f.y][f.i];
			Range r = { n.x1, n.x2 };
			shape.rowRanges[f.y - shape.rect.yMin].push_back(r);
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

	std::vector<std::vector<PerimeterPoint>> points = PerimeterPointsAxisX(in, max);

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


