#pragma once

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <math.h>
#include <limits>
#include <bitset>

#include "ImgProcTypes.h"
#include "Values.h"
#include "ImgProc.h"
#include "Presets.h"
#include "LinearRegression.h"
#include "GaussNewton.h"

bool AXGreaterThanBX(const FloatVec2& a, const FloatVec2& b) {
	return a.x > b.x;
}
bool AYGreaterThanBY(const FloatVec2& a, const FloatVec2& b) {
	return a.y > b.y;
}
FloatVec2 GridCellVertexByIndex(CellQuad& cell, int i) {
	switch (i) {
	case 0:
		return cell.botLeft;
	case 1:
		return cell.topLeft;
	case 2:
		return cell.botRight;
	case 3:
		return cell.topRight;
	}
	return FloatVec2{ 0.0,0.0 };
}


namespace Calibrate {

	// find grid rectangle/bounds
	Rectangle GridRectangle(std::vector<ShapeRLE> shapes) {

		ShapeRLE selected = ShapeRLE{};
		int largest = 0;
		for (ShapeRLE s : shapes) {
			int boundArea = s.height * s.width;
			if (boundArea > largest)
			{
				largest = boundArea;
				selected = s;
			}
		}
		
		return selected.bounds;
	}

	// find the grid center, and return its center of mass coordinates in view space for the camera
	FloatVec2 GridCenter(std::vector<ShapeRLE> shapes, Rectangle bounds) {

		float xSpan = bounds.xMax - bounds.xMin;
		float ySpan = bounds.yMax - bounds.yMin;
		FloatVec2 estimatedCenter = { bounds.xMin + (xSpan / 2.0f), bounds.yMin + (ySpan / 2.0f) };
		float sizeModifier = 1.25f;
		FloatVec2 estimatedSizeMax = { sizeModifier * xSpan / 10.0f, sizeModifier * ySpan / 10.0f };
		
		bool acceptable = false;
		ShapeRLE closest = shapes.front;
		float smallest = std::numeric_limits<float>::max();
		for (ShapeRLE s : shapes) {
			if (estimatedSizeMax.x > s.width && estimatedSizeMax.y > s.height) {
				float d = Distance(s.areaCenter, estimatedCenter);
				if (d < smallest) {
					smallest = d;
					closest = s;
					acceptable = true;
				}
			}
		}

		if (acceptable) {
			return closest.areaCenter;
		}
		else {
			return FloatVec2{bounds.xMin + (xSpan / 2.0f), bounds.yMin + (ySpan / 2.0f)};
		}
	}

	void FindLines(cv::Mat& img, std::vector<IntLine>* line, std::vector<PixelCoord> pixels) {

		std::vector<IntLine> unsorted = std::vector<IntLine>(0);

		// emit lines for each pixel
		for (PixelCoord px : pixels) {

			// check top
			if (Blocked(img, { px.x , px.y + 1 }, 127)) {
				IntVec2 a = { px.x, px.y };
				IntVec2 b = { px.x + 1, px.y };
				unsorted.push_back(IntLine{ a,b });
			}

			// check bottom
			if (Blocked(img, { px.x	 , px.y - 1 }, 127)) {
				IntVec2 a = { px.x, px.y + 1 };
				IntVec2 b = { px.x + 1, px.y + 1 };
				unsorted.push_back(IntLine{ a,b });
			}

			// check left
			if (Blocked(img, { px.x +-1, px.y }, 127)) {
				IntVec2 a = { px.x, px.y };
				IntVec2 b = { px.x, px.y + 1 };
				unsorted.push_back(IntLine{ a,b });
			}

			//check right 
			if (Blocked(img, { px.x + 1, px.y }, 127)) {
				IntVec2 a = { px.x + 1, px.y };
				IntVec2 b = { px.x + 1, px.y + 1};
				unsorted.push_back(IntLine{ a,b });
			}
		}

		// sort lines by joining shared coordinates
		std::vector<IntLine> sorted = std::vector<IntLine>(0);
		if (unsorted.size()) {
			sorted.push_back(unsorted[0]);
			unsorted.erase(unsorted.begin() + 0);
		}

		// check match location for search reversal optimization & etc later
		while (unsorted.size()) {

			IntLine current = sorted[sorted.size() - 1];
			IntLine next = IntLine{ IntVec2{ -1, -1}, IntVec2{ -1, -1} };
			bool matchFound = false;
			int matchIndex = -1;

			for (int i = 0; i < unsorted.size(); i++) {
				next = unsorted[i];
				if (IsSame(current.b, next.a)) {
					matchFound = true;
					matchIndex = i;
					break;
				} else if (IsSame(current.b, next.b)) {
					matchFound = true;
					matchIndex = i;
					next = SwapAB(next);
					break;
				} else {
					continue;
				}
			}

			if (matchFound) {
				unsorted.erase(unsorted.begin() + matchIndex);
				sorted.push_back(next);
			} else {
				// no match found, totally gonked
			}
		}

		for (int i = 0; i < sorted.size(); i++) {
			line->push_back(sorted[i]);
		}
	}

	void ErodeAndDilate(cv::Mat* in, cv::Mat* out) {
		int kernelSize = 7;
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernelSize, kernelSize));
		cv::Point point = cv::Point(-1, -1);
		int iterations = 3;
		cv::erode(*in, *out, kernel);
		cv::dilate(*out, *out, kernel);
	}

	// find all the grid lines, grid center, and line intersections.
	Grid PolySearchGrid(cv::Mat img) {

		// extract shape information
		cv::threshold(img, img, 100, 255, cv::THRESH_BINARY);
		cv::Mat eroded;
		ErodeAndDilate(&img, &eroded);
		std::vector<ShapeRLE> shapes = ExtractShapesRLE(&img);
		std::vector<ShapeRLE> shapesEroded = ExtractShapesRLE(&eroded);

		// find initial bounds & center
		Rectangle bounds = GridRectangle(shapesEroded);
		FloatVec2 center = GridCenter(shapesEroded, bounds);
		std::vector<GridLine> lines = std::vector<GridLine>(0);
		
		// estimate size for sliding perimeter slope window
		int gridSize = 10;
		float cellPerimeterEstimate = (2.0f * (bounds.xMax - bounds.xMin + bounds.yMax - bounds.yMin)) / gridSize;
		int windowSize = floorf(cellPerimeterEstimate / 8.0f); // can use edge count divided by (sides + mod) as well
		if (windowSize % 2 != 0) {
			// ensure window size is even so that it has a center
			windowSize -= 1;
		}

		// set basic information to estimate positions
		int cellCount = Squared(gridSize);
		float xSpan = bounds.xMax - bounds.xMin;
		float ySpan = bounds.yMax - bounds.yMin;
		float sizeModifier = 1.0f / gridSize;
		FloatVec2 estimatedCellSize = FloatVec2{ sizeModifier * xSpan, sizeModifier * ySpan};
		float sizeModifierMax = 2.0f;
		float estimatedCellAreaMax = sizeModifierMax * estimatedCellSize.x * estimatedCellSize.y;

		// estimate the position of the center of grid cells and set their value in the array
		std::vector<FloatVec2> estimatedCellCenters = std::vector<FloatVec2>(cellCount);
		for (int i = 0; i < cellCount; i++) {
			float x = (0.5 + (i % gridSize)) * estimatedCellSize.x;
			float y = (0.5 + floorf(i / gridSize)) * estimatedCellSize.y;
			estimatedCellCenters[i] = FloatVec2{x, y};
		}

		// use local normalized positions to insert shape indices into a table for testing
		std::vector<std::vector<int>> sortedShapeIndices = std::vector<std::vector<int>>(cellCount);
		for (int i = 0; i < shapes.size(); i++) {
			int xGrid = (int)floorf(gridSize * (shapes[i].areaCenter.x - bounds.xMin) / xSpan);
			int yGrid = (int)floorf(gridSize * (shapes[i].areaCenter.y - bounds.yMin) / ySpan);
			int cellIndex = xGrid + (gridSize * yGrid);
			sortedShapeIndices[cellIndex].push_back(i);
		}

		// select the shape with the best fit for each grid cell, and store its index in the corresponding element
		std::vector<int> cellShapeIndices = std::vector<int>(cellCount);
		for (int i = 0; i < sortedShapeIndices.size(); i++) {
			float smallest = std::numeric_limits<float>::max();	
			int smallestIndex = 0;
			for (int j = 0; j < sortedShapeIndices[i].size(); j++) {
				int shapeIndex = sortedShapeIndices[i][j];
				float d = Distance(estimatedCellCenters[i], shapes[shapeIndex].areaCenter);
				if (d < smallest && shapes[shapeIndex].area < estimatedCellAreaMax) {
					smallest = d;
					smallestIndex = shapeIndex;
				}
			}
			cellShapeIndices[i] = smallestIndex;
		}

		// generate per-pixel permimeter paths for each grid cells corresponding shape
		std::vector<LinePerimeter> perimeterPaths = std::vector<LinePerimeter>(cellCount);
		PixelCoord px = PixelCoord{ floorf(bounds.xMin), floorf(bounds.yMin) };
		std::vector<CellQuad> gridCellQuads = std::vector<CellQuad>(cellCount);
		int cellQuadIndex = 0;
		for (int i : cellShapeIndices) {
			ShapeRLE shape = shapes[i];
			bool searching = true;

			// for each shape find the polygon of the shape
			while (searching) {
				PixelCoord startPixel = PixelCoord{ shape.rowRanges[0][0].x1, shape.bounds.yMin };
				std::vector<PixelCoord> perimeterPixels = PerimeterPixels(img, startPixel);
				std::vector<IntLine> lines; 
				FindLines(img, &lines, perimeterPixels);

				std::vector<Axis> lineAxises;
				for (IntLine l : lines) {
					if (l.a.x == l.b.x) {
						lineAxises.push_back(X_AXIS);
					} else if (l.a.y == l.b.y) {
						lineAxises.push_back(Y_AXIS);
					} else {
						// something is gonked big hugeness
					}
				}

				// using a window of approximately the length of a perfect grid cell side, score the windows to find corners
				// tuple -> score , walkable , index
				// int might actually get maxxed out sometimes... idk
				std::vector<std::tuple<int, bool, int>> windowScores = std::vector<std::tuple<int,bool, int>>(lineAxises.size());
				for (int g = 0; g < lineAxises.size(); g++) {
					// score each window. lower is a corner, higher is a line. max line score is windowSize
					int score = 0;
					for (int p = 0; p < windowSize; p++) {
						// index in both directions by windows size - 1  from window center index
						Axis lineAxis = lineAxises[(g + p) % lineAxises.size()];
						if (lineAxis == X_AXIS) {
							score++;
						} else {
							score--;
						}
					}
					windowScores[g] = std::make_tuple(score, true, g);
				}

				// set values for finding corners
				float estimatedPerimeter = (estimatedCellSize.x + estimatedCellSize.y) * 2.0f;
				float measuredPerimeter = lines.size();
				int minSpacing = floorf(static_cast<float>(windowSize) * measuredPerimeter / measuredPerimeter);
				int removesPer = minSpacing * 2;

				// find remaining corners
				int numFound = 1;
				int numCorners = 4;
				std::vector<int> cornersLow = std::vector<int>(0);
				while (numFound != numCorners) {
					
					// search for next corner
					int bestScore = std::numeric_limits<int>::max();
					int indexOfBest = -1;
					for (std::tuple<int, bool, int> score : windowScores) {
						if (std::get<1>(score) && int(abs(std::get<0>(score))) < bestScore) {
							bestScore = std::get<0>(score);
							indexOfBest = std::get<2>(score);
						}
					}

					// store corner. make close by unwalkable
					cornersLow.push_back(indexOfBest);
					for (int r = 0; r < removesPer; r++) {
						int centerLow = indexOfBest + (windowSize / 2);
						int inset = minSpacing;
						int index = (r + windowSize + centerLow - inset) % windowScores.size();
						std::get<1>(windowScores[index]) = false;
					}
					
					numFound++;
				}

				// split perimeter at corners & fit a line to each side of the grid cell
				std::vector<std::vector<IntLine>> cellWalls = std::vector<std::vector<IntLine>>(0);
				for (int f = 0; f < cornersLow.size(); f++) {
					
					// get the indices of the corners to grab points from between.
					int begin = cornersLow[f];
					int end = cornersLow[(f + 1) % 4] + 1;

					// push each cell wall line segment into the array
					std::vector<IntLine> cellWallLineSegments = std::vector<IntLine>(0);
					for (int z = begin; z < end; z++) {
						int segmentIndex = z % lines.size();
						cellWallLineSegments.push_back(lines[segmentIndex]);
					}
					cellWalls.push_back(cellWallLineSegments);
				}

				// use segments to get just the points for each cell wall
				std::vector<std::vector<IntVec2>> wallsPoints = std::vector<std::vector<IntVec2>>(0);
				for (std::vector<IntLine> segs : cellWalls) {
					wallsPoints.push_back(std::vector<IntVec2>{segs[0].a});
					for (IntLine seg : segs) {
						wallsPoints[wallsPoints.begin].push_back(seg.b);
					}
				}

				// for each cell walls points, fit a line to the point data
				std::tuple<std::vector<Float64SlopeIntercept>, Degrees> slopeInterceptsAndRotation = LinearRegressionMultiR2D2(wallsPoints, 17.3f);
				std::vector<Float64SlopeIntercept>& slopeIntercepts = std::get<0>(slopeInterceptsAndRotation);
				Degrees linesRotation = std::get<1>(slopeInterceptsAndRotation);

				// find intersections and emit vertices, then rotate back.
				std::vector<FloatVec2> vertices = std::vector<FloatVec2>(0);
				for (int c = 0; c < slopeIntercepts.size(); c++) {
					for (int g = 0; g < slopeIntercepts.size() - 1; g++) {
						int k = (c + g) % 4;
						Float64SlopeIntercept f0 = slopeIntercepts[c];
						Float64SlopeIntercept f1 = slopeIntercepts[k];
						double minDiff = 0.0000001;
						if (f0.m - f1.m) {
							continue;
						}
						double x = (f0.b - f1.b) / (f1.m - f0.m);
						double y0 = (f0.m * x) + f0.b;
						double y1 = (f1.m * x) + f1.b;
						double y = (y0 + y1) / 2.0;
						bool xTruncated = std::numeric_limits<float>::min() > x || std::numeric_limits<float>::max() < x;
						bool yTruncated = std::numeric_limits<float>::min() > y || std::numeric_limits<float>::max() < y;
						if (xTruncated || yTruncated) {
							continue;
						}
						// rotate vertices back to origin while still represented as doubles
						double reverseRotation = -1.0 * linesRotation;
						double x = (x * cos(reverseRotation)) - (y * sin(reverseRotation));
						double y = (y * cos(reverseRotation)) + (x * sin(reverseRotation));
						vertices.push_back(FloatVec2{ x, y });
					}
				}

				// test vertices distance to shape area center, and pick 4 closest vertices
				std::vector<std::tuple<float,int>> magnitudes = std::vector<std::tuple<float, int>>(vertices.size());
				for (int k = 0; k < vertices.size(); k++) {
					float x = vertices[k].x - shape.areaCenter.x;
					float y = vertices[k].y - shape.areaCenter.y;
					magnitudes[k] = std::make_tuple(sqrtf((x * x) + (y * y)), k);
				}

				// pick quad vertices
				std::vector<FloatVec2> quadVertices = std::vector<FloatVec2>(4);
				std::sort(magnitudes.begin(), magnitudes.end());
				for (int k = 0; k < 4; k++) {

					// store vertex
					int vertIndex = std::get<1>(magnitudes[k]);
					quadVertices[k] = vertices[vertIndex];
				}

				// sort into topLeft, topRight, botLeft, botRight & push final quad
				std::sort(quadVertices.begin(), quadVertices.end(), AXGreaterThanBX);
				std::sort(quadVertices.begin(), quadVertices.begin() + 1, AYGreaterThanBY);
				std::sort(quadVertices.end() - 1, quadVertices.end(), AYGreaterThanBY);
				
				// push final quad into grid cell quads
				CellQuad result = CellQuad{ quadVertices[0], quadVertices[1], quadVertices[2], quadVertices[3] };
				gridCellQuads[cellQuadIndex] = result;
				cellQuadIndex++;
				searching = false;
			}
		}

		// populate sorted grid cell matrix
		std::array<std::array<GridCell, 10>, 10> gridCells;
		for (int i = 0; i < gridCells.size(); i++) {
			for (int j = 0; j < gridCells[i].size(); j++) {
				int shapeIndex = cellShapeIndices[i + (10 * j)];
				gridCells[i][j] = GridCell{ shapes[shapeIndex], gridCellQuads[i] }; 
			}
		}

		// calc 11 x 11 line points matrix 
		const int pointsPer = 11;
		std::array<std::array<FloatVec2, pointsPer>, pointsPer> gridLinePoints;
		for (int i = 0; i < pointsPer; i++) {
			for (int j = 0; j < pointsPer; j++) {

				// if it lies on the farthest corners, dont emit the points
				bool check = static_cast<int>(i == 0) + 
					static_cast<int>(i == pointsPer - 1) + 
					static_cast<int>(j == 0) + 
					static_cast<int>(j == pointsPer - 1);
				if (check == 2) {
					gridLinePoints[i][j] = FloatVec2{ 0.0, 0.0 };
					continue;
				} 

				// up , down, left, right
				IntVec2 botLeft = IntVec2{ i - 1,j - 1 };
				IntVec2 topLeft = IntVec2{ i - 1, j };
				IntVec2 botRight = IntVec2{ i, j - 1 };
				IntVec2 topRight = IntVec2{ i, j };

				// vertex index per cell quad for opposite corner -> cells vertex closest to the line point
				std::array<int, 4> mapping = { 3, 2, 1, 0 };
				
				// indices for the grid cell to grab a vertex from
				std::array<IntVec2, 4> indices = { botLeft, topLeft, botRight, topRight };

				// grab the vertices and average them for the line point
				std::vector<FloatVec2> vertices = std::vector<FloatVec2>(0);
				for (int k = 0; k < indices.size(); k++) {
					bool indicesValid = NumInRangeMinMax(i, 0, gridCells.size()) && NumInRangeMinMax(i, 0, gridCells.size());
					if (indicesValid) {
						IntVec2 index = indices[k];
						CellQuad quad = gridCells[index.x][index.y].quad;
						int vertexIndex = mapping[k];
						FloatVec2 vertex = GridCellVertexByIndex(quad, vertexIndex);
						vertices.push_back(vertex);
					}
				}
				double xSum = 0.0;
				double ySum = 0.0;
				for (FloatVec2 v : vertices) {
					xSum += v.x;
					ySum += v.y;
				}
				xSum /= vertices.size();
				ySum /= vertices.size();

				// store resulting point
				gridLinePoints[i][j] = FloatVec2{ xSum, ySum };

			}
		}
		


		return Grid{ FloatRectangle{bounds.xMin, bounds.yMin, bounds.xMax, bounds.yMax}, gridLinePoints, gridCells, center };
	}

	// takes known measurements and returns the corresponding grid for those measurements
	Grid CalculateGrid(float xFov, IntVec2 imgSize, FloatVec2 sensorSizeInches, Inches gridDistInches, Inches gridSizeInches) {

		float alpha = xFov * TO_RADIANS;
		float a = sensorSizeInches.x / 2.0f;
		float beta = (90.0f * TO_RADIANS) - (xFov / (2.0f * TO_RADIANS));
		float c = a / sinf(alpha);
		float b = sqrtf((c * c) - (a * a));
		float nearPlane = b;

		float a2 = sensorSizeInches.y / 2;
		float c2 = sqrtf(powf(a2, 2) + pow(nearPlane, 2));
		float yFov = asinf(a2 / c2) * TO_DEGREES;

		// construct initial
		Grid output = Grid{};
		const int m = 11;
		float gridScale = gridSizeInches / m;
		for (int i = 0; i < m; i++) {
			for (int j = 0; j < m; j++) {
				output.linePointMat[i][j].x = (nearPlane * (i + static_cast<float>(-m) / 2.0f) * gridScale) / gridDistInches;
				output.linePointMat[i][j].y = (nearPlane * (j + static_cast<float>(-m) / 2.0f) * gridScale) / gridDistInches;
			}
		}

		return output;
	}

	// takes in a distorted and undistorted grid, and returns the approximate solutions for the lens distortion polynomials
	LensDistortion LensDistortionForGrids(Grid distorted, Grid reference) {

		LensDistortion output = {};

		// calculate the magnitudes to use for radial distortion summing
		std::vector<double> xi = Magnitudes(reference.linePointMat);
		std::vector<double> yi = Magnitudes(distorted.linePointMat);

		output.translation = { 0, 0 };
		output.distortion2 = ToPoly2(GaussNewtonPolyN(xi, yi, 2));
		output.distortion3 = ToPoly3(GaussNewtonPolyN(xi, yi, 3));
		output.distortion4 = ToPoly4(GaussNewtonPolyN(xi, yi, 4));
		output.distortion5 = ToPoly5(GaussNewtonPolyN(xi, yi, 5));

		return output;
	}

	// returns the magnitudes of the vectors for each point in the linePointMat
	std::vector<double> Magnitudes(std::array<std::array<FloatVec2, 11>,11> input) {

		std::vector<double> output = std::vector<double>();

		for (std::array<FloatVec2, 11> col : input) {
			for (FloatVec2 p : col) {
				double magnitude = sqrt((p.x * p.x) + (p.y * p.y));
				output.push_back(magnitude);
			}
		}

		return output;
	}

	// input measures are in inches
	StereoLensDistortion Distortion(cv::Mat& l, cv::Mat& r, float imgDist, float imgSize, float gridSize, float parallax, float fov) {

		// solve for rectilinear position. imdDist should be greater than parallax.
		FloatVec2 linearCenter { 0.0f, 0.0f };
		float a = parallax / 2.0f;
		float b = imgDist;
		float beta =  (b / a) * TO_DEGREES;
		float alpha = 180.0f - (90 + beta);
		float scale = 2.0f * alpha / fov;
		float width = (float)l.cols;
		float height = (float)l.rows;
		int expectedOffsetX = (int)floor(scale * width / 2.0f);

		// expected location of the center of the calibration image with no distortion, and cameras perfectly aligned
		PixelCoord expectedRectilinearLeft = { expectedOffsetX + (width / 2), height / 2 };
		PixelCoord expectedRectilinearRight = { (width / 2) - expectedOffsetX, height / 2 };

		// identify grid lines, grid center, and line intersections. 
		Grid gridLeft = PolySearchGrid(l);
		Grid gridRight = PolySearchGrid(r);

		// approximate the undistorted left and right grids based on known measurements
		OV9750 cam = OV9750{};
		Inches gridDistance = 0.0f;
		Grid undistortedLeft = CalculateGrid(cam.xFovDegrees, cam.imageSize, FloatVec2{ cam.width, cam.height }, gridDistance, gridSize);
		Grid undistortedRight = CalculateGrid(cam.xFovDegrees, cam.imageSize, FloatVec2{ cam.width, cam.height }, gridDistance, gridSize);

		// fit curves to the thing or whatever
		LensDistortion distortionLeft = LensDistortionForGrids(gridLeft, undistortedLeft);
		LensDistortion distortionRight = LensDistortionForGrids(gridRight, undistortedRight);
		StereoLensDistortion distortion{ distortionLeft, distortionRight };


		return distortion;
	}



}

