#pragma once

#include <vector>

struct Float64SlopeIntercept {
	double m;
	double b;
};
struct Float64SlopeInterceptRotation {
	double m;
	double b;
	double degrees;
};

struct IndexRange {
	int begin;
	int endExclusive;
};


struct FittedCurve {

};


typedef int Threshold;
typedef int PerimeterPoint;
typedef float Millimeters;
typedef float Inches;
typedef double Degrees;
typedef double Radians;

#define MILLIMETERS_TO_INCHES 0.0393701;
#define INCHES_TO_MILLIMETERS 25.4;

struct CornerPinRect
{
	int x;
	int y;
	int w;
	int h;
};

struct PixelCoord
{
	int x;
	int y;
};

struct Rectangle
{
	int xMin;
	int yMin;
	int xMax;
	int yMax;
};
struct FloatRectangle
{
	float xMin;
	float yMin;
	float xMax;
	float yMax;
};


struct Span
{
	int x;
	int length;
};
struct IndexedFillRange
{
	int x1;
	int x2;
	int len;
	int i;
};
struct FillRange
{
	int x1;
	int x2;
	int len;
};
struct UnorderedSpan
{
	int y;
	Span span;
};
struct Range {
	int x1;
	int x2;
};
struct FloatVec2
{
	float x;
	float y;
};
struct FloatVec3
{
	float x;
	float y;
	float z;
};
struct FloatLine {
	FloatVec2 a;
	FloatVec2 b;
};
struct IntVec2 {
	int x;
	int y;
};
struct IntLine {
	IntVec2 a;
	IntVec2 b;
};
struct RectangleSize
{
	int width;
	int height;
};
struct ColorRGBi
{
	int red;
	int green;
	int blue;
};
struct IJK
{
	int i;
	int j;
	int k;
};
struct FillNodeIndex
{
	int y;
	int i;
};
struct FillNode
{
	bool walked;
	bool joined;
	int id;
	FillNodeIndex index;
	int x1;
	int x2;
	int len;
	std::vector<int> ids;
	std::vector<FillNodeIndex> connections;
};
struct Blob
{
	Rectangle rect;
	RectangleSize size;

	float mass;
	float density;
	float area;

	FloatVec2 centerOfMass;
	std::vector<FloatVec2> temporalMassCenter;
	std::vector<float> temporalMassCenterScores;
	float massCenterVariance;

	FloatVec2 centerOfArea;
	int temporalSize = 10;
	int temporalEntries = 0;
	int temporalAreaPointer = 0;
	std::vector<bool> temporalAreaCenterInitialized = std::vector<bool>(10, false);
	std::vector<FloatVec2> temporalAreaCenter = std::vector<FloatVec2>(10, FloatVec2{ 0.0,0.0 });
	std::vector<float> temporalAreaCenterScores = std::vector<float>(10, 0.0);
	std::vector<FloatVec2> temporalAreaCenterMoves = std::vector<FloatVec2>(10, FloatVec2{ 0.0,0.0 });
	std::vector<float> temporalAreaCenterVelocities = std::vector<float>(10, 0.0);
	float areaCenterVariance;

	ColorRGBi color;
	std::vector<FillNodeIndex> indices;
	std::vector<FillNode> nodes;
	std::vector<UnorderedSpan> unorderedSpans;
	std::vector<std::vector<IndexedFillRange>> ySortedRanges;
	std::vector<std::vector<FillRange>> sortedRanges;

	bool render;
};
struct ShapeRLE {
	int area;
	FloatVec2 areaCenter;
	int width;
	int height;
	Rectangle bounds;
	std::vector<std::vector<Range>> rowRanges;
};


struct BlobFrame
{
	std::vector<Blob> blobs;
	std::vector<FillNodeIndex> indices;
	std::vector<std::vector<FillNode>> nodes;
};



struct BlobState
{
	std::vector<std::vector<Blob>> butt;
};


struct Chain
{

};
struct Trace
{

};
struct Vertex
{

};
struct Cluster
{

};
struct Nexus
{

};
struct Flock
{

};
struct Constellation
{

};
struct ConstellationState
{

};
struct View
{

};
struct ViewState
{

};
struct PolyN {
	int n;
	float a;
	float b;
	float c;
	float d;
	float e;
};
struct Poly2 {
	float a;
	float b;
};
struct Poly3 {
	float a;
	float b;
	float c;
};

struct Poly4 {
	float a;
	float b;
	float c;
	float d;
};

struct Poly5 {
	float a;
	float b;
	float c;
	float d;
	float e;
};
struct LensDistortion {
	FloatVec2 translation;
	Poly2 distortion2;
	Poly3 distortion3;
	Poly4 distortion4;
	Poly5 distortion5;
};
struct StereoLensDistortion {
	LensDistortion left;
	LensDistortion right;
};


struct CellQuad {
	FloatVec2 botLeft;
	FloatVec2 topLeft;
	FloatVec2 botRight;
	FloatVec2 topRight;
};
struct GridCell {
	ShapeRLE shape;
	CellQuad quad;
};
struct Grid {
	FloatRectangle bounds;
	std::array<std::array<FloatVec2, 11>, 11> linePointMat;
	std::array<std::array<GridCell, 10>, 10> cellMat;
	FloatVec2 center;
};

struct LineSegment {
	FloatVec2 a;
	FloatVec2 b;
};

using GridLine = std::vector<FloatVec2>;
using LinePerimeter = std::vector<IntLine>;

enum Axis {
	X_AXIS = 0,
	Y_AXIS = 1
};


struct FloatQuad {
	std::array<FloatVec2, 4> vertices;
};





/*
struct PerimeterSearch
{
	PixelCoord current;
	direction look;
	direction move;
};
*/