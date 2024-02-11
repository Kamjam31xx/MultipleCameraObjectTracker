#pragma once

#include <vector>
/*struct CapVars {
	int scale = 2;
	int width = 2560 / scale;
	int height = 960 / scale;
	int exposure = -8.50;
};
struct PostProcVars {
	float minArea = 150;
	float distanceScale = 1.0;
	float modPos = 0.1;
	float modArea = 0.05;
	float modRect = 0.06;
	float modPost = 1.0;
	float ratioModRect = 1.1;
	float discard = 0.15;
	float accept = 2.4;
};
struct ConvolutionVars {
	double thresh = 100.0;
	double threshMax = 255.0;
	int erosion = 3;
	int dilation = 3;
	int sobelThresh = 100;
	bool sobel = false;

	bool blur = false;
	bool erode = false;
	bool dilate = false;
	bool color = false;
};
struct LogVars {
	float rate = 1.0;
	float time = 0.0;
};*/

struct CameraSettings {
	double exposure = -8.50;
	int resolutionScale = 2;
	int width = 2560 / resolutionScale;
	int height = 960 / resolutionScale;
};

struct ProcessSettings {
	float areaMinSize = 150.0;
	float distScale = 1.0;
	float distMod = 0.1;
	float areaMod = 0.05;
	float rectMod = 0.06;
	float rectAreaRatioMod = 1.1;
	float postMod = 1.0;
	float discardThreshold = 0.15;
	float acceptThreshold = 2.4;
	bool wait = false;
	bool advanceFrame = false;
	bool frameAdvanced = false;
	bool blur = false;
	bool erode = false;
	bool dilate = false;
	bool color = false;
	int cvwait = 1;
	float cvwaitSlider = 1.0;
	bool lag = false;
	bool sobel = false;
	int sobelThresh = 100;
	int temporalSteps = 5;
	int temporalFrames = 10;
	double threshold = 100.0;
	double thresholdMax = 255.0;
	int erosion = 3;
	int dilation = 3;
	cv::Size kernelSize{ 3, 3 };
	float logRate = 1.0;
	float logTime = 0.0;
};

/*
enum direction
{
	UP = 0,
	RIGHT = 1,
	DOWN = 2,
	LEFT = 3,
	NONE = -1
};*/

enum direction
{
	POS_X = 0,
	NEG_Y = 1,
	NEG_X = 2,
	POS_Y = 3,
	NONE = 4
};

typedef int Threshold;
typedef int PerimeterPoint;

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
struct BlobState
{
	std::vector<std::vector<Blob>> butt;
};
struct BlobFrame
{
	std::vector<Blob> blobs;
	std::vector<FillNodeIndex> indices;
	std::vector<std::vector<FillNode>> nodes;
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



/*
struct PerimeterSearch
{
	PixelCoord current;
	direction look;
	direction move;
};
*/