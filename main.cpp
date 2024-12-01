//#include "stdafx.h"

/*

parimeter walk flood fill
	find wall, walk wall, record xMax and yMax in array for dynamic programming algorithm. use touples for polarity/direction of travel.

minkowski shape sum center of mass and intersection etc thing



to-do.....
generate vector of all detected blobs temporally
generate temporal blob trees
score trees

to-do....
implement splines
use splines for continuation prediction (quadratic continuation)


*/


/*
	add blob to blob manhattan distance -> to -> nearest neighbor mass & nearest neighbor center of mass -> check if that matches temporal past blobs
	- aka a blob gets split into 2 blobs, detect if adding 2 blobs brings back an abstraction resembling that blobby-boi
*/

// // double time = glfwGetTime(); // typically the most accurate time source on each platform

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
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
#include <utility>
#include <bitset>
#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>



#include "ImgProcTypes.h"
#include "DeltaTime.h"
#include "FrameRate.h"
#include "ImgProc.h"
#include "Keyboard.h"

#include "Shader.h"

#include "CallbacksGLFW.h"

#include "linmath.h"

IntVec2 FrontResolution = { 2560, 960 };

DeltaTime Timer;
DeltaTime VelocityTimer;
FrameRate FPS = FrameRate(10);
cv::VideoCapture cap(1);
AppSettings appSettings = AppSettings{};
ImageProcessSettings imgProcessing = ImageProcessSettings{};
TrackingSettings tracking = TrackingSettings{};
LogSettings logging = LogSettings{};
RenderSettings rendering = RenderSettings{};
CaptureSettings capturing = CaptureSettings{};
CalibrationSettings calibration = CalibrationSettings{};
CameraSettings frontCamera = CameraSettings{ -8.50f, 2, FrontResolution.x, FrontResolution.y}; // load camera settings
int lastDownscaling = 9;

std::string fragmentShaderSource = "#version 440 \n\
in vec3 color;\n\
in vec2 textureCoord;\n\
out vec4 fragment;\n\
layout(binding = 1) uniform sampler2D cameraTexture;\n\
void main(){\n\
	vec2 tex = vec2(textureCoord.x, textureCoord.y); \n \
	vec4 cameraColor = vec4(texture(cameraTexture, vec2(tex.x, -tex.y)).xyz, 1.0);\n\
	fragment = cameraColor;\n\
}";

std::string vertexShaderSource = "#version 440 \n \
layout(location = 0) in vec2 uv;\n\
layout(location = 1) in vec3 pos;\n\
out vec3 color;\n\
out vec2 textureCoord;\n\
uniform mat4 MVP;\n\
void main() {\n\
	gl_Position = MVP * vec4(pos.xy, 0.0, 1.0);\n\
	color = vec3(uv.x, uv.y, 0.0);\n\
	textureCoord = uv;\n\
}";

const int indices[6] = { 0, 1, 2, 0, 2, 3 };
float s = 1.0;
float sRatio = 2560.0f / 960.0f;
static const Vertex vertices[4] = {
	{0.0f, 0.0f, -s, -s * sRatio, -s}, // top left
	{1.0f, 0.0f, -s,  s * sRatio, -s}, // bottom right
	{1.0f, 1.0f, -s,  s * sRatio,  s}, // top right
	{0.0f, 1.0f, -s, -s * sRatio,  s} // bottom left
	//u v z x y
};
struct VisibilityGUI {
	bool settings = SHOW;
	bool mode = SHOW;
	bool imageProcessing = SHOW;
	bool tracking = SHOW;
	bool logging = SHOW;
	bool rendering = SHOW;
	bool capturing = SHOW;
	bool calibration = HIDE;
	bool cameras = SHOW;
	bool selectedCamera = 0;

};

bool InitSettings() {

	log("initializing settings...");

	appSettings = AppSettings{
		TRACKING_MODE,
		&imgProcessing,
		&tracking,
		&logging,
		&rendering,
		&capturing,
		&calibration,
		std::vector<CameraSettings*>{&frontCamera}
	};

	return SUCCESS;
}
bool InitGLFW() {

	log("initializing GLFW...");

	if (!glfwInit()) {
		log("glfw initialization failed");
		return FAILURE;
	}

	return SUCCESS;
}
GLFWwindow* InitWindow(GLint width, GLint height, std::string name) {

	log("initializing window...");

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 4);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	GLFWwindow* window = glfwCreateWindow(width, height, name.c_str(), NULL, NULL);

	return window;
}
bool ValidateWindow(GLFWwindow* window) {

	log("validating window...");

	if (!window) {
		glfwTerminate();
		log("window initialization failed");
		return FAILURE;
	}

	return FAILURE;
}
bool ValidateLoaderGLAD(GLFWwindow* window) {

	log("validating GLAD loader...");

	glfwMakeContextCurrent(window);
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		glfwTerminate();
		log("GLAD loader failed");
		return FAILURE;
	}

	return SUCCESS;
}
bool InitViewport(GLFWwindow* window, GLint* width, GLint* height, GLboolean vSync) {

	log("initializing viewport...");
	glfwGetFramebufferSize(window, width, height);
	glViewport(0, 0, *width, *height);
	glfwSwapInterval(vSync); // glfwWaitEvents() alternative see docs https://www.glfw.org/docs/latest/group__window.html#ga554e37d781f0a997656c26b2c56c835e

	return SUCCESS;
}
void SetCallbacksGLFW(GLFWwindow* window) {

	glfwSetErrorCallback(error_callback);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

}
bool InitCapturing(cv::VideoCapture* capture, CameraSettings* settings) {

	log("initializing capturing...");

	if (capture->isOpened() != true) {
		log("capture is not open");
		return FAILURE;
	}

	int denominator = (int)(pow(2, settings->resolutionScale));
	int effectiveWidth = settings->width / denominator;
	int effectiveHeight = settings->height / denominator;

	capture->set(cv::CAP_PROP_AUTO_EXPOSURE, 0);
	capture->set(cv::CAP_PROP_EXPOSURE, settings->exposure);
	capture->set(cv::CAP_PROP_FRAME_HEIGHT, effectiveHeight);
	capture->set(cv::CAP_PROP_FRAME_WIDTH, effectiveWidth);

	return SUCCESS;
}
bool InitTimers(int fpsBufferLength) {

	log("initializing timers...");

	Timer.tick();
	VelocityTimer.tick();

	FPS = FrameRate(fpsBufferLength);

	return SUCCESS;
}
bool InitScreenSpaceQuad(GLuint* VAO, GLuint* IBO, GLuint* VBO, GLint locationPos, GLint locationUV) {

	log("initializing screen-space quad...");

	glGenVertexArrays(1, VAO);
	glBindVertexArray(*VAO);

	glGenBuffers(1, IBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, *IBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices[0]) * 6, indices, GL_STATIC_DRAW);

	glGenBuffers(1, VBO);
	glBindBuffer(GL_ARRAY_BUFFER, *VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	glEnableVertexAttribArray(locationPos);
	glVertexAttribPointer(locationPos, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, pos));
	glEnableVertexAttribArray(locationUV);
	glVertexAttribPointer(locationUV, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, uv));

	return SUCCESS;
}
bool InitCameraTexture(GLuint* id, GLenum textureUnit, cv::Mat* mat) {

	log("initializing camera texture...");

	if (mat->empty()) {
		log("texture data matrix is empty");
		return FAILURE;
	}

	glGenTextures(1, id);
	glBindTexture(GL_TEXTURE_2D, *id);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, mat->cols, mat->rows, 0, GL_RGB, GL_UNSIGNED_BYTE, mat->data);

	return SUCCESS;
}
void ConvertToRGBU8(cv::Mat* mat) {

	mat->convertTo(*mat, CV_8UC3);
	cv::cvtColor(*mat, *mat, cv::COLOR_BGR2RGB);
}
void UpdateTextureData(GLuint id, cv::Mat* mat) {

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, mat->cols, mat->rows, 0, GL_RGB, GL_UNSIGNED_BYTE, mat->data);
}
void UseTexture(GLuint id, GLenum textureUnit) {

	glActiveTexture(textureUnit);
	glBindTexture(GL_TEXTURE_2D, id);
}
void UpdateViewport(GLFWwindow* window, GLint* width, GLint* height, float* aspectRatio, FloatVec4 color) {

	glClearColor(color.x, color.y, color.z, color.w);
	glClear(GL_COLOR_BUFFER_BIT);
	glfwGetFramebufferSize(window, width, height);
	*aspectRatio = *width / (float)(*height);
	glViewport(0, 0, *width, *height);
}
void ButtonShowHide(bool* visible, int id, std::string name) {
	ImGui::PushID(id);
	if (ImGui::Button(std::string(*visible ? "Hide < " + name : "Show > " + name).c_str())) {
		*visible = !*visible;
	}
	ImGui::PopID();
}
void MetricsGUI(DeltaTime* timer, FrameRate* rate) {
	
	ImGui::Text(std::string("time:" + std::to_string(timer->getDeltaMilliSeconds()) + " ms").c_str());
	ImGui::Text(std::string("rate:" + std::to_string(rate->getRate()) + " fps").c_str());
	ImGui::SliderInt("FPS buffer length", &FPS.length, 1, 100);
}
void SettingsGUI(GLFWwindow* window, VisibilityGUI* show, DeltaTime* timer, FrameRate* fps) {
	// read example : https://github.com/ocornut/imgui/blob/master/examples/example_glfw_opengl3/main.cpp
	if (glfwGetWindowAttrib(window, GLFW_ICONIFIED) != 0)
	{
		ImGui_ImplGlfw_Sleep(10);
		return;
	}
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
	ImGui::Begin("ImGui");

	MetricsGUI(timer, fps);

	ButtonShowHide(&show->settings, 100, "Settings");
	if (show->settings) {

		// idfk NOT WORKY AT ALL 
		if (ImGui::BeginMenuBar()) {
			ImGui::PushID(200);
			if (ImGui::MenuItem("Tracking Mode")) {
				appSettings.mode = TRACKING_MODE;
			}
			if (ImGui::MenuItem("Calibration Mode")) {
				appSettings.mode = CALIBRATION_MODE;
			}
			ImGui::PopID();
		}

		// image processing
		ButtonShowHide(&show->imageProcessing, 101, "Image Processing");
		if (show->imageProcessing) {
			ImGui::SliderFloat("threshold", &imgProcessing.threshold, 0.0f, imgProcessing.thresholdMax);
			ImGui::Checkbox("erode", &imgProcessing.erode);
			ImGui::Checkbox("dilate", &imgProcessing.dilate);
			ImGui::Checkbox("blur", &imgProcessing.blur);
			ImGui::SliderInt("downscaling", &imgProcessing.downScaling, 0, 8);
			if (imgProcessing.downScaling != lastDownscaling) {
				lastDownscaling = imgProcessing.downScaling;
				int denominator = (int)(pow(2, imgProcessing.downScaling));
				int effectiveWidth = frontCamera.width / denominator;
				int effectiveHeight = frontCamera.height / denominator;
				cap.set(cv::CAP_PROP_FRAME_HEIGHT, effectiveHeight);
				cap.set(cv::CAP_PROP_FRAME_WIDTH, effectiveWidth);
			}
			//ImGui::SliderInt("sobel threshold", &imgProcessing.sobelThreshold, 0, imgProcessing.thresholdMax);
			//ImGui::Checkbox("sobel", &imgProcessing.sobel);

		}

		// tracking
		ButtonShowHide(&show->tracking, 102, "Tracking");
		if (show->tracking) {
			ImGui::SliderFloat("minimum area", &tracking.areaMinSize, 8.0f, tracking.areaMaxSize);
			ImGui::SliderFloat("maximum area", &tracking.areaMaxSize, tracking.areaMinSize, 4000.0f);
			ImGui::SliderFloat("area mod", &tracking.areaMod, 0.01f, 1.0f);
			ImGui::SliderFloat("distance scale", &tracking.distScale, 0.5f, 1.5f);
			ImGui::SliderFloat("distance mod", &tracking.distMod, 0.01f, 1.0f);
			ImGui::SliderFloat("rectangle mod", &tracking.rectMod, 0.01f, 1.0f);
			ImGui::SliderFloat("area rect ratio mod", &tracking.rectAreaRatioMod, 0.1, 2.0f);
			ImGui::SliderFloat("post mod", &tracking.postMod, 0.5f, 1.5f);
			ImGui::SliderFloat("discard threshold", &tracking.discardThreshold, 0.1f, 4.0f);
			ImGui::SliderFloat("accept threshold", &tracking.acceptThreshold, 0.1f, 4.0f);
			ImGui::SliderInt("temporal steps", &tracking.temporalSteps, 1, 30);
			ImGui::SliderInt("temporal frames", &tracking.temporalFrames, 1, 30);
		}

		ButtonShowHide(&show->rendering, 103, "Rendering");
		if (show->rendering) {
			ImGui::Checkbox("color", &rendering.color);
			ImGui::Checkbox("invert", &rendering.invert);
		}

		// capturing
		ButtonShowHide(&show->capturing, 104, "Capturing");
		if (show->capturing) {
			ImGui::SliderFloat("exposure", &frontCamera.exposure, -16.0f, 0.0f);
			cap.set(cv::CAP_PROP_EXPOSURE, frontCamera.exposure);
			ImGui::Checkbox("hault", &capturing.hault);
			ImGui::SliderInt("wait", &capturing.wait, 1, 100);
			if (ImGui::Button("advance frame")) {
				capturing.advanceFrame = true; // frame advanced will get dundiddered in the stuffs er whatever
			}
		}
	}
	ImGui::End();

}
void SegmentImage() {
	// to-do : add all image segmentation
}

int main()
{
	// to-do : handle initialization failures

	InitSettings();

	InitCapturing(&cap, &frontCamera);
	InitTimers(10);

	InitGLFW();
	GLint windowWidth = 1230;
	GLint windowHeight = 480;
	GLFWwindow* window = InitWindow(windowWidth, windowHeight, "OpenGL Window");
	ValidateWindow(window);
	ValidateLoaderGLAD(window);
	SetCallbacksGLFW(window);
	InitViewport(window, &windowWidth, &windowHeight, GL_TRUE);

	Shader shader;
	ShaderSource shaderSource = ShaderSource{ vertexShaderSource, "", fragmentShaderSource };
	shader.CreateFromSource(shaderSource);
	shader.UseShader();

	const GLint locationPos = 0;
	const GLint locationUV = 1;
	GLuint VAO = 0;
	GLuint IBO = 0;
	GLuint VBO = 0;
	InitScreenSpaceQuad(&VAO, &IBO, &VBO, locationPos, locationUV);

	GLuint textureID;
	GLenum textureUnit = GL_TEXTURE1;
	const int bitDepth = 8;
	const int channels = 3;
	const int capWidth = 2560;
	const int capHeight = 960;
	cv::Mat initialCapture;
	cap >> initialCapture;
	ConvertToRGBU8(&initialCapture);
	InitCameraTexture(&textureID, textureUnit, &initialCapture);

	const char* glsl_version = "#version 130";
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); 
	(void)io; // idk tbh -> was in example docs
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
	ImGui::StyleColorsDark();
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init(glsl_version);
	
	int front = 0;
	std::vector<cv::Mat> frame = { cv::Mat(), cv::Mat() };
	std::vector<cv::Mat> detect = { cv::Mat(), cv::Mat() };
	std::vector<cv::Mat> blurredFrame = { cv::Mat(), cv::Mat() };
	std::vector<cv::Mat> lightMaskFrame = { cv::Mat(), cv::Mat() };
	cv::Mat erosionElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * imgProcessing.erosion + 1, 2 * imgProcessing.erosion + 1));
	cv::Mat dilationElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * imgProcessing.dilation + 1, 2 * imgProcessing.dilation + 1));

	std::array<cv::Mat, 2> fillSource = { cv::Mat(), cv::Mat() };
	std::array<cv::Mat, 2> fillResult = { cv::Mat(), cv::Mat() };
	std::array<cv::Mat, 2> inFill = { cv::Mat(), cv::Mat() };
	std::array<cv::Mat, 2> upScaled = { cv::Mat(), cv::Mat() };
	std::array<cv::Mat, 2> leftCamera = { cv::Mat(), cv::Mat() };
	std::array<cv::Mat, 2> rightCamera = { cv::Mat(), cv::Mat() };
	std::array<cv::Mat, 2> leftResult = { cv::Mat(), cv::Mat() };
	std::array<cv::Mat, 2> rightResult = { cv::Mat(), cv::Mat() };

	BlobFrame lastLeft = BlobFrame{};
	BlobFrame lastRight = BlobFrame{};
	
	
	cv::waitKey(20); // wait for stuff

	cv::Mat capturedFrame;
	float aspectRatio = 1.0f;
	glfwPollEvents();
	
	VisibilityGUI show = VisibilityGUI{};
	while (!glfwWindowShouldClose(window)) {
		
		SettingsGUI(window, &show, &Timer, &FPS);

		UpdateViewport(window, &windowWidth, &windowHeight, &aspectRatio, FloatVec4{ 0.2f, 0.2f, 0.2f, 1.0f });

		mat4x4 m, p, mvp;
		mat4x4_identity(m);
		mat4x4_rotate_Z(m, m, 0.0f);// sinf((float)glfwGetTime() / 2.0f) * 3.14159 / 4.0f);
		mat4x4_ortho(p, -aspectRatio, aspectRatio, -1.f, 1.f, 1.f, -1.f);
		mat4x4_mul(mvp, p, m);


		if (appSettings.mode == TRACKING_MODE) {
			if (capturing.hault) {
				if (capturing.advanceFrame) {
					cap >> frame[front];
					capturing.frameAdvanced = true;
					capturing.advanceFrame = false;
				}
			} else {
				cap >> frame[front];
			}

			// wait for frame capture from camera device 
			cv::waitKey(capturing.wait);

			if (frame.empty()) {
				break;
			} else {
				// IMPLEMENT FUNCTION POINTERS TO ELIMINATE BRANCHING
				cv::cvtColor(frame[front], detect[front], cv::COLOR_BGR2GRAY);
				if (imgProcessing.blur) {
					cv::GaussianBlur(frame[front], frame[front], imgProcessing.kernelSize, 0);
				}
				/*
				cv::Mat derivativeMap;
				if (imgProcessing.sobel) {
					cv::Mat dx;
					cv::Mat dy;

					cv::Sobel(detect[front], dx, CV_8UC3, 1, 0);
					cv::Sobel(detect[front], dy, CV_8UC3, 0, 1);

					// Convert the derivative maps to absolute values
					cv::Mat dx_abs;
					cv::Mat dy_abs;
					cv::convertScaleAbs(dx, dx_abs);
					cv::convertScaleAbs(dy, dy_abs);

					cv::add(dx_abs, dy_abs, derivativeMap);
				} */

				cv::threshold(detect[front], detect[front], imgProcessing.threshold, imgProcessing.thresholdMax, cv::THRESH_BINARY);

				/*if (imgProcessing.sobel)
				{
					cv::subtract(255, derivativeMap, derivativeMap);
					cv::threshold(derivativeMap, derivativeMap, imgProcessing.sobelThresh, imgProcessing.thresholdMax, cv::THRESH_BINARY);
					derivativeMap.copyTo(detect[front]);
				}*/
				if (imgProcessing.erode) {
					cv::erode(detect[front], detect[front], erosionElement);
				}
				if (imgProcessing.dilate) {
					cv::dilate(detect[front], detect[front], dilationElement);
				}
				cv::cvtColor(detect[front], fillResult[front], cv::COLOR_GRAY2BGR);
				int width = fillResult[front].cols / 2;
				int height = fillResult[front].rows;
				leftCamera[front] = cv::Mat(fillResult[front], cv::Rect(0, 0, width, height));
				rightCamera[front] = cv::Mat(fillResult[front], cv::Rect(width, 0, width, height));

				leftCamera[front].copyTo(leftResult[front]); // replace with bounds per function to avoid copying and stuff
				rightCamera[front].copyTo(rightResult[front]);

				// initialize pixel blob container stuffs & then put the blobs of pixels in dem right der aye
				BlobFrame leftNow = BlobFrame{
					std::vector<ShapeDataRLE>(),
					std::vector<FillNodeIndex>(),
					std::vector<std::vector<FillNode>>(leftCamera[front].cols, std::vector<FillNode>())
				};
				BlobFrame rightNow = BlobFrame{
					std::vector<ShapeDataRLE>(),
					std::vector<FillNodeIndex>(),
					std::vector<std::vector<FillNode>>(rightCamera[front].cols, std::vector<FillNode>())
				};
				GetBlobs(&leftCamera[front], &leftResult[front], 255, tracking.areaMinSize, leftNow); // ADD areaMaxSize
				GetBlobs(&rightCamera[front], &rightResult[front], 255, tracking.areaMinSize, rightNow);

				// tracking dumb idiot way
				VelocityTimer.tick();
				float deltaTime = VelocityTimer.getDeltaMilliSeconds();
				if (rendering.color)
				{
					ColorTrack_Test(&leftResult[front], lastLeft, leftNow, deltaTime, tracking);
					ColorTrack_Test(&rightResult[front], lastRight, rightNow, deltaTime, tracking);
				}

				// invert all colors
				if (rendering.invert) {
					cv::subtract(255, leftResult[front], leftResult[front]);
					cv::subtract(255, rightResult[front], rightResult[front]);
				}

				lastLeft = leftNow;
				lastRight = rightNow;

				leftResult[front].copyTo(capturedFrame);
				ConvertToRGBU8(&capturedFrame);
				UpdateTextureData(textureID, &capturedFrame);// switch to PBO (Pixel Buffer Object)
				UseTexture(textureID, textureUnit);

			}
		}
		else if (appSettings.mode == CALIBRATION_MODE) {
			// to-do 
		}
		else {
			// yuh dun fuckter 
		}

		/*
		cap >> capturedFrame;
		if (!capturedFrame.empty()) {
			ConvertToRGBU8(&capturedFrame);
			UpdateTextureData(textureID, &capturedFrame);// switch to PBO (Pixel Buffer Object)
		}
		UseTexture(textureID, textureUnit);
		*/

		shader.SetMVP(&mvp);
		glBindVertexArray(VAO);
		glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(window);
		glfwPollEvents();

		Timer.tick();
		FPS.pushFrameTime(Timer.getDeltaMilliSeconds());
	}

	glfwTerminate();

	cap.release();

	cv::destroyAllWindows();



	return 0;

}
