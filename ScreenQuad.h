#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>

class ScreenQuad
{
public:
	ScreenQuad(int windowWidth, int windowHeight, GLfloat depth);
	void ReverseWinding();
	void ShiftWinding(int shift);

private:
	void ScaleToWindow(int windowWidth, int windowHeight, GLfloat depth);

	int windowWidth;
	int windowHeight;
	GLfloat z = 1.0f;
	GLfloat u = 0.5f;
	size_t offsetX = 0;
	size_t offsetY = 1;
	std::vector<GLuint> indices = { 0,1,2,3 };
	std::vector<GLfloat> vertices = { u, u, z, u, -u, z, -u, -u, z, -u, u, z };
};

