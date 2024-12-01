#include "ScreenQuad.h"

ScreenQuad::ScreenQuad(int windowWidth, int windowHeight, GLfloat depth)
{
	this->windowWidth = windowWidth;
	this->windowHeight = windowHeight;
	this->z = depth;
}
// reverse the winding while keeping the same start index
void ScreenQuad::ReverseWinding() {
	std::vector<GLint> reversed;
	for (int i = 0; i < indices.size(); i++) {
		int j = (indices.size() - (1 + i) + 1) % indices.size();
		reversed.push_back(indices[j]);
	}
	for (int i = 0; i < indices.size(); i++) {
		indices[i] = reversed[i];
	}
}
void ScreenQuad::ShiftWinding(int shift) {
	std::vector<GLfloat> shifted;
	shift = shift % indices.size();
	for (int i = 0; i < indices.size(); i++) {
		shifted[i] = indices[(i + shift) % indices.size()];
	}
	for (int i = 0; i < indices.size(); i++) {
		indices[i] = shifted[i];
	}
}
void ScreenQuad::ScaleToWindow(int windowWidth, int windowHeight, GLfloat depth)
{
}