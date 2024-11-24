#pragma once

#include <vector>
#include <tuple>
#include <array>
#include <iostream>
#include <stdexcept>
#include "ImgProcTypes.h"

struct StateChangeVertices32F {

};

class Vertex32F {
public:
	Vertex32F(std::string& layout, std::vector<float&> data);
	Vertex32F(Vertices32F& parent, std::string& layout, std::vector<float&> data);


protected:

private:
	std::vector<float> data;
};

class Vertices32F {
public:
	Vertices32F();

	void setLayout(std::string layout);

	void 

protected:

private:
	std::vector<float> data;
	std::vector<StateChangeVertices32F> changes;
	
};