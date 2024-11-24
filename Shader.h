#pragma once
#include <glad/glad.h>

#include <vector>
#include <string>
#include <stdio.h>
#include <iostream>
#include <fstream>

class Shader
{
public:
	Shader();

	void CreateFromString(const char* vertexCode, const char* fragmentCode);
	void CreateFromFiles(const char* vertexLocation, const char* geometryLocation, const char* fragmentLocation);
	void Validate() const;

	std::string ReadFile(const char* fileLocation);

	void UseShader();
	void ClearShader();

	void SetBrightness(GLfloat brightness);
	GLuint GetBrightnessLocation();

	~Shader();

private:

	int pointLightCount;
	int spotLightCount;
	GLuint shaderID, brightness;
	GLuint uniformBrightness;

	void CompileShader(const char* vertexCode, const char* geometryCode, const char* fragmentCode);
	void AddShader(GLuint theProgram, const char* shaderCode, GLenum shaderType);

	void CompileProgram();
};