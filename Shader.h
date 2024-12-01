#pragma once
#include <glad/glad.h>
#include "linmath.h"
#include "ImgProcTypes.h"

#include <vector>
#include <string>
#include <stdio.h>
#include <iostream>
#include <fstream>

class Shader
{
public:
	Shader();

	void CreateFromSource(ShaderSource source);
	void CreateFromStrings(std::string vertexCode, std::string geometryCode, std::string fragmentCode);
	void CreateFromFiles(const char* vertexLocation, const char* geometryLocation, const char* fragmentLocation);
	void Validate() const;

	std::string ReadFile(const char* fileLocation);

	void UseShader();
	void ClearShader();

	void SetMVP(mat4x4* MVP);

	GLuint GetShaderID();
	GLuint GetLocationMVP();

	~Shader();

private:

	GLuint shaderID, uniformMVP;

	void CompileShader(const char* vertexCode, const char* geometryCode, const char* fragmentCode);
	void AddShader(GLuint theProgram, const char* shaderCode, GLenum shaderType);

	void CompileProgram();
};