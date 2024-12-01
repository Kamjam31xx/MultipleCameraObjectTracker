#include "Shader.h"
#include <thread>

Shader::Shader()
{
	shaderID = 0;
}

void Shader::CreateFromSource(ShaderSource source) {
	CreateFromStrings(source.vertex, source.geometry, source.fragment);
}
void Shader::CreateFromStrings(std::string vertexCodeStr, std::string geometryCodeStr, std::string fragmentCodeStr) {
	const char* vertexCode = vertexCodeStr == "" ? nullptr : vertexCodeStr.c_str();
	const char* geometryCode = geometryCodeStr == "" ? nullptr : geometryCodeStr.c_str();
	const char* fragmentCode = fragmentCodeStr == "" ? nullptr : fragmentCodeStr.c_str();
	CompileShader(vertexCode, geometryCode, fragmentCode);
}
void Shader::CreateFromFiles(const char* vertexLocation, const char* geometryLocation, const char* fragmentLocation)
{
	const char* vertexCode = vertexLocation == nullptr ? nullptr : ReadFile(vertexLocation).c_str();
	const char* geometryCode = geometryLocation == nullptr ? nullptr : ReadFile(geometryLocation).c_str();
	const char* fragmentCode = fragmentLocation == nullptr ? nullptr : ReadFile(fragmentLocation).c_str();
	CompileShader(vertexCode, geometryCode, fragmentCode);
}
std::string Shader::ReadFile(const char* fileLocation)
{
	std::string content;

	std::ifstream fileStream(fileLocation, std::ios::in);
	if (!fileStream.is_open()) {
		printf("Failed to read %s! File doesn't exist.");
		content = "";
	}
	std::string line = "";
	while (!fileStream.eof()) {
		std::getline(fileStream, line);
		content.append(line + "\n");
	}
	fileStream.close();

	return content;
}
void Shader::CompileShader(const char* vertexCode, const char* geometryCode, const char* fragmentCode)
{
	shaderID = glCreateProgram();
	if (!shaderID) {
		printf("Error creating shader program");
		return;
	}
	int numShaders = 3;
	std::vector<const char*> shaderCodes = { vertexCode, geometryCode, fragmentCode };
	std::vector<int> shaderTypes{ GL_VERTEX_SHADER , GL_GEOMETRY_SHADER, GL_FRAGMENT_SHADER };
	for (int i = 0; i < numShaders; i++) {
		if (shaderCodes[i] != nullptr) {
			AddShader(shaderID, shaderCodes[i], shaderTypes[i]);
		}
	}
	CompileProgram();
	glDeleteShader(shaderID);
}
void Shader::Validate() const
{
	GLint result = 0;
	GLchar eLog[1024] = { 0 };

	glValidateProgram(shaderID);
	glGetProgramiv(shaderID, GL_VALIDATE_STATUS, &result);
	if (!result) {
		glGetProgramInfoLog(shaderID, sizeof(eLog), NULL, eLog);
		printf("Error validating program: '%s' \n", eLog);
		return;
	}
}

void Shader::CompileProgram()
{
	GLint result = 0;
	GLchar eLog[1024] = { 0 };

	glLinkProgram(shaderID);
	glGetProgramiv(shaderID, GL_LINK_STATUS, &result);
	if (!result) {
		glGetProgramInfoLog(shaderID, sizeof(eLog), NULL, eLog);
		printf("Error linking program: '%s' \n", eLog);
		return;
	}
	uniformMVP = glGetUniformLocation(shaderID, "MVP");
}

void Shader::SetMVP(mat4x4* MVP) {
	glUniformMatrix4fv(uniformMVP, 1, GL_FALSE, (const GLfloat*)MVP);
}
GLuint Shader::GetShaderID() {
	return shaderID;
}
GLuint Shader::GetLocationMVP() {
	return uniformMVP;
}

void Shader::UseShader()
{
	glUseProgram(shaderID);
}
void Shader::ClearShader()
{
	if (shaderID != 0) {
		glDeleteProgram(shaderID);
		shaderID = 0;
	}
}
void Shader::AddShader(GLuint theProgram, const char* shaderCode, GLenum shaderType)
{
	GLuint theShader = glCreateShader(shaderType);

	const GLchar* theCode[1];
	theCode[0] = shaderCode;

	GLint codeLength[1];
	codeLength[0] = strlen(shaderCode);

	glShaderSource(theShader, 1, theCode, codeLength);
	glCompileShader(theShader);

	GLint result = 0;
	GLchar eLog[1024] = { 0 };

	glGetShaderiv(theShader, GL_COMPILE_STATUS, &result);
	if (!result) {
		glGetProgramInfoLog(theShader, sizeof(eLog), NULL, eLog);
		printf("Error compiling the %d shader: '%s' \n", shaderType, eLog);
		return;
	}
	glAttachShader(theProgram, theShader);
}

Shader::~Shader()
{
	ClearShader();
}