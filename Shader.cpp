#include "Shader.h"
#include <thread>

Shader::Shader()
{
	shaderID = 0;
}

void Shader::CreateFromString(const char* vertexCode, const char* fragmentCode)
{
	CompileShader(vertexCode, fragmentCode, nullptr);
}
void Shader::CreateFromFiles(const char* vertexLocation, const char* geometryLocation, const char* fragmentLocation)
{
	std::string vertexString;
	std::string fragmentString;
	std::string geometryString;

	vertexString = ReadFile(vertexLocation);
	fragmentString = ReadFile(fragmentLocation);
	geometryString = ReadFile(geometryLocation);

	const char* vertexCode = vertexString.c_str();
	const char* geometryCode = geometryString.c_str();
	const char* fragmentCode = fragmentString.c_str();
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
	uniformBrightness = glGetUniformLocation(shaderID, "brightness");
}

void Shader::SetBrightness(GLfloat brightness) {
	glUniform1f(uniformBrightness, brightness);
}
GLuint Shader::GetBrightnessLocation() {
	return uniformBrightness;
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