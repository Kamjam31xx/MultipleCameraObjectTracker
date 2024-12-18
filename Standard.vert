#version 440

layout(location = 0) in vec2 uv;
layout(location = 1) in vec3 pos;

out vec3 color;
out vec2 textureCoord;

uniform mat4 MVP;

void main() {
	gl_Position = MVP * vec4(pos.xy, 0.0, 1.0);
	color = vec3(uv.x, uv.y, 0.0);
	textureCoord = uv;
}