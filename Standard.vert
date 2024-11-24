#version 440

uniform mat4 MVP;

in vec2 uv;
in vec3 pos;

out vec3 color;

void main() {
	gl_position = MVP * vec4(pos, 0.0, 1.0);
	color = vec3(uv.x, uv.y, 0.0);
}