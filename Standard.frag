#version 440
in vec3 color;
in vec2 textureCoord;
out vec4 fragment;
layout (binding = 1) uniform sampler2D cameraTexture;

void main()
{
    float xCol = gl_FragCoord.x / 2560.0;
    float yCol = gl_FragCoord.y / 960.0;

    xCol /= 2.0;
    yCol /= 2.0;

    xCol += color.x / 2.0;
    yCol += color.y / 2.0;

    vec4 cameraColor = vec4(texture(cameraTexture, textureCoord).xyz, 1.0);

    fragment = cameraColor;// + vec4(xCol, yCol,0.5, 1.0);
}