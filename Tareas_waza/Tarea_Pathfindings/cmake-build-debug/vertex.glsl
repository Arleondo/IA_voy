#version 330 core

layout (location = 0) in vec2 aPos;

uniform float zoom;

void main()
{
    gl_Position = vec4(aPos * zoom, 0.0, 1.0);
}