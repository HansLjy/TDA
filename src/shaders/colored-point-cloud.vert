#version 460 core

layout(location = 0) in vec3 coord_in;
layout(location = 5) in vec3 color_in;
uniform mat4 mvp;

out vec3 color_out;

void main() {
    gl_Position = mvp * vec4(coord_in, 1.0f);
    color_out = color_in;
}