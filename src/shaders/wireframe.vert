#version 460 core

layout(location = 0) in vec3 coord_in;

uniform mat4 mvp;

void main() {
    gl_Position = mvp * vec4(coord_in, 1.0);
}