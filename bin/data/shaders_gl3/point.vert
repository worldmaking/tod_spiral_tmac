#version 150

uniform mat4 modelViewProjectionMatrix;
uniform float size;
in vec4 position;
out vec4 frontColor;

void main() {

    gl_Position   = modelViewProjectionMatrix * position;
    gl_PointSize  = size;
    frontColor = vec4(0.3, 1., 1., 1.) ;
}