#version 150

uniform mat4 modelViewProjectionMatrix;
uniform float size;
in vec4 position;
in vec3 normal;
out vec4 frontColor;
out vec3 vertex;

void main() {
    vertex = position.xyz;
    gl_Position   = modelViewProjectionMatrix * position;
    gl_PointSize  = size * normal.z;
    frontColor = vec4(normal.xy, 1., 1.) ;
}