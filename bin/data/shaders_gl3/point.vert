#version 150

uniform mat4 modelViewProjectionMatrix;
uniform float size;
uniform vec3 translation;
in vec4 position;
in vec3 normal;
out vec4 frontColor;
out vec3 vertex;

void main() {
    vertex = position.xyz;
    gl_Position   = modelViewProjectionMatrix * position;
    if (gl_Position.w > 0.) {
        gl_PointSize  = 5. * size * normal.z / gl_Position.w;
    } else {
        gl_PointSize = 0.;
    }
    frontColor = vec4(normal.xy, 1., 1.) ;
}