
#version 150

//#include "lib.glsl"

float DIM = 32.;

uniform sampler3D uGradient;
uniform mat4	ciViewMatrix;
uniform mat4	ciProjectionMatrix;
uniform float uNow;

in vec3		ciPosition;
//in vec2		ciTexCoord0;
in vec3		ciNormal;

out vec4 view;
out vec3 normal;
out vec3 color;

void main( void )
{
	normal = ciNormal;
	
	vec3 P = ciPosition.xyz;
	
	vec3 grad = texture(uGradient, P/DIM).xyz;
	color = 0.5 + grad;
	//normal = grad;
	
	// wobbly offset according to position:
	vec3 t = uNow * vec3(3., 4., 5.);
	float scale = 3.141592653589793 * .9;
	float range = .1;
	
	P += range * sin(t + P * scale) - 0.5;
	
	
	view = ciViewMatrix * vec4(P, 1.);
	
	gl_Position = ciProjectionMatrix * view;
	
}
