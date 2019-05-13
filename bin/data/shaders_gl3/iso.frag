#version 150

//#include "lib.glsl"

vec3 light0dir = vec3(1, 0, 1);
vec3 light2dir = vec3(-1, 0, -1);
vec3 light1dir = vec3(0, -1, 1);
vec3 light3dir = vec3(1, 1, -1);

//Haru
vec3 light0color = vec3(0.4, 0.2, 0.6); //(0.9, 0.5, 0.6);
vec3 light1color = vec3(0.55, 0.55, 0.95); //(0.0, 0.55, 0.75);
vec3 light2color = vec3(0.03, 0.65, 0.65);//(0.13, 0.35, 0.65)(0.0, 0.55, 0.65);
vec3 light3color = vec3(0.05, 0.4, 0.7);//(0.1, 0.4, 0.7)(0.0, 0.5, 0.7)

/* original
 vec3 light0color = vec3(0.0, 0.7, 0.6);
 vec3 light1color = vec3(0.0, 0.55, 0.75);
 vec3 light2color = vec3(0.0, 0.65, 0.65);
 vec3 light3color = vec3(0.0, 0.6, 0.7);
*/

float gamma = 2.2;
vec3 gamma3 = vec3(1.0 / gamma);


uniform float uAlpha;

in vec3 normal;
in vec4 view;
in vec3 color;
out vec4 outColor;

void main( void ) {
	
	vec3 nn = normalize(normal);
	
	vec3 lighting
		= abs(dot(light0dir, nn)) * light0color
		+ abs(dot(light1dir, nn)) * light1color
		+ abs(dot(light2dir, nn)) * light2color
		+ abs(dot(light3dir, nn)) * light3color
	;
	
	float glancing = 1.1 - abs(dot(normalize(view.xyz), nn));
	glancing = 3. * pow(glancing, 3.);
	
	lighting *= glancing*2.;
	

	outColor = vec4(lighting, 1.);
	
	outColor *= uAlpha;
	
	// gamma correct
	outColor.rgb = pow(outColor.rgb, gamma3);
	
	//outColor = vec4(color, 0.2);
}