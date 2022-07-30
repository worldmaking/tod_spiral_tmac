#version 330
precision mediump float;
in vec3 v_color;
in vec3 v_normal;
in vec3 v_lightdir;
in vec3 v_vertex;
out vec4 outColor;

/*
	TODO: this lighting model doesn't really make sense
	- the lightdir and normal are world space
	- the v_vertex is camera space
	- the math is weird
	- the computations are wasteful on vec3
	- also missing is the skin texture
*/

vec4 highTone = vec4(1.0, 0.692, 0.07, 0.); // diffuse 0.19

float halfLambert(in vec3 v1, in vec3 v2) {
	return dot(v1, v2) * 0.5 + 0.5;
}

void main() {
	vec3 normal = normalize(v_normal);
	vec3 lightdir = normalize(v_lightdir);
	float ndotl = dot(normal, lightdir);
	float attenuation = 1.; //(.5/length(v_vertex)); -- doesn't make sense.

	vec3 indl = vec3(max(0., dot(normal, lightdir)));
	indl += 8. * halfLambert(v_vertex, -lightdir);
	indl *= v_color.rgb * attenuation;
	vec3 rim = vec3(1.-max(0., dot(normal, v_vertex)));
	rim = rim * rim;
	rim *= max(0., dot(normal, lightdir)) * highTone.rgb * attenuation;

	vec4 final_color = vec4(0.15*indl + 0.5*rim, 1.);

	outColor = final_color;
}