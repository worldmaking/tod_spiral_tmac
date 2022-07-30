#version 330
uniform mat4 u_viewmatrix;
uniform mat4 u_projmatrix;
uniform vec3 u_lightpos;

in vec3 a_position;
in vec3 a_normal;
in vec2 a_texCoord;

in vec4 i_quat;
in vec3 i_pos;
in float i_age;
in vec2 i_scale;
in vec3 i_color;

out vec3 v_color;
out vec3 v_normal;
out vec3 v_lightdir;
out vec3 v_vertex;

const float pi = 3.141592653589793;

// //	q must be a normalized quaternion
vec3 quat_rotate(vec4 q, vec3 v) {
	vec4 p = vec4(
				q.w*v.x + q.y*v.z - q.z*v.y,	// x
				q.w*v.y + q.z*v.x - q.x*v.z,	// y
				q.w*v.z + q.x*v.y - q.y*v.x,	// z
				-q.x*v.x - q.y*v.y - q.z*v.z	// w
				);
	return vec3(
				p.x*q.w - p.w*q.x + p.z*q.y - p.y*q.z,	// x
				p.y*q.w - p.w*q.y + p.x*q.z - p.z*q.x,	// y
				p.z*q.w - p.w*q.z + p.y*q.x - p.x*q.y	// z
				);
}

vec3 beetle_animate(vec3 vertex, float phase, float amp) {
	// distort the geometry:
	float x = vertex.x;
	float y = vertex.y;
	float z = vertex.z;
	
	// yshift is proportional to z
	// zshift moves toward origin if yshift has same sign as y
	// and zshift is proportional to yshift
	float s = sin(phase * 2. * pi);
	float c = cos(phase * 2. * pi);
	
	float expansion = (1. + 0.06 * s);
	float wingpush = (abs(x) * 0.4 * (0.5+sin((z + phase*2.) * pi)));
	float wingopen = (y * abs(x) * 0.4 * (0.5*c));
	
	// wing component: greater as x increases:
	float side = sign(x);
	
	float yshift = (abs(z) * 0.15 * s) + (-0.04*s) + wingopen;
	float zshift = amp * ( (-yshift * z * y) + wingpush);
	
	return vec3(
				expansion*x,
				expansion*y + yshift,
				expansion*z + zshift
				);
}

void main() {
	v_normal = a_normal;
	// normal in world space:
	v_normal = quat_rotate(i_quat, v_normal);

	vec3 vertex = beetle_animate(a_position.xyz, i_age * 5., 1.);
 	vertex = quat_rotate(normalize(i_quat), vertex * i_scale.xxy) + i_pos;
	vec4 view_vertex = u_viewmatrix * vec4(vertex, 1);
	// vertex in view space:
	v_vertex = view_vertex.xyz;
	gl_Position = u_projmatrix * view_vertex;

	v_color = i_color;
	v_lightdir = u_lightpos - vertex;

}