#version 150

uniform mat4 matrix;
in vec4 position;
in vec3 v3NormalIn;
in vec2 v2TexCoordsIn;
out vec2 v2TexCoord;
void main()
{
	v2TexCoord = v2TexCoordsIn;
	gl_Position = matrix * vec4(position.xyz, 1);
}