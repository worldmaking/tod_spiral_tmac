#version 150

uniform sampler2D diffuse;
in vec2 v2TexCoord;
out vec4 outputColor;
void main()
{
   outputColor = texture( diffuse, v2TexCoord);
}