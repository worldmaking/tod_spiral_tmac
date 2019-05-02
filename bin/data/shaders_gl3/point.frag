#version 150

uniform sampler2D tex;
in vec4 frontColor;
out vec4 fragColor;

void main (void) {
    fragColor = texture2D(tex, gl_PointCoord.xy) * frontColor;
}