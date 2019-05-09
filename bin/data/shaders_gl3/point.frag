#version 150

uniform sampler2D tex;
in vec4 frontColor;
in vec3 vertex;
out vec4 fragColor;

void main (void) {
    fragColor = texture2D(tex, gl_PointCoord.xy) * vec4(frontColor.rgb, 1);
}