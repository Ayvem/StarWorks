#version 450

// Triangle.frag — interpolated vertex color, written as linear values;
// the sRGB swapchain performs the gamma encoding.

layout(location = 0) in vec3 vColor;
layout(location = 0) out vec4 outColor;

void main()
{
    outColor = vec4(vColor, 1.0);
}
