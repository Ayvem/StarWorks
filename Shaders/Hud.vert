#version 450

// ============================================================================
// Hud.vert — screen-space pass. The instance model matrix maps glyph/panel
// local space DIRECTLY to Vulkan NDC (x right, y down); no camera involved.
// Instance layout matches Renderer::InstanceData (Mesh.vert shares it).
// ============================================================================

struct InstanceData
{
    mat4 model;
    vec4 tint;
};
layout(std430, set = 0, binding = 1) readonly buffer InstanceBuffer
{
    InstanceData instances[];
} uInstances;

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec4 inColor;
layout(location = 3) in vec2 inUv;

layout(location = 0) out vec4 vColor;

void main()
{
    gl_Position = uInstances.instances[gl_InstanceIndex].model * vec4(inPosition, 1.0);
    gl_Position.z = 0.5; // fixed depth; the HUD pipeline does not test depth
    vColor = inColor * uInstances.instances[gl_InstanceIndex].tint;
}
