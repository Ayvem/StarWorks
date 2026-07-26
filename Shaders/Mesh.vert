#version 450

// ============================================================================
// Mesh.vert — static mesh vertex shader.
// Camera data arrives in a per-frame UBO (set 0, binding 0); the model
// matrix arrives as a push constant. Both layouts are mirrored in
// Renderer.hpp (CameraUniforms / MeshPushConstants) — keep them in sync.
// ============================================================================

layout(set = 0, binding = 0) uniform CameraUniforms
{
    mat4 viewProjection;
    vec4 cameraPosition;    // xyz = world position
    vec4 sunPosition;       // xyz = camera-relative sun position, w = shadow count
    vec4 shadowSpheres[8];  // xyz = camera-relative center, w = radius
    vec4 fogColorDensity;   // xyz = horizon color, w = fog density
    vec4 skyAmbient;        // xyz = sky-scattered ambient
    vec4 qualityTime;       // x = quality tier, y = world seconds, z = air style
    vec4 atmosphereBody;    // xyz = camera-relative planet centre, w = radius
} uCamera;

// Per-instance data, written by the renderer each frame. Indexed by
// gl_InstanceIndex (which starts at the draw's firstInstance), so one
// indexed draw renders a whole batch of instances of the same mesh.
// Layout mirrors Renderer::InstanceData (std430, 80-byte stride).
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

layout(location = 0) out vec3 vWorldNormal;
layout(location = 1) out vec4 vColor;
layout(location = 2) out vec2 vUv;
layout(location = 3) out vec3 vWorldPosition;
layout(location = 4) flat out float vTintAlpha; // material routing
layout(location = 5) out vec3 vModelPosition;   // planet body-frame direction
// M26: the per-fragment planet path shades in the BODY frame (that is where
// the heightfield lives), so it needs the sun direction expressed there and
// the body's radius in meters. Both are derived from the instance matrix —
// no extra uniform, no CPU work per planet.
layout(location = 6) out vec3 vSunDirBody;
layout(location = 7) flat out float vBodyRadius;
layout(location = 8) out vec3 vViewDirBody;

void main()
{
    const mat4 model = uInstances.instances[gl_InstanceIndex].model;

    // "World" here is camera-relative space: the camera sits at the origin
    // (large-world f64 positions are narrowed CPU-side).
    const vec4 worldPosition = model * vec4(inPosition, 1.0);
    gl_Position = uCamera.viewProjection * worldPosition;

    // Assumes uniform scale (engine convention for now). A dedicated normal
    // matrix will accompany skinned/non-uniformly-scaled meshes later.
    vWorldNormal = normalize(mat3(model) * inNormal);
    vColor = inColor * uInstances.instances[gl_InstanceIndex].tint;
    vUv = inUv;
    vWorldPosition = worldPosition.xyz;
    vTintAlpha = uInstances.instances[gl_InstanceIndex].tint.a;
    vModelPosition = inPosition;

    // Uniform scale is the engine convention, so mat3(model) is rotation *
    // radius: its transpose maps a world direction back into the body frame
    // (the length is irrelevant, the fragment normalizes).
    const mat3 rotationScale = mat3(model);
    vSunDirBody = transpose(rotationScale) *
                  (uCamera.sunPosition.xyz - worldPosition.xyz);
    vViewDirBody = transpose(rotationScale) * (-worldPosition.xyz);
    vBodyRadius = length(rotationScale[0]);
}
