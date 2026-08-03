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
/// THE CAMERA, IN BODY UNITS (unit-sphere space, so 1.0 is the surface).
/// `flat`, because it is the same for every fragment of one planet and must
/// not be interpolated: the fragment shader intersects the view ray with the
/// TRUE sphere from here, instead of trusting the interpolated mesh position.
layout(location = 9) flat out vec3 vCameraPosBody;

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
    // radius, and its transpose maps a world direction back into the body
    // frame.
    //
    // NORMALIZE BEFORE THE MATRIX, NOT AFTER. "The length is irrelevant, the
    // fragment normalizes" was true of the mathematics and false of f32.
    // Saturn is 1.43e12 m from the sun and 5.82e7 m in radius, so the old
    // product came out at 8e19 — and normalize() squares its argument first.
    // 6.9e39 is past the f32 ceiling of 3.4e38, so the length was INFINITY,
    // the direction was NaN, and every lambert term on the planet evaluated
    // to zero.
    //
    // The effect was not subtle and it was not local: EVERY OUTER PLANET IN
    // THE GAME WAS UNLIT. Jupiter, Saturn, Uranus and Neptune were drawn with
    // nothing but the 2% cold ambient, which is exactly why they looked like
    // grey balls no matter what the surface shader did. Terra survived on
    // luck — 6.37e6 x 1.5e11 squares to 9e35, one part in four hundred under
    // the ceiling.
    //
    // Normalizing first keeps every intermediate near 1.
    const mat3 rotationScale = mat3(model);
    vBodyRadius = length(rotationScale[0]);
    const mat3 rotation = rotationScale / max(vBodyRadius, 1.0e-6);
    vSunDirBody = transpose(rotation) *
                  normalize(uCamera.sunPosition.xyz - worldPosition.xyz);
    vViewDirBody = transpose(rotation) * normalize(-worldPosition.xyz);
    // model[3] is the body's centre in camera-relative space, so its negative
    // is the camera seen from the body. In body axes and body radii.
    vCameraPosBody =
        (transpose(rotation) * (-model[3].xyz)) / max(vBodyRadius, 1.0e-6);
}
