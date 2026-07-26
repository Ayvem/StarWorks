#version 450

// ============================================================================
// Triangle.vert — first triangle of the engine.
// Vertices are generated from gl_VertexIndex (no vertex buffers yet; those
// arrive with the GPU memory allocator in the next milestone).
// Positions are in world space; the camera's view-projection matrix arrives
// as a push constant (must match TrianglePushConstants in Renderer.cpp).
// ============================================================================

layout(push_constant) uniform PushConstants
{
    mat4 viewProjection;
} pc;

layout(location = 0) out vec3 vColor;

// Counter-clockwise in world space (+Y up, camera looking down -Z).
const vec3 kPositions[3] = vec3[](
    vec3( 0.0,  0.6, 0.0),
    vec3(-0.6, -0.6, 0.0),
    vec3( 0.6, -0.6, 0.0)
);

const vec3 kColors[3] = vec3[](
    vec3(1.0, 0.35, 0.1),  // thruster orange
    vec3(0.2, 0.6, 1.0),   // ice blue
    vec3(0.9, 0.9, 0.95)   // hull white
);

void main()
{
    gl_Position = pc.viewProjection * vec4(kPositions[gl_VertexIndex], 1.0);
    vColor = kColors[gl_VertexIndex];
}
