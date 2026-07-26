// ============================================================================
// Shaders/Clouds.glsl — M28: weather, and the shadow it throws.
//
// The cloud DECK is one analytic function of (body-frame direction, time).
// That is the whole trick: the shell fragment shader evaluates it to draw
// clouds, and the GROUND fragment shader evaluates the SAME function at the
// point where its ray to the sun pierces the shell, to know whether it is in
// shade. One definition, two consumers — a cloud cannot cast a shadow that
// does not match its own shape, because there is nothing to keep in sync.
//
// Two layers, drifting at different rates like real weather:
//   CUMULUS — low, opaque, domain-warped, hard-thresholded (defined edges);
//   CIRRUS  — high, sheer, stretched into veils, faster.
//
// The shell mesh is glued to the body's rotation; ALL drift happens here, so
// the ground path can reproduce it exactly from the world clock alone.
// ============================================================================
#ifndef SW_CLOUDS_GLSL
#define SW_CLOUDS_GLSL

#include "Noise.glsl"

/// Shell altitude as a fraction of the body radius (the game builds the
/// cloud mesh at 1.005 — keep the two in step).
const float kCloudShellRatio = 1.005;

/// Drift rates, radians per second around the body's axis. Cumulus lag the
/// ground slightly; cirrus run ahead of it.
const float kCumulusDrift = 5.4e-6;
const float kCirrusDrift = 1.9e-5;

/// Base frequencies on the unit sphere. These are the numbers that decide
/// HOW BIG a cloud is, and they were the single worst inherited constant in
/// the renderer: the old per-vertex shell used 3.6, i.e. cells some 1,800 km
/// across. From a 400 km orbit that is ONE cloud filling the entire screen —
/// which reads as a grey film over the planet, not as weather. At 22 the
/// cells are ~290 km with detail down to a few km: fields with gaps, and an
/// ocean you can see between them.
const float kCumulusFrequency = 22.0;
const float kCirrusFrequency = 26.0;

/// Octave budget for a cloud fragment, from its screen footprint in radians
/// of body angle — the same rule the terrain uses, for the same reason.
float cloudLodOctaves(float footprint, float maxOctaves)
{
    float ratio = 1.0 / max(kCumulusFrequency * footprint * 2.5, 1.0e-9);
    return clamp(log2(max(ratio, 1.0)) / log2(2.03), 2.0, maxOctaves);
}

/// Rotate a body-frame direction around the Y axis (the spin axis of every
/// body in the game so far). Cheap and exact — no matrix, no uniform.
vec3 rotateAroundAxis(vec3 dir, float angle)
{
    float s = sin(angle);
    float c = cos(angle);
    return vec3(dir.x * c + dir.z * s, dir.y, dir.z * c - dir.x * s);
}

/// Cumulus coverage in [0,1] — the opaque layer, and the ONLY one that
/// casts a shadow on the ground.
/// `octaves` is the LOD dial: fewer octaves when a pixel covers more sky.
float cloudCumulus(vec3 dir, float time, float octaves)
{
    vec3 drifted = rotateAroundAxis(dir, time * kCumulusDrift);
    // Domain warp: without it the cells look like a lattice of potatoes;
    // with it they shear into fronts and spirals.
    vec3 warped = drifted * kCumulusFrequency + vec3(1.7, 9.2, 3.8);
    float wx = fbm3(warped * 0.45 + vec3(11.3, 2.7, 5.1), 2, 77778u);
    float wy = fbm3(warped * 0.45 + vec3(4.9, 17.1, 8.3), 2, 77779u);
    warped += (vec3(wx, wy, wx * 0.5 + wy * 0.5) - 0.375) * 1.6;

    float field = fbm3(warped, int(octaves), 77777u);
    // A TIGHT threshold is what gives cumulus their edge; a wide one gives
    // the blurry blobs of the old per-vertex shell.
    return smoothstepf(0.50, 0.60, field);
}

/// Cirrus veils: higher, faster, stretched (the direction is squashed along
/// the axis before sampling, which shears the noise into bands).
float cloudCirrus(vec3 dir, float time, float octaves)
{
    vec3 drifted = rotateAroundAxis(dir, time * kCirrusDrift);
    vec3 stretched = drifted * kCirrusFrequency;
    stretched.y *= 2.5; // squashed along the axis: veils shear into bands
    float field = fbm3(stretched + vec3(31.7, 5.3, 19.9), int(octaves), 33111u);
    // Cirrus are RARE and THIN. A veil that covers most of the sky at a
    // quarter opacity does not read as high cloud, it reads as a dirty
    // lens — and over deep water, whose albedo is a few percent, it erases
    // the ocean entirely.
    return smoothstepf(0.56, 0.74, field) * 0.40;
}

/// Total optical coverage seen from outside, with a polar fade: the shell is
/// a UV sphere and every vertex pinches to a point at the caps, where any
/// per-fragment field turns into a star of streaks.
float cloudCoverage(vec3 dir, float time, float octaves, out float cumulus,
                    out float cirrus)
{
    cumulus = cloudCumulus(dir, time, octaves);
    cirrus = cloudCirrus(dir, time, max(octaves - 1.0, 2.0));
    float polar = abs(dir.y);
    float fade = 1.0 - smoothstepf(0.90, 0.975, polar);
    cumulus *= fade;
    cirrus *= fade;
    return clamp(cumulus + cirrus * (1.0 - cumulus), 0.0, 1.0);
}

/// Shadow cast on a ground point at `dir` (body frame) whose surface sits
/// `elevation` metres above a body of radius `radius`, lit from `sunDirBody`.
///
/// The ray from the ground toward the sun is intersected ANALYTICALLY with
/// the cloud shell (one quadratic), and the coverage is sampled where it
/// exits. No shadow map, no extra pass, and the shadow lands exactly under
/// the cloud that casts it — including the long shadows near the terminator,
/// where the ray crosses the deck far from the vertical.
float cloudShadow(vec3 dir, float elevation, float radius, vec3 sunDirBody,
                  float time, float octaves, float strength)
{
    if (strength <= 0.0)
    {
        return 1.0;
    }
    float sunUp = dot(sunDirBody, dir);
    if (sunUp <= 0.02)
    {
        return 1.0; // sun at or below the horizon: nothing left to shade
    }
    vec3 origin = dir * (radius + elevation);
    float shellRadius = radius * kCloudShellRatio;
    float b = dot(origin, sunDirBody);
    float c = dot(origin, origin) - shellRadius * shellRadius;
    float disc = b * b - c;
    if (disc <= 0.0)
    {
        return 1.0;
    }
    float t = -b + sqrt(disc); // the exit point, above the ground
    vec3 pierce = normalize(origin + sunDirBody * t);
    float cumulus = cloudCumulus(pierce, time, octaves);
    float polar = abs(pierce.y);
    cumulus *= 1.0 - smoothstepf(0.90, 0.975, polar);
    return 1.0 - cumulus * strength;
}

#endif // SW_CLOUDS_GLSL
