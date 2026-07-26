// ============================================================================
// Shaders/Noise.glsl — GLSL TWIN of Engine/Source/Math/Noise.hpp.
//
// Line-by-line port, same constants, same order of operations. The CPU
// header is the source of truth: physics collides with what IT returns, and
// the only reason the player sees mountains where the ship crashes is that
// these two files agree bit for bit.
//
// Verified automatically by Tools/glsl_parity/check_parity.py, which
// transpiles this file to C++ and diffs it against the header over tens of
// thousands of directions. Keep the restricted style it expects: no
// uniforms, no textures, no built-in smoothstep in shared math, explicit
// float literals.
// ============================================================================
#ifndef SW_NOISE_GLSL
#define SW_NOISE_GLSL

float hashToUnitFloat(uint x)
{
    x ^= 2747636419u;
    x *= 2654435769u;
    x ^= x >> 16;
    x *= 2654435769u;
    x ^= x >> 16;
    return float(x & 0xFFFFFFu) / 16777216.0;
}

float latticeHash(ivec3 cell, uint seed)
{
    uint h = uint(cell.x) * 374761393u + uint(cell.y) * 668265263u +
             uint(cell.z) * 1274126177u + seed * 2246822519u;
    return hashToUnitFloat(h);
}

/// Trilinear value noise in [0,1), C1-smooth (smoothstep fade).
float valueNoise3(vec3 p, uint seed)
{
    vec3 cell = floor(p);
    vec3 f = p - cell;
    vec3 u = f * f * (3.0 - 2.0 * f);
    ivec3 c = ivec3(cell);
    float x00 = mix(latticeHash(c, seed),
                    latticeHash(c + ivec3(1, 0, 0), seed), u.x);
    float x10 = mix(latticeHash(c + ivec3(0, 1, 0), seed),
                    latticeHash(c + ivec3(1, 1, 0), seed), u.x);
    float x01 = mix(latticeHash(c + ivec3(0, 0, 1), seed),
                    latticeHash(c + ivec3(1, 0, 1), seed), u.x);
    float x11 = mix(latticeHash(c + ivec3(0, 1, 1), seed),
                    latticeHash(c + ivec3(1, 1, 1), seed), u.x);
    return mix(mix(x00, x10, u.y), mix(x01, x11, u.y), u.z);
}

/// Fractal Brownian motion, ~[0,1).
float fbm3(vec3 p, int octaves, uint seed)
{
    float sum = 0.0;
    float amplitude = 0.5;
    vec3 q = p;
    for (int i = 0; i < octaves; ++i)
    {
        sum += amplitude * valueNoise3(q, seed + uint(i) * 101u);
        q *= 2.03;
        amplitude *= 0.5;
    }
    return sum;
}

/// Explicit smoothstep — the CPU twin cannot call the built-in, so neither
/// does shared math here. (Named smoothstepf like its C++ counterpart.)
float smoothstepf(float edge0, float edge1, float x)
{
    float t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

/// Ridged multifractal in ~[0,1]: folded octaves (sharp crest lines),
/// squared, and weighted by the previous octave so detail only accumulates
/// along existing ridges — mountain RANGES instead of blobs.
float ridged3(vec3 p, int octaves, uint seed)
{
    float sum = 0.0;
    float amplitude = 0.5;
    float normalization = 0.0;
    float weight = 1.0;
    vec3 q = p;
    for (int i = 0; i < octaves; ++i)
    {
        float n = valueNoise3(q, seed + uint(i) * 131u);
        float ridge = 1.0 - abs(2.0 * n - 1.0);
        ridge = ridge * ridge;
        ridge *= weight;
        weight = clamp(ridge * 2.0, 0.0, 1.0);
        sum += amplitude * ridge;
        normalization += amplitude;
        q *= 2.07;
        // Gain 0.55 (see the CPU twin): halving amplitudes make an almost
        // brownian, slope-less surface; this puts real ridge steepness back.
        amplitude *= 0.55;
    }
    return sum / normalization;
}

/// Billow noise in ~[0,1]: |2n-1|, the inverse fold of ridged. Rounded
/// lobes: foothills, dunes, basin rims.
float billow3(vec3 p, int octaves, uint seed)
{
    float sum = 0.0;
    float amplitude = 0.5;
    float normalization = 0.0;
    vec3 q = p;
    for (int i = 0; i < octaves; ++i)
    {
        float n = valueNoise3(q, seed + uint(i) * 167u);
        sum += amplitude * abs(2.0 * n - 1.0);
        normalization += amplitude;
        q *= 2.11;
        amplitude *= 0.5;
    }
    return sum / normalization;
}

/// Domain warping: one pass, three decorrelated 2-octave fBms.
/// (A 2-octave fbm3 averages ~0.375, hence the recentering constant.)
vec3 warpDomain3(vec3 p, uint seed, float strength)
{
    float wx = fbm3(p + vec3(19.13, 33.41, 7.77), 2, seed);
    float wy = fbm3(p + vec3(51.37, 11.93, 27.29), 2, seed + 17u);
    float wz = fbm3(p + vec3(3.71, 45.17, 63.59), 2, seed + 37u);
    return p + (vec3(wx, wy, wz) - 0.375) * (strength * 2.0);
}

#endif // SW_NOISE_GLSL
