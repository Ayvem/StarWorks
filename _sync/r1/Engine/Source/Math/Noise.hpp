#pragma once

// ============================================================================
// Math/Noise.hpp
// Deterministic 3D value noise + fBm family (header-only).
//
// THE single source of procedural randomness for planetary content: the
// terrain heightfield (collision!), the globe vertex colors and the cloud
// coverage all sample these functions — same seed, same result, forever.
// No std::rand, no state: pure functions of (position, seed).
//
// MIRROR CONTRACT (M25): every function below has a line-by-line GLSL twin
// in Shaders/Noise.glsl. The CPU is the source of truth; the shader is the
// port. Tools/glsl_parity/check_parity.py transpiles the GLSL and compares
// both over tens of thousands of directions — any divergence between what
// physics collides with and what the player sees is a build failure.
// ============================================================================

#include "Core/Types.hpp"
#include "Math/Math.hpp"

#include <cmath>

namespace sw::math
{
    [[nodiscard]] inline f32 hashToUnitFloat(u32 x)
    {
        x ^= 2747636419u;
        x *= 2654435769u;
        x ^= x >> 16;
        x *= 2654435769u;
        x ^= x >> 16;
        return static_cast<f32>(x & 0xFFFFFF) / 16777216.0f;
    }

    [[nodiscard]] inline f32 latticeHash(i32 x, i32 y, i32 z, u32 seed)
    {
        const u32 h = static_cast<u32>(x) * 374761393u +
                      static_cast<u32>(y) * 668265263u +
                      static_cast<u32>(z) * 1274126177u + seed * 2246822519u;
        return hashToUnitFloat(h);
    }

    /// Trilinear value noise in [0,1), C1-smooth (smoothstep fade).
    [[nodiscard]] inline f32 valueNoise3(const Vec3& p, u32 seed)
    {
        const Vec3 cell = glm::floor(p);
        const Vec3 f = p - cell;
        const Vec3 u = f * f * (3.0f - 2.0f * f);
        const i32 x = static_cast<i32>(cell.x);
        const i32 y = static_cast<i32>(cell.y);
        const i32 z = static_cast<i32>(cell.z);

        auto corner = [&](i32 dx, i32 dy, i32 dz) {
            return latticeHash(x + dx, y + dy, z + dz, seed);
        };
        const f32 x00 = glm::mix(corner(0, 0, 0), corner(1, 0, 0), u.x);
        const f32 x10 = glm::mix(corner(0, 1, 0), corner(1, 1, 0), u.x);
        const f32 x01 = glm::mix(corner(0, 0, 1), corner(1, 0, 1), u.x);
        const f32 x11 = glm::mix(corner(0, 1, 1), corner(1, 1, 1), u.x);
        return glm::mix(glm::mix(x00, x10, u.y), glm::mix(x01, x11, u.y), u.z);
    }

    /// Fractal Brownian motion, ~[0,1).
    [[nodiscard]] inline f32 fbm3(const Vec3& p, i32 octaves, u32 seed)
    {
        f32 sum = 0.0f;
        f32 amplitude = 0.5f;
        Vec3 q = p;
        for (i32 i = 0; i < octaves; ++i)
        {
            sum += amplitude * valueNoise3(q, seed + static_cast<u32>(i) * 101u);
            q *= 2.03f;
            amplitude *= 0.5f;
        }
        return sum;
    }

    // ------------------------------------------------------------------------
    // M25 — the shapes value-noise fBm alone cannot make.
    //
    // fbm3 is isotropic and round: it produces "potato" continents and hills,
    // never CRESTS or VALLEYS. The three functions below are the classical
    // fixes, all built from the very same valueNoise3 lattice (so a world's
    // seed still determines everything) and all written with explicit
    // arithmetic — no pow(), no built-in smoothstep — so the GLSL twin can
    // be bit-identical.
    // ------------------------------------------------------------------------

    /// Explicit smoothstep. Written out (instead of glm::smoothstep) because
    /// the GLSL mirror must evaluate the exact same expression.
    [[nodiscard]] inline f32 smoothstepf(f32 edge0, f32 edge1, f32 x)
    {
        const f32 t = glm::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
    }

    /// Ridged multifractal in ~[0,1]: each octave is folded (1 - |2n-1|) so
    /// its maxima become SHARP LINES instead of round blobs, squared for
    /// crest definition, and weighted by the previous octave so detail only
    /// grows where a ridge already exists (that feedback is what turns noise
    /// into mountain RANGES with spurs and cols).
    [[nodiscard]] inline f32 ridged3(const Vec3& p, i32 octaves, u32 seed)
    {
        f32 sum = 0.0f;
        f32 amplitude = 0.5f;
        f32 normalization = 0.0f;
        f32 weight = 1.0f;
        Vec3 q = p;
        for (i32 i = 0; i < octaves; ++i)
        {
            const f32 n = valueNoise3(q, seed + static_cast<u32>(i) * 131u);
            f32 ridge = 1.0f - std::abs(2.0f * n - 1.0f);
            ridge = ridge * ridge;
            ridge *= weight;
            weight = glm::clamp(ridge * 2.0f, 0.0f, 1.0f);
            sum += amplitude * ridge;
            normalization += amplitude;
            q *= 2.07f;
            // Gain 0.55, not 0.5: with lacunarity 2.07 the classical halving
            // gives an almost BROWNIAN surface (H ~ 0.95) whose slope barely
            // grows with detail — visually, mountains without steepness. A
            // slightly slower amplitude decay puts the roughness of real
            // ranges back in (H ~ 0.85) without touching their height.
            amplitude *= 0.55f;
        }
        return sum / normalization;
    }

    /// Billow noise in ~[0,1]: |2n-1| — the inverse fold of ridged. Rounded,
    /// bulging lobes: foothills, dune fields, lunar basin rims.
    [[nodiscard]] inline f32 billow3(const Vec3& p, i32 octaves, u32 seed)
    {
        f32 sum = 0.0f;
        f32 amplitude = 0.5f;
        f32 normalization = 0.0f;
        Vec3 q = p;
        for (i32 i = 0; i < octaves; ++i)
        {
            const f32 n = valueNoise3(q, seed + static_cast<u32>(i) * 167u);
            sum += amplitude * std::abs(2.0f * n - 1.0f);
            normalization += amplitude;
            q *= 2.11f;
            amplitude *= 0.5f;
        }
        return sum / normalization;
    }

    /// Domain warping: displace the sample point by a vector field made of
    /// three decorrelated 2-octave fBms. One pass is enough to break the
    /// axis-aligned lattice signature of value noise — coastlines and ridges
    /// stop looking like a grid of potatoes and start meandering.
    /// (A 2-octave fbm3 averages ~0.375, hence the recentering constant.)
    [[nodiscard]] inline Vec3 warpDomain3(const Vec3& p, u32 seed, f32 strength)
    {
        const f32 wx = fbm3(p + Vec3{19.13f, 33.41f, 7.77f}, 2, seed);
        const f32 wy = fbm3(p + Vec3{51.37f, 11.93f, 27.29f}, 2, seed + 17u);
        const f32 wz = fbm3(p + Vec3{3.71f, 45.17f, 63.59f}, 2, seed + 37u);
        return p + (Vec3{wx, wy, wz} - 0.375f) * (strength * 2.0f);
    }
} // namespace sw::math
