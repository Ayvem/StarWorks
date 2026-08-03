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
            // QUINTIC FADE, and it is not a refinement — it is the difference between
        // a planet and a planet with a grid drawn on it.
        //
        // The cubic smoothstep (3t^2 - 2t^3) has zero FIRST derivative at the
        // lattice planes and a discontinuous SECOND one. Value noise built on it
        // is C1: the value never jumps, so nothing looks wrong in isolation — but
        // every operation that sharpens the field (a smoothstep plateau, a belt
        // edge, a high octave read at close range) differentiates it, and out
        // comes the lattice. On a gas giant, where the churn octaves are sampled
        // six times finer in latitude than in longitude, that lattice projects
        // onto the sphere as a LATITUDE-LONGITUDE GRID, four grey levels deep,
        // over the whole disc. It was in every screenshot of Saturn ever taken of
        // this game and it was blamed on the mesh three times before a headless
        // capture and a high-pass filter found it here.
        //
        // The quintic (6t^5 - 15t^4 + 10t^3) is Perlin's own 2002 correction: same
        // endpoints, same first derivative, and a second derivative that is zero
        // at both ends too. The lattice stops existing. It costs two multiplies.
        const Vec3 u = f * f * f * (f * (f * 6.0f - 15.0f) + 10.0f);
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

    // ------------------------------------------------------------------------
    // IMPACT CRATERS.
    //
    // Every body with no atmosphere and no resurfacing engine is a four
    // billion year record of bombardment, and that record has a SHAPE. From
    // the centre outward a simple crater is: a floor, darker than its
    // surroundings because it holds impact melt and because it is the one
    // part of the structure that never sees a low sun; a wall climbing to a
    // rim crest at roughly half the structure's radius, bright because it is
    // freshly excavated bedrock; and past that crest a blanket of ejecta,
    // brightest at the rim and thinning to nothing about one crater radius
    // further out. The youngest few per cent add RAYS — filaments of ejecta
    // that flew clear of the blanket entirely and are still bright because
    // nothing has gardened them yet. Rays are the single most recognisable
    // thing about Luna at full phase, and no amount of fBm will ever draw
    // one, because fBm has no notion of a CENTRE.
    //
    // The construction is the classical cellular (Worley) one: one impact
    // site per lattice cell, placed by three hashes of the cell index, and
    // the profile read off the distance from the sample point to that site.
    // Two decisions are worth the ink:
    //
    //   THE SEARCH IS EIGHT CELLS, NOT TWENTY-SEVEN. A site sits anywhere in
    //   its cell and its influence never exceeds 0.48 of a cell. Take the
    //   2x2x2 block at floor(p - 0.5): per axis it drops the cell on the far
    //   side of p, whose sites are all at least 1 - frac(p) > 0.5 away when
    //   frac(p) < 0.5, and at least frac(p) >= 0.5 away otherwise. So every
    //   site that can possibly reach p is in the block, and the textbook
    //   3x3x3 loop is 19 cells of wasted hashing.
    //
    //   CONTRIBUTIONS ARE SUMMED, NOT RESOLVED TO THE NEAREST SITE. A
    //   nearest-site field cuts every overlap along a Voronoi chord, and a
    //   straight chord across a crater rim reads as a scar, not as geology.
    //   Summing draws what overlapping craters actually look like: the
    //   younger rim running across the older floor.
    //
    // MIRROR CONTRACT: Shaders/Noise.glsl carries all of this line for line.
    // The one unavoidable difference is the lattice hash call — the GLSL
    // takes an ivec3, the C++ takes three ints — exactly as valueNoise3
    // already does.
    // ------------------------------------------------------------------------

    /// One size class of impact craters, as a signed brightness field: below
    /// zero on the floor, above it on the rim crest and on the ejecta, and
    /// exactly zero on ground no impact has touched (so classes can be summed
    /// and each faded out on its own scale).
    [[nodiscard]] inline f32 craterField3(const Vec3& p, u32 seed)
    {
        const Vec3 cell = glm::floor(p - 0.5f);
        const i32 bx = static_cast<i32>(cell.x);
        const i32 by = static_cast<i32>(cell.y);
        const i32 bz = static_cast<i32>(cell.z);
        // RIM WOBBLE. A crater excavated out of real rock is polygonal, not
        // circular: the cavity opens along whatever joints and faults were
        // already there. Perfect circles are the single loudest tell that a
        // field like this was generated, and they cost one noise sample to
        // lose — scaling every site's radius by a value noise read at SIX
        // times the cell frequency puts three or four lobes around a large
        // crater and leaves a small one alone. Reading it at the sample point
        // rather than at the site means one evaluation serves all eight cells
        // and neighbouring craters wobble together, as ground sharing a
        // fracture set does.
        const f32 wobble =
            valueNoise3(p * 6.0f + Vec3{11.7f, 3.9f, 27.1f}, seed + 8123u) - 0.5f;
        f32 sum = 0.0f;
        for (i32 k = 0; k < 2; ++k)
        {
            for (i32 j = 0; j < 2; ++j)
            {
                for (i32 i = 0; i < 2; ++i)
                {
                    const i32 cx = bx + i;
                    const i32 cy = by + j;
                    const i32 cz = bz + k;
                    // TWO sites per cell, not one. A cratered surface is
                    // SATURATED — new craters land on old ones and there is
                    // no unmarked ground left. One site per cell puts a ball
                    // of mean radius 0.23 in a unit cell, which covers 6% of
                    // the ground; even five size classes of that read as
                    // scattered dots on a smooth ball. Two sites and a
                    // fatter size floor take one class to 19%, and five
                    // classes to about two thirds of the surface, which is
                    // what the lunar highlands actually look like.
                    for (i32 n = 0; n < 2; ++n)
                    {
                        const u32 sub = seed + static_cast<u32>(n) * 26417u;
                        const Vec3 site =
                            Vec3{static_cast<f32>(cx), static_cast<f32>(cy),
                                 static_cast<f32>(cz)} +
                            Vec3{latticeHash(cx, cy, cz, sub),
                                 latticeHash(cx, cy, cz, sub + 5051u),
                                 latticeHash(cx, cy, cz, sub + 7919u)};
                        // Crater populations are power laws — the count above
                        // a diameter D goes as roughly D^-2, so small craters
                        // outnumber large ones by orders of magnitude.
                        // Squaring a uniform hash is the cheapest thing that
                        // has that shape, and it is the difference between a
                        // bombarded surface and a tray of identical dents.
                        // 0.13 keeps the smallest ones from vanishing; the
                        // 0.45 ceiling is not a taste decision — with the
                        // wobble it reaches 0.477, and anything at or past
                        // 0.5 would put sites outside the eight-cell search.
                        const f32 sizeHash = latticeHash(cx, cy, cz, sub + 1699u);
                        const f32 radius = (0.13f + 0.32f * sizeHash * sizeHash) *
                                           (1.0f + wobble * 0.24f);
                        const f32 offset = glm::length(site - p);
                        if (offset < radius)
                        {
                            // u = 0 at the impact point, 1 at the outer
                            // edge of the continuous ejecta; the rim crest
                            // sits at 0.62, so the blanket is 0.6 crater
                            // radii wide (the low end of what the lunar
                            // record shows — anything wider merges
                            // neighbouring blankets into one bright wash).
                            //
                            // The one number that decides whether these read
                            // as craters at all is where the FLOOR ends. Put
                            // it at 0.3 of the rim radius, as a first pass
                            // did, and every crater comes out as a small
                            // black dot inside a fat white ring: a fried
                            // egg, not a bowl. Real simple craters are floor
                            // most of the way out to the wall, with the
                            // bright rim a thin band on top of it — which is
                            // 0.30 to 0.58 here against a crest at 0.62.
                            const f32 u = offset / radius;
                            const f32 bowl = 1.0f - smoothstepf(0.34f, 0.56f, u);
                            const f32 rim = smoothstepf(0.50f, 0.64f, u) *
                                            (1.0f - smoothstepf(0.64f, 0.76f, u));
                            const f32 blanket = smoothstepf(0.70f, 0.80f, u) *
                                                (1.0f - smoothstepf(0.80f, 1.0f, u));
                            // Freshness. A crater is bright the day it is
                            // made and fades into its surroundings as
                            // micrometeorites garden the exposed rock back to
                            // regolith; the floor of 0.30 is the ghost
                            // craters that are barely there at all.
                            const f32 fresh =
                                0.30f + 0.70f * latticeHash(cx, cy, cz, sub + 3803u);
                            sum += fresh *
                                   (1.15f * rim + 0.42f * blanket - 0.95f * bowl);
                        }
                    }
                }
            }
        }
        return sum;
    }

    /// The mean of craterField3 over uniformly distributed p, measured over
    /// four million samples. It is POSITIVE — rim and ejecta cover about
    /// three times the ground the floors do — so a ladder that fades a class
    /// out with distance has to give this back, or the body visibly darkens
    /// as the camera pulls away and brightens as it comes in.
    inline constexpr f32 kCraterFieldMean = 0.0248f;

    /// One class of YOUNG impacts: a bright fresh crater plus the ray system
    /// it threw. Returns the rays; `crater` comes back as the crater itself.
    ///
    /// Rays need a centre that stays findable from far outside the crater, so
    /// this class lives on its own coarse lattice and keeps the crater small
    /// — the rim sits at 0.12 of the influence radius, the rays run to 1.0.
    /// That is a reach of eight crater radii. Tycho's rays reach thirty-five,
    /// but at thirty-five every system on the globe overlaps every other and
    /// the moon turns white.
    [[nodiscard]] inline f32 craterRays3(const Vec3& p, u32 seed, f32& crater)
    {
        crater = 0.0f;
        const Vec3 cell = glm::floor(p - 0.5f);
        const i32 bx = static_cast<i32>(cell.x);
        const i32 by = static_cast<i32>(cell.y);
        const i32 bz = static_cast<i32>(cell.z);
        f32 best = 4.0f;
        Vec3 bestSite = p;
        f32 bestYouth = 0.0f;
        for (i32 k = 0; k < 2; ++k)
        {
            for (i32 j = 0; j < 2; ++j)
            {
                for (i32 i = 0; i < 2; ++i)
                {
                    const i32 cx = bx + i;
                    const i32 cy = by + j;
                    const i32 cz = bz + k;
                    const Vec3 site =
                        Vec3{static_cast<f32>(cx), static_cast<f32>(cy),
                             static_cast<f32>(cz)} +
                        Vec3{latticeHash(cx, cy, cz, seed),
                             latticeHash(cx, cy, cz, seed + 4231u),
                             latticeHash(cx, cy, cz, seed + 6151u)};
                    const f32 offset = glm::length(site - p);
                    if (offset < best)
                    {
                        best = offset;
                        bestSite = site;
                        bestYouth = latticeHash(cx, cy, cz, seed + 8677u);
                    }
                }
            }
        }
        // Nearest site only, not summed: two ray systems that overlap are
        // rare enough that drawing the nearer one costs nothing, and it saves
        // seven of the eight radial noise samples below.
        //
        // One site in five survives the age test. That sounds generous until
        // you notice that a class only ever draws the sites within half a
        // cell of the sample point, so the sky-visible count is what a
        // hemisphere of Luna actually carries: a handful of loud systems and
        // a scattering of small bright pits.
        const f32 young = smoothstepf(0.80f, 0.93f, bestYouth);
        if (young <= 0.0f || best >= 0.48f)
        {
            return 0.0f;
        }

        const f32 t = best / 0.48f;
        const f32 bowl = 1.0f - smoothstepf(0.055f, 0.105f, t);
        const f32 rim =
            smoothstepf(0.070f, 0.120f, t) * (1.0f - smoothstepf(0.120f, 0.185f, t));
        const f32 blanket =
            smoothstepf(0.150f, 0.220f, t) * (1.0f - smoothstepf(0.220f, 0.380f, t));
        crater = young * (1.20f * rim + 0.75f * blanket - 0.90f * bowl);

        // THE RAYS, without a single trigonometric function.
        //
        // Sample the noise at a point that has been NORMALISED about the
        // impact site and its value is constant along every line leaving that
        // site: the field is radial by construction. Adding a little of the
        // un-normalised offset back in breaks the filaments into segments
        // with distance, which is what stops them reading as bicycle spokes;
        // adding the site position decorrelates one system from the next.
        const Vec3 spoke = p - bestSite;
        const Vec3 unit = spoke / std::max(best, 1.0e-6f);
        const f32 rayNoise =
            fbm3(unit * 9.0f + bestSite * 0.7f + spoke * 1.7f, 2, seed + 271u);
        // Frequency 9 puts about sixty features around the site; keeping the
        // top quarter of a 2-octave fBm (which spans [0, 0.75]) leaves the
        // fifteen or so rays a fresh crater really has.
        const f32 filaments = smoothstepf(0.40f, 0.56f, rayNoise);
        // They start outside the ejecta blanket, not at the rim: the ground
        // between blanket and rays is the darkest part of a young structure.
        const f32 radial =
            smoothstepf(0.13f, 0.24f, t) * (1.0f - smoothstepf(0.24f, 1.0f, t));
        return young * filaments * radial;
    }

    /// The whole impact record of a body in one call: `classes` general size
    /// classes a factor 2.9 apart, plus three classes of young rayed impacts
    /// a factor 3.6 apart.
    ///
    /// `footprint` is the angular size of one sample in the same radians `dir`
    /// is swept through — the fragment path passes its pixel footprint, the
    /// vertex path passes the reciprocal of its mesh's Nyquist frequency.
    /// Every class is faded out the moment its cells stop covering enough of
    /// those samples to be anything but noise, and the loop breaks there:
    /// from orbit a body costs two classes, on final approach it costs eight.
    [[nodiscard]] inline f32 craterTerrain3(const Vec3& dir, f32 baseFrequency,
                                            u32 seed, f32 footprint, i32 classes,
                                            f32& rays)
    {
        f32 field = 0.0f;
        f32 mean = 0.0f;
        rays = 0.0f;

        f32 frequency = baseFrequency;
        f32 weight = 1.0f;
        for (i32 i = 0; i < classes; ++i)
        {
            // Full strength while a cell spans about seventeen samples, gone
            // by five. A crater is roughly half a cell across and its rim is
            // a tenth of one, so five samples per cell is already a rim
            // thinner than the sample: past that the class can only crawl.
            const f32 fade = 1.0f - smoothstepf(0.060f, 0.200f, footprint * frequency);
            if (fade <= 0.0f)
            {
                break;
            }
            field += weight * fade *
                     craterField3(dir * frequency, seed + static_cast<u32>(i) * 613u);
            mean += weight * fade * kCraterFieldMean;
            // 2.9 rather than 2: six classes then span three decades of
            // diameter, which on Luna is 140 km walled plains down to 700 m
            // pits. Doubling would need ten classes for the same reach and
            // each one costs eight cells of hashing. Note that the last
            // classes are FREE from orbit — the break above never reaches
            // them until a pixel covers a few hundred metres of ground.
            frequency *= 2.9f;
            weight *= 0.94f;
        }

        // The young ones fade out four times earlier than the general
        // population, because a ray is a tenth the width of the crater that
        // threw it: unfaded, they were the one part of this that sparkled.
        f32 rayFrequency = baseFrequency * 0.8f;
        f32 rayWeight = 1.0f;
        for (i32 i = 0; i < 3; ++i)
        {
            const f32 fade =
                1.0f - smoothstepf(0.015f, 0.060f, footprint * rayFrequency);
            if (fade <= 0.0f)
            {
                break;
            }
            f32 fresh = 0.0f;
            const f32 streaks = craterRays3(
                dir * rayFrequency, seed + 4409u + static_cast<u32>(i) * 877u, fresh);
            field += rayWeight * fade * fresh;
            rays += rayWeight * fade * streaks;
            rayFrequency *= 3.6f;
            rayWeight *= 0.72f;
        }
        return field - mean;
    }
} // namespace sw::math
