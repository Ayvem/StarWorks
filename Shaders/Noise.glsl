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

/// Trilinear value noise in [0,1), C2-smooth (quintic fade).
float valueNoise3(vec3 p, uint seed)
{
    vec3 cell = floor(p);
    vec3 f = p - cell;
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
    vec3 u = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
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

// ----------------------------------------------------------------------------
// IMPACT CRATERS. The CPU twin carries the full reasoning; the short version:
// a crater is a dark floor, a bright rim crest at 0.62 of the structure's
// radius and an ejecta blanket thinning out to 1.0, drawn on a hashed cubic
// lattice. Two facts keep it cheap and keep it looking like geology —
//   a site's influence never exceeds 0.48 of a cell, so the 2x2x2 block at
//   floor(p - 0.5) provably contains every site that can reach p and the
//   textbook 3x3x3 search is 19 cells of wasted hashing;
//   contributions are SUMMED, not resolved to the nearest site, because a
//   nearest-site field cuts every overlap along a Voronoi chord and a straight
//   chord across a crater rim reads as a scar.
// ----------------------------------------------------------------------------

/// One size class of impact craters, as a signed brightness field around zero:
/// below it on the floor, above it on the rim and the ejecta, exactly zero on
/// ground no impact has touched.
float craterField3(vec3 p, uint seed)
{
    ivec3 base = ivec3(floor(p - 0.5));
    // RIM WOBBLE. A crater excavated from real rock is polygonal, not
    // circular: the cavity opens along joints that were already there, and
    // perfect circles are the loudest tell that a field like this was
    // generated. Scaling every site's radius by a noise read at six times the
    // cell frequency costs one sample and puts three or four lobes around a
    // large rim. Read at the SAMPLE point, not the site, so one evaluation
    // serves all eight cells and neighbours wobble together.
    float wobble = valueNoise3(p * 6.0 + vec3(11.7, 3.9, 27.1), seed + 8123u) - 0.5;
    float sum = 0.0;
    for (int k = 0; k < 2; ++k)
    {
        for (int j = 0; j < 2; ++j)
        {
            for (int i = 0; i < 2; ++i)
            {
                ivec3 c = base + ivec3(i, j, k);
                // Two sites per cell: a cratered surface is SATURATED, and
                // one ball of mean radius 0.25 per unit cell covers only 6%
                // of the ground — five size classes of that read as scattered
                // dots on a smooth ball. Two sites take one class to 19%.
                for (int n = 0; n < 2; ++n)
                {
                    uint sub = seed + uint(n) * 26417u;
                    vec3 site = vec3(c) + vec3(latticeHash(c, sub),
                                               latticeHash(c, sub + 5051u),
                                               latticeHash(c, sub + 7919u));
                    // Impactor populations are power laws: many small craters
                    // for every large one. Squaring a uniform hash is the
                    // cheapest honest approximation of that shape, and 0.48
                    // is exactly what makes the eight-cell search exact.
                    // (0.13 keeps the smallest from vanishing; with the
                    // wobble the largest reaches 0.477, and 0.5 is where the
                    // eight-cell search would start missing sites.)
                    float sizeHash = latticeHash(c, sub + 1699u);
                    float radius = (0.13 + 0.32 * sizeHash * sizeHash) *
                                   (1.0 + wobble * 0.24);
                    float offset = length(site - p);
                    if (offset < radius)
                    {
                        // u = 0 at the impact point, 1 at the outer edge of
                        // the ejecta. Where the FLOOR ends is the number that
                        // decides whether these read as craters: end it at 0.3
                        // and every one comes out a black dot in a fat white
                        // ring — a fried egg, not a bowl.
                        float u = offset / radius;
                        float bowl = 1.0 - smoothstepf(0.34, 0.56, u);
                        float rim = smoothstepf(0.50, 0.64, u) *
                                    (1.0 - smoothstepf(0.64, 0.76, u));
                        float blanket = smoothstepf(0.70, 0.80, u) *
                                        (1.0 - smoothstepf(0.80, 1.0, u));
                        // Freshness: bright the day it is made, gardened
                        // back into the background by micrometeorites over a
                        // few hundred million years.
                        float fresh = 0.30 + 0.70 * latticeHash(c, sub + 3803u);
                        sum += fresh * (1.15 * rim + 0.42 * blanket - 0.95 * bowl);
                    }
                }
            }
        }
    }
    return sum;
}

/// The mean of craterField3 over uniformly distributed p, measured over four
/// million samples. Positive — rim and ejecta cover about three times the
/// ground the floors do — so a ladder that fades a class out with distance
/// has to give it back or the body changes brightness as the camera moves.
const float kCraterFieldMean = 0.0248;

/// One class of YOUNG impacts: a bright fresh crater and the RAY SYSTEM it
/// threw. Its own coarse lattice, and a small crater on it — rim at 0.12 of
/// the influence radius, rays out to 1.0, a reach of eight crater radii.
/// Tycho's rays reach thirty-five, but at thirty-five every system on the
/// globe overlaps every other and the moon turns white.
float craterRays3(vec3 p, uint seed, out float crater)
{
    crater = 0.0;
    ivec3 base = ivec3(floor(p - 0.5));
    float best = 4.0;
    vec3 bestSite = p;
    float bestYouth = 0.0;
    for (int k = 0; k < 2; ++k)
    {
        for (int j = 0; j < 2; ++j)
        {
            for (int i = 0; i < 2; ++i)
            {
                ivec3 c = base + ivec3(i, j, k);
                vec3 site = vec3(c) + vec3(latticeHash(c, seed),
                                           latticeHash(c, seed + 4231u),
                                           latticeHash(c, seed + 6151u));
                float offset = length(site - p);
                if (offset < best)
                {
                    best = offset;
                    bestSite = site;
                    bestYouth = latticeHash(c, seed + 8677u);
                }
            }
        }
    }
    // Nearest site only, not summed: overlapping ray systems are rare
    // enough that drawing the nearer one saves seven of the eight radial
    // noise samples for nothing visible. One site in five is young enough.
    float young = smoothstepf(0.80, 0.93, bestYouth);
    if (young <= 0.0 || best >= 0.48)
    {
        return 0.0;
    }

    float t = best / 0.48;
    float bowl = 1.0 - smoothstepf(0.055, 0.105, t);
    float rim = smoothstepf(0.070, 0.120, t) * (1.0 - smoothstepf(0.120, 0.185, t));
    float blanket = smoothstepf(0.150, 0.220, t) * (1.0 - smoothstepf(0.220, 0.380, t));
    crater = young * (1.20 * rim + 0.75 * blanket - 0.90 * bowl);

    // THE RAYS, without a trigonometric function anywhere. Noise sampled at
    // a point NORMALISED about the impact site has the same value all the way
    // along every line leaving it, so the field is radial by construction;
    // adding back a little of the un-normalised offset breaks the filaments
    // into segments instead of bicycle spokes, and adding the site position
    // decorrelates one system from the next.
    vec3 spoke = p - bestSite;
    vec3 unit = spoke / max(best, 1.0e-6);
    float rayNoise = fbm3(unit * 9.0 + bestSite * 0.7 + spoke * 1.7, 2, seed + 271u);
    float filaments = smoothstepf(0.40, 0.56, rayNoise);
    // They start outside the blanket, not at the rim: the ground between
    // blanket and rays is the darkest part of a young structure.
    float radial = smoothstepf(0.13, 0.24, t) * (1.0 - smoothstepf(0.24, 1.0, t));
    return young * filaments * radial;
}

/// The whole impact record of a body: `classes` general size classes a factor
/// 2.9 apart, plus three classes of young rayed impacts a factor 3.6 apart.
///
/// `footprint` is the angular size of one sample in the same radians `dir` is
/// swept through — the fragment path passes its pixel footprint, the vertex
/// path the reciprocal of its mesh's Nyquist frequency. Each class is faded
/// out once its cells stop covering enough samples to be anything but noise,
/// and the loop BREAKS there: from orbit a body costs two classes, on final
/// approach it costs nine.
float craterTerrain3(vec3 dir, float baseFrequency, uint seed, float footprint,
                     int classes, out float rays)
{
    float field = 0.0;
    float mean = 0.0;
    rays = 0.0;

    float frequency = baseFrequency;
    float weight = 1.0;
    for (int i = 0; i < classes; ++i)
    {
        // Full strength while a cell spans about seventeen samples, gone by
        // five: a crater is half a cell across and its rim a tenth of one, so
        // past five samples per cell the class can only crawl.
        float fade = 1.0 - smoothstepf(0.060, 0.200, footprint * frequency);
        if (fade <= 0.0)
        {
            break;
        }
        field += weight * fade * craterField3(dir * frequency, seed + uint(i) * 613u);
        mean += weight * fade * kCraterFieldMean;
        frequency *= 2.9;
        weight *= 0.94;
    }

    float rayFrequency = baseFrequency * 0.8;
    float rayWeight = 1.0;
    for (int i = 0; i < 3; ++i)
    {
        // Four times earlier than the general population, because a ray is
        // a tenth the width of the crater that threw it: unfaded, the rays
        // were the one part of this that sparkled.
        float fade = 1.0 - smoothstepf(0.015, 0.060, footprint * rayFrequency);
        if (fade <= 0.0)
        {
            break;
        }
        float fresh = 0.0;
        float streaks =
            craterRays3(dir * rayFrequency, seed + 4409u + uint(i) * 877u, fresh);
        field += rayWeight * fade * fresh;
        rays += rayWeight * fade * streaks;
        rayFrequency *= 3.6;
        rayWeight *= 0.72;
    }
    return field - mean;
}

#endif // SW_NOISE_GLSL
