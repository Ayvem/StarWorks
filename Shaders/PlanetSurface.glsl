// ============================================================================
// Shaders/PlanetSurface.glsl — M26: lighting the heightfield PER PIXEL.
//
// This module is GPU-ONLY on purpose. Terrain.glsl is a strict twin of the
// CPU header (it is what physics collides with, so it may not drift by a
// single bit); everything here is a VIEW of that function — LOD sampling,
// gradients, self-shadowing, biome colors — and none of it changes where the
// ground is. Its one hard rule: at ground level, at full quality, the
// sampler below reduces EXACTLY to terrainElevationSigned().
//
// The three things that make a planet look like the reference frame:
//   1. NORMALS FROM THE HEIGHTFIELD, per pixel, at the pixel's own footprint
//      (finite differences on the real function — mountains get lit flanks
//      and shadowed valleys instead of a smooth painted blob);
//   2. AN OCTAVE LOD driven by screen footprint — without it, detail smaller
//      than a pixel crawls and boils the moment the camera moves;
//   3. BIOMES from altitude, SLOPE, latitude and humidity — bare rock where
//      the ground is steep, snow where it is high and cold, green where it
//      is wet, ochre everywhere else.
// ============================================================================
#ifndef SW_PLANET_SURFACE_GLSL
#define SW_PLANET_SURFACE_GLSL

#include "Noise.glsl"
#include "Terrain.glsl"
#include "Clouds.glsl"

// Quality tiers (uCamera.qualityTime.x).
const float kQualityLow = 0.0;
const float kQualityMedium = 1.0;
const float kQualityHigh = 2.0;

/// How much the relief gradient is amplified before it tilts the normal.
/// Planetary slopes are genuinely shallow (a 9 km range spread over 100 km
/// of ground is a 5 degree ramp); without a boost the shading is a whisper.
/// This is the SAME kind of honest cheat as the M22 vertex exaggeration —
/// it tilts normals only: silhouette, geometry and collision are untouched.
const float kReliefExaggeration = 5.5;

/// Hard ceiling on the tilt the exaggeration may produce, as a tangent
/// (1.0 = 45 degrees). Without it, exaggerating a steep ridge swings its
/// normal PAST the terminator: neighbouring pixels come out fully lit and
/// fully black, and a mountain range reads as a field of holes. This is the
/// one place where the honest cheat needs a leash.
const float kMaxNormalTilt = 1.0;

const vec3 kHumidityOffset = vec3(63.11, 27.43, 15.91);

// ----------------------------------------------------------------------------
// Fractional-octave noise: the LOD dial.
//
// An INTEGER octave count pops when it changes. These variants fade the last
// octave in and out, so detail arrives continuously as the camera descends.
// With an integral count they reduce exactly to the Noise.glsl originals.
// ----------------------------------------------------------------------------
float ridged3f(vec3 p, float octaves, uint seed)
{
    float sum = 0.0;
    float amplitude = 0.5;
    float normalization = 0.0;
    float weight = 1.0;
    vec3 q = p;
    int count = int(ceil(octaves));
    float lastWeight = 1.0 - (float(count) - octaves);
    for (int i = 0; i < count; ++i)
    {
        float fade = (i == count - 1) ? lastWeight : 1.0;
        float n = valueNoise3(q, seed + uint(i) * 131u);
        float ridge = 1.0 - abs(2.0 * n - 1.0);
        ridge = ridge * ridge;
        ridge *= weight;
        weight = clamp(ridge * 2.0, 0.0, 1.0);
        sum += amplitude * ridge * fade;
        normalization += amplitude * fade;
        q *= 2.07;
        amplitude *= 0.55; // matches ridged3 in Noise.glsl
    }
    return sum / max(normalization, 1.0e-6);
}

float billow3f(vec3 p, float octaves, uint seed)
{
    float sum = 0.0;
    float amplitude = 0.5;
    float normalization = 0.0;
    vec3 q = p;
    int count = int(ceil(octaves));
    float lastWeight = 1.0 - (float(count) - octaves);
    for (int i = 0; i < count; ++i)
    {
        float fade = (i == count - 1) ? lastWeight : 1.0;
        float n = valueNoise3(q, seed + uint(i) * 167u);
        sum += amplitude * abs(2.0 * n - 1.0) * fade;
        normalization += amplitude * fade;
        q *= 2.11;
        amplitude *= 0.5;
    }
    return sum / max(normalization, 1.0e-6);
}

/// Domain warp with a chosen octave count (2 = the CPU reference). The
/// recentering constant follows the octave count: a k-octave fbm3 averages
/// 0.5 * (1 - 2^-k).
vec3 warpDomain3o(vec3 p, uint seed, float strength, int octaves)
{
    float center = 0.5 * (1.0 - exp2(-float(octaves)));
    float wx = fbm3(p + vec3(19.13, 33.41, 7.77), octaves, seed);
    float wy = fbm3(p + vec3(51.37, 11.93, 27.29), octaves, seed + 17u);
    float wz = fbm3(p + vec3(3.71, 45.17, 63.59), octaves, seed + 37u);
    return p + (vec3(wx, wy, wz) - center) * (strength * 2.0);
}

// ----------------------------------------------------------------------------
// The LOD sampler.
//
// `land` and `belt` are the two LOW-FREQUENCY terms of the heightfield
// (continental mask ~2700 km wavelength, orogeny belts ~3400 km). Over the
// few kilometres a gradient tap or a shadow march covers they are constant,
// so the caller samples them ONCE and passes them in — that alone removes 8
// of the ~22 noise evaluations per extra tap.
// ----------------------------------------------------------------------------
float terrainElevationFast(TerrainParams terrain, vec3 dir, float land, float belt,
                           float reliefOctaves, int warpOctaves)
{
    if (land <= 0.0)
    {
        float deep = -land;
        float shelf = smoothstepf(0.0, 0.05, deep);
        float basin = smoothstepf(0.03, 0.40, deep);
        vec3 warpedSea = warpDomain3o(
            dir * (terrain.frequency * terrain.reliefFrequency * 0.5) + kAbyssOffset,
            terrain.seed + 911u, terrain.warpStrength, warpOctaves);
        float seaRidge = ridged3f(warpedSea, 3.0, terrain.seed + 977u);
        float depth = terrain.oceanDepth * (0.22 * shelf + 0.78 * basin);
        depth -= terrain.oceanDepth * 0.45 * seaRidge * basin;
        return -depth;
    }

    float coast = smoothstepf(0.0, 0.10, land);
    vec3 warped = warpDomain3o(
        dir * (terrain.frequency * terrain.reliefFrequency) + kReliefOffset,
        terrain.seed + 7u, terrain.warpStrength, warpOctaves);
    float rawRidge = ridged3f(warped, reliefOctaves, terrain.seed + 31u);
    float ridge = rawRidge * rawRidge; // see terrainElevationSignedLod
    float billow = billow3f(warped * 0.45 + kBillowOffset, 3.0, terrain.seed + 53u);

    float h = 0.42 * land * land + terrain.ridgeWeight * belt * ridge +
              terrain.plainsWeight * (1.0 - belt) * ridge +
              terrain.billowWeight * billow * (0.35 + 0.65 * land);
    h = h + terrain.erosion * (h * h - h);
    float scaled = h * terrain.terraceCount;
    float floored = floor(scaled);
    float stepped =
        (floored + smoothstepf(0.35, 1.0, scaled - floored)) / terrain.terraceCount;
    h = h + terrain.terraceStrength * belt * (stepped - h);

    if (h > 0.75) // soft ceiling, see terrainElevationSignedLod
    {
        h = 0.75 + (h - 0.75) / (1.0 + (h - 0.75) * 4.0);
    }

    float detailFade = smoothstepf(11.0, 13.0, reliefOctaves);
    if (detailFade > 0.0 && terrain.detailWeight > 0.0)
    {
        float rough = ridged3f(dir * terrain.detailFrequency + kDetailOffset,
                               (reliefOctaves >= 14.0) ? 4.0 : 3.0,
                               terrain.seed + 4111u);
        h += terrain.detailWeight * detailFade * (0.25 + 0.75 * belt) *
             (rough - 0.30);
    }
    h = max(h, 0.0);
    return h * terrain.amplitude * coast;
}

/// Orogeny belt mask — the low-frequency term the fast sampler needs.
float terrainBelt(TerrainParams terrain, vec3 dir)
{
    float orogeny =
        fbm3(dir * (terrain.frequency * 0.8) + kBeltOffset, 3, terrain.seed + 601u);
    return smoothstepf(terrain.beltThreshold, terrain.beltThreshold + 0.22, orogeny);
}

/// Octave budget for a fragment whose direction footprint is `footprint`
/// (radians of body angle covered by one pixel). Octave k of the relief has
/// wavelength 1/(frequency*reliefFrequency*2.07^k); anything finer than a
/// few pixels can only alias, so it is faded out.
float terrainLodOctaves(TerrainParams terrain, float footprint, float maxOctaves)
{
    float base = terrain.frequency * terrain.reliefFrequency;
    float ratio = 1.0 / max(base * footprint * 2.5, 1.0e-9);
    float octaves = log2(max(ratio, 1.0)) / log2(2.07);
    return clamp(octaves, 1.0, maxOctaves);
}

/// Per-pixel surface normal in the BODY frame: the analytic gradient of the
/// heightfield, taken at the pixel's own footprint (so it antialiases
/// itself) and folded into the sphere normal. `slope` comes back as the
/// gradient magnitude in metres per metre — the biome layer needs it.
vec3 terrainNormalBody(TerrainParams terrain, vec3 dir, float land, float belt,
                       float radius, float footprint, float reliefOctaves,
                       int warpOctaves, float exaggeration, float centre,
                       out float slope)
{
    // At least a couple of metres of separation: below that the difference
    // of two nearly equal floats is noise, not a gradient.
    float epsilon = max(footprint, 2.0 / max(radius, 1.0));
    vec3 reference = (abs(dir.y) < 0.95) ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
    vec3 tangentX = normalize(cross(reference, dir));
    vec3 tangentY = cross(dir, tangentX);

    // Two taps, not three: `centre` is the elevation the caller already
    // computed. It MUST come from the same sampler and the same parameters
    // as the taps below — feed it a value computed with a different octave
    // or warp count and the difference stops being a derivative and becomes
    // the offset between two functions, which reads as slopes of several
    // hundred percent and turns the planet into a field of black holes.
    float alongX = terrainElevationFast(terrain, normalize(dir + tangentX * epsilon),
                                        land, belt, reliefOctaves, warpOctaves);
    float alongY = terrainElevationFast(terrain, normalize(dir + tangentY * epsilon),
                                        land, belt, reliefOctaves, warpOctaves);

    float metres = max(radius * epsilon, 1.0e-3);
    float slopeX = (alongX - centre) / metres;
    float slopeY = (alongY - centre) / metres;
    slope = length(vec2(slopeX, slopeY));
    vec2 tilt = vec2(slopeX, slopeY) * exaggeration;
    float tiltLength = length(tilt);
    if (tiltLength > kMaxNormalTilt)
    {
        tilt *= kMaxNormalTilt / tiltLength; // same direction, sane angle
    }
    return normalize(dir - (tangentX * tilt.x + tangentY * tilt.y));
}

/// Terrain self-shadowing: march toward the sun along the ground and ask
/// whether anything rises above the light ray. Six steps with quadratic
/// spacing (dense near the shading point, where ridges actually occlude)
/// are enough to put valleys behind crests into shadow — the single effect
/// that reads as "real mountains" in the reference frame.
float terrainSelfShadow(TerrainParams terrain, vec3 dir, vec3 sunDirBody,
                        float land, float belt, float radius, float here,
                        float reliefOctaves)
{
    float sunUp = dot(sunDirBody, dir);
    if (sunUp <= 0.0)
    {
        return 1.0; // below the horizon: the lambert term already killed it
    }
    vec3 horizontal = sunDirBody - dir * sunUp;
    float horizontalLength = length(horizontal);
    if (horizontalLength < 1.0e-4)
    {
        return 1.0; // sun at the zenith: nothing can shadow anything
    }
    vec3 towardSun = horizontal / horizontalLength;
    float raySlope = sunUp / horizontalLength;

    // How far a full-amplitude obstacle could still block this ray.
    float reach = clamp(terrain.amplitude / max(raySlope, 0.08), 2.0e3, 9.0e4);
    float occlusion = 0.0;
    for (int i = 1; i <= 6; ++i)
    {
        float t = float(i) / 6.0;
        float distance = reach * t * t;
        vec3 sampleDir = normalize(dir + towardSun * (distance / radius));
        float height = terrainElevationFast(terrain, sampleDir, land, belt,
                                            reliefOctaves, 1);
        float rayHeight = here + raySlope * distance;
        // A ridge standing 8% of the amplitude (~720 m on Terra) above the
        // ray shadows fully; below that the edge fades, which is what a sun
        // of finite angular size actually draws. (4% made every fold in the
        // ground throw a hard black shadow.)
        occlusion = max(occlusion,
                        clamp((height - rayHeight) / (terrain.amplitude * 0.08),
                              0.0, 1.0));
    }
    return 1.0 - occlusion * 0.65; // never pitch black: sky light fills in
}


// ============================================================================
// M27 — THE OCEAN.
//
// Water is not a blue sphere with a shiny dot. What sells it is four things:
// depth (the sea floor from M25 finally has a use), a coastal SURF band that
// moves, micro-waves that break the specular into a scintillating streak
// instead of a hard point, and a Fresnel term that is actually Fresnel —
// water reflects 2% face-on and nearly everything at grazing angles, which
// is why the horizon-side of an ocean goes white under a low sun.
//
// The wave detail obeys the same rule as the terrain: it only exists when a
// pixel is small enough to resolve it. From orbit the waves are folded into
// a WIDER specular lobe instead — the physically honest way to draw a
// sub-pixel rough surface, and exactly what turns the glint into a trail.
// ============================================================================

/// Animated micro-wave normal in the tangent plane, plus the sub-pixel
/// roughness that survives when the waves themselves cannot be resolved.
vec3 oceanWaveNormal(vec3 dir, float bodyRadius, float footprint, float time,
                     out float resolved)
{
    // ~260 m swell and its ~90 m chop, drifting in different directions.
    float footprintMetres = footprint * bodyRadius;
    resolved = 1.0 - smoothstepf(60.0, 260.0, footprintMetres);
    if (resolved <= 0.001)
    {
        return dir;
    }
    vec3 reference = (abs(dir.y) < 0.95) ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
    vec3 tangentX = normalize(cross(reference, dir));
    vec3 tangentY = cross(dir, tangentX);

    float swell = bodyRadius / 260.0;
    float chop = bodyRadius / 90.0;
    vec3 driftA = vec3(0.031, 0.0, 0.017) * time;
    vec3 driftB = vec3(-0.019, 0.011, 0.026) * time;
    float epsilon = 0.35 / swell; // finite difference across ~1/3 of a swell

    // Two octaves, gradient by finite differences in the tangent plane.
    float here = valueNoise3(dir * swell + driftA, 8081u) +
                 0.45 * valueNoise3(dir * chop + driftB, 8082u);
    float alongX = valueNoise3(normalize(dir + tangentX * epsilon) * swell + driftA, 8081u) +
                   0.45 * valueNoise3(normalize(dir + tangentX * epsilon) * chop + driftB, 8082u);
    float alongY = valueNoise3(normalize(dir + tangentY * epsilon) * swell + driftA, 8081u) +
                   0.45 * valueNoise3(normalize(dir + tangentY * epsilon) * chop + driftB, 8082u);

    // Wave steepness in metres of height per metre of ground.
    float amplitude = 1.6; // metres, peak to trough of the swell
    float metres = max(epsilon * bodyRadius, 1.0);
    vec2 gradient = vec2(alongX - here, alongY - here) * (amplitude / metres);
    return normalize(dir - (tangentX * gradient.x + tangentY * gradient.y) * resolved);
}

/// Sea surface: color by depth, coastal foam, waves, ice. `water` comes back
/// as 1 for open water (the caller switches to a water Fresnel), 0 for ice.
void planetOcean(TerrainParams terrain, vec3 dir, float elevation, float latitude,
                 float iceEdge, float bodyRadius, float footprint, float time,
                 float detail, out vec3 albedo, out float specular, out float gloss,
                 out vec3 normalBody, out float water)
{
    float depth = clamp(-elevation / max(terrain.oceanDepth, 1.0), 0.0, 1.0);
    float shelf = 1.0 - smoothstepf(0.02, 0.30, depth);

    // Depth palette: abyssal plain -> open ocean -> turquoise shelf. The
    // shelf band is exactly where the M25 bathymetry rises toward the coast.
    vec3 abyss = vec3(0.008, 0.032, 0.105);
    vec3 open = vec3(0.020, 0.105, 0.250);
    vec3 shallows = vec3(0.065, 0.330, 0.410);
    albedo = mix(mix(abyss, open, 1.0 - depth), shallows, shelf * shelf) *
             (1.0 + detail * 0.06);

    float resolved = 0.0;
    normalBody = oceanWaveNormal(dir, bodyRadius, footprint, time, resolved);

    // SURF: the band where the sea floor comes up to meet the beach, keyed
    // to the REAL depth in metres (waves break in the last few metres of
    // water, so the band is narrow on a steep coast and wide on a flat one —
    // for free, because the bathymetry decides).
    float surfBand = 1.0 - smoothstepf(1.0, 12.0, -elevation);
    float churn = fbm3(dir * 260.0 + vec3(0.0, time * 0.07, 0.0), 2, 6161u);
    float foam = clamp(surfBand * (0.55 + 1.8 * (churn - 0.40)), 0.0, 1.0);
    albedo = mix(albedo, vec3(0.82, 0.87, 0.90), foam * 0.9);

    // Sub-pixel waves widen the highlight: that is the sun GLITTER TRAIL.
    // Foam is rough and kills it locally.
    gloss = mix(0.52, 0.90, resolved) * (1.0 - 0.75 * foam);
    specular = 1.0;
    water = 1.0;

    // Polar sea ice, with a ragged edge and a low sheen.
    float ice = smoothstepf(iceEdge, iceEdge + 0.02, latitude);
    if (ice > 0.0)
    {
        vec3 iceAlbedo = vec3(0.88, 0.92, 0.95) * (1.0 + detail * 0.10);
        albedo = mix(albedo, iceAlbedo, ice);
        gloss = mix(gloss, 0.45, ice);
        specular = mix(specular, 0.30, ice);
        normalBody = mix(normalBody, dir, ice);
        water = 1.0 - ice;
    }
}

/// Terra's polar cap edge: a latitude with a soft noisy boundary, shared by
/// the land biome and the sea ice so the cap is ONE coherent shape.
float terraIceEdge(vec3 dir)
{
    return 0.91 + 0.03 * (fbm3(dir * 6.0, 3, 555u) - 0.5);
}

// ----------------------------------------------------------------------------
// BIOMES — albedo from altitude, slope, latitude and humidity.
// Mirrored on the CPU by colorizeSurfaceVertex() (far LOD vertex colors).
// ----------------------------------------------------------------------------
void planetBiome(TerrainParams terrain, int style, vec3 dir, float elevation,
                 float land, float slope, out vec3 albedo, out float specular,
                 out float gloss)
{
    specular = 0.0;
    gloss = 0.0;
    float detail = fbm3(dir * 42.0, 3, 90210u) - 0.5;
    float relief = clamp(elevation / terrain.amplitude, 0.0, 1.0);
    float latitude = abs(dir.y);
    float rock = smoothstepf(0.025, 0.09, slope);

    if (style == 0) // Terra
    {
        // The cap edge costs three noise samples and only matters near the
        // poles; everywhere else a constant keeps every comparison false.
        float iceEdge = (latitude > 0.84) ? terraIceEdge(dir) : 1.0;

        // Humidity: wet along the coasts and in the wet belts, dry deep
        // inside continents — this is what makes the world read as ARID
        // with green edges instead of a uniform golf course.
        float wet = fbm3(dir * 1.9 + kHumidityOffset, 3, terrain.seed + 3131u);
        float humidity = clamp(wet * 1.9 - 0.30 +
                                   0.40 * (1.0 - smoothstepf(0.0, 0.30, land)) -
                                   0.35 * smoothstepf(0.12, 0.55, relief),
                               0.0, 1.0);

        vec3 desert = vec3(0.615, 0.505, 0.310);
        vec3 steppe = vec3(0.470, 0.425, 0.240);
        vec3 grass = vec3(0.245, 0.360, 0.150);
        vec3 forest = vec3(0.120, 0.245, 0.105);
        vec3 ground = mix(desert, steppe, smoothstepf(0.15, 0.45, humidity));
        ground = mix(ground, grass, smoothstepf(0.42, 0.64, humidity));
        ground = mix(ground, forest, smoothstepf(0.64, 0.86, humidity));

        // Beaches: only where the ground is flat and just above the water.
        float beach = (1.0 - smoothstepf(0.0, 70.0, elevation)) * (1.0 - rock);
        ground = mix(ground, vec3(0.720, 0.660, 0.460), beach);

        // Soil thins with altitude, then gives way to bare rock on slopes.
        ground = mix(ground, vec3(0.420, 0.360, 0.245),
                     smoothstepf(0.10, 0.42, relief));
        vec3 stone = vec3(0.380, 0.350, 0.320) * (1.0 + detail * 0.28);
        ground = mix(ground, stone, rock);

        // Snow: high and cold, and it does not stick to cliffs.
        // Snow line: ~6.7 km at the equator, ~4 km at 37 deg, sea level at
        // the poles (the real curve is quadratic in latitude), and snow does
        // not stick to cliffs.
        float snowLine =
            max(0.02, 0.75 * (1.0 - 1.05 * latitude * latitude) - 0.04 * humidity);
        float snow = smoothstepf(snowLine, snowLine + 0.10, relief + detail * 0.05) *
                     (1.0 - smoothstepf(0.07, 0.16, slope));
        albedo = mix(ground * (1.0 + detail * 0.22), vec3(0.90, 0.92, 0.95), snow);

        // Polar caps (land ice; the sea ice lives in planetOcean).
        float ice = smoothstepf(iceEdge, iceEdge + 0.015, latitude);
        albedo = mix(albedo, vec3(0.92, 0.94, 0.97) * (1.0 + detail * 0.12), ice);
        specular = mix(0.0, 0.30, max(snow, ice));
        gloss = mix(0.0, 0.45, max(snow, ice));
    }
    else if (style == 1) // Luna: maria over cratered highlands
    {
        // The shore between mare and highland used to be a hard `m < 0.47`
        // step. On a fractal field that draws a crisp wandering edge, and
        // from any distance a crisp wandering bright edge on a grey world
        // reads as a WEATHER FRONT. Real maria have soft margins. The CPU
        // twin (colorizeSurfaceVertex) carries the same numbers, so walking
        // down from orbit never crosses a seam.
        float m = fbm3(dir * 3.1 + vec3(2.9, 8.1, 0.4), 4, 4242u);
        float maria = smoothstepf(0.435, 0.515, m);
        float fine = fbm3(dir * 11.0, 3, 4343u) - 0.5;
        float g = mix(0.235, 0.415, maria) + 0.10 * fine + detail * 0.08 +
                  relief * 0.10;
        // Crater walls and scarps expose brighter, fresher regolith.
        g = mix(g, g * 1.18 + 0.03, rock);
        albedo = vec3(g, g, g * 1.04);
    }
    else // Mars: rust, dark basalt on the scarps, CO2 caps
    {
        float capEdge = 0.93 + 0.02 * (fbm3(dir * 5.0, 3, 771u) - 0.5);
        vec3 lowlands = vec3(0.360, 0.170, 0.090);
        vec3 highlands = vec3(0.660, 0.360, 0.180);
        vec3 dust = vec3(0.720, 0.520, 0.330);
        vec3 ground = mix(lowlands, highlands, smoothstepf(0.02, 0.45, relief));
        ground = mix(ground, dust, smoothstepf(0.45, 0.85, relief));
        ground = mix(ground, vec3(0.230, 0.150, 0.110), rock); // basalt scarps
        albedo = ground * (1.0 + detail * 0.24);
        float cap = smoothstepf(capEdge, capEdge + 0.015, latitude);
        albedo = mix(albedo, vec3(0.90, 0.88, 0.86) * (1.0 + detail * 0.10), cap);
        specular = cap * 0.25;
        gloss = cap * 0.40;
    }
}

// ============================================================================
// THE PROCEDURAL PLANET PATH — one call, everything a ground fragment needs.
//
// Which LOD to sample, the surface normal at that LOD, whether a ridge stands
// between this point and the sun, and what the ground is made of. Shading
// then happens in the BODY FRAME: dot products do not care which orthonormal
// frame they are taken in, and the heightfield gradient is naturally
// expressed there.
//
// `footprint` is the body angle covered by one pixel (length(fwidth(dir)) in
// the fragment shader). Tools/planet_preview renders this very function on
// the CPU, which is how the look is validated without a GPU in the loop.
// ============================================================================
void planetShading(int style, vec3 dirBody, float quality, float bodyRadius,
                   vec3 sunDirBody, float footprint, float time, out vec3 albedo,
                   out float specular, out float gloss, out vec3 normalBody,
                   out float selfShadow, out float water)
{
    TerrainParams terrain = terrainPreset(style);

    // The two low-frequency terms, sampled ONCE and reused by every extra
    // tap below (they are constant over the kilometres those taps span).
    float land = terrainLandFraction(terrain, dirBody);
    float belt = terrainBelt(terrain, dirBody);

    // QUALITY TIERS. These are the only real performance lever the player
    // has, so they have to be genuinely far apart: the octave count is paid
    // THREE times per fragment (elevation + two gradient taps), which makes
    // it by far the most effective dial in the shader.
    //   HIGH   full octave stack (10 on Terra), self-shadowing, cloud shadows
    //   MEDIUM 6 octaves, cloud shadows, no sun march
    //   LOW    3 octaves, sphere normals, nothing else
    // Capped at 12 even at HIGH: octaves beyond that describe ground under
    // ~100 m, which from any distance the globe is drawn at is smaller than
    // a pixel — and up close the terrain PATCH (real geometry, full octave
    // stack) is what covers those metres.
    float octaves = terrainLodOctaves(terrain, max(footprint, 1.0e-9),
                                      min(float(terrain.reliefOctaves), 13.0));
    int warpOctaves = 2;
    if (quality < 1.5)
    {
        octaves = min(octaves, 6.0);
        // One warp octave below HIGH: three noise samples saved on each of
        // the three elevation evaluations. The warp only has to be
        // CONSISTENT between them, not exact.
        warpOctaves = 1;
    }
    if (quality < 0.5) { octaves = min(octaves, 3.0); }

    float elevation =
        terrainElevationFast(terrain, dirBody, land, belt, octaves, warpOctaves);

    // The sea is flat: computing a heightfield gradient for it and then
    // throwing the result away was pure waste over a third of the planet.
    bool isWater = elevation <= 0.0 && terrain.oceanDepth > 0.0;
    // And well past the terminator there is no light for a normal to catch:
    // the night hemisphere skips the gradient entirely. The cut has to sit
    // DEEP in the dark side — a per-pixel normal can tilt by ten degrees or
    // more, so a gate placed right at the wrapped-diffuse zero draws a hard
    // seam down the planet (it did, at -0.25). At -0.42 no normal, however
    // tilted, can still catch light, and the seam has nothing to reveal.
    bool sunlit = dot(dirBody, sunDirBody) > -0.42;
    float slope = 0.0;
    normalBody = dirBody;
    if (quality >= 0.5 && !isWater && sunlit)
    {
        // Same octave and warp counts as `elevation` above — see the note in
        // terrainNormalBody about why that is not optional.
        normalBody = terrainNormalBody(terrain, dirBody, land, belt, bodyRadius,
                                       footprint, octaves, warpOctaves,
                                       kReliefExaggeration, elevation, slope);
    }

    water = 0.0;
    // One evaluation of the cap edge for the whole fragment, and only near
    // the poles at that (it was being sampled three times per water pixel).
    float latitude = abs(dirBody.y);
    float iceEdge = (latitude > 0.84) ? terraIceEdge(dirBody) : 1.0;
    if (isWater && latitude <= iceEdge + 0.02)
    {
        // Sea surface (M27): depth colour, surf, micro-waves, sea ice. The
        // sea floor stays where the bathymetry put it; what is drawn here is
        // the water ON TOP of it, at sea level — the very surface physics
        // collides with.
        float detail = fbm3(dirBody * 42.0, 3, 90210u) - 0.5;
        planetOcean(terrain, dirBody, elevation, latitude, iceEdge, bodyRadius,
                    footprint, time, detail, albedo, specular, gloss, normalBody,
                    water);
    }
    else
    {
        planetBiome(terrain, style, dirBody, elevation, land, slope, albedo,
                    specular, gloss);
        if (elevation <= 0.0 && terrain.oceanDepth > 0.0)
        {
            normalBody = dirBody; // polar sea ice: flat
        }
    }

    // ---- shadows, faded by how much of the world one pixel covers (M30) ---
    // A shadow whose caster is smaller than a pixel is not a shadow, it is
    // noise. Both terms fade out on their own scale — the ridges first
    // (hundreds of metres), the clouds much later (tens of kilometres) —
    // so nothing ever pops as the camera pulls away.
    float metresPerPixel = footprint * bodyRadius;

    selfShadow = 1.0;
    if (quality >= 1.5 && elevation > 0.0 && sunlit)
    {
        // A high sun casts nothing worth marching for: terrain slopes here
        // top out around 0.3, so above ~35 degrees of elevation every step
        // would come back unshadowed. Skipping it removes the march from
        // most of the day side.
        float sunElevation = dot(sunDirBody, dirBody);
        float ridgeFade = (1.0 - smoothstepf(800.0, 3000.0, metresPerPixel)) *
                          (1.0 - smoothstepf(0.45, 0.62, sunElevation));
        if (ridgeFade > 0.01)
        {
            float marched = terrainSelfShadow(terrain, dirBody, sunDirBody, land,
                                              belt, bodyRadius, elevation,
                                              min(octaves, 4.0));
            selfShadow = mix(1.0, marched, ridgeFade);
        }
    }
    // CLOUD SHADOWS (M28): worlds with a deck — the ones with an ocean, for
    // now — sample the very cloud their ray to the sun crosses. It lives
    // here, not in the fragment shader, so the CPU preview shows it too.
    if (terrain.oceanDepth > 0.0 && quality >= 0.5 && sunlit)
    {
        // Cloud detail follows the pixel like everything else: a shadow
        // sampled at full detail from a marble would crawl.
        float cloudOctaves = cloudLodOctaves(footprint, (quality >= 1.5) ? 6.0 : 4.0);
        float cloudFade = 1.0 - smoothstepf(1.2e4, 4.0e4, metresPerPixel);
        if (cloudFade > 0.01)
        {
            // 0.5, not 0.75: under a real deck the ground still gets a lot
            // of light scattered off the sky and off the neighbouring
            // clouds. At 0.75 the shadows read as holes punched in the
            // planet.
            float castShadow = cloudShadow(dirBody, max(elevation, 0.0), bodyRadius,
                                           sunDirBody, time, cloudOctaves, 0.5);
            selfShadow *= mix(1.0, castShadow, cloudFade);
        }
    }
}

#endif // SW_PLANET_SURFACE_GLSL
