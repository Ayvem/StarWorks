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
    vec3 abyss = vec3(0.004, 0.024, 0.115);
    vec3 open = vec3(0.012, 0.095, 0.290);
    vec3 shallows = vec3(0.050, 0.375, 0.470);
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
    // ...AND IT IS A METRES-SCALE FEATURE, so it fades on a metres-scale
    // footprint. Left unfaded it put a near-white rim on every coast and
    // every island in the world, and from orbit that is not surf, it is
    // confetti — it was most of why the planet read as speckle rather than
    // as continents. Twelve metres of water is invisible from space; the
    // band vanishes once a pixel covers more than about a kilometre of it.
    float surfFade = 1.0 - smoothstepf(1.0e-5, 1.6e-4, footprint);
    float foam = clamp(surfBand * (0.55 + 1.8 * (churn - 0.40)), 0.0, 1.0) *
                 surfFade;
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
// CRATERS IN THE PALETTE.
//
// craterTerrain3 hands back the impact record as a signed brightness field
// around zero plus a separate, purely additive ray term. Turning that into
// colour is the same operation on every airless body, so it is written once:
//
//   the FLOOR is the same rock less well lit and holding darker melt, so it
//   multiplies the ground colour down;
//   the RIM and the EJECTA are freshly excavated material, so they both lift
//   the ground colour AND blend toward what is underneath — grey regolith on
//   a rock world, but clean white ice on a satellite of Jupiter or Saturn,
//   where an impact is the only thing that ever digs through the dirty crust.
//   That single distinction is why Callisto reads as bright pits in soot and
//   Luna reads as bright rims in grey;
//   the RAYS are that same fresh material thrown clear, which is why Tycho's
//   are plainly visible where they cross a mare.
// ----------------------------------------------------------------------------

/// How much of a feature carried by noise at `frequency` survives a fragment
/// whose footprint is `footprint` body-radians. Full strength while one
/// wavelength covers about sixteen samples, gone by four.
///
/// The tectonic terrains below — Europa's bands, Enceladus' sulci, Ganymede's
/// grooves — are all THIN lanes inside a wide cell, so they are the first
/// thing in the palette to alias and the last thing that should be allowed
/// to. Same rule and same numbers the crater ladder uses on its size classes;
/// the vertex twin passes the reciprocal of its mesh's Nyquist frequency and
/// gets the same answer.
float detailFade(float frequency, float footprint)
{
    return 1.0 - smoothstepf(0.060, 0.250, footprint * frequency);
}

/// `floorGain` is how far the floor is allowed to fall relative to the rim's
/// rise, and it is a property of the GROUND, not of the crater. Luna's
/// regolith reflects 12%, so a floor holding melt and shadow can lose most of
/// that and still read as rock. Callisto's surface already reflects 2%: take
/// 70% off it and the crater is a hole cut in the moon. So the dark worlds
/// keep their floors shallow and let the excavated ice do all the work —
/// which is also what the photographs show, bright rings on soot rather than
/// black discs in grey.
vec3 paintCraters(vec3 ground, float field, float rays, float contrast,
                  float floorGain, vec3 fresh, float freshAmount, float rayAmount)
{
    float signedField = field * contrast;
    // Floors bottom out at -0.72: a crater floor is dim, never black, because
    // the sky above it is half filled with sunlit wall.
    float shadowed = clamp(signedField * floorGain, -0.72, 0.0);
    float excavated = clamp(signedField, 0.0, 1.40);
    vec3 albedo = ground * (1.0 + shadowed + excavated * 0.80);
    albedo = mix(albedo, fresh, clamp(excavated * freshAmount, 0.0, 0.85));
    return mix(albedo, fresh, clamp(rays * rayAmount, 0.0, 0.80));
}

// ----------------------------------------------------------------------------
// BIOMES — albedo from altitude, slope, latitude and humidity.
// Mirrored on the CPU by colorizeSurfaceVertex() (far LOD vertex colors).
//
// `footprint` is the body angle one pixel covers. Everything the crater
// ladder draws is faded out against it, on each size class's own scale — the
// same discipline the terrain octaves and the gas giant churn already follow,
// and the only reason a bombarded surface does not boil when seen from orbit.
// ----------------------------------------------------------------------------
void planetBiome(TerrainParams terrain, int style, vec3 dir, float elevation,
                 float land, float slope, float footprint, out vec3 albedo,
                 out float specular, out float gloss)
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

        vec3 desert = vec3(0.680, 0.535, 0.290);
        vec3 steppe = vec3(0.505, 0.440, 0.205);
        vec3 grass = vec3(0.215, 0.385, 0.125);
        vec3 forest = vec3(0.075, 0.235, 0.080);
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
        // NAMED FOR WHAT IT IS. The mask runs 0 over the mare basalt and 1
        // over the highlands (0.235 is the dark end of the mix below), which
        // is the opposite of what it used to be called — and the crater
        // density hanging off it is the reason that mattered enough to fix.
        float highland = smoothstepf(0.435, 0.515, m);
        float fine = fbm3(dir * 11.0, 3, 4343u) - 0.5;
        float g = mix(0.235, 0.415, highland) + 0.10 * fine + detail * 0.08 +
                  relief * 0.10;
        // Crater walls and scarps expose brighter, fresher regolith.
        g = mix(g, g * 1.18 + 0.03, rock);
        albedo = vec3(g, g, g * 1.04);

        // THE CRATER RECORD, and the fact that it is not uniform. The maria
        // are basalt floods roughly three billion years younger than the
        // highlands they drowned, and the crater count is the clock that
        // says so: a mare carries something like a third of the craters per
        // square kilometre that the highlands do, and its largest ones are
        // gone entirely. Painting both at the same density is most of the
        // reason the old Luna read as one grey speckle with a stain on it.
        // Base frequency 7: cells of 1/7 radian, which on a 1737 km moon is
        // 250 km, so the coarsest class draws 140 km walled plains and the
        // sixth 700 m pits. The last two classes are free from orbit — the
        // ladder breaks out of them until a pixel covers a few hundred
        // metres of ground.
        float rays = 0.0;
        float craters = craterTerrain3(dir, 7.0, 4242u, footprint, 6, rays);
        float density = mix(0.34, 1.0, highland);
        // Rays are barely discounted over a mare: the material is highland
        // ejecta that flew there, not local rock, so the age of the ground
        // it landed on has nothing to do with it.
        albedo = paintCraters(albedo, craters * density, rays * mix(0.80, 1.0, highland),
                              1.15, 1.00, vec3(0.62, 0.62, 0.64), 0.38, 0.62);
    }
    else if (style == 3) // Mercury: Luna's bones, baked warm
    {
        float m = fbm3(dir * 3.4 + vec3(11.2, 4.4, 7.7), 4, 7171u);
        float plains = smoothstepf(0.44, 0.52, m);
        float fine = fbm3(dir * 12.0, 3, 7272u) - 0.5;
        float g = mix(0.245, 0.385, plains) + 0.09 * fine + detail * 0.08 +
                  relief * 0.12;
        g = mix(g, g * 1.16 + 0.03, rock);
        albedo = vec3(g * 1.11, g * 0.99, g * 0.85);

        // Mercury took the same bombardment as Luna with none of the mare
        // flooding to erase it, so its intercrater plains are the most
        // heavily battered ground in the inner system: full density
        // everywhere, and a contrast a third higher than Luna's because
        // there is no atmosphere, no water and no volcanism to soften an
        // edge once it is cut. The smooth plains inside Caloris get the same
        // discount the maria get, and for the same reason.
        float rays = 0.0;
        // 9.5, not Luna's 7: Mercury is 1.4 times the radius, and the same
        // impactor population makes the same SIZE of crater on both, not the
        // same fraction of the globe.
        float craters = craterTerrain3(dir, 9.5, 7171u, footprint, 6, rays);
        float density = mix(1.0, 0.65, plains);
        albedo = paintCraters(albedo, craters * density, rays,
                              0.95, 0.95, vec3(0.60, 0.55, 0.46), 0.26, 0.70);
    }
    else if (style == 4) // Io: sulfur plains, SO2 frost, eruption floors
    {
        // THE ONE BODY ON THIS LIST WITH NO CRATERS AT ALL, and that is not
        // an omission. Io resurfaces itself: tidal kneading by Jupiter pumps
        // something like a hundred gigawatts per square kilometre through
        // its interior, four hundred volcanoes lay down a centimetre of
        // sulfur and silicate a year, and nothing older than about a million
        // years survives anywhere on it. Voyager looked for impact craters
        // and found precisely zero. Painting even a faint crater ladder here
        // would be painting a lie — so what follows is the other thing Io
        // has instead: eruption centres, and what they throw.
        float p = fbm3(dir * 3.0 + vec3(3.1, 9.2, 5.5), 4, 4411u);
        vec3 sulfur = vec3(0.78, 0.68, 0.30);
        vec3 burnt = vec3(0.55, 0.30, 0.14);
        vec3 frost = vec3(0.90, 0.88, 0.78);
        albedo = mix(sulfur, burnt, smoothstepf(0.55, 0.75, p));
        albedo = mix(albedo, frost, 1.0 - smoothstepf(0.30, 0.42, p));
        float e = fbm3(dir * 7.0, 3, 4422u);
        float vents = smoothstepf(0.66, 0.74, e);
        albedo = mix(albedo, vec3(0.16, 0.12, 0.08), vents);

        // PLUME DEPOSITS. An Ionian plume rises four hundred kilometres and
        // falls back as a near-perfect ring: Pele's is 1400 km across and
        // red with short-chain sulfur, Prometheus' is a white SO2 frost
        // annulus that has not moved in twenty years. The ring is the ONE
        // circular feature on this moon, and it is drawn here as a lane of
        // the same fbm the vents come from, thresholded to an annulus
        // AROUND each vent instead of the vent itself — so a deposit can
        // only ever exist where there is something erupting to make it.
        float apron = smoothstepf(0.56, 0.64, e) * (1.0 - smoothstepf(0.64, 0.70, e));
        float sulfurRed = fbm3(dir * 5.0 + vec3(6.1, 1.7, 8.3), 3, 4433u);
        vec3 plume = mix(vec3(0.94, 0.93, 0.88),  // SO2 frost, Prometheus
                         vec3(0.72, 0.17, 0.13),  // short-chain sulfur, Pele
                         smoothstepf(0.42, 0.58, sulfurRed));
        albedo = mix(albedo, plume, apron * 0.75);

        // Sulfur allotropes are temperature-sensitive, and the poles are the
        // cold trap: SO2 frost collects there and the surface goes white
        // where it never gets warm enough to sublime.
        albedo = mix(albedo, vec3(0.88, 0.90, 0.92),
                     smoothstepf(0.62, 0.92, latitude) * 0.45);

        // FLOW FIELDS. Io's plains are not smooth, they are lapped in
        // overlapping lava flows tens of kilometres long, and the newest of
        // them are dark because the sulfur has not had time to weather.
        // Without this the moon is a flat wash from any distance that can
        // see between its volcanoes — which is exactly the distance a ship
        // in a low orbit sits at. Footprint-gated like everything else.
        float flowFade = detailFade(46.0, footprint);
        if (flowFade > 0.0)
        {
            float flow = fbm3(dir * 46.0 + vec3(2.3, 7.9, 4.1), 3, 4455u);
            albedo *= 1.0 + (flow - 0.5) * 0.34 * flowFade;
            albedo = mix(albedo, vec3(0.40, 0.22, 0.11),
                         smoothstepf(0.60, 0.74, flow) * flowFade * 0.55);
        }
        albedo *= (1.0 + detail * 0.20);
    }
    else if (style == 5 || style == 9 || style == 13) // young ice worlds
    {
        // Europa / Enceladus / Triton: bright ice, and the family trait is
        // the CRACKS — thin dark lanes where |fbm - 0.5| pinches to zero.
        //
        // They are also the three bodies in this list whose surfaces are
        // YOUNG: an ocean underneath keeps repaving them, so the crater
        // record is almost blank. Europa has about two dozen craters over
        // 5 km on the whole moon; a face of Callisto has thousands. That
        // difference — not the colour — is what tells the two apart on
        // sight, and it is why the ladders below run three classes at a
        // tenth of the contrast the old worlds get.
        vec3 base = (style == 9)    ? vec3(0.90, 0.94, 0.99)
                    : (style == 13) ? vec3(0.80, 0.72, 0.68)
                                    : vec3(0.80, 0.84, 0.91);
        float c =
            abs(fbm3(dir * 4.5 + terrain.noiseOffset, 4, terrain.seed + 17u) - 0.5) *
            2.0;
        float lineae = 1.0 - smoothstepf(0.03, 0.11, c);
        vec3 crackTint = (style == 5)   ? vec3(0.52, 0.38, 0.26)  // Europa rust
                         : (style == 9) ? vec3(0.55, 0.72, 0.86)  // tiger stripes
                                        : vec3(0.42, 0.36, 0.34); // Triton streaks
        // Enceladus keeps its stripes to the south polar terrain.
        float where = (style == 9) ? smoothstepf(0.45, 0.75, -dir.y) : 1.0;
        albedo = base * (1.0 + detail * 0.10 + relief * 0.10);
        // Europa keeps this network at less than half strength: the real
        // lineae are the long stretched bands below, and at 0.8 the isotropic
        // crazing on top of them turned the whole moon dusty rust.
        float crackAmount = (style == 5) ? 0.34 : 0.80;
        albedo = mix(albedo, crackTint, lineae * where * crackAmount);

        if (style == 5)
        {
            // EUROPA'S LINEAE, the real ones. The bands that cross this moon
            // are hundreds of kilometres long and only tens wide, and no
            // isotropic noise will ever draw that: |fbm - 0.5| gives a
            // NETWORK, which is a cracked windscreen, not a globe pulled
            // apart by tides. Sampling the field seven times finer across
            // one axis than along it stretches every cell into a ribbon —
            // the same trick the gas giants use for their zonal ribbons —
            // and two sets at right angles cross the way Europa's actually
            // do. Each band is dark rust at the centre with a paler margin,
            // because that is the profile a triple band has.
            vec3 alongX = vec3(dir.x * 21.0, dir.y * 3.0, dir.z * 3.2);
            vec3 alongZ = vec3(dir.x * 3.2, dir.y * 3.1, dir.z * 19.0);
            float bandA =
                abs(fbm3(alongX + terrain.noiseOffset, 3, terrain.seed + 61u) - 0.5) *
                2.0;
            float bandB =
                abs(fbm3(alongZ + terrain.noiseOffset, 3, terrain.seed + 83u) - 0.5) *
                2.0;
            float band = detailFade(21.0, footprint);
            float core = max(1.0 - smoothstepf(0.012, 0.055, bandA),
                             1.0 - smoothstepf(0.012, 0.055, bandB)) * band;
            float margin = max(smoothstepf(0.035, 0.075, bandA) *
                                   (1.0 - smoothstepf(0.075, 0.150, bandA)),
                               smoothstepf(0.035, 0.075, bandB) *
                                   (1.0 - smoothstepf(0.075, 0.150, bandB))) *
                           band;
            albedo = mix(albedo, vec3(0.86, 0.88, 0.92), margin * 0.55);
            albedo = mix(albedo, vec3(0.46, 0.30, 0.19), core * 0.75);

            // RIDGED PLAINS, five times finer, for the approach. From a
            // hundred kilometres up the great bands are one dark smear
            // crossing the frame and the moon comes out as a blank sheet;
            // what is actually there at that scale is a dense weave of
            // double ridges a few kilometres apart, each a pair of pale
            // crests with a groove between. It costs two noise samples and
            // it only exists while a pixel can resolve it.
            float weaveFade = detailFade(96.0, footprint);
            if (weaveFade > 0.0)
            {
                float fineA =
                    abs(fbm3(vec3(dir.x * 96.0, dir.y * 14.0, dir.z * 15.0), 3,
                             terrain.seed + 109u) -
                        0.5) *
                    2.0;
                float fineB =
                    abs(fbm3(vec3(dir.x * 15.0, dir.y * 13.0, dir.z * 88.0), 3,
                             terrain.seed + 127u) -
                        0.5) *
                    2.0;
                float crest = max(smoothstepf(0.03, 0.08, fineA) *
                                      (1.0 - smoothstepf(0.08, 0.16, fineA)),
                                  smoothstepf(0.03, 0.08, fineB) *
                                      (1.0 - smoothstepf(0.08, 0.16, fineB)));
                float groove = max(1.0 - smoothstepf(0.008, 0.030, fineA),
                                   1.0 - smoothstepf(0.008, 0.030, fineB));
                albedo = mix(albedo, vec3(0.93, 0.95, 0.98),
                             crest * weaveFade * 0.40);
                albedo = mix(albedo, vec3(0.58, 0.46, 0.36),
                             groove * weaveFade * 0.45);
            }
        }
        if (style == 9)
        {
            // THE TIGER STRIPES. Four sulcus fractures, roughly parallel,
            // 130 km long and 35 km apart, all of them inside the south
            // polar terrain — and every one flanked by the coarse blue-green
            // ice the plumes drop back on. Stretched 9:1 the same way
            // Europa's bands are, and masked to the pole hard enough that
            // the northern hemisphere keeps its craters.
            float south = smoothstepf(0.35, 0.78, -dir.y);
            vec3 alongStripe = vec3(dir.x * 2.6, dir.y * 2.4, dir.z * 34.0);
            float stripe =
                abs(fbm3(alongStripe + terrain.noiseOffset, 2, terrain.seed + 97u) -
                    0.5) *
                2.0;
            float lane = detailFade(34.0, footprint);
            float flank = smoothstepf(0.04, 0.14, stripe) *
                          (1.0 - smoothstepf(0.14, 0.30, stripe)) * lane;
            float fracture = (1.0 - smoothstepf(0.015, 0.055, stripe)) * lane;
            albedo = mix(albedo, vec3(0.62, 0.80, 0.90), flank * south * 0.60);
            albedo = mix(albedo, vec3(0.34, 0.52, 0.64), fracture * south * 0.85);
        }
        if (style == 13)
        {
            // Triton's cantaloupe terrain: a close-packed field of 30 km
            // dimples with no rims and no ejecta, which is what makes it
            // read as MELT rather than as bombardment. It is the crater
            // field with only its floor term surviving, so it comes free.
            float dimpleRays = 0.0;
            float dimples = craterTerrain3(dir, 26.0, terrain.seed + 151u, footprint,
                                           2, dimpleRays);
            albedo *= 1.0 + clamp(dimples, -0.7, 0.0) * 0.42;
            // The southern cap. Voyager 2 flew past a hemisphere covered in
            // nitrogen and methane frost so fresh it reflects 80% of what
            // hits it — the brightest large surface in the solar system —
            // and pink because four billion years of ultraviolet has been
            // turning its methane into tholins.
            albedo = mix(albedo, vec3(0.94, 0.86, 0.80),
                         smoothstepf(-0.10, 0.55, -dir.y) * 0.50);
        }
        albedo = mix(albedo, albedo * 1.12, rock);

        // What little bombardment these three have kept. Enceladus carries
        // its craters everywhere EXCEPT the south polar terrain, which is
        // resurfaced and measurably crater-free; that contrast between a
        // battered northern hemisphere and a blank southern one is the
        // moon's whole portrait.
        float rays = 0.0;
        float youngIce =
            craterTerrain3(dir, (style == 9) ? 5.0 : 9.0, terrain.seed + 733u,
                           footprint, 3, rays);
        float density = (style == 9) ? (1.0 - smoothstepf(0.20, 0.70, -dir.y))
                        : (style == 5) ? 0.30
                                       : 0.45;
        albedo = paintCraters(albedo, youngIce * density, rays * density,
                              (style == 5) ? 0.30 : 0.55, 0.70,
                              vec3(0.96, 0.98, 1.00), 0.40, 0.50);
        specular = 0.35;
        gloss = 0.55;
    }
    else if (style == 6 || style == 7 || style == 10 || style == 11 ||
             style == 12) // old ice-rock worlds
    {
        float m = fbm3(dir * 3.4 + terrain.noiseOffset, 4, terrain.seed + 29u);
        float brightTerrain = smoothstepf(0.42, 0.58, m);
        vec3 dark = vec3(0.27, 0.25, 0.23);
        vec3 light = vec3(0.55, 0.54, 0.52);
        if (style == 7) { dark *= 0.62; light *= 0.72; }   // Callisto, oldest
        if (style == 10) { dark *= 1.12; light *= 1.10; }  // Rhea, frosty
        if (style == 12)
        {
            dark *= vec3(1.05, 0.95, 0.87);   // Oberon, reddened
            light *= vec3(1.05, 0.96, 0.88);
        }
        albedo = mix(dark, light, brightTerrain);
        if (style == 6) // GANYMEDE'S GROOVED TERRAIN
        {
            // Ganymede is two surfaces welded together: dark, ancient,
            // saturated cratered terrain, and pale sulci — belts of parallel
            // furrows a few kilometres apart and hundreds of kilometres long,
            // cut when the crust was pulled apart. The old `abs(fbm - 0.5)`
            // network drew neither; what it drew was crazing.
            //
            // Grooves are PARALLEL, so the field that makes them has to be
            // anisotropic: sampled fourteen times finer across the furrows
            // than along them, a value-noise cell stops being a blob and
            // becomes a lane. Two sets at right angles, each behind its own
            // low-frequency patch mask, give belts running different ways
            // across the globe — which is what turns it from corduroy into
            // tectonics.
            float setA = abs(fbm3(vec3(dir.x * 62.0, dir.y * 4.4, dir.z * 4.4), 3,
                                  6611u) -
                             0.5) *
                         2.0;
            float setB = abs(fbm3(vec3(dir.x * 4.6, dir.y * 58.0, dir.z * 5.0), 3,
                                  6612u) -
                             0.5) *
                         2.0;
            // A third set at 45 degrees in the xz plane. Two sets alone are
            // two families of parallels at right angles, and on a sphere that
            // reads as a wireframe globe; the oblique one is what stops the
            // belts pointing at the body axes and makes them look chosen by
            // tectonics rather than by the coordinate system.
            float su = (dir.x + dir.z) * 0.70710678;
            float sv = (dir.z - dir.x) * 0.70710678;
            float setC =
                abs(fbm3(vec3(su * 55.0, dir.y * 4.8, sv * 4.6), 3, 6615u) - 0.5) * 2.0;
            // Thresholds low enough that each set claims about half the
            // globe: the union of three then covers most of the pale terrain,
            // which is what Ganymede is — sulci everywhere EXCEPT the dark
            // ancient patches, rather than a few stripes on a grey ball.
            float patchA = smoothstepf(0.38, 0.55, fbm3(dir * 2.4, 3, 6613u));
            float patchB = smoothstepf(0.40, 0.57, fbm3(dir * 2.7, 3, 6614u));
            float patchC = smoothstepf(0.39, 0.56, fbm3(dir * 2.9, 3, 6616u));
            float lane = detailFade(62.0, footprint);
            float grooves =
                max(max((1.0 - smoothstepf(0.05, 0.17, setA)) * patchA,
                        (1.0 - smoothstepf(0.05, 0.17, setB)) * patchB),
                    (1.0 - smoothstepf(0.05, 0.17, setC)) * patchC) *
                brightTerrain * lane;
            // Bright crest, dark furrow floor: without the second term a
            // sulcus is a flat pale stripe, and half of what makes grooved
            // terrain legible is the ribbing inside the belt.
            albedo = mix(albedo, light * vec3(1.42, 1.45, 1.50), grooves * 0.90);
            float ribs = max(smoothstepf(0.06, 0.15, setA) *
                                 (1.0 - smoothstepf(0.15, 0.26, setA)) * patchA,
                             smoothstepf(0.06, 0.15, setC) *
                                 (1.0 - smoothstepf(0.15, 0.26, setC)) * patchC) *
                         lane;
            albedo = mix(albedo, light * 0.66, ribs * brightTerrain * 0.50);
            brightTerrain = max(brightTerrain, grooves);
        }
        float fine = fbm3(dir * 11.0, 3, terrain.seed + 43u) - 0.5;
        albedo *= (1.0 + fine * 0.16 + detail * 0.10 + relief * 0.10);
        albedo = mix(albedo, albedo * 1.18, rock); // fresh crater walls

        // THE BOMBARDMENT RECORD OF THE OLDEST SURFACES IN THE SYSTEM.
        //
        // On an icy body an impact is the only thing that ever digs through
        // the dark lag of dust and radiation-processed organics that four
        // billion years leaves on top, so a crater there is not a grey ring
        // on grey ground — it is a WHITE hole punched in soot. That is what
        // `fresh` and `freshAmount` carry, and it is the whole reason
        // Callisto looks the way it does.
        //
        //   Callisto  the most heavily cratered surface known: nothing has
        //             resurfaced it since it formed, so it gets the full
        //             five classes at the highest contrast in the file, and
        //             the brightest excavated ice against the darkest ground.
        //   Ganymede  half its face is younger grooved terrain, so the
        //             density is cut where the sulci are.
        //   Rhea, Titania, Oberon  old, airless and icy: heavily cratered,
        //             a little softer than Callisto because their smaller
        //             gravity leaves shallower structures.
        float rays = 0.0;
        float density = 1.0;
        float contrast = 0.85;
        float floorGain = 0.62;
        float freshAmount = 0.50;
        vec3 freshIce = vec3(0.88, 0.90, 0.92);
        if (style == 7) // Callisto
        {
            contrast = 1.30;
            floorGain = 0.20;
            freshAmount = 0.62;
            freshIce = vec3(0.86, 0.87, 0.88);
        }
        if (style == 6) // Ganymede: the sulci are young ground
        {
            density = mix(1.0, 0.40, brightTerrain);
            contrast = 0.85;
            floorGain = 0.48;
        }
        if (style == 10) { freshIce = vec3(0.94, 0.96, 0.98); } // Rhea, frostiest
        if (style == 12) { freshIce = vec3(0.90, 0.87, 0.82); } // Oberon, reddened
        float craters =
            craterTerrain3(dir, 6.5, terrain.seed + 29u, footprint, 6, rays);
        albedo = paintCraters(albedo, craters * density, rays * density, contrast,
                              floorGain, freshIce, freshAmount, 0.60);
        specular = 0.18;
        gloss = 0.35;
    }
    else if (style == 8) // Titan: dune fields under the haze
    {
        vec3 dunes = vec3(0.34, 0.24, 0.13);
        vec3 beltDark = vec3(0.19, 0.13, 0.07);
        vec3 upland = vec3(0.58, 0.47, 0.29);
        float equator = 1.0 - smoothstepf(0.25, 0.55, latitude);
        albedo = mix(dunes, beltDark, equator);
        albedo = mix(albedo, upland, smoothstepf(0.30, 0.75, relief));
        albedo *= (1.0 + detail * 0.14);

        // Titan HAS craters — Menrva, Sinlap, Selk — and it has almost none
        // of them, because it is the one body on this list with weather. A
        // 1.5 bar nitrogen atmosphere burns up the small impactors before
        // they arrive, and methane rain and blowing sand fill in what does
        // get through in a few hundred million years. So: two classes only,
        // no small ones at all, and a fifth of Callisto's contrast.
        float rays = 0.0;
        float craters = craterTerrain3(dir, 5.0, 8811u, footprint, 2, rays);
        albedo = paintCraters(albedo, craters, rays * 0.15, 0.26, 0.80,
                              vec3(0.56, 0.46, 0.30), 0.20, 0.15);

        // ...and then the haze puts it all behind a curtain. An orange
        // photochemical smog several hundred kilometres deep scatters so
        // hard that Voyager saw a featureless ball; every contrast on the
        // ground gets pulled 45% of the way toward that curtain's colour,
        // which is what makes Titan the SOFT one in a system of hard edges.
        albedo = mix(albedo, vec3(0.46, 0.33, 0.17), 0.38);
        specular = 0.10;
        gloss = 0.25;
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
// SATURN'S RINGS — the radial structure, in ONE place (F19).
//
// This function is the ring: where it starts, where it ends, the Cassini
// division, the Encke gap and the ringlet banding in between. It is called
// twice from two different worlds — by buildRingMesh in GameInternal.hpp to
// author the annulus mesh, and by the planet's own shading to work out how
// much sunlight a given ring radius takes away from the cloud tops beneath.
// Written once, mirrored exactly, because a shadow cast by a different set
// of gaps than the ones you can see is worse than no shadow at all.
//
// `r` is in BODY RADII. Returns opacity in [0, 1].
// ============================================================================
float ringOpacity(float r)
{
    // Ringlets. Real Saturn has thousands; three octaves over the radius is
    // what reads as "structure" rather than as a gradient at every distance
    // the player ever sees the thing from.
    float band = fbm3(vec3(r * 7.3, 0.5, 0.5), 3, 21210u);
    float fine = fbm3(vec3(r * 31.0, 2.5, 4.5), 2, 21211u);
    float opacity = 0.62 * (0.55 + 0.60 * band) * (0.82 + 0.36 * fine);
    // The two edges fade, and the two famous gaps cut.
    opacity *= smoothstepf(1.24, 1.30, r);
    opacity *= 1.0 - smoothstepf(2.17, 2.27, r);
    float cassini = smoothstepf(1.92, 1.95, r) * (1.0 - smoothstepf(1.99, 2.03, r));
    opacity *= 1.0 - 0.94 * cassini;
    float encke = smoothstepf(2.20, 2.215, r) * (1.0 - smoothstepf(2.225, 2.24, r));
    opacity *= 1.0 - 0.85 * encke;
    return clamp(opacity, 0.0, 1.0);
}

/// How much sunlight survives the rings on its way to a point on the planet.
/// The fragment is at `dirBody` on a unit sphere whose EQUATOR is the ring
/// plane (body y = 0, the same plane the latitude bands are measured from);
/// march toward the sun and see where that ray crosses y = 0. If it crosses
/// inside the annulus, the rings are between this ground and the sun, and
/// the band of shadow they lay across the cloud tops is the thing every
/// photograph of Saturn has and no game ever bothers with.
float ringShadow(vec3 dirBody, vec3 sunDirBody)
{
    // A ray parallel to the ring plane never meets it; so does one heading
    // away from it. Both are the day side of the solstice, and both are
    // correctly "no shadow".
    if (abs(sunDirBody.y) < 1.0e-4)
    {
        return 1.0;
    }
    float t = -dirBody.y / sunDirBody.y;
    if (t <= 0.0)
    {
        return 1.0; // the ring plane is behind this fragment, not above it
    }
    vec3 hit = dirBody + sunDirBody * t;
    float r = length(vec2(hit.x, hit.z));
    if (r < 1.24 || r > 2.27)
    {
        return 1.0;
    }
    // GRAZING LIGHT CROSSES MORE RING, exactly as it crosses more air — but
    // that scaling belongs to the OPTICAL DEPTH, not to the opacity, and
    // the first version of this multiplied the opacity directly. An opacity
    // is already 1 - exp(-tau); scaling it by twelve does not mean "twelve
    // times as much material", it means "minus eleven", and the result was
    // half of Saturn painted flat black behind a hard edge. So: back to
    // tau, scale THAT, and come out through the exponential again.
    //
    // The slant is capped at 3. Beyond it the honest answer is that almost
    // no light gets through, and the honest answer is also a black hole in
    // the middle of the planet: the sun is never exactly in the ring plane
    // for long, and a cap costs one edge case to keep a real photograph's
    // worth of shadow instead of a silhouette.
    float opacity = ringOpacity(r);
    float tau = -log(max(1.0 - opacity, 1.0e-3));
    float slant = min(1.0 / max(abs(sunDirBody.y), 0.10), 3.0);
    return clamp(exp(-tau * slant), 0.18, 1.0);
}

// ============================================================================
// GAS WORLDS (style >= 20): no ground, no heightfield — the visible surface
// IS the cloud deck, banded by latitude with a turbulent warp. Mirrored in
// gasGiantAlbedo() in Game/Source/GameInternal.hpp so the vertex-coloured
// LODs (which is all anyone sees beyond close orbit) agree with this shader.
// 20 Jupiter, 21 Saturn, 22 Uranus, 23 Neptune, 24 Venus (whose "bands" are
// the faint chevrons of a cloud deck nobody has ever seen through).
// ============================================================================
vec3 gasGiantAlbedo(int style, vec3 dir, float footprint)
{
    vec3 zone;
    vec3 belt;
    float bands;      // base frequency of the band field, pole to pole
    float edge;       // how sharp a belt's edge is (small = hard-edged)
    float shear;      // how hard the zonal flow bends those edges
    float contrast;
    float churnAmount;
    if (style == 20) // Jupiter: the loudest banding in the system
    {
        zone = vec3(1.00, 0.96, 0.86); belt = vec3(0.42, 0.24, 0.14);
        bands = 6.0; edge = 0.07; shear = 0.90; contrast = 1.00;
        churnAmount = 0.16;
    }
    else if (style == 21) // Saturn: the same machinery under a haze
    {
        zone = vec3(1.00, 0.94, 0.76); belt = vec3(0.56, 0.42, 0.24);
        bands = 5.0; edge = 0.11; shear = 0.50; contrast = 0.85;
        churnAmount = 0.10;
    }
    else if (style == 22) // Uranus: almost featureless, and that is the point
    {
        zone = vec3(0.72, 0.90, 0.90); belt = vec3(0.50, 0.72, 0.80);
        bands = 3.0; edge = 0.26; shear = 0.25; contrast = 0.35;
        churnAmount = 0.05;
    }
    else if (style == 23) // Neptune: dark blue with bright methane cirrus
    {
        zone = vec3(0.38, 0.54, 0.94); belt = vec3(0.09, 0.16, 0.50);
        bands = 4.0; edge = 0.13; shear = 0.70; contrast = 0.95;
        churnAmount = 0.14;
    }
    else // Venus: a cloud deck, not a banded atmosphere
    {
        zone = vec3(1.00, 0.95, 0.78); belt = vec3(0.74, 0.64, 0.44);
        bands = 3.0; edge = 0.24; shear = 0.60; contrast = 0.45;
        churnAmount = 0.08;
    }

    // ZONAL SHEAR, which is the whole trick. What bends a belt is
    // turbulence that has been stretched along the flow by winds running
    // hundreds of metres a second: sampling the noise SIX TIMES finer in
    // latitude than in longitude turns round eddies into ribbons, and
    // ribbons are what a gas giant is made of.
    vec3 flow = vec3(dir.x, dir.y * 6.0, dir.z);
    float swirl = fbm3(flow * 1.7, 4, 2020u + uint(style)) - 0.5;
    // The eddy displacement is FADED BY FOOTPRINT like everything else, and
    // it is the one that most needed it: it is the finest thing that reaches
    // the band field, and the band field is then SHARPENED by a smoothstep
    // 0.22 wide — a gain of four and a half on whatever noise arrives. Three
    // fbm octaves of `flow * 5.3` under a six-fold latitude stretch carry 127
    // cycles per radian, so a pixel wider than 1/254 of a radian cannot
    // resolve it and gets an aliased latitude ripple instead.
    float eddyFade = 1.0 - smoothstepf(0.0012, 0.0034, footprint);
    float eddy = (eddyFade > 0.0)
                     ? (fbm3(flow * 5.3, 3, 4040u + uint(style)) - 0.5) * eddyFade
                     : 0.0;

    // The latitude the bands are READ at — displaced by that turbulence, so
    // the edges wander around the globe instead of ruling straight lines.
    float lat = dir.y + (swirl + eddy * 0.4) * shear * 0.05;

    // THE BAND FIELD, IN FIVE OCTAVES.
    //
    // A planet whose belts are all the same width and the same darkness is
    // a beach ball, and one sine wave cannot draw anything else — which is
    // exactly what the whole disc looked like. Octaves give major belts
    // with minor ones inside them and no two the same width, which is what
    // a gas giant actually is; and because the field is a function of
    // LATITUDE ALONE, its cells ARE bands, at every scale, all the way down
    // to the sixteenth of a band the fifth octave draws.
    float field = fbm3(vec3(0.37, lat * bands, 0.71), 5, 3030u + uint(style));

    // Sharpened into PLATEAUS: broad zones, narrow transitions. A raw fbm
    // is a smooth gradient and reads as haze — the edge is where a belt
    // becomes a belt.
    // ...and the sharpener is WIDENED BY THE FOOTPRINT, which is the whole
    // of antialiasing in one line. A smoothstep 0.14 wide across a field that
    // changes by `bands * footprint` per pixel is a step function once a
    // pixel covers more than a fourteenth of a band — and a step function
    // sampled once per pixel is aliasing, which on a banded planet reads as a
    // latitude ripple that crawls when the camera moves. Jupiter has the
    // sharpest edge in the system (0.07) and showed it worst.
    //
    // Two and a half pixels of transition is the least that reads as an edge
    // rather than a staircase — six was tried first and it turned Jupiter into
    // a soft-focus photograph. Up close the term vanishes and the belts get
    // their authored hardness back.
    float softEdge = max(edge, 2.5 * bands * footprint);
    float blend = smoothstepf(0.5 - softEdge, 0.5 + softEdge, field);

    // ...and not every belt gets the full belt colour. A slow second field
    // decides how deep each one runs, which is the whole difference between
    // BANDED and striped.
    float depth = 0.60 + 0.85 * fbm3(vec3(1.7, lat * bands * 0.21, 4.3), 2,
                                     7070u + uint(style));
    vec3 albedo = mix(zone, belt, clamp(blend * depth * contrast, 0.0, 1.0));

    // THE EQUATORIAL ZONE. Every one of these planets has one, it is the
    // brightest thing on the disc, and it is what makes the banding read as
    // a system with an axis rather than as wallpaper.
    float equator = 1.0 - smoothstepf(0.03, 0.17, abs(dir.y));
    albedo = mix(albedo, zone * 1.04, equator * 0.45);

    // CHURN: the structure a close pass is entitled to see, in two octave
    // bands, each faded out by the FOOTPRINT once a pixel covers more of the
    // planet than the detail is wide.
    //
    // THE THRESHOLDS ARE NYQUIST, NOT TASTE, and getting them wrong is what
    // put a grid over Saturn. `flow` stretches y by SIX, so an octave written
    // as `flow * 26` carries content at 26 cycles per radian in longitude and
    // ONE HUNDRED AND FIFTY-SIX in latitude — and three fbm octaves take the
    // finest of those to 624. A pixel must be smaller than half a cycle of
    // the finest thing in the band or that thing aliases, which for 624
    // cycles is a footprint of 1/1250. The old threshold let it stay 82% on
    // at a footprint of 0.0031, four times past its own Nyquist limit, and
    // the aliasing came out anisotropic — six to one, exactly like the
    // stretch — so it read as LATITUDE LINES rather than as noise. Which is
    // why it was blamed on the mesh, on the tessellation, on the derivative
    // and on the noise fade before it was blamed on this.
    //
    // Each band now ends at its own limit: 0.0008 for the 26/156 band, and a
    // quarter of that for the 60/360 one.
    float coarseFade = 1.0 - smoothstepf(0.00030, 0.00080, footprint);
    if (coarseFade > 0.0)
    {
        float churn = fbm3(flow * 26.0, 3, 5050u + uint(style)) - 0.5;
        albedo *= 1.0 + churn * churnAmount * 1.8 * coarseFade;
    }
    float fineFade = 1.0 - smoothstepf(0.00012, 0.00034, footprint);
    if (fineFade > 0.0)
    {
        float ripple = fbm3(flow * 60.0, 2, 6060u + uint(style)) - 0.5;
        albedo *= 1.0 + ripple * churnAmount * 1.1 * fineFade;
    }

    // THE POLAR HOOD: colder, greyer, and with the banding washing out —
    // the zonal jets do not reach the pole.
    float hood = smoothstepf(0.62, 0.94, abs(dir.y));
    albedo = mix(albedo, mix(albedo, zone, 0.45) * vec3(0.74, 0.79, 0.90), hood);

    if (style == 20) // the Great Red Spot, fixed in the body frame
    {
        // Longitude distance via cos(lon - lon0) — no mod(), whose negative
        // behaviour differs between GLSL and C.
        float lxz = max(length(vec2(dir.x, dir.z)), 1.0e-5);
        float cl = (dir.x * (-0.42) + dir.z * 0.91) / lxz;
        float dl = acos(clamp(cl, -1.0, 1.0));
        float radial = length(vec2(dl * 0.55, (dir.y + 0.34) * 2.2));
        float spot = 1.0 - smoothstepf(0.10, 0.17, radial);
        // A collar of dragged cloud around it, which is most of what makes
        // the spot read as a STORM rather than a painted oval.
        float collar = smoothstepf(0.10, 0.15, radial) *
                       (1.0 - smoothstepf(0.17, 0.26, radial));
        albedo = mix(albedo, vec3(0.72, 0.33, 0.22), spot * 0.85);
        albedo = mix(albedo, vec3(0.93, 0.86, 0.74), collar * 0.35);
    }
    if (style == 23) // Neptune's bright methane streaks
    {
        float streak =
            smoothstepf(0.60, 0.72, fbm3(flow * vec3(3.0, 1.4, 3.0), 3, 2323u));
        albedo = mix(albedo, vec3(0.88, 0.93, 0.99), streak * 0.55);
    }
    return albedo;
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
    // Gas worlds first: no terrain, no gradient, no ocean — the albedo IS
    // the whole answer, and none of the machinery below applies.
    if (style >= 20)
    {
        albedo = gasGiantAlbedo(style, dirBody, footprint);
        specular = 0.06;
        gloss = 0.30;
        normalBody = dirBody;
        // SATURN WEARS ITS OWN RINGS' SHADOW. Style 21 is the only body in
        // the game with a ring system; every other gas giant returns 1 from
        // the branch below anyway, but the test keeps the four noise samples
        // off the ones that cannot use them.
        selfShadow = (style == 21) ? ringShadow(dirBody, sunDirBody) : 1.0;
        water = 0.0;
        return;
    }

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
    float sunHeight = dot(dirBody, sunDirBody);
    bool sunlit = sunHeight > -0.42;
    float slope = 0.0;
    normalBody = dirBody;
    if (quality >= 0.5 && !isWater && sunlit)
    {
        // Same octave and warp counts as `elevation` above — see the note in
        // terrainNormalBody about why that is not optional.
        normalBody = terrainNormalBody(terrain, dirBody, land, belt, bodyRadius,
                                       footprint, octaves, warpOctaves,
                                       kReliefExaggeration, elevation, slope);
        // ...AND FADE THE GATE OUT RATHER THAN SWITCHING IT OFF. The cut is
        // sound for the DIRECT term — past -0.42 no normal can catch the sun
        // — but `slope` does not only feed lighting: it feeds planetBiome,
        // and planetBiome is ALBEDO. Dropping it to zero at a threshold puts
        // a step in the rock/grass mix along a contour of solar incidence,
        // and the night side is lit (ambient, airglow, an aurora), so the
        // step is not hidden by darkness. The fade is free — the gradient is
        // already computed everywhere in this band — so it is fixed rather
        // than argued about.
        //
        // WHAT IT IS NOT: the vertical line down the terra-terminator frame.
        // That was the suspect, the fade was written for it, and it changed
        // the frame by nothing measurable. Column means across the seam run
        // 49, 54, 51: a five-level BRIGHT ridge about 130 km wide sitting on
        // the boundary, which is the twilight arc, which is supposed to be
        // there. Do not go looking for that bug again.
        float gate = smoothstepf(-0.42, -0.32, sunHeight);
        normalBody = normalize(mix(dirBody, normalBody, gate));
        slope *= gate;
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
        planetBiome(terrain, style, dirBody, elevation, land, slope, footprint,
                    albedo, specular, gloss);
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
