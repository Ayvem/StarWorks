#version 450
#extension GL_GOOGLE_include_directive : require

// ============================================================================
// Mesh.frag — the M21 visual overhaul.
//
//  - Blinn-Phong SPECULAR + fresnel sheen, driven by a per-vertex material
//    channel (uv.x = specular strength, uv.y = glossiness) — oceans glint,
//    hulls shine, regolith stays matte (uv = 0).
//  - AERIAL PERSPECTIVE: exponential distance fog toward the horizon color
//    + sky-scattered ambient, both fed per frame by the game from the
//    camera's altitude and the local sun elevation (sunsets included).
//  - ATMOSPHERE SHELLS (instance tint alpha > 2.5): fresnel limb glow —
//    a bright rim of air when seen from orbit, a glowing horizon band
//    from inside — lit day-side by the real sun direction.
//  - CINEMATIC GRADE (M24): Hable filmic curve (gentler shoulder than
//    ACES), tunable contrast / saturation / black lift, WRAP diffuse for a
//    soft terminator, penumbra'd eclipse shadows, and a hash dither that
//    kills gradient banding. All knobs are the named constants below.
//  - PER-PIXEL TERRAIN (M26): the procedural planet path shades in the
//    BODY FRAME with a normal taken from the gradient of the real
//    heightfield at the pixel's own footprint, an octave LOD that keeps
//    detail from crawling, optional terrain self-shadowing, and biomes
//    driven by altitude / slope / latitude / humidity.
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

layout(location = 0) in vec3 vWorldNormal;
layout(location = 1) in vec4 vColor;
layout(location = 2) in vec2 vUv;
layout(location = 3) in vec3 vWorldPosition;
layout(location = 4) flat in float vTintAlpha;
layout(location = 5) in vec3 vModelPosition;
layout(location = 6) in vec3 vSunDirBody;
layout(location = 7) flat in float vBodyRadius;
layout(location = 8) in vec3 vViewDirBody;

layout(location = 0) out vec4 outColor;

// EARLY DEPTH TEST. This shader contains `discard` (the cloud deck and the
// atmosphere shell both use it), and a shader that MAY discard normally
// forces the driver to postpone the depth test until after shading — so
// every fragment of planet hidden behind the terrain patch, the rocket or
// another part still paid for its noise octaves before being thrown away.
// Declaring the mode is safe here because the only depth-WRITING pipeline
// (the opaque one) never discards, and the transparent and HUD pipelines do
// not write depth at all.
layout(early_fragment_tests) in;

const vec3 kSunColor = vec3(1.0, 0.96, 0.9);
const vec3 kAmbient = vec3(0.020, 0.024, 0.035); // faint cold space ambient

// ============================================================================
// SHARED PLANET MATH (M25) — included, never duplicated.
//
// Noise.glsl and Terrain.glsl are the GLSL TWINS of Engine/Source/Math/
// Noise.hpp and Engine/Source/Planet/Terrain.hpp. The CPU samples those very
// headers for terrain COLLISION, patch geometry and site placement; this
// shader samples their port. Tools/glsl_parity/check_parity.py proves the
// two agree bit for bit — that is the whole reason a coastline seen from
// orbit is the coastline the ship splashes into.
// ============================================================================
#include "Noise.glsl"
#include "Terrain.glsl"
#include "PlanetSurface.glsl"
#include "Clouds.glsl"
#include "Atmosphere.glsl"

// ============================================================================
// CINEMATIC GRADE — the whole look lives in these knobs.
// ============================================================================
const float kExposure = 1.0;    // linear pre-exposure
const float kContrast = 0.98;   // < 1 = flatter, film-like (pivot 0.42)
const float kSaturation = 0.82; // < 1 = desaturated, cinematic
const vec3 kBlackLift = vec3(0.010, 0.012, 0.016); // faintly cool raised blacks
/// Soft terminator (0 = hard lambert). It exists to stand in for the
/// atmospheric scattering and bounce light that blur a PLANET's day/night
/// line — and it was being applied to hulls too, which lifted every surface
/// facing away from the sun to 18% and left the rocket a flat grey tube.
/// Objects get a nearly hard lambert; only the procedural planet keeps the
/// wrap, where it is physically standing in for something real.
const float kDiffuseWrap = 0.22;
const float kDiffuseWrapObject = 0.03;
const float kPenumbra = 0.35;    // eclipse softness (fraction of occluder radius)

// Hable (Uncharted 2) filmic curve: gentler shoulder and far less
// saturation push than the ACES fit — highlights roll off instead of
// clipping toward primaries.
vec3 hable(vec3 x)
{
    const float A = 0.15;
    const float B = 0.50;
    const float C = 0.10;
    const float D = 0.20;
    const float E = 0.02;
    const float F = 0.30;
    return ((x * (A * x + C * B) + D * E) / (x * (A * x + B) + D * F)) - E / F;
}

vec3 gradeCinematic(vec3 color)
{
    // Filmic curve, normalized to a 4.0 white point.
    const vec3 mapped = hable(color * kExposure) / hable(vec3(4.0)).x;
    // Contrast around a filmic pivot (in the tonemapped domain).
    vec3 graded = (mapped - 0.42) * kContrast + 0.42;
    // Desaturate toward luminance.
    const float luminance = dot(graded, vec3(0.2126, 0.7152, 0.0722));
    graded = mix(vec3(luminance), graded, kSaturation);
    // Raised, slightly cool blacks: shadows breathe instead of crushing.
    graded = kBlackLift + graded * (1.0 - kBlackLift);
    return clamp(graded, 0.0, 1.0);
}

// Screen-space hash dither: +-0.5/255 hides the banding that smooth sky
// and terminator gradients otherwise show on an 8-bit swapchain.
float ditherOffset(vec2 fragCoord)
{
    const float noise = fract(sin(dot(fragCoord, vec2(12.9898, 78.233))) * 43758.5453);
    return (noise - 0.5) / 255.0;
}

void main()
{
    const vec3 viewDir = normalize(-vWorldPosition); // camera at the origin
    const float fragmentDistance = length(vWorldPosition);

    // ---- PROCEDURAL PLANET SURFACE (instance tint alpha > 3.5) ---------------
    // Close orbit: vertex colors blur when the globe fills the screen, so
    // the game swaps the surface to per-fragment shading. The model-space
    // position IS the body-frame direction (the mesh rotates with the
    // planet), so features stay glued to the ground.
    vec3 surfaceAlbedo = vColor.rgb;
    float surfaceAlpha = vColor.a;
    float materialSpecular = clamp(vUv.x, 0.0, 1.0);
    float materialGloss = clamp(vUv.y, 0.0, 1.0);
    // M26: the planet path shades in the BODY FRAME (where the heightfield
    // and its gradient live). Dot products are frame-agnostic, so only the
    // triplet (normal, light, view) has to agree — these three do.
    bool proceduralPlanet = false;
    vec3 planetNormalBody = vec3(0.0, 1.0, 0.0);
    float terrainShadow = 1.0;
    float waterMask = 0.0;
    if (vTintAlpha > 3.5)
    {
        const int style = int(round((vTintAlpha - 3.6) * 10.0));
        const vec3 dirBody = normalize(vModelPosition);
        // Screen footprint of this fragment, in radians of body angle: the
        // single number that decides how much detail may exist here.
        const float footprint = max(length(fwidth(dirBody)), 1.0e-9);
        planetShading(style, dirBody, uCamera.qualityTime.x, vBodyRadius,
                      normalize(vSunDirBody), footprint, uCamera.qualityTime.y,
                      surfaceAlbedo, materialSpecular, materialGloss,
                      planetNormalBody, terrainShadow, waterMask);
        surfaceAlpha = 1.0;
        proceduralPlanet = true;
    }
    // ---- CLOUD DECK (instance tint alpha in (3.15, 3.5]) --------------------
    // M28: the shell mesh is only a support surface now. Coverage, edges,
    // layers and drift are all evaluated PER FRAGMENT from Clouds.glsl —
    // the same function the ground samples for its shadows.
    else if (vTintAlpha > 3.15)
    {
        const vec3 dirBody = normalize(vModelPosition);
        const vec3 sunDirBody = normalize(vSunDirBody);
        const vec3 viewDirBody = normalize(vViewDirBody);
        const float footprint = max(length(fwidth(dirBody)), 1.0e-9);
        // Same LOD logic as the terrain: cloud detail finer than a pixel can
        // only crawl.
        // The deck is a second full-screen pass over the planet disc, so it
        // follows the quality tier too.
        const float maxOctaves = (uCamera.qualityTime.x >= 1.5)
                                     ? 6.0
                                     : ((uCamera.qualityTime.x >= 0.5) ? 5.0 : 4.0);
        const float octaves = cloudLodOctaves(footprint, maxOctaves);
        float cumulus = 0.0;
        float cirrus = 0.0;
        const float coverage = cloudCoverage(dirBody, uCamera.qualityTime.y,
                                             octaves, cumulus, cirrus);
        if (coverage < 0.003)
        {
            discard; // clear sky: let the planet through untouched
        }

        // THICKNESS: sample the deck again a short way toward the sun. If
        // there is more cloud there, this fragment sits in the shaded core
        // of the mass; if there is less, it is a lit flank.
        const float sunUp = dot(sunDirBody, dirBody);
        float core = 0.0;
        if (cumulus > 0.02)
        {
            const vec3 tangentSun = normalize(sunDirBody - dirBody * sunUp);
            const float ahead =
                cloudCumulus(normalize(dirBody + tangentSun * 0.0022),
                             uCamera.qualityTime.y, octaves);
            core = clamp(cumulus * 0.55 + ahead * 0.75, 0.0, 1.0);
        }

        const float dayWrap = clamp(sunUp * 0.75 + 0.32, 0.0, 1.0);
        // Sunlit white, shaded blue-grey underside, ambient sky fill.
        const vec3 sunlit = vec3(1.02, 1.00, 0.98);
        const vec3 shaded = vec3(0.36, 0.40, 0.50);
        vec3 cloudColor = mix(shaded, sunlit, dayWrap * (1.0 - 0.65 * core));

        // SILVER LINING: light forward-scattered through a thin edge when
        // the camera looks toward the sun. Thin cloud only — the core stays
        // dark, which is exactly what makes the rim read as a rim.
        const float forward = clamp(dot(-viewDirBody, sunDirBody), 0.0, 1.0);
        const float phase = pow(forward, 8.0);
        const float thinEdge = (1.0 - cumulus) * cumulus * 4.0;
        cloudColor += vec3(1.0, 0.96, 0.88) * (phase * thinEdge * 1.6 * dayWrap);

        cloudColor += uCamera.skyAmbient.xyz * 0.6 + kAmbient;
        // `coverage` already includes the cirrus term: adding it again here
        // was double counting, and it is what put a permanent milky film
        // over the whole planet.
        const float alpha = clamp(coverage * (0.15 + 0.85 * dayWrap), 0.0, 1.0);
        outColor = vec4(gradeCinematic(cloudColor) + ditherOffset(gl_FragCoord.xy),
                        alpha);
        return;
    }
    // ---- ATMOSPHERE SHELL material (instance tint alpha in (2.5, 3.15]) -----
    // M29: no longer a fresnel trick. The shell is where the sky is DRAWN:
    // the ray is marched through the real air column and returns the light
    // scattered into it. Seen from orbit that is the limb (thick and pale
    // over the day side, red where it crosses the terminator); seen from the
    // ground it is the sky itself, sunsets included.
    //
    // If the ray hits the planet, the shell steps aside: that fragment's own
    // aerial perspective already accounts for the air in front of it, and
    // painting it twice is how atmospheres end up looking like fog banks.
    else if (vTintAlpha > 2.5)
    {
        const vec3 centre = uCamera.atmosphereBody.xyz;
        const float radius = uCamera.atmosphereBody.w;
        if (radius <= 0.0)
        {
            discard;
        }
        const vec3 rayDir = normalize(vWorldPosition);
        // Does the planet itself block this ray?
        const vec3 toCentre = -centre;
        const float b = dot(toCentre, rayDir);
        const float c = dot(toCentre, toCentre) - radius * radius;
        const float disc = b * b - c;
        if (disc > 0.0 && -b - sqrt(disc) > 0.0)
        {
            discard; // the surface is in front: its own scattering covers it
        }

        const AtmosphereParams air =
            atmospherePreset(int(round(uCamera.qualityTime.z)));
        const vec3 sunDir = normalize(uCamera.sunPosition.xyz - vWorldPosition);
        vec3 transmittance = vec3(1.0);
        const int steps = (uCamera.qualityTime.x >= 1.5) ? 8 : 5;
        const vec3 sky = atmosphereScatter(rayDir, 1.0e12, centre, radius, sunDir,
                                           air, steps, transmittance);
        const float opacity = clamp(1.0 - min(transmittance.r,
                                              min(transmittance.g, transmittance.b)),
                                    0.0, 1.0);
        if (opacity < 0.002)
        {
            discard; // vacuum
        }
        outColor = vec4(gradeCinematic(sky * kSunColor) +
                            ditherOffset(gl_FragCoord.xy),
                        opacity);
        return;
    }

    // ---- EMISSIVE convention: alpha in (1, 2] = self-lit --------------------
    // (star, beacons, plasma, glow discs). Opacity = alpha - 1; the 1..2
    // interpolation range gives soft radial falloffs for free.
    else if (vColor.a > 1.0001)
    {
        outColor = vec4(gradeCinematic(vColor.rgb),
                        clamp(vColor.a - 1.0, 0.0, 1.0));
        return;
    }

    // Light direction PER FRAGMENT from the star's actual position.
    const vec3 toSun = uCamera.sunPosition.xyz - vWorldPosition;
    const float distanceToSun = length(toSun);
    const vec3 sunDirWorld = toSun / max(distanceToSun, 1.0e-3);

    // The SHADING FRAME: body frame for procedural planets (normal from the
    // heightfield gradient), world frame for everything else. The eclipse
    // test below stays in world space — it is about positions, not normals.
    const vec3 normal = proceduralPlanet ? planetNormalBody : normalize(vWorldNormal);
    const vec3 lightDir = proceduralPlanet ? normalize(vSunDirBody) : sunDirWorld;
    const vec3 shadeViewDir = proceduralPlanet ? normalize(vViewDirBody) : viewDir;
    // WRAP diffuse: light bleeds a little past the geometric terminator
    // (atmosphere/bounce stand-in) — the day/night line becomes a soft
    // gradient instead of a hard cut. Normalized so noon stays at 1.
    const float wrap = proceduralPlanet ? kDiffuseWrap : kDiffuseWrapObject;
    const float wrapped = (dot(normal, lightDir) + wrap) / (1.0 + wrap);
    const float lambert = max(wrapped, 0.0) * max(sign(wrapped), 0.0);

    // Analytic eclipse: no light behind a planet (stable small-number form).
    // Analytic eclipse with a PENUMBRA: instead of a binary hit test, the
    // ray's closest-approach distance to the occluder sphere fades the
    // light across kPenumbra of the radius — planetary shadows get the
    // soft edge a real sun's angular size produces.
    float shadow = 1.0;
    const int shadowCount = int(uCamera.sunPosition.w);
    for (int i = 0; i < shadowCount; ++i)
    {
        const vec3 m = vWorldPosition - uCamera.shadowSpheres[i].xyz;
        const float r = uCamera.shadowSpheres[i].w;
        const float b = dot(m, sunDirWorld);
        if (b >= 0.0 || -b > distanceToSun)
        {
            continue; // occluder behind the fragment or beyond the sun
        }
        const float missSq = dot(m, m) - b * b; // closest approach, squared
        const float miss = sqrt(max(missSq, 0.0));
        const float inner = r * (1.0 - kPenumbra);
        const float occlusion = 1.0 - smoothstep(inner, r * (1.0 + kPenumbra), miss);
        shadow = min(shadow, 1.0 - occlusion);
        if (shadow <= 0.0)
        {
            break;
        }
    }

    // ---- material: uv.x = specular strength, uv.y = glossiness ----------------
    // (overridden by the procedural planet path above.)
    const float specularStrength = materialSpecular;
    const float glossiness = materialGloss;
    vec3 specular = vec3(0.0);
    if (specularStrength > 0.001 && lambert > 0.0)
    {
        const vec3 halfway = normalize(lightDir + shadeViewDir);
        const float exponent = mix(12.0, 260.0, glossiness);
        const float highlight = pow(max(dot(normal, halfway), 0.0), exponent);
        const float grazing = 1.0 - max(dot(normal, shadeViewDir), 0.0);
        float fresnel;
        float energy = 1.0;
        if (waterMask > 0.5)
        {
            // WATER (M27): real Schlick with water's F0 = 0.02. Face-on the
            // sea barely reflects; at grazing angles it becomes a mirror —
            // that asymmetry is what makes an ocean read as water and puts
            // the blinding sheet of light on the far side of the glint.
            fresnel = 0.02 + 0.98 * pow(grazing, 5.0);
            // ...and because 2% of a mirror is nothing without the SUN'S
            // radiance behind it, normalize the Blinn-Phong lobe: a tighter
            // lobe concentrates the same energy into a brighter, smaller
            // highlight. This is what turns a dim dot into sun glitter.
            energy = (exponent + 8.0) / 25.13;
        }
        else
        {
            // Authored materials keep the M21 response curve.
            fresnel = (0.35 + 0.65 * pow(grazing, 3.0)) * specularStrength;
        }
        specular = kSunColor * (highlight * fresnel * energy * shadow);
    }

    // Ambient = cold space floor + the game's sky-scattered light; a hint
    // of fill from the sky color on upward-facing surfaces.
    const vec3 ambient = kAmbient + uCamera.skyAmbient.xyz;
    // terrainShadow: a ridge standing between this fragment and the sun
    // (M26, HIGH only). It dims DIRECT light only — the sky still fills
    // shadowed valleys, which is exactly how a shadowed slope reads.
    const vec3 lit =
        surfaceAlbedo * (ambient + kSunColor * (lambert * shadow * terrainShadow)) +
        specular * terrainShadow;

    // ---- AERIAL PERSPECTIVE (M29) ------------------------------------------
    // The same scattering integral as the sky, run over the segment between
    // the camera and THIS fragment: distant ground loses contrast to
    // extinction and gains the colour of the air in front of it. A mountain
    // 200 km away goes blue from orbit and amber at sunset, for the same
    // reason the sky does — because it is the same air.
    vec3 finalColor = lit;
    if (uCamera.atmosphereBody.w > 0.0)
    {
        const AtmosphereParams air =
            atmospherePreset(int(round(uCamera.qualityTime.z)));
        vec3 transmittance = vec3(1.0);
        const int steps = (uCamera.qualityTime.x >= 1.5) ? 6
                                                          : ((uCamera.qualityTime.x >= 0.5)
                                                                 ? 4
                                                                 : 3);
        const vec3 haze = atmosphereScatter(
            normalize(vWorldPosition), fragmentDistance,
            uCamera.atmosphereBody.xyz, uCamera.atmosphereBody.w, sunDirWorld,
            air, steps, transmittance);
        finalColor = lit * transmittance + haze * kSunColor;
    }
    else if (uCamera.fogColorDensity.w > 0.0)
    {
        // Legacy exponential fog, kept for bodies with no air parameters.
        const float fogFactor =
            1.0 - exp(-uCamera.fogColorDensity.w * fragmentDistance);
        finalColor = mix(lit, uCamera.fogColorDensity.xyz, clamp(fogFactor, 0.0, 0.97));
    }

    const vec3 graded =
        gradeCinematic(finalColor) + ditherOffset(gl_FragCoord.xy);
    outColor = vec4(graded, surfaceAlpha);
}
