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
layout(location = 9) flat in vec3 vCameraPosBody;

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

/// The grade WITHOUT the black lift. Everything that paints over an already
/// graded frame has to use this one, and the atmosphere shell is the reason
/// it exists.
///
/// The lift is an image-wide decision — shadows breathe instead of crushing —
/// and it belongs to the frame, not to each surface in it. A TRANSLUCENT pass
/// that grades its own contribution adds the lift a second time, and the
/// shell's alpha is taken FROM the graded colour: so a ray through pure
/// vacuum, whose radiance is exactly zero, came out at the lift's own value
/// (0.016) and painted 1.6% of a bright grey over everything inside the mesh.
/// Terra wore a hard-edged grey halo out to the shell's radius — which the
/// F20 aurora work had just pushed from 83 km to 350 km, turning a small
/// mistake into a large one. The vacuum test `coverage < 0.002` could never
/// fire, because the floor was eight times the threshold.
vec3 gradeCinematicCore(vec3 color)
{
    // Filmic curve, normalized to a 4.0 white point.
    const vec3 mapped = hable(color * kExposure) / hable(vec3(4.0)).x;
    // Contrast around a filmic pivot (in the tonemapped domain).
    vec3 graded = (mapped - 0.42) * kContrast + 0.42;
    // Desaturate toward luminance.
    const float luminance = dot(graded, vec3(0.2126, 0.7152, 0.0722));
    graded = mix(vec3(luminance), graded, kSaturation);
    return clamp(graded, 0.0, 1.0);
}

vec3 gradeCinematic(vec3 color)
{
    // Raised, slightly cool blacks: shadows breathe instead of crushing.
    const vec3 graded = kBlackLift + gradeCinematicCore(color) * (1.0 - kBlackLift);
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
    // The body-frame view direction, EXACT and per fragment. See the planet
    // branch: the interpolated vViewDirBody is piecewise-linear across the
    // mesh's facets, and anything with a steep response to it (the limb
    // darkening's pow(facing, 0.45), the footprint's 1/cos) turns those
    // creases into a visible lattice-longitude grid.
    vec3 planetViewDirBody = vec3(0.0, 0.0, 1.0);
    // Distance to the planet's TRUE surface along this pixel's ray. The
    // interpolated one is faceted, and the aerial-perspective march is clipped
    // by it — see the note where it is used.
    float planetHitDistance = 0.0;
    // A GAS GIANT'S VISIBLE SURFACE IS THE TOP OF ITS OWN ATMOSPHERE, so
    // there is no air between it and the camera to march. Running the aerial
    // perspective over it is not merely wasted work, it is a second copy of
    // the same air — and a six-step march is sensitive enough to where it is
    // cut off that the mesh's facets came back through it as a grid worth
    // three per cent of the disc. Rocky worlds keep it: on Terra the column
    // between the ground and orbit is real, and it is the blue.
    bool gasSurface = false;
    float terrainShadow = 1.0;
    float waterMask = 0.0;
    if (vTintAlpha > 3.5)
    {
        const int style = int(round((vTintAlpha - 3.6) * 10.0));
        // THE SHADING DIRECTION IS INTERSECTED, NOT INTERPOLATED.
        //
        // `normalize(vModelPosition)` is the obvious answer and it is wrong in
        // a way that took a headless capture to see: the interpolated position
        // lies on the FLAT FACET, not on the sphere, so the direction field it
        // gives is the gnomonic chart of a polyhedron. Its value is
        // continuous but its derivative steps at every facet edge, and the
        // cloud field is sampled at frequencies fine enough to beat against
        // that — so the planet came out with a latitude-longitude GRID drawn
        // across it, about four grey levels deep, in every screenshot anyone
        // ever took of Saturn.
        //
        // The honest direction owes nothing to the mesh: intersect this
        // fragment's view ray with the TRUE unit sphere, in body space, from a
        // camera position that is `flat` and therefore exact. One quadratic,
        // no derivatives, no tessellation. The mesh goes back to being what it
        // should always have been — a silhouette and a depth value.
        vec3 dirBody = normalize(vModelPosition);
        {
            const vec3 rayOrigin = vCameraPosBody;
            const float originDistanceSquared = dot(rayOrigin, rayOrigin);
            // Past ten thousand radii the body is a couple of pixels wide and
            // f32 has run out of room to subtract 1 from `dot(ro, ro)`
            // meaningfully. The facet chart is fine at that size.
            if (originDistanceSquared < 1.0e8)
            {
                const vec3 rayDir = normalize(dirBody - rayOrigin);
                const float half_b = dot(rayOrigin, rayDir);
                const float discriminant = half_b * half_b - (originDistanceSquared - 1.0);
                if (discriminant > 0.0)
                {
                    const float hit = -half_b - sqrt(discriminant);
                    dirBody = normalize(rayOrigin + rayDir * hit);
                    planetHitDistance = hit * vBodyRadius;
                }
            }
        }
        // SCREEN FOOTPRINT of this fragment, in radians of body angle: the
        // single number that decides how much detail may exist here.
        //
        // ANALYTIC, NOT fwidth(), and that is the fix for the worst artefact
        // this renderer has shipped. `fwidth(normalize(vModelPosition))` asks
        // the RASTERISER how fast the body direction is changing, and the
        // rasteriser answers with a number that carries the mesh in it: on a
        // UV sphere the screen-space derivative steps at every facet
        // boundary, the detail octaves fade on smoothstep curves sitting
        // exactly across those steps, and a latitude-longitude GRID appears
        // over the whole planet. Saturn wore one in every screenshot.
        //
        // The honest quantity owes nothing to the triangles: a pixel subtends
        // uCamera.qualityTime.w radians at the camera, the fragment is
        // `fragmentDistance` away, and a surface tilted away from the view
        // spreads that footprint by 1/cos(incidence). It is the same formula
        // Tools/planet_preview has always used on the CPU — which is exactly
        // why the preview never showed the bug the game had.
        // ...and the view direction from the SAME exact geometry: the camera
        // is `flat` and the surface point is the intersection above, so this
        // owes nothing to the interpolator either. Using the interpolated
        // vViewDirBody here put a crease at every facet edge, and pow(facing,
        // 0.45) below turned each crease into a line.
        planetViewDirBody = normalize(vCameraPosBody - dirBody);
        const float incidence = max(abs(dot(dirBody, planetViewDirBody)), 0.06);
        const float footprint =
            max(uCamera.qualityTime.w * fragmentDistance /
                    max(vBodyRadius * incidence, 1.0),
                1.0e-9);
        planetShading(style, dirBody, uCamera.qualityTime.x, vBodyRadius,
                      normalize(vSunDirBody), footprint, uCamera.qualityTime.y,
                      surfaceAlbedo, materialSpecular, materialGloss,
                      planetNormalBody, terrainShadow, waterMask);
        if (style >= 20)
        {
            // LIMB DARKENING, and it is the difference between a sphere and
            // a painted ball. On a rock the disc really is nearly uniform —
            // you see the same dirt at the centre and at the edge. On a gas
            // giant you do not: near the limb your line of sight enters the
            // cloud deck at a grazing angle and never reaches as deep, so it
            // gathers less scattered light. Every photograph of Jupiter has
            // this dark rim, and without it the banding sits on the disc
            // like a decal.
            //
            // A power law on the facing cosine, which is the cheap standard
            // stand-in for the Minnaert law and indistinguishable from it at
            // the angles a planet is ever seen at.
            const float facing = max(dot(dirBody, planetViewDirBody), 0.0);
            surfaceAlbedo *= 0.35 + 0.65 * pow(facing, 0.45);
        }
        surfaceAlpha = 1.0;
        proceduralPlanet = true;
        gasSurface = style >= 20;
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
        // THE SHELL RETURNS A RADIANCE, NOT A PAINT COLOUR, and the blend has
        // to be told so. What belongs in the frame is `graded + background *
        // transmittance`; a src-alpha blend spells that as rgb = graded /
        // alpha with alpha = coverage, so the coverage is taken FROM the
        // graded colour and the colour pre-divided by it.
        //
        // Using `1 - transmittance` instead — which is what this did — counts
        // the same (1 - T) twice and dissolves the faint outer limb. Worse,
        // it hands alpha ~ 0 to any band that EMITS and has no extinction of
        // its own, which is exactly what the F19 nightglow is: above about
        // 72 km it was being discarded outright.
        // VACUUM IS TESTED ON THE RADIANCE, BEFORE THE GRADE, and it has to
        // be. Every term of the grade has a floor: the black lift adds 0.016
        // and the contrast pivot adds another 0.008, so `gradeCinematic(0)` is
        // 0.024 and even `gradeCinematicCore(0)` is 0.0084 — four times the
        // 0.002 the vacuum test was comparing against. A ray through empty
        // space above the air therefore painted a pale grey at 1% alpha, and
        // since the shell now reaches 350 km for the aurora, Terra wore a flat
        // grey disc half again its own radius with a hard edge where the mesh
        // ended. That is the halo in every orbital screenshot.
        //
        // 2e-4 of radiance is a fiftieth of a grey level once it has been
        // through the curve. Below that there is nothing to draw.
        const vec3 radiance = sky * kSunColor;
        if (max(radiance.r, max(radiance.g, radiance.b)) < 2.0e-4)
        {
            discard; // vacuum, and now it really is vacuum
        }
        const vec3 graded = gradeCinematicCore(radiance);
        const float coverage = clamp(max(graded.r, max(graded.g, graded.b)),
                                     0.0, 1.0);
        if (coverage < 0.002)
        {
            discard;
        }
        // Pre-divided by its own maximum channel, so the result peaks at
        // exactly 1 and nothing clips.
        outColor = vec4(graded / max(coverage, 1.0e-4) +
                            ditherOffset(gl_FragCoord.xy),
                        coverage);
        return;
    }

    // ---- SOFT EMISSIVE: STAR DOME AND SUN GLARE (tint alpha in (2.2, 2.5]) --
    // Anything whose opacity FADES TO ZERO at its own rim needs this route,
    // for one reason: the emissive convention below uses the INTERPOLATED
    // alpha as its flag, and zero opacity is alpha exactly 1.0 — the wrong
    // side of the test. Whatever falls through is shaded as an ordinary lit
    // surface, so every soft shape draws a solid OUTLINE of itself. The star
    // dome came out as a field of wireframe quads; the sun's glare layers
    // came out as two concentric grey rings around it. A good demonstration,
    // twice, of why a flag and a value should not share a channel.
    //
    // The instance tint alpha is `flat`, so it cannot interpolate across a
    // threshold, which makes it the right place for a flag. It carries the
    // day fade too, in (2.25, 2.45]:
    //
    //   opacity = vertexAlpha * fade - 1,  vertexAlpha = 1 + brightness
    //
    // ...which erases the faint stars first and the brightest last as the sky
    // comes up, for free, because that is what the arithmetic already does.
    else if (vTintAlpha > 2.2)
    {
        const float fade = clamp((vTintAlpha - 2.25) * 5.0, 0.0, 1.0);
        const float vertexAlpha = vColor.a / max(vTintAlpha, 1.0e-3);
        const float opacity = clamp(vertexAlpha * fade - 1.0, 0.0, 1.0);
        if (opacity < 0.0015)
        {
            discard; // below one grey level: an empty part of the sky
        }
        outColor = vec4(gradeCinematic(vColor.rgb), opacity);
        return;
    }

    // ---- RING SYSTEM (instance tint alpha in (2.05, 2.2]) -------------------
    // ORDER MATTERS AND IT BIT ONCE: this chain is else-if on a DESCENDING
    // threshold, so a branch testing `> 2.05` placed above the star dome's
    // `> 2.2` swallows the dome — the whole sky came out shaded as ring ice
    // and nine thousand stars vanished from the menu. Keep the thresholds
    // monotonically decreasing down the chain.
    // An annulus of ice, and it is NOT a surface: it is a plane-parallel slab
    // of scatterers a few metres thick and seventy thousand kilometres wide.
    // Lighting it with a lambert on a vertex normal, which is what it had,
    // gets the one thing wrong that everybody knows about Saturn's rings —
    // that they look completely different from the two sides. Lit from the
    // front, the dense B ring is the BRIGHTEST thing in the system; lit from
    // behind, it is the DARKEST, because dense is exactly what light cannot
    // get through. Voyager's departure shot is famous for it.
    //
    // So this is single scattering through a slab of optical depth tau,
    // Chandrasekhar's plane-parallel case, which is four exponentials:
    //
    //   reflected      w * mu0/(mu0+mu) * (1 - exp(-tau*(1/mu + 1/mu0)))
    //   transmitted    what survives the crossing, times what it hits on the
    //                  way out — bright where the ring is THIN
    //
    // The radius comes from the model position, so every ringlet ringOpacity
    // knows about is drawn at pixel resolution rather than at the mesh's 160
    // radial steps. The mesh is now geometry and nothing else.
    else if (vTintAlpha > 2.05)
    {
        // The mesh is authored in body radii, so this IS the ring radius.
        const float ringRadius = length(vModelPosition);
        const float opacity = ringOpacity(ringRadius);
        if (opacity < 0.004)
        {
            discard; // a gap: the Cassini division is empty, not dim
        }
        const float tau = -log(max(1.0 - opacity, 1.0e-3));

        // C is thin, dark and slightly blue-grey; B is the bright one and the
        // most strongly reddened by whatever dirties the ice; A sits between.
        // Cassini imaged this, and it is most of what makes a ring system read
        // as a structure with a history rather than as a grey disc.
        const float toB = smoothstepf(1.50, 1.58, ringRadius);
        const float toA = smoothstepf(2.00, 2.05, ringRadius);
        vec3 tone = mix(vec3(0.52, 0.53, 0.58), vec3(0.86, 0.76, 0.58), toB);
        tone = mix(tone, vec3(0.72, 0.67, 0.58), toA);
        // Brightness follows the same structure the opacity does: a dense
        // ringlet is both more opaque AND more reflective, and multiplying the
        // two is what makes the fine radial banding read at a distance where
        // the opacity alone has already saturated.
        tone *= 0.60 + 0.58 * opacity;

        const vec3 planeNormal = normalize(vWorldNormal);
        const vec3 toSun = normalize(uCamera.sunPosition.xyz - vWorldPosition);
        const vec3 toEye = normalize(-vWorldPosition);
        const float sunSide = dot(planeNormal, toSun);
        const float eyeSide = dot(planeNormal, toEye);
        // The floors are not cosmetic: both cosines go to zero when the ring
        // is edge-on, and they are divisors.
        const float muSun = max(abs(sunSide), 0.06);
        const float muEye = max(abs(eyeSide), 0.06);
        // Dirty water ice. 0.6 is the value that comes out of Cassini's
        // photometry for the B ring and it is high — these are snowballs.
        const float singleAlbedo = 0.78;

        float lit;
        if (sunSide * eyeSide > 0.0)
        {
            // FRONT-LIT. Saturates at unity optical depth: past that the slab
            // reflects everything it is going to reflect, which is why the B
            // ring has structure at its edges and none in its middle.
            lit = singleAlbedo * (muSun / (muSun + muEye)) *
                  (1.0 - exp(-tau * (1.0 / muSun + 1.0 / muEye)));
            // OPPOSITION SURGE: at zero phase every particle hides its own
            // shadow and the ring flares. Real and worth about 40%, over a
            // couple of degrees.
            const float phase = clamp(dot(toSun, toEye), 0.0, 1.0);
            lit *= 1.0 + 0.45 * pow(phase, 40.0);
        }
        else
        {
            // BACK-LIT, and the sign flips: the light has to CROSS the slab,
            // so what arrives falls off as exp(-tau) and the dense ringlets go
            // black while the gaps glow. Plus a forward-scattering peak,
            // because ice grains a few microns across throw most of what they
            // scatter within a few degrees of straight on — that is the
            // sunbeam through the C ring.
            const float crossed = exp(-tau / muSun);
            const float forward = clamp(-dot(toSun, toEye), 0.0, 1.0);
            lit = singleAlbedo * crossed * (1.0 - exp(-tau / muEye)) * 0.85 +
                  crossed * 2.2 * pow(forward, 7.0);
        }

        // THE PLANET'S SHADOW ACROSS THE ANNULUS, which is what makes a ring
        // look like it is in orbit around something rather than painted on.
        // The same occluder spheres the surfaces use, in the same closed form.
        float shadowed = 1.0;
        const int ringShadowCount = int(uCamera.sunPosition.w);
        for (int i = 0; i < ringShadowCount; ++i)
        {
            const vec3 m = vWorldPosition - uCamera.shadowSpheres[i].xyz;
            const float sphereRadius = uCamera.shadowSpheres[i].w;
            const float along = dot(m, toSun);
            if (along >= 0.0)
            {
                continue; // the occluder is behind this piece of ring
            }
            const float miss = sqrt(max(dot(m, m) - along * along, 0.0));
            const float inner = sphereRadius * (1.0 - kPenumbra);
            shadowed = min(shadowed,
                           smoothstep(inner, sphereRadius * (1.0 + kPenumbra), miss));
        }

        // Saturnshine: the planet is a huge lit disc a radius away, and it
        // throws enough light back to keep the shadowed ring from being a
        // hole cut in the frame. Faint, warm, and it fades with distance from
        // the planet the way an inverse square does.
        const vec3 planetshine =
            vec3(0.055, 0.050, 0.040) / (ringRadius * ringRadius);
        const vec3 radiance =
            tone * (kSunColor * (lit * shadowed) + planetshine * opacity);

        const vec3 graded = gradeCinematicCore(radiance);
        // HOW MUCH OF THE PIXEL THE RING BLOCKS, which is not its opacity at
        // normal incidence: a slab seen at a grazing angle is thicker along
        // the line of sight by 1/mu, and that is why the rings close up into a
        // solid band as they turn edge-on.
        const float coverage = clamp(1.0 - exp(-tau / muEye), 0.0, 1.0);
        outColor = vec4(graded / max(coverage, 1.0e-3) +
                            ditherOffset(gl_FragCoord.xy),
                        coverage);
        return;
    }
    // ---- THE PHOTOSPHERE (instance tint alpha in (2.03, 2.05]) --------------
    // A star's disc, and the only surface in the game that is supposed to
    // CLIP. The tint carries a radiance of eighteen, four times the grade's
    // white point, so the middle of the disc is pure white with a wide margin
    // — and that margin is the point: it keeps the white region's EDGE in the
    // same place as the sun's apparent size changes from Mercury to Neptune,
    // instead of the whole disc fading as it shrinks.
    //
    // LIMB DARKENING is real and strong on a star — the sun's edge is about
    // 40% of its centre in visible light, because looking at the limb you see
    // higher, cooler gas. At a radiance of eighteen it stays clipped almost to
    // the edge and shows only as the warm rim it also is, which is exactly
    // what a photograph of the sun looks like.
    else if (vTintAlpha > 2.03)
    {
        const vec3 outward = normalize(vWorldNormal);
        const vec3 toEye = normalize(-vWorldPosition);
        const float mu = clamp(dot(outward, toEye), 0.0, 1.0);
        // Eddington's law with u = 0.62, the visible-light value.
        const float darkening = 0.38 + 0.62 * pow(mu, 0.72);
        // The limb is a couple of hundred kelvin cooler, so it reddens as it
        // dims. Both together, or the edge looks like a cut rather than a
        // curve.
        const vec3 limbTint = mix(vec3(1.0, 0.72, 0.42), vec3(1.0, 1.0, 1.0),
                                  smoothstepf(0.0, 0.45, mu));
        outColor = vec4(gradeCinematic(vColor.rgb * (darkening * limbTint)) +
                            ditherOffset(gl_FragCoord.xy),
                        1.0);
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
    const vec3 shadeViewDir = proceduralPlanet ? planetViewDirBody : viewDir;
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
        // A BODY DOES NOT ECLIPSE ITSELF. Its own night side is the lambert
        // term's business, and the lambert term has a soft terminator on
        // purpose — a wrap that stands in for the scattering and bounce light
        // that blur a real day/night line. The eclipse test has no such thing:
        // it goes from lit to black across the penumbra measured in MISS
        // DISTANCE, which at the terminator is a straight vertical cut down
        // the middle of the disc. Saturn grew one the moment it entered the
        // occluder list.
        //
        // Anything within four thousandths of a radius of the surface is ON
        // it — 25 km on Terra, above the highest ground and below any orbit.
        if (dot(m, m) <= r * r * 1.008)
        {
            continue;
        }
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
    if (uCamera.atmosphereBody.w > 0.0 && !gasSurface)
    {
        const AtmosphereParams air =
            atmospherePreset(int(round(uCamera.qualityTime.z)));
        vec3 transmittance = vec3(1.0);
        const int steps = (uCamera.qualityTime.x >= 1.5) ? 6
                                                          : ((uCamera.qualityTime.x >= 0.5)
                                                                 ? 4
                                                                 : 3);
        // THE MARCH IS CLIPPED BY THE DISTANCE TO THE FRAGMENT, and that
        // distance must not be the interpolated one. `fragmentDistance` is
        // measured to the flat FACET, which on a UV sphere sits up to a
        // sagitta inside the true surface — eleven kilometres on Saturn — and
        // steps from facet to facet. A six-step march is sensitive to where
        // it is cut off at the percent level, so that step came out as a
        // percent-deep LATITUDE-LONGITUDE GRID over the whole planet, three
        // quarters of the entire high-frequency error in the frame. The
        // analytic hit distance from the sphere intersection above is smooth,
        // and it is free — it was already computed.
        const float marchLimit =
            (proceduralPlanet && planetHitDistance > 0.0) ? planetHitDistance
                                                          : fragmentDistance;
        const vec3 haze = atmosphereScatter(
            normalize(vWorldPosition), marchLimit,
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
