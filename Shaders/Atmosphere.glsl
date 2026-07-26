// ============================================================================
// Shaders/Atmosphere.glsl — M29: air, as physics rather than as a gradient.
//
// ONE function, three consumers:
//   1. AERIAL PERSPECTIVE — every lit fragment (terrain, patch, rockets) is
//      extinguished and veiled by the air between it and the camera;
//   2. THE LIMB FROM SPACE — the shell paints what the ray gathers as it
//      grazes the planet: thick and white-blue over the day side, reddened
//      where it crosses the terminator;
//   3. THE SKY FROM THE GROUND — the same shell, seen from inside, IS the
//      sky: zenith deep blue, horizon pale, sunsets for free.
//
// Because one integral serves all three, a descent from orbit to the pad is
// continuous by construction: nothing switches, the ray simply gets shorter.
//
// The model is analytic SINGLE SCATTERING: Rayleigh (wavelength-dependent,
// the blue) plus Mie (aerosols, the white haze and the sun's halo), with the
// optical depth toward the sun taken from the Kasten-Young airmass formula
// instead of a nested integral. Six steps along the view ray are enough at
// this scale; there is no lookup table, no precomputation, no extra pass.
//
// All positions are CAMERA-RELATIVE metres, matching the rest of the engine.
// ============================================================================
#ifndef SW_ATMOSPHERE_GLSL
#define SW_ATMOSPHERE_GLSL

#include "Noise.glsl" // smoothstepf: the same explicit curve everything else uses

struct AtmosphereParams
{
    vec3 betaRayleigh;  // scattering coefficient per metre, per channel
    float betaMie;      // scattering coefficient per metre (grey)
    float heightRayleigh; // scale height of the molecular atmosphere, metres
    float heightMie;      // scale height of the aerosol layer, metres
    float top;            // top of the atmosphere above the surface, metres
    float mieAnisotropy;  // Henyey-Greenstein g: forward scattering strength
    /// Gains on the two in-scattered terms. They are NOT free art direction.
    /// The engine's direct lighting omits the 1/pi of a Lambertian surface,
    /// so a physically scaled sky has to be lifted by about that much to sit
    /// correctly against the ground; RAYLEIGH also carries an allowance for
    /// the multiple scattering this single-scattering model omits (a third
    /// to a half of a real sky). MIE gets none of that: it is already a
    /// narrow forward lobe, and amplifying it is exactly how a planet ends
    /// up under a sheet of grey haze when the sun is behind the camera.
    float intensityRayleigh;
    float intensityMie;
};

/// Per-body air. Style ids match the terrain presets: 0 Terra, 1 Luna (none),
/// 2 Mars. Real coefficients for Terra; Mars keeps a thin, dust-reddened air
/// whose Rayleigh term is inverted (blue is absorbed by the dust, not
/// scattered) — which is the whole reason its sky is butterscotch.
AtmosphereParams atmospherePreset(int style)
{
    AtmosphereParams a;
    if (style == 2) // Mars
    {
        a.betaRayleigh = vec3(15.2e-6, 9.1e-6, 4.6e-6);
        a.betaMie = 42.0e-6;
        a.heightRayleigh = 11100.0;
        a.heightMie = 3500.0;
        a.top = 70000.0;
        a.mieAnisotropy = 0.62;
        a.intensityRayleigh = 2.4;
        a.intensityMie = 1.1;
    }
    else // Terra
    {
        a.betaRayleigh = vec3(5.8e-6, 13.5e-6, 33.1e-6);
        a.betaMie = 21.0e-6;
        a.heightRayleigh = 8000.0;
        a.heightMie = 1200.0;
        a.top = 80000.0;
        a.mieAnisotropy = 0.76;
        a.intensityRayleigh = 3.0;
        a.intensityMie = 1.0;
    }
    return a;
}

/// Rayleigh phase: the 1 + cos^2 lobe that makes the sky brightest along and
/// against the sun and darkest at 90 degrees from it.
float phaseRayleigh(float cosTheta)
{
    return 0.0596831 * (1.0 + cosTheta * cosTheta); // 3 / (16 pi)
}

/// Henyey-Greenstein: the strongly forward lobe of aerosols — the white
/// glare around the sun and the bright haze at the horizon.
float phaseMie(float cosTheta, float g)
{
    float g2 = g * g;
    float denominator = 1.0 + g2 - 2.0 * g * cosTheta;
    return 0.0795775 * (1.0 - g2) / max(denominator * sqrt(max(denominator, 1.0e-4)),
                                        1.0e-4);
}

/// Relative airmass for a ray leaving at zenith angle `cosZenith`.
///
/// This used to be the Kasten-Young formula, and it carried TWO bugs at once:
/// it costs an acos and a pow per sample (six per fragment, the single most
/// expensive line in the atmosphere), and — much worse — it is only defined
/// for a sun ABOVE the horizon. Fed a negative cosine it returned a value
/// near ZERO instead of "blocked", so the night side of the planet glowed as
/// if it were noon. What follows is a rational fit: exactly 1 at the zenith,
/// ~38 at the horizon, no transcendentals. Below the horizon the caller
/// stops asking (see the shadow test in the march).
float airmass(float cosZenith)
{
    float c = max(cosZenith, 0.0);
    return 1.0 / (c * 0.9737 + 0.0263);
}

/// Optical depth from an altitude toward the sun, both species, analytic.
vec3 sunOpticalDepth(float altitude, float cosZenithSun, AtmosphereParams a)
{
    float mass = airmass(cosZenithSun);
    float verticalRayleigh = a.heightRayleigh * exp(-altitude / a.heightRayleigh);
    float verticalMie = a.heightMie * exp(-altitude / a.heightMie);
    return (a.betaRayleigh * verticalRayleigh + vec3(a.betaMie * verticalMie)) * mass;
}

/// Is this parcel of air in the planet's own shadow? Exact and cheap: the
/// ray toward the sun is blocked when it points below the local horizon AND
/// its closest approach to the centre falls inside the planet. This is what
/// draws the terminator on the atmosphere — and what the old airmass
/// formula silently skipped.
float sunVisibility(float distanceToCentre, float cosZenithSun, float planetRadius)
{
    if (cosZenithSun > 0.0)
    {
        return 1.0;
    }
    float sinZenithSquared = max(0.0, 1.0 - cosZenithSun * cosZenithSun);
    float closestApproach = distanceToCentre * sqrt(sinZenithSquared);
    // A soft edge over the last ~30 km: the sun has an angular size, and a
    // hard step would draw a visible seam across the sky at dusk.
    return smoothstepf(planetRadius - 3.0e4, planetRadius + 3.0e4, closestApproach);
}

/// Distance along `rayDir` at which the ray leaves a sphere of `radius`
/// centred on `centre` (0 if it never enters it). `entry` comes back as the
/// distance at which it enters (0 when the origin is already inside).
float sphereExit(vec3 origin, vec3 rayDir, vec3 centre, float radius,
                 out float entry)
{
    vec3 toCentre = origin - centre;
    float b = dot(toCentre, rayDir);
    float c = dot(toCentre, toCentre) - radius * radius;
    float disc = b * b - c;
    entry = 0.0;
    if (disc <= 0.0)
    {
        return 0.0;
    }
    float root = sqrt(disc);
    float near = -b - root;
    float far = -b + root;
    if (far <= 0.0)
    {
        return 0.0; // entirely behind the camera
    }
    entry = max(near, 0.0);
    return far;
}

/// THE integral. Marches the segment of `rayDir` that lies inside the
/// atmosphere and returns the light scattered INTO the ray, plus the
/// transmittance that survives along it.
///
/// `maxDistance` clips the march at whatever the ray hit first (a mountain,
/// a hull, or the far side of the air). `planetCentre` is camera-relative.
/// `steps` is the quality dial: 6 is visually converged at planetary scale,
/// 3 still reads correctly and costs half.
vec3 atmosphereScatter(vec3 rayDir, float maxDistance, vec3 planetCentre,
                       float planetRadius, vec3 sunDir, AtmosphereParams a,
                       int steps, out vec3 transmittance)
{
    transmittance = vec3(1.0);
    float entry = 0.0;
    float exitDistance = sphereExit(vec3(0.0), rayDir, planetCentre,
                                    planetRadius + a.top, entry);
    if (exitDistance <= 0.0)
    {
        return vec3(0.0); // the ray never touches this planet's air
    }
    float start = entry;
    float end = min(exitDistance, maxDistance);
    if (end <= start)
    {
        return vec3(0.0);
    }

    // Adaptive step count: a ray that only crosses a few kilometres of air
    // (a rocket in front of the camera on the pad) does not need six samples
    // of it. Saves most of the cost of the near field, where the frame is
    // full of parts.
    float span = end - start;
    int stepCount = clamp(int(span / 1.5e4) + 2, 2, max(steps, 2));
    float segment = span / float(stepCount);
    float cosTheta = dot(rayDir, sunDir);
    float phaseR = phaseRayleigh(cosTheta);
    float phaseM = phaseMie(cosTheta, a.mieAnisotropy);

    vec3 inscatter = vec3(0.0);
    for (int i = 0; i < stepCount; ++i)
    {
        vec3 samplePoint = rayDir * (start + segment * (float(i) + 0.5));
        vec3 up = samplePoint - planetCentre;
        float distanceToCentre = length(up);
        float altitude = max(distanceToCentre - planetRadius, 0.0);
        up /= max(distanceToCentre, 1.0);

        float densityRayleigh = exp(-altitude / a.heightRayleigh);
        float densityMie = exp(-altitude / a.heightMie);

        // Light reaching this parcel of air, then scattered toward us —
        // provided the planet is not standing between the two.
        float cosZenithSun = dot(up, sunDir);
        float visibility = sunVisibility(distanceToCentre, cosZenithSun, planetRadius);
        vec3 sunlight = vec3(0.0);
        if (visibility > 0.0)
        {
            sunlight = exp(-sunOpticalDepth(altitude, cosZenithSun, a)) * visibility;
        }

        vec3 scattered =
            a.betaRayleigh * (densityRayleigh * phaseR * a.intensityRayleigh) +
            vec3(a.betaMie * densityMie * phaseM * a.intensityMie);
        inscatter += transmittance * sunlight * scattered * segment;

        // ...and what this segment takes out of the ray behind it.
        vec3 extinction = (a.betaRayleigh * densityRayleigh +
                           vec3(a.betaMie * densityMie * 1.11)) * segment;
        transmittance *= exp(-extinction);
    }
    return inscatter;
}

#endif // SW_ATMOSPHERE_GLSL
