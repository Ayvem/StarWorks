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
// optical depth toward the sun taken from an airmass formula instead of a
// nested integral. Six steps along the view ray are enough at this scale;
// there is no lookup table, no precomputation, no extra pass.
//
// F19 — THE VIEW FROM ORBIT. Five things were keeping the limb flat. Four of
// them were physics the model had left out rather than art direction, and the
// fifth was a quadrature that could not represent what it was integrating:
//
//   - the TANGENT-HEIGHT sun path. Past the terminator the ray to the sun no
//     longer climbs out of the air, it DIVES THROUGH IT and comes back up on
//     the far side of the limb. The old code clamped the sun's airmass at its
//     horizon value and read the density at the parcel's own altitude, so the
//     light stopped reddening exactly where a real twilight starts. That one
//     substitution is the whole orange wedge;
//   - OZONE, which absorbs and does not scatter. It is invisible looking up
//     (28 thousandths of an optical depth straight overhead) and worth an
//     optical depth of ONE along a horizontal path at 25 km — which is why
//     the top of a real limb, and the last ten minutes of a real sunset, are
//     blue-violet instead of grey;
//   - a bounded MULTIPLE-SCATTERING gain. Single scattering is a bad
//     approximation exactly where the column is thick, and the column is only
//     thick on the limb and in the twilight band: the gain is 3.3x there and
//     1.3x at nadir, which is the difference between a rim that glows and a
//     rim that washes;
//   - the NIGHTGLOW layer at the top of the column, integrated analytically
//     because a 20 km shell falls straight between two samples of any march
//     that can afford to exist. It is what stops the dark limb from being a
//     cut to black;
//   - and an ENERGY-CONSERVING segment, plus samples placed where the air is
//     rather than evenly along the ray. A rectangle rule cannot know that the
//     slab it is integrating is opaque, and a limb ray is eighteen optical
//     depths of opaque: the old march came out 30% too bright and too blue on
//     the brightest pixel of the rim, and by a different amount every frame.
//     Eight steps now land within one grey level of a hundred and twenty-eight.
//
// F20 — THE AURORA, and the room to put it in. F19 left the nightglow layer
// 25 km below its real altitude and left aurorae out altogether, for one
// reason that had nothing to do with light: the atmosphere shell mesh was
// drawn at 83 km, and a layer above the mesh has no fragment to be drawn
// into. Raising that shell to 350 km (GameScene.cpp) is what this feature is
// built on, and it changes three things here:
//
//   - the two EMISSIVE layers are gathered BEFORE the march is allowed to
//     decline the ray. Both of them now live above `top`, so a ray that
//     grazes at 120 km never enters the 80 km air sphere at all and used to
//     be answered with black;
//   - the nightglow moves to its measured altitude, 85-105 km, where the
//     limb arc belongs. It was at 70 km only because that was where a
//     fragment existed;
//   - and the aurorae themselves: two oxygen lines, on an oval rather than a
//     cap, folded into curtains, and only where the sky is dark enough to
//     see them. They are not part of the air column — they are the air being
//     HIT — so they are emission added at the end, never sunlight run through
//     the scattering gain.
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
    /// correctly against the ground — which is all Terra's 3.0 is, and it no
    /// longer carries anything else: the allowance it used to make for the
    /// multiple scattering this model omitted is now `multiScatter` below,
    /// where it can be large on the limb and small at nadir instead of being
    /// the same everywhere. MIE gets no lift of either kind: it is already a
    /// narrow forward lobe, and amplifying it is exactly how a planet ends
    /// up under a sheet of grey haze when the sun is behind the camera.
    float intensityRayleigh;
    float intensityMie;
    /// OZONE, or whatever plays its part on this world: a species that
    /// ABSORBS and does not scatter, and that lives in a layer rather than
    /// under a scale height. Modelled as a tent — zero at `absorbPeak -
    /// absorbWidth`, one at the peak, zero again at `absorbPeak +
    /// absorbWidth` — which is the standard fit to the real profile and has
    /// the useful property that its column integrates in closed form.
    vec3 betaAbsorb;
    float absorbPeak;
    float absorbWidth;
    /// The fraction of the light removed from the ray that finds its way
    /// back into it after further bounces. See atmosphereScatter: it buys a
    /// gain of 1/(1 - f) on an opaque column and almost nothing on a thin
    /// one, which is the shape multiple scattering actually has.
    float multiScatter;
    /// NIGHTGLOW: chemiluminescence in the upper air, present all night and
    /// independent of the sun. Radiance emitted per metre of path at the
    /// centre of the layer; the colour is the line spectrum (557.7 nm green
    /// dominates what the eye sees).
    vec3 airglow;
    float airglowAltitude; // centre of the emitting shell, metres
    float airglowWidth;    // half-thickness of it, metres
    /// AURORA: radiance emitted per metre of path at the peak of each of the
    /// two oxygen layers, on the brightest thread of a curtain. Green is the
    /// 557.7 nm line and lives low; red is 630.0 nm and lives high. A world
    /// with no magnetic field to funnel anything leaves both at zero and pays
    /// one comparison for it.
    vec3 auroraGreen;
    vec3 auroraRed;
};

/// Per-body air. Style ids match the terrain presets: 0 Terra, 1 Luna (none),
/// 2 Mars. Real coefficients for Terra; Mars keeps a thin, dust-reddened air
/// whose Rayleigh term is inverted (blue is absorbed by the dust, not
/// scattered) — which is the whole reason its sky is butterscotch.
AtmosphereParams atmospherePreset(int style)
{
    AtmosphereParams a;
    // Defaults for the four F19 terms. A world with no ozone-like absorber
    // and no nightglow leaves these alone and behaves exactly as it did
    // before; only `multiScatter` is given a value everywhere, because every
    // atmosphere has one.
    a.betaAbsorb = vec3(0.0);
    a.absorbPeak = 25000.0;
    a.absorbWidth = 15000.0;
    a.multiScatter = 0.35;
    a.airglow = vec3(0.0);
    a.airglowAltitude = 0.0;
    a.airglowWidth = 1.0;
    a.auroraGreen = vec3(0.0);
    a.auroraRed = vec3(0.0);
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
        // Dust scatters far more than it absorbs, so what leaves a Martian
        // limb has usually bounced more than once — but it does absorb, which
        // is why this sits well under Terra's.
        a.multiScatter = 0.45;
        // No aurora, and the reason is the interesting part: Mars lost its
        // dynamo, so there is no dipole to gather the incoming electrons into
        // an oval. What it has instead is patchy — a crustal-field aurora
        // over the southern highlands and a diffuse proton glow with no
        // geometry at all — and neither is the thing this file draws.
    }
    else if (style == 20 || style == 21) // Jupiter, Saturn
    {
        // HYDROGEN SCATTERS WEAKLY, AND THE HAZE IS WARM. Until now every
        // world that was not Mars was handed Terra's preset, so the two
        // biggest planets in the game wore an Earth-blue sky over
        // butterscotch cloud — and the blue is exactly what was greying
        // out the banding in the reported frame. Hydrogen's Rayleigh
        // cross-section is a third of air's and far less blue-weighted;
        // what you actually see on a limb here is the ammonia haze, which
        // is Mie and gold.
        //
        // The layer that matters is the one ABOVE the cloud tops — what we
        // are drawing IS the deck, so a deep column would only extinguish
        // the thing it is supposed to sit on. (Tried at 320 km first, with
        // Saturn's real 59 km scale height: it dimmed the disc by a fifth
        // and bought nothing.) A hundred kilometres of haze over it gilds
        // the limb and leaves the bands alone.
        a.betaRayleigh = vec3(3.0e-6, 4.0e-6, 6.4e-6);
        a.betaMie = 18.0e-6;
        a.heightRayleigh = 20000.0;
        a.heightMie = 8000.0;
        a.top = 1.2e5;
        a.mieAnisotropy = 0.70;
        a.intensityRayleigh = 0.9;
        a.intensityMie = 1.2;
        a.multiScatter = 0.30;
    }
    else if (style == 22 || style == 23) // Uranus, Neptune
    {
        // METHANE EATS RED, and that absorption — not scattering — is why
        // these two are the deepest blue limbs in the system. Modelled
        // here as a strongly blue-weighted Rayleigh term with very little
        // aerosol above it.
        a.betaRayleigh = vec3(2.0e-6, 11.0e-6, 31.0e-6);
        a.betaMie = 9.0e-6;
        a.heightRayleigh = 15000.0;
        a.heightMie = 7000.0;
        a.top = 1.0e5;
        a.mieAnisotropy = 0.60;
        a.intensityRayleigh = 2.0;
        a.intensityMie = 0.5;
        a.multiScatter = 0.30;
    }
    else if (style == 24) // Venus
    {
        // ALL MIE, NO SKY. Sulphuric acid droplets, an optical depth
        // nobody has ever seen the ground through, and a limb the colour
        // of old ivory.
        a.betaRayleigh = vec3(8.6e-6, 8.0e-6, 6.8e-6);
        a.betaMie = 78.0e-6;
        a.heightRayleigh = 15900.0;
        a.heightMie = 6000.0;
        a.top = 9.0e4;
        a.mieAnisotropy = 0.55;
        a.intensityRayleigh = 1.0;
        a.intensityMie = 1.9;
        // On Venus multiple scattering IS the atmosphere: the cloud deck is
        // almost perfectly non-absorbing, a photon bounces hundreds of times
        // before it leaves, and that is the entire reason the planet has a
        // bond albedo of 0.77 and is the brightest thing in anyone's sky. The
        // highest value here, and still capped far short of what the physics
        // would give, because 1/(1 - f) is a series that runs away.
        a.multiScatter = 0.78;
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
        // OZONE. The measured extinction at the peak of the layer, per metre,
        // per channel: red is barely touched, GREEN takes three times the
        // hit, blue almost none. That ratio is the Chappuis band, and it is
        // the reason a deep twilight goes blue-violet rather than simply
        // dark — the green has been eaten out of the middle of the spectrum
        // by a hundred kilometres of horizontal ozone. Straight up it is
        // worth 1.881e-6 * 15 km = 0.028 of optical depth, i.e. nothing,
        // which is exactly why it can be added without touching a single
        // ground shot.
        a.betaAbsorb = vec3(0.650e-6, 1.881e-6, 0.085e-6);
        a.absorbPeak = 25000.0;
        a.absorbWidth = 15000.0;
        // Rayleigh scattering absorbs NOTHING, so on an infinitely deep
        // column every photon eventually comes back out and the series
        // 1/(1 - f) has no finite answer. What bounds a real column is the
        // ground under it, the ozone through it, and the fact that it is not
        // infinite: 0.70 is a gain of 3.3 on the opaque foot of the limb and
        // 1.3 looking straight down from orbit, which is the ratio the two
        // ends of that range should have.
        a.multiScatter = 0.70;
        // NIGHTGLOW. Oxygen and OH recombining in the upper air; green at
        // 557.7 nm with a warm floor from the sodium layer, which is what
        // photographs of the night limb from orbit actually show.
        //
        // F20 puts it where it is measured — 95 +- 10 km, i.e. the 85-105 km
        // shell the rocket-borne photometers find — instead of the 70 km F19
        // had to settle for. That 70 km was never physics: it was the highest
        // altitude the 83 km shell mesh could still draw. The shell is at
        // 350 km now and the layer no longer has to hide under it.
        a.airglow = vec3(0.80e-8, 2.8e-8, 0.90e-8);
        a.airglowAltitude = 95000.0;
        a.airglowWidth = 10000.0;
        // AURORA. Both lines come from the same atom — atomic oxygen, hit by
        // an electron that fell down a field line — and they separate purely
        // by how long the excited state lives against how often the air
        // collides with it.
        //
        // 557.7 nm (green, the 1S->1D transition) has 0.7 s to radiate. Below
        // 90 km a collision arrives first and the emission is quenched dead,
        // which is why the bottom edge of a real curtain is the sharpest line
        // in the sky; above ~150 km there is not enough atomic oxygen left to
        // hit. So the green is a band, and it is the bright one.
        //
        // 630.0 nm (red, 1D->3P) has to survive 110 seconds. That is hopeless
        // below 200 km and easy above it, so the red sits ON TOP of the green
        // and runs to the top of the display — fainter per metre, but over
        // twice the depth, which is why a strong aurora is green with a red
        // crown rather than red with a green base.
        //
        // The green is written with a little blue in it and almost no red. A
        // pure 557.7 nm line is OUTSIDE the sRGB gamut on the red side (its
        // linear red coordinate is negative), so it cannot be spelled here
        // exactly; what a camera records is that clipped green plus the N2
        // bands beside it, which is the faintly teal colour every photograph
        // of an arc has.
        //
        // THE MAGNITUDES ARE NOT PHYSICAL AND THE FIRST VERSION'S WERE. A good
        // IBC III display puts the green thread at a linear 0.06 seen from
        // orbit, which is what this shipped with, and the result measured out
        // at sRGB 93 green over a night side sitting at 58 — a grey-green fog
        // twenty levels above the ground it was supposed to be lighting.
        // Nothing was wrong with the radiance; the fault is downstream, and it
        // is unfixable from here. The grade lifts blacks to sRGB 37 so shadows
        // breathe, and it desaturates by a fifth for film. Both are right for
        // a lit frame and both are poison for a faint one: an additive lift
        // eats chroma in proportion to how dark the subject is, so the dimmer
        // the aurora the greyer it gets.
        //
        // So the amplitude is set from the OUTPUT instead — 4.5x the honest
        // number, which lands the brightest thread near linear 1.0 and comes
        // out of the grade at roughly (135, 250, 153) with the diffuse oval
        // still an eighth of that. That is a long-exposure photograph rather
        // than an eye, which is the only aurora anyone has actually seen.
        //
        // The hue is measured off those photographs too: linear (0.13, 1, 0.22)
        // for the green line, (1, 0.06, 0.10) for the red. The old green
        // carried 0.365 of blue against its green, which is a teal, and teal
        // survives a desaturating grade even worse than green does.
        a.auroraGreen = vec3(0.68e-6, 5.20e-6, 1.15e-6);
        a.auroraRed = vec3(1.50e-6, 0.090e-6, 0.150e-6);
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
/// ~38 at the horizon, no transcendentals. Below the horizon this is not the
/// right question at all and the caller asks a different one — see
/// sunOpticalDepthGrazing.
float airmass(float cosZenith)
{
    float c = max(cosZenith, 0.0);
    return 1.0 / (c * 0.9737 + 0.0263);
}

/// Vertical column of the ozone-like tent ABOVE `altitude`, in metres of
/// peak-density air. The tent integrates in closed form, which is the whole
/// reason it is a tent: two clamped ramps and no branches.
///
/// Below the layer it is the tent's full area (its half-width, since the
/// triangle is 2w wide and 1 tall); through the lower flank it falls as a
/// quadratic, through the upper flank it finishes as the mirror quadratic.
float absorbColumn(float altitude, AtmosphereParams a)
{
    float w = max(a.absorbWidth, 1.0);
    float lower = clamp((altitude - (a.absorbPeak - w)) / w, 0.0, 1.0);
    float upper = clamp((altitude - a.absorbPeak) / w, 0.0, 1.0);
    return w * (1.0 - 0.5 * lower * lower) - 0.5 * w * upper * (2.0 - upper);
}

/// Everything above `altitude`, straight up, as an optical depth per channel:
/// the two exponential species plus the absorber's tent.
vec3 verticalColumn(float altitude, AtmosphereParams a)
{
    float verticalRayleigh = a.heightRayleigh * exp(-altitude / a.heightRayleigh);
    float verticalMie = a.heightMie * exp(-altitude / a.heightMie);
    return a.betaRayleigh * verticalRayleigh + vec3(a.betaMie * verticalMie) +
           a.betaAbsorb * absorbColumn(altitude, a);
}

/// Optical depth from an altitude toward a sun ABOVE the local horizon.
vec3 sunOpticalDepth(float altitude, float cosZenithSun, AtmosphereParams a)
{
    return verticalColumn(altitude, a) * airmass(cosZenithSun);
}

/// ...and toward a sun BELOW it — the twilight wedge, which is the single
/// most beautiful thing a planet does and which the model used to miss
/// entirely.
///
/// Once the sun has set for a parcel of air, the ray it still receives does
/// not climb out of the atmosphere: it goes DOWN, skims the planet at a
/// tangent height of `distanceToCentre * sin(zenith) - radius`, and climbs
/// out on the far side. So the light is filtered by the air at the TANGENT
/// height, not at the parcel's own, and it crosses that air twice.
///
/// Written as `2 * column(tangent) - column(parcel)` the outbound half is
/// the full grazing column and the inbound half is the same thing with the
/// part above the parcel taken back off — which makes it exactly continuous
/// with the formula above at the moment of sunset (tangent = parcel, one
/// horizontal column, no seam) and doubles smoothly as the sun goes under.
///
/// The consequence, measured through this function at a parcel 20 km up,
/// with the ozone above included: with the sun ON its horizon the beam that
/// arrives is (0.65, 0.31, 0.42) — already rose, and already with less GREEN
/// in it than blue, which is the ozone and which is why the deep part of a
/// twilight is purple and not brown. Two degrees under it is (0.52, 0.17,
/// 0.19). Three degrees, (0.31, 0.05, 0.02) — red. At four and a half the
/// tangent is underground and there is nothing at all. Higher parcels last
/// longer (30 km is still lit, and still red, at five), which is exactly the
/// shape that makes the lit band a WEDGE: it climbs as it goes round the limb
/// and runs out. Five degrees of sun is 550 km of terminator.
vec3 sunOpticalDepthGrazing(float altitude, float tangentAltitude,
                            AtmosphereParams a)
{
    return (2.0 * verticalColumn(tangentAltitude, a) -
            verticalColumn(altitude, a)) * airmass(0.0);
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
    // A soft edge, because the sun is a disc and not a point — but a NARROW
    // one. This used to be 30 km either side, which was never the penumbra:
    // differentiate closestApproach with respect to the sun's zenith cosine
    // near the horizon and the sun's half-degree of angular size works out at
    // about a kilometre of it. The 30 km was standing in for the reddening
    // that the grazing optical depth above now does properly, and it was
    // costing a third of the brightness of the twilight band while doing it.
    // Three kilometres is a few times the real penumbra, which is enough to
    // keep the edge off the sample grid and no more.
    return smoothstepf(planetRadius - 3.0e3, planetRadius + 3.0e3, closestApproach);
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

/// How much of the ray lies inside a sphere of `radius`, clipped to
/// [0, maxDistance]. Zero if it misses, or if the whole intersection is
/// behind the camera or beyond whatever the ray already hit.
float sphereSpan(vec3 rayDir, vec3 centre, float radius, float maxDistance)
{
    vec3 toCentre = -centre; // the camera is the origin
    float b = dot(toCentre, rayDir);
    float c = dot(toCentre, toCentre) - radius * radius;
    float disc = b * b - c;
    if (disc <= 0.0)
    {
        return 0.0;
    }
    float root = sqrt(disc);
    float near = max(-b - root, 0.0);
    float far = min(-b + root, maxDistance);
    return max(far - near, 0.0);
}

/// THE NIGHTGLOW, integrated in closed form because no affordable march can
/// see it. The emitting layer is 20 km thick on a 6371 km ball: a limb ray at
/// planetary scale takes ~150 km steps, so a sampled version would flicker on
/// and off between frames as the shell fell between two samples.
///
/// It does not need a march. The length of ray inside a spherical shell is
/// the length inside the outer sphere minus the length inside the inner one,
/// and both are clipped identically — so the subtraction stays correct when
/// the planet cuts the ray short. The result has the limb brightening built
/// in for free and for the right reason: straight down the ray crosses 20 km
/// of the layer, along the limb it crosses ONE THOUSAND, and a factor of
/// fifty is why airglow is a sharp arc from orbit and invisible from
/// underneath it.
///
/// It is a NIGHT term, and that is chemistry rather than convenience. The
/// oxygen that recombines up there was split by daylight hours earlier, so
/// the 557.7 nm layer lights up after sunset and stays lit until dawn; the
/// sunlit side has a different, far bluer dayglow that no exposure which can
/// see a cloud will ever record. Fading it out across the terminator is what
/// keeps a green fringe off the bright limb, where nothing should be able to
/// see it anyway.
///
/// The fade is judged at the ray's LOWEST point, which for the rays that
/// matter — the ones grazing the limb — is the tangent point, i.e. exactly
/// the place the arc is being drawn.
vec3 airglowEmission(vec3 rayDir, vec3 planetCentre, float planetRadius,
                     vec3 sunDir, float maxDistance, AtmosphereParams a)
{
    if (a.airglowAltitude <= 0.0)
    {
        return vec3(0.0);
    }
    float outer = planetRadius + a.airglowAltitude + a.airglowWidth;
    float inner = planetRadius + a.airglowAltitude - a.airglowWidth;
    float path = sphereSpan(rayDir, planetCentre, outer, maxDistance) -
                 sphereSpan(rayDir, planetCentre, inner, maxDistance);
    if (path <= 0.0)
    {
        return vec3(0.0);
    }
    vec3 low = rayDir * clamp(dot(planetCentre, rayDir), 0.0, maxDistance) -
               planetCentre;
    float cosZenithSun = dot(low, sunDir) / max(length(low), 1.0);
    // Full strength once the sun is 3 degrees under, gone by 7 degrees up:
    // the band therefore appears out of the twilight instead of switching on.
    return a.airglow * path * smoothstepf(0.12, -0.05, cosZenithSun);
}

/// Where the i-th node of the march falls, as a fraction of the span.
///
/// A UNIFORM march is the wrong instrument for a planet. A limb ray crosses
/// well over a thousand kilometres of shell, of which the couple of hundred
/// either side of its lowest point hold essentially all of the air; eight
/// evenly spaced samples put two of them there and six in vacuum, and the
/// answer then depends on precisely where those two landed — and that error
/// moves with the camera, which is how a limb ends up shimmering as you fly
/// past it.
///
/// So the nodes are squeezed toward `pivot`, the point of the ray closest to
/// the planet's centre, which is its lowest point, which is where the air is.
/// The same warp does the right thing everywhere else without being asked: on
/// the pad the pivot lands at the camera, looking down from orbit it lands on
/// the ground, and in both cases the samples crowd where the density is.
///
/// The exponent is 2 and it was chosen by measurement, not by taste. Against
/// a 128-step reference at eight steps, with the segment integral below in
/// place, the mean error over a column of the sunset horizon falls from 1.8
/// grey levels to 0.4, and over the limb from 0.8 to 0.5. Cubed is WORSE
/// than squared: it piles four samples into the flat top of the density
/// profile and leaves the shoulders, which are 300 km wide at planetary
/// scale, to one sample each (limb 1.1, twilight band 1.5).
float marchNode(float u, float pivot)
{
    if (u <= pivot)
    {
        float s = (pivot - u) / max(pivot, 1.0e-6);
        return pivot - pivot * s * s;
    }
    float t = (u - pivot) / max(1.0 - pivot, 1.0e-6);
    return pivot + (1.0 - pivot) * t * t;
}

// ============================================================================
// THE AURORA (F20).
//
// Everything else in this file is sunlight being redirected. This is not:
// electrons from the tail of the magnetosphere fall down field lines, hit
// oxygen at a hundred kilometres and the oxygen radiates. The sun sets its
// GEOMETRY — the oval hangs off the magnetic axis and leans toward midnight,
// because that is where the field lines the electrons ride come back down —
// but contributes none of its light.
// ============================================================================

/// The base and the ceiling of the emitting volume, metres above the surface.
///
/// The BASE is chemistry and it is hard: below 90 km the air collides with an
/// excited oxygen atom faster than the atom can radiate, so the emission stops
/// dead. It is the sharpest edge in the sky and it is drawn as one.
///
/// The CEILING is not chemistry, it is geometry: a real red tail runs past 400
/// km, and this one ends at 340 because the atmosphere shell in GameScene.cpp
/// is drawn at kTerraRadius * 1.0550 (350 km) and nothing above that mesh has
/// a fragment to be drawn into. The profile is faded out over the 60 km below
/// it rather than cut, so the ceiling is a limit and not a line.
const float kAuroraBase = 90000.0;
const float kAuroraCeiling = 340000.0;
const int kAuroraSteps = 12;

/// The magnetic axis: the spin axis (world Y — the cloud deck and every
/// SpinComponent in the game agree on that) tilted 11 degrees, which is
/// Terra's real dipole offset and is the first of the two reasons this is an
/// oval round the pole rather than a cap on it.
///
/// It is a CONSTANT in the world frame, and that is a compromise worth being
/// explicit about: the shell shader is handed a camera-relative centre and a
/// radius and has no handle on the planet's rotation, so this axis cannot be
/// glued to the ground the way a real dipole is. What that loses is the daily
/// wobble of the magnetic pole around the geographic one. What it keeps is
/// the part you can see from a cockpit: the oval stays fixed with respect to
/// the SUN and the ground turns underneath it, which is exactly what a real
/// oval does over a night.
///
/// kAuroraU and kAuroraV are any two unit vectors perpendicular to it. They
/// carry no meaning; they are the graph paper the curtains are drawn on.
const vec3 kAuroraAxis = vec3(0.190809, 0.981627, 0.0);
const vec3 kAuroraU = vec3(0.981627, -0.190809, 0.0);
const vec3 kAuroraV = vec3(0.0, 0.0, 1.0);

/// The oval and the curtains standing on it. `up` is the unit direction from
/// the planet's centre to the parcel and `toMidnight` the unit anti-sunward
/// direction perpendicular to the magnetic axis. The result is 0 everywhere
/// off the oval, about 0.8 along a smooth ribbon of it, and up to 1.8 on the
/// brightest thread of a broken-up arc — the preset's two colours are the
/// radiance of a thread, not of an average.
///
/// NOTHING HERE DEPENDS ON ALTITUDE, and that is the whole trick. A curtain is
/// a sheet of air standing on a bundle of field lines: its shape is a property
/// of WHERE it is, not of how high you look. Feed the same pattern to samples
/// at 100 km and at 300 km and the march draws a vertical sheet for free —
/// which is the difference between curtains and a green smear.
float auroraSheet(vec3 up, vec3 toMidnight)
{
    // MAGNETIC COLATITUDE, as its cosine, about an axis displaced toward
    // midnight by 4.5 degrees — the second reason this is an oval. A real
    // oval is not concentric with the pole: the field lines that carry the
    // precipitation are stretched down-tail, so the ring sits at magnetic
    // latitude 67 at midnight and 76 at noon. One tilt of the axis produces
    // both numbers, and produces them from the sun direction, which means the
    // oval keeps facing the night without anything having to tell it where
    // the night is.
    //
    // Both hemispheres at once: the conjugate points north and south share a
    // field line, and only one of the two can be anywhere near the ring, so
    // the larger cosine IS the local one and costs a max instead of a second
    // exponential.
    float alongAxis = dot(up, kAuroraAxis);
    float towardMidnight = dot(up, toMidnight) * 0.078702; // tan(4.5 deg)
    float colatitude =
        max(alongAxis + towardMidnight, -alongAxis + towardMidnight) * 0.996911;

    // MAGNETIC LONGITUDE — everything below is a function of it, and it has
    // to be, because a ray is a sheet standing ACROSS the arc: its brightness
    // varies along the ring and is constant through its width. Plane waves in
    // the projected coordinates were tried first and cannot express that. A
    // plane wave has one direction, the ring turns through 360 degrees, so on
    // two opposite stretches of the oval the stripes run along the arc
    // instead of across it, and two crossing waves turn the whole thing into
    // a lattice of beads.
    //
    // The proper coordinate is the azimuth about the axis, which is an atan2,
    // which this file cannot afford (and which the CPU twin's shim does not
    // carry). What follows is the DIAMOND ANGLE: the same angle measured
    // round a square instead of a circle. It is one divide, it is exactly
    // monotone, it closes seamlessly at the wrap provided the wave counts are
    // whole numbers — and it is wrong by up to four degrees of azimuth, which
    // spreads the spokes unevenly by about a third. That last part is not a
    // defect to apologise for: real rays are not evenly spaced either.
    float x = dot(up, kAuroraU);
    float y = dot(up, kAuroraV);
    float diamond = y / (abs(x) + abs(y) + 1.0e-6);
    float turn = (x >= 0.0) ? diamond : (2.0 - diamond); // one full turn = 4
    float wave = 1.5707963 * turn; // ...so sin(n * wave) has n periods on it

    // FOLDS: nine and five of them round the ring, i.e. wavelengths of 1370
    // and 2470 km. That is the scale of the big arcs that fill a sky, not of
    // the fine structure inside them, which no march that can afford to exist
    // could carry anyway. Thirteen and eight were tried first and the ring
    // came out as a polygon: the fold has to be long compared with the
    // 120 km ribbon it is displacing or the corners read as straight lines.
    float fold = 0.55 * sin(9.0 * wave + 1.7) + 0.45 * sin(5.0 * wave + 4.2);

    // THE ARC. Gaussian across the ring, with the fold displacing its centre
    // by up to a degree and a half of latitude — that displacement is what
    // makes a curtain snake instead of ring the pole like a hoop.
    //
    // The two widths are different on purpose. A real oval has a hard
    // poleward edge (the boundary of the polar cap, where the open field
    // lines start and the precipitation simply stops) and a soft equatorward
    // one that trails off into the diffuse aurora over several degrees. In
    // cosine units, at 18.5 degrees of colatitude, 0.0083 is 1.5 degrees and
    // 0.0221 is 4.
    float offset = colatitude - (0.948 + 0.0080 * fold);
    float width = (offset > 0.0) ? 0.0083 : 0.0221;
    float diffuse = exp(-(offset * offset) / (width * width));
    if (diffuse <= 0.003)
    {
        return 0.0;
    }
    // TWO THINGS ARE BEING DRAWN HERE, and conflating them is what makes an
    // aurora look like a green fog bank. `diffuse` above is the OVAL: the
    // statistical band, hundreds of kilometres wide, faint, always there. The
    // DISCRETE ARC is the curtain, and it is thin — a real one is tens of
    // kilometres across and the reason you can see through it edge-on. 0.0060
    // in cosine units is 1.1 degrees, or 120 km; narrower than that and the
    // twelve-sample march starts to step over it between one pixel and the
    // next.
    float arc = exp(-(offset * offset) / (0.0060 * 0.0060));
    // RAYS: two spoke sets, 94 and 71 to the turn — 130 and 175 km apart on
    // the ring, which is the spacing of the discrete arcs a substorm breaks
    // into. Two of them because the beat between two incommensurate counts is
    // what stops a comb from reading as corduroy, and squared because the dark
    // gaps between real rays are wider than the rays. The `offset` term in the
    // phase leans them a few degrees off radial, since the field lines they
    // stand on are not radial either.
    //
    // A real arc is not rayed along its whole length: it runs smooth for
    // hundreds of kilometres and then breaks up. `breakup` — three to a turn,
    // the scale of a substorm bulge — decides which, and fading between a flat
    // ribbon and the rayed version is also the only defence this has against
    // its own march. On a grazing ray the samples are hundreds of kilometres
    // apart and cannot resolve a 130 km spoke; what aliases is then a
    // brightness ripple along a band, not a band that flickers on and off.
    float combA = 0.5 + 0.5 * sin(94.0 * wave + 3.0 * fold + 40.0 * offset);
    float combB = 0.5 + 0.5 * sin(71.0 * wave + 1.5 * fold - 25.0 * offset);
    float breakup = 0.5 + 0.5 * sin(3.0 * wave + 2.0);
    return diffuse * 0.20 + arc * mix(0.80, 0.25 + 1.45 * combA * combA * combB,
                                      breakup);
}

/// The aurora gathered along `rayDir`, as radiance. Zero everywhere the ray
/// misses the emitting shell, everywhere the sky is too bright to see one, and
/// on every world whose preset left the two colours at zero.
vec3 auroraEmission(vec3 rayDir, vec3 planetCentre, float planetRadius,
                    vec3 sunDir, float maxDistance, AtmosphereParams a)
{
    if (a.auroraGreen.y <= 0.0)
    {
        return vec3(0.0);
    }
    float outer = planetRadius + kAuroraCeiling;
    float inner = planetRadius + kAuroraBase;
    // THE CHEAP GATE FIRST, and it earns its place: this function is called
    // by the aerial-perspective path too, i.e. once per lit fragment in the
    // frame. Two quadratics say whether any of the ray is inside the shell at
    // all, and for everything standing on the ground — every rocket, every
    // terrain patch, the whole near field — the answer is no.
    if (sphereSpan(rayDir, planetCentre, outer, maxDistance) -
            sphereSpan(rayDir, planetCentre, inner, maxDistance) <=
        0.0)
    {
        return vec3(0.0);
    }
    // ...then daylight, judged at the ray's lowest point. The threshold is 23
    // degrees rather than nought because `up` swings by up to 16 degrees
    // between that lowest point and the top of a 340 km shell: a tighter test
    // would cut a curtain that the far end of the ray can still legitimately
    // see, and it would cut it along a line.
    vec3 low = rayDir * clamp(dot(planetCentre, rayDir), 0.0, maxDistance) -
               planetCentre;
    if (dot(low, sunDir) / max(length(low), 1.0) > 0.40)
    {
        return vec3(0.0);
    }

    float entry = 0.0;
    float exitDistance = sphereExit(vec3(0.0), rayDir, planetCentre, outer, entry);
    float end = min(exitDistance, maxDistance);
    if (end <= entry)
    {
        return vec3(0.0);
    }
    float span = end - entry;
    // WHERE TO PUT THE SAMPLES. The scattering march squeezes its nodes
    // toward the ray's lowest point because that is where the air is. Here
    // that answer is right only sometimes: a ray grazing the limb crosses
    // four thousand kilometres of this shell and spends the middle two
    // thousand of them inside the green band, so the tangent point is exactly
    // the place to look — but a ray fired UP from the pad has its lowest
    // point at the camera, ninety kilometres BELOW anything that emits, and
    // the same rule would pile all twelve samples into the empty part and
    // leave the curtain to one.
    //
    // So the pivot is the deepest point of the ray THAT IS STILL IN THE
    // SHELL: the closest approach when that is above the base, and the
    // crossing of the base when it is not. The two agree in the limit — as a
    // tangent height falls through 90 km the crossing slides onto the tangent
    // point — so nothing jumps at the boundary between the two cases.
    float pivotDistance = clamp(dot(planetCentre, rayDir), entry, end);
    float innerEntry = 0.0;
    float innerExit = sphereExit(vec3(0.0), rayDir, planetCentre, inner, innerEntry);
    if (innerExit > 0.0)
    {
        // Under the base at some point. Looking up from beneath it, the
        // shell begins where the ray leaves the inner sphere; looking down
        // from above, it ends where the ray enters it. A ray that does both —
        // one that dips below 90 km and comes back out without touching the
        // ground, which is a thin annulus of the limb — is pivoted on the
        // near crossing and undersamples the far one, behind the whole planet.
        pivotDistance = (length(planetCentre) < inner || innerEntry <= entry)
                            ? innerExit
                            : innerEntry;
    }
    float pivot = clamp((clamp(pivotDistance, entry, end) - entry) / max(span, 1.0),
                        0.0, 1.0);

    // MIDNIGHT, as a direction perpendicular to the magnetic axis. Degenerate
    // only when the sun stands exactly over the magnetic pole, where "toward
    // midnight" has no meaning and any tangent will do.
    vec3 midnight = kAuroraAxis * dot(sunDir, kAuroraAxis) - sunDir;
    float midnightLength = length(midnight);
    vec3 toMidnight =
        (midnightLength > 1.0e-3) ? midnight / midnightLength : kAuroraU;

    vec3 sum = vec3(0.0);
    float nodeNear = entry;
    for (int i = 0; i < kAuroraSteps; ++i)
    {
        float nodeFar =
            entry + span * marchNode(float(i + 1) / float(kAuroraSteps), pivot);
        float segment = nodeFar - nodeNear;
        vec3 samplePoint = rayDir * (nodeNear + segment * 0.5);
        nodeNear = nodeFar;
        vec3 up = samplePoint - planetCentre;
        float distanceToCentre = length(up);
        float altitude = distanceToCentre - planetRadius;
        up /= max(distanceToCentre, 1.0);

        // The two vertical profiles, in the order the atom decides: green
        // switched on hard at the quenching altitude and gone by 185 km, red
        // only where a 110-second state can survive and still climbing where
        // the green has run out.
        float green = smoothstepf(kAuroraBase, 106000.0, altitude) *
                      (1.0 - smoothstepf(118000.0, 185000.0, altitude));
        float red = smoothstepf(165000.0, 235000.0, altitude) *
                    (1.0 - smoothstepf(265000.0, kAuroraCeiling, altitude));
        if (green + red <= 0.0)
        {
            continue;
        }
        // The aurora does not stop at dawn — but it becomes invisible, and
        // that is what is being modelled. A bright arc is a few kilorayleighs
        // against a day sky four orders of magnitude brighter, so it appears
        // when the stars do: nothing while the sun is up, all of it once the
        // sun is 8 degrees under, which is the end of civil twilight.
        float night = smoothstepf(0.0, -0.14, dot(up, sunDir));
        if (night <= 0.0)
        {
            continue;
        }
        sum += (a.auroraGreen * green + a.auroraRed * red) *
               (auroraSheet(up, toMidnight) * night * segment);
    }
    return sum;
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
    // THE EMISSIVE LAYERS FIRST, because both of them are now ABOVE `top` and
    // the march below is about to turn away any ray that misses the 80 km air
    // sphere. A ray grazing at 120 km misses it and still crosses a thousand
    // kilometres of nightglow and, if the oval is under it, a curtain: that
    // ray used to be answered with black, which is precisely the black the
    // 83 km shell was hiding.
    vec3 glow = airglowEmission(rayDir, planetCentre, planetRadius, sunDir,
                                maxDistance, a);
    vec3 aurora = auroraEmission(rayDir, planetCentre, planetRadius, sunDir,
                                 maxDistance, a);
    float entry = 0.0;
    float exitDistance = sphereExit(vec3(0.0), rayDir, planetCentre,
                                    planetRadius + a.top, entry);
    if (exitDistance <= 0.0)
    {
        // The ray never touches this planet's air, so nothing extinguishes
        // what the layers above it emitted.
        return glow + aurora;
    }
    float start = entry;
    float end = min(exitDistance, maxDistance);
    if (end <= start)
    {
        return glow + aurora;
    }

    // Adaptive step count: a ray that only crosses a few kilometres of air
    // (a rocket in front of the camera on the pad) does not need six samples
    // of it. Saves most of the cost of the near field, where the frame is
    // full of parts.
    float span = end - start;
    int stepCount = clamp(int(span / 1.5e4) + 2, 2, max(steps, 2));
    // The lowest point of the ray, expressed as a fraction of the span: the
    // camera sits at the origin, so the closest approach to the planet's
    // centre is simply dot(centre, direction). Everything the march does
    // about sample placement follows from this one number.
    float pivot = clamp((clamp(dot(planetCentre, rayDir), start, end) - start) /
                            max(span, 1.0),
                        0.0, 1.0);
    float cosTheta = dot(rayDir, sunDir);
    float phaseR = phaseRayleigh(cosTheta);
    float phaseM = phaseMie(cosTheta, a.mieAnisotropy);

    vec3 inscatter = vec3(0.0);
    // The same sum with the sun taken out of it: what this ray WOULD have
    // gathered if every parcel along it were in full sunlight. Dividing one
    // by the other at the end gives the average survival of the beam on its
    // way in, which is the other half of the multiple-scattering estimate.
    vec3 potential = vec3(0.0);
    float nodeNear = start;
    for (int i = 0; i < stepCount; ++i)
    {
        float nodeFar =
            start + span * marchNode(float(i + 1) / float(stepCount), pivot);
        float segment = nodeFar - nodeNear;
        vec3 samplePoint = rayDir * (nodeNear + segment * 0.5);
        nodeNear = nodeFar;
        vec3 up = samplePoint - planetCentre;
        float distanceToCentre = length(up);
        float altitude = max(distanceToCentre - planetRadius, 0.0);
        up /= max(distanceToCentre, 1.0);

        float densityRayleigh = exp(-altitude / a.heightRayleigh);
        float densityMie = exp(-altitude / a.heightMie);

        // Light reaching this parcel of air, then scattered toward us —
        // provided the planet is not standing between the two, and filtered
        // by whichever path the beam had to take to get here: over the air
        // while the sun is up, THROUGH it once the sun has set.
        float cosZenithSun = dot(up, sunDir);
        float visibility = sunVisibility(distanceToCentre, cosZenithSun, planetRadius);
        vec3 sunlight = vec3(0.0);
        if (visibility > 0.0)
        {
            vec3 depth;
            if (cosZenithSun >= 0.0)
            {
                depth = sunOpticalDepth(altitude, cosZenithSun, a);
            }
            else
            {
                float sinZenith = sqrt(max(0.0, 1.0 - cosZenithSun * cosZenithSun));
                float tangent = max(distanceToCentre * sinZenith - planetRadius, 0.0);
                depth = sunOpticalDepthGrazing(altitude, tangent, a);
            }
            sunlight = exp(-depth) * visibility;
        }

        vec3 scattered =
            a.betaRayleigh * (densityRayleigh * phaseR * a.intensityRayleigh) +
            vec3(a.betaMie * densityMie * phaseM * a.intensityMie);
        // What this segment takes out of the ray behind it. Mie's 1.11 is its
        // absorbing fraction; the ozone tent is pure absorption and so appears
        // HERE and in the sun path, but never in `scattered`.
        vec3 extinction = a.betaRayleigh * densityRayleigh +
                          vec3(a.betaMie * densityMie * 1.11) +
                          a.betaAbsorb * max(0.0, 1.0 - abs(altitude - a.absorbPeak) /
                                                            max(a.absorbWidth, 1.0));
        vec3 survives = exp(-extinction * segment);

        // ENERGY-CONSERVING SEGMENT, and on the limb it is the difference
        // between a plausible answer and a wrong one. A rectangle —
        // scattering x density x length — has no idea that the segment it is
        // integrating is opaque, so a ray grazing the surface, where the blue
        // channel is eighteen optical depths deep, gathers light from air
        // that cannot possibly be seen through the air in front of it.
        // Integrating the segment exactly instead (a homogeneous slab has a
        // closed form) makes it impossible to scatter more light than the
        // slab removed, and it caps at scattering/extinction — the single
        // scattering albedo — which is what an opaque column actually looks
        // like. Measured on the brightest pixel of the limb, blue channel,
        // eight steps against a 128-step reference: 171 where the answer is
        // 132 with a rectangle, 133 with this. It is also what lets eight
        // steps and five look the same, which matters because the quality
        // tiers pass different numbers.
        inscatter += transmittance * sunlight * scattered *
                     (vec3(1.0) - survives) / max(extinction, vec3(1.0e-12));
        potential += transmittance * scattered *
                     (vec3(1.0) - survives) / max(extinction, vec3(1.0e-12));
        transmittance *= survives;
    }

    // MULTIPLE SCATTERING, as one number rather than a second integral.
    //
    // A photon that leaves the beam does not vanish; Rayleigh scattering
    // absorbs nothing at all, so it goes on bouncing and some share of it
    // arrives here anyway. Summing that series gives a gain of 1/(1 - f)
    // where f is the share that comes back, and the share that comes back
    // can only be large where a lot of light left the beam in the first
    // place. Light leaves it on BOTH legs — on the way in from the sun and
    // on the way out to the camera — so f is driven by the product of the
    // two survivals, and `inscatter / potential` is the first of them
    // averaged over the samples that actually contributed.
    //
    // That is precisely the shape wanted. Looking straight down from 400 km
    // both legs are short, Terra's gain is 1.3, and the ocean shot barely
    // moves. Along the limb neither is: the gain reaches its ceiling of 3.3
    // and the rim gets a brightness single scattering has no way to produce.
    // In the twilight band the view leg is thick and the SUN leg is far
    // thicker still — 2.6 on the red that survives — which is why a
    // single-scattering twilight always comes out too dark.
    //
    // Per channel throughout, because blue bounces more often than red. That
    // is also why the foot of the rim goes white while its top stays blue.
    //
    // The 0.25 floor is a fuse, not a tuning knob: it holds any preset that
    // ever gets a multiScatter above 0.75 to a gain of four.
    vec3 beamSurvival = clamp(inscatter / max(potential, vec3(1.0e-9)), 0.0, 1.0);
    inscatter /= max(vec3(1.0) - a.multiScatter *
                                     (vec3(1.0) - transmittance * beamSurvival),
                     vec3(0.25));

    // THE EMISSIVE LAYERS last, and never through the multiple-scattering
    // gain: they are emission, not sunlight. Seen from below they do have to
    // cross the air that is between — which is the whole atmosphere, at
    // whatever angle the ray took — so a camera under a layer gets it through
    // `transmittance` and a camera above it gets it whole. The aurora's own
    // floor is 90 km, so the crossover is its own and not the nightglow's:
    // from the pad both are dimmed by the same air, from 200 km up neither is.
    float cameraAltitude = length(planetCentre) - planetRadius;
    return inscatter +
           glow * mix(vec3(1.0), transmittance,
                      smoothstepf(a.airglowAltitude + a.airglowWidth,
                                  a.airglowAltitude - a.airglowWidth,
                                  cameraAltitude)) +
           aurora * mix(vec3(1.0), transmittance,
                        smoothstepf(kAuroraBase + 20000.0, kAuroraBase - 20000.0,
                                    cameraAltitude));
}

#endif // SW_ATMOSPHERE_GLSL
