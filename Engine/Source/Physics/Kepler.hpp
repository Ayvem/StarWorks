#pragma once

// ============================================================================
// Physics/Kepler.hpp
// Analytic two-body (Keplerian) orbits — the "on rails" half of the physics
// architecture, and the math behind trajectory prediction.
//
// v2 (hierarchical star system): orbits are RELATIVE to their primary —
// evaluate() returns primary-relative position/velocity; the caller adds
// the primary's own world state (which may itself be on rails around its
// parent: Sun -> planets -> moons). KeplerOrbit no longer stores a center.
//
// Both ELLIPTIC (e < 1) and HYPERBOLIC (e > 1) trajectories are supported:
// rails objects stay elliptic by policy, but patched-conics prediction
// needs hyperbolic arcs (a lunar flyby IS hyperbolic relative to the moon).
// The near-parabolic band |e-1| < 1e-6 is rejected (numerically hostile,
// physically a measure-zero case).
//
// Conventions (engine-wide): +Y is "north", the reference plane is XZ.
// Storage layout: the perifocal basis (P toward periapsis, Q 90° ahead in
// the direction of motion) is PRECOMPUTED, so evaluate() is just an anomaly
// solve + two trig calls + a linear combination.
// ============================================================================

#include "Core/Types.hpp"
#include "Math/Math.hpp"

#include <cmath>
#include <type_traits>

namespace sw::phys
{
    struct KeplerOrbit
    {
        f64 mu = 0.0;                 // gravitational parameter GM of the primary
        f64 semiMajorAxis = 0.0;      // meters; NEGATIVE for hyperbolic orbits
        f64 eccentricity = 0.0;       // [0,1) elliptic, (1,inf) hyperbolic
        f64 meanMotion = 0.0;         // rad/s (sqrt(mu/|a|^3), precomputed)
        f64 meanAnomalyAtEpoch = 0.0; // radians (hyperbolic: unwrapped)
        f64 epochSeconds = 0.0;       // simulation time of the elements
        WorldVec3 basisP{1.0, 0.0, 0.0}; // unit vector toward periapsis
        WorldVec3 basisQ{0.0, 0.0, 1.0}; // unit vector 90 deg ahead (motion side)

        [[nodiscard]] bool isHyperbolic() const { return eccentricity > 1.0; }
    };
    static_assert(std::is_trivially_copyable_v<KeplerOrbit>);

    namespace kepler
    {
        /// Builds an ELLIPTIC orbit from classical elements (radians;
        /// Y-up convention: inclination vs the XZ plane, RAAN about +Y from
        /// +X, argument of periapsis in-plane from the ascending node).
        [[nodiscard]] KeplerOrbit fromElements(f64 mu, f64 semiMajorAxis, f64 eccentricity,
                                               f64 inclination, f64 raan,
                                               f64 argumentOfPeriapsis, f64 meanAnomalyAtEpoch,
                                               f64 epochSeconds);

        /// Builds an orbit from a PRIMARY-RELATIVE position/velocity pair at
        /// `epochSeconds`. Elliptic only unless allowHyperbolic; returns
        /// false (out untouched) for rejected states (near-parabolic,
        /// radial, or hyperbolic when not allowed).
        [[nodiscard]] bool fromStateVectors(f64 mu, const WorldVec3& relativePosition,
                                            const WorldVec3& relativeVelocity,
                                            f64 epochSeconds, KeplerOrbit& out,
                                            bool allowHyperbolic = false);

        /// MEAN ANOMALY FROM TRUE ANOMALY ON A HYPERBOLA, asymptote included.
        ///
        /// Its own entry point because the whole content of it is a guard,
        /// and a guard nothing can call is a guard nothing can test. A
        /// hyperbola only reaches out to nu_inf = acos(-1/e), where the atanh
        /// argument is exactly 1 — and fromStateVectors' radial-trajectory
        /// check keeps every state IT accepts a MEASURED six orders of
        /// magnitude short of that: 1 - argument >= 1e-6 * e/sqrt(e^2-1), so
        /// 3.28e-6 at e = 1.05 and 1.00e-6 at e = 100, against a clamp that
        /// engages at 1e-12. The cliff is unreachable from there, reachable
        /// from here, and here is where it is proved harmless. `eccentricity`
        /// must be > 1; at or past the asymptote the answer is the one AT the
        /// clamp rather than an infinity or a NaN.
        [[nodiscard]] f64 hyperbolicMeanAnomaly(f64 eccentricity, f64 trueAnomaly);

        /// PRIMARY-RELATIVE position (and optionally velocity) at time t.
        ///
        /// Returns FALSE, writes zeros and logs, for an orbit that is not one
        /// — mu <= 0, a degenerate semi-latus rectum, a non-finite element.
        /// A default-constructed `KeplerOrbit` is exactly that case and it
        /// used to answer with a NaN velocity, which the rails->dynamic
        /// hand-off wrote straight into a DynamicBodyComponent. Deliberately
        /// NOT [[nodiscard]]: most callers only want a position and a zero is
        /// a safe one, but anything taking a VELOCITY from it must check.
        /// THE SAME EVALUATION AT FULL TIME PRECISION.
        ///
        /// `wholeSeconds` is an exact integer second count and `fraction` is
        /// in [0, 1) — see sim::Simulation::wholeSeconds() for why the clock
        /// is carried that way. For a CLOSED orbit the elapsed interval is
        /// reduced modulo the period before it is multiplied by the mean
        /// motion, and because the whole part is exact that reduction is
        /// exact: a session that has run for an interstellar crossing gets
        /// the same answer as one that started a second ago. An OPEN orbit
        /// has no period to reduce by and falls through to the plain path,
        /// which is correct — a hyperbola's mean anomaly is unwrapped by
        /// definition and a craft on one is not there for ten thousand years.
        ///
        /// Measured against the single-double path at 3.3e11 s of simulated
        /// time: 2.18 m of per-tick position noise becomes 0.27 mm.
        bool evaluateSplit(const KeplerOrbit& orbit, f64 wholeSeconds, f64 fraction,
                           WorldVec3& outPosition, WorldVec3* outVelocity = nullptr);

        bool evaluate(const KeplerOrbit& orbit, f64 timeSeconds, WorldVec3& outPosition,
                      WorldVec3* outVelocity = nullptr);

        /// Orbital period; only meaningful for elliptic orbits.
        [[nodiscard]] f64 period(const KeplerOrbit& orbit);

        /// Periapsis distance (valid for both conic types).
        [[nodiscard]] f64 periapsis(const KeplerOrbit& orbit);
        /// Apoapsis distance; +infinity for hyperbolic orbits.
        [[nodiscard]] f64 apoapsis(const KeplerOrbit& orbit);

        /// THE TIME AT WHICH TO TAKE THE n-th SAMPLE OF A DRAWN ARC.
        ///
        /// `fraction` walks 0..1 across [t0, t1] — but not at a constant
        /// rate in TIME. At a constant rate in ECCENTRIC anomaly, which is
        /// very nearly constant in arc length, and that distinction is the
        /// difference between a drawable orbit and a wrong one: on a Terra-
        /// to-Luna transfer (e = 0.965) the craft crosses most of its
        /// angular sweep in a few hours out of ten days, so samples spaced
        /// evenly in time leave one chord that cuts 2 800 km off the
        /// periapsis — a line drawn straight through the planet.
        ///
        /// Hyperbolic orbits are parameterised the same way, by hyperbolic
        /// anomaly. A degenerate orbit falls back to linear in time.
        [[nodiscard]] f64 timeAtArcFraction(const KeplerOrbit& orbit, f64 t0, f64 t1,
                                            f64 fraction);

        /// Circular-orbit speed at radius r around a body of parameter mu.
        /// Zero for a body that is not there or a radius that is not one:
        /// sqrt(0/0) is a NaN, and a NaN speed handed to a manoeuvre planner
        /// propagates further than the mistake that produced it.
        [[nodiscard]] inline f64 circularOrbitSpeed(f64 mu, f64 radius)
        {
            if (!(mu > 0.0) || !(radius > 0.0))
            {
                return 0.0;
            }
            return std::sqrt(mu / radius);
        }
    } // namespace kepler
} // namespace sw::phys
