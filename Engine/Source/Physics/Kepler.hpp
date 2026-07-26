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

        /// PRIMARY-RELATIVE position (and optionally velocity) at time t.
        void evaluate(const KeplerOrbit& orbit, f64 timeSeconds, WorldVec3& outPosition,
                      WorldVec3* outVelocity = nullptr);

        /// Orbital period; only meaningful for elliptic orbits.
        [[nodiscard]] f64 period(const KeplerOrbit& orbit);

        /// Periapsis distance (valid for both conic types).
        [[nodiscard]] f64 periapsis(const KeplerOrbit& orbit);
        /// Apoapsis distance; +infinity for hyperbolic orbits.
        [[nodiscard]] f64 apoapsis(const KeplerOrbit& orbit);

        /// Circular-orbit speed at radius r around a body of parameter mu.
        [[nodiscard]] inline f64 circularOrbitSpeed(f64 mu, f64 radius)
        {
            return std::sqrt(mu / radius);
        }
    } // namespace kepler
} // namespace sw::phys
