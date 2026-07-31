#include "Physics/Kepler.hpp"

#include "Core/Assert.hpp"

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <atomic>
#include <limits>

namespace sw::phys::kepler
{
    namespace
    {
        constexpr f64 kTwoPi = 6.283185307179586476925286766559;
        constexpr f64 kParabolicBand = 1.0e-6; // |e-1| rejected below this

        [[nodiscard]] f64 wrapAngle(f64 angle)
        {
            angle = std::fmod(angle, kTwoPi);
            return (angle < 0.0) ? angle + kTwoPi : angle;
        }

        /// Elliptic Kepler equation M = E - e*sin(E), Newton-Raphson.
        [[nodiscard]] f64 solveEllipticAnomaly(f64 meanAnomaly, f64 eccentricity)
        {
            const f64 m = wrapAngle(meanAnomaly);
            f64 anomaly = (eccentricity < 0.8) ? m : 3.14159265358979323846;
            for (int iteration = 0; iteration < 16; ++iteration)
            {
                const f64 f = anomaly - eccentricity * std::sin(anomaly) - m;
                const f64 fPrime = 1.0 - eccentricity * std::cos(anomaly);
                const f64 step = f / fPrime;
                anomaly -= step;
                if (std::abs(step) < 1.0e-14)
                {
                    break;
                }
            }
            return anomaly;
        }

        /// Hyperbolic Kepler equation M = e*sinh(H) - H, Newton-Raphson.
        /// M is NOT wrapped (hyperbolic anomaly is unbounded).
        [[nodiscard]] f64 solveHyperbolicAnomaly(f64 meanAnomaly, f64 eccentricity)
        {
            // Robust start: H ~ asinh(M/e) (exact for e >> 1).
            f64 anomaly = std::asinh(meanAnomaly / eccentricity);
            for (int iteration = 0; iteration < 32; ++iteration)
            {
                const f64 f = eccentricity * std::sinh(anomaly) - anomaly - meanAnomaly;
                const f64 fPrime = eccentricity * std::cosh(anomaly) - 1.0;
                const f64 step = f / fPrime;
                anomaly -= step;
                if (std::abs(step) < 1.0e-13)
                {
                    break;
                }
            }
            return anomaly;
        }
    } // namespace

    KeplerOrbit fromElements(f64 mu, f64 semiMajorAxis, f64 eccentricity, f64 inclination,
                             f64 raan, f64 argumentOfPeriapsis, f64 meanAnomalyAtEpoch,
                             f64 epochSeconds)
    {
        SW_ASSERT(mu > 0.0 && semiMajorAxis > 0.0 && eccentricity >= 0.0 &&
                      eccentricity < 1.0,
                  "fromElements builds elliptic orbits (mu={}, a={}, e={})", mu,
                  semiMajorAxis, eccentricity);

        // Y-up frame: reference plane XZ, node measured about +Y from +X.
        const glm::dvec3 up{0.0, 1.0, 0.0};
        const glm::dvec3 xAxis{1.0, 0.0, 0.0};

        const glm::dquat nodeRotation = glm::angleAxis(raan, up);
        const glm::dvec3 nodeDirection = nodeRotation * xAxis;
        const glm::dquat inclinationRotation = glm::angleAxis(inclination, nodeDirection);
        const glm::dvec3 orbitNormal = inclinationRotation * up;
        const glm::dquat periapsisRotation = glm::angleAxis(argumentOfPeriapsis, orbitNormal);

        KeplerOrbit orbit{};
        orbit.mu = mu;
        orbit.semiMajorAxis = semiMajorAxis;
        orbit.eccentricity = eccentricity;
        orbit.meanMotion = std::sqrt(mu / (semiMajorAxis * semiMajorAxis * semiMajorAxis));
        orbit.meanAnomalyAtEpoch = wrapAngle(meanAnomalyAtEpoch);
        orbit.epochSeconds = epochSeconds;
        orbit.basisP = periapsisRotation * nodeDirection;
        orbit.basisQ = glm::cross(orbitNormal, orbit.basisP);
        return orbit;
    }

    f64 hyperbolicMeanAnomaly(f64 eccentricity, f64 trueAnomaly)
    {
        // nu -> H -> M (hyperbolic).
        //
        // THE ASYMPTOTE IS A CLIFF, and atanh goes over it silently. A
        // hyperbola only reaches true anomalies out to nu_inf = acos(-1/e),
        // and AT that angle sqrt((e-1)/(e+1)) * tan(nu/2) is exactly 1:
        // atanh is +infinity there and NaN one ulp past it. A NaN mean
        // anomaly is the worst possible thing to hand back, because it is
        // stored, not used — every position the rails ever evaluate from
        // that orbit is NaN afterwards, and the first visible sign is a
        // vessel that has left the world.
        //
        // MEASURED, nothing that comes through fromStateVectors reaches it,
        // and not by a little: its radial-trajectory guard needs
        // h > 1e-6 * r * v, which on a hyperbola is exactly
        // sqrt(e^2-1)/(e cosh H - 1) > 1e-6, and that holds the argument to
        // 1 - argument >= 1e-6 * e/sqrt(e^2-1). Bisected against the real
        // function, the closest state it will ACCEPT sits at 1 - argument =
        // 3.2796e-6 for e = 1.05, 1.3416e-6 for e = 1.5 and 1.0000e-6 for
        // e = 100 — six orders of magnitude short of this clamp, and none of
        // it this code's doing: the guard's threshold was picked for an
        // entirely different reason and the argument is 1 - p/r, so it is
        // one bad guard away from the cliff. Hence the clamp, and hence
        // this being a function anyone can call: a guard whose only caller
        // cannot reach it is a guard no test can hold to account.
        //
        // The clamp costs nothing where it does not engage and stays in f64
        // where it does: at 1 - 1e-12 the anomaly is H = 28.324190 and
        // sinh(H) = 1.000022e12, so M = 1.500033e12 for e = 1.5 — a large
        // number, but a number.
        const f64 factor = std::sqrt((eccentricity - 1.0) / (eccentricity + 1.0));
        const f64 tanHalfNu = std::tan(0.5 * trueAnomaly);
        const f64 argument = std::clamp(factor * tanHalfNu, -1.0 + 1.0e-12, 1.0 - 1.0e-12);
        const f64 hyperbolicAnomaly = 2.0 * std::atanh(argument);
        return eccentricity * std::sinh(hyperbolicAnomaly) - hyperbolicAnomaly;
    }

    bool fromStateVectors(f64 mu, const WorldVec3& relativePosition,
                          const WorldVec3& relativeVelocity, f64 epochSeconds,
                          KeplerOrbit& out, bool allowHyperbolic)
    {
        const WorldVec3 r = relativePosition;
        const f64 radius = glm::length(r);
        const f64 speedSq = glm::dot(relativeVelocity, relativeVelocity);
        if (radius <= 0.0 || mu <= 0.0)
        {
            return false;
        }

        const WorldVec3 angularMomentum = glm::cross(r, relativeVelocity);
        const f64 hLength = glm::length(angularMomentum);
        if (hLength <= 1.0e-6 * radius * std::sqrt(speedSq + 1.0e-12))
        {
            return false; // radial trajectory: not representable
        }
        const WorldVec3 orbitNormal = angularMomentum / hLength;

        const WorldVec3 eccVector =
            glm::cross(relativeVelocity, angularMomentum) / mu - r / radius;
        const f64 eccentricity = glm::length(eccVector);
        if (std::abs(eccentricity - 1.0) < kParabolicBand)
        {
            return false; // near-parabolic: numerically hostile, keep dynamic
        }

        const bool hyperbolic = eccentricity > 1.0;
        if (hyperbolic && !allowHyperbolic)
        {
            return false;
        }

        const f64 energy = 0.5 * speedSq - mu / radius;
        if (!hyperbolic && energy >= -1.0e-9)
        {
            return false; // elliptic bookkeeping needs bound energy
        }
        const f64 semiMajorAxis = -mu / (2.0 * energy); // negative if hyperbolic

        const WorldVec3 basisP =
            (eccentricity > 1.0e-9) ? eccVector / eccentricity : r / radius;
        const WorldVec3 basisQ = glm::cross(orbitNormal, basisP);

        // True anomaly at epoch, with quadrant from the radial velocity.
        const f64 cosNu = std::clamp(glm::dot(basisP, r / radius), -1.0, 1.0);
        f64 trueAnomaly = std::acos(cosNu);
        if (glm::dot(r, relativeVelocity) < 0.0)
        {
            trueAnomaly = -trueAnomaly; // symmetric range works for both conics
        }

        f64 meanAnomaly = 0.0;
        if (hyperbolic)
        {
            meanAnomaly = hyperbolicMeanAnomaly(eccentricity, trueAnomaly);
        }
        else
        {
            const f64 halfNu = 0.5 * trueAnomaly;
            const f64 eccentricAnomaly =
                2.0 * std::atan2(std::sqrt(1.0 - eccentricity) * std::sin(halfNu),
                                 std::sqrt(1.0 + eccentricity) * std::cos(halfNu));
            meanAnomaly = eccentricAnomaly - eccentricity * std::sin(eccentricAnomaly);
            meanAnomaly = wrapAngle(meanAnomaly);
        }

        out.mu = mu;
        out.semiMajorAxis = semiMajorAxis;
        out.eccentricity = eccentricity;
        const f64 absA = std::abs(semiMajorAxis);
        out.meanMotion = std::sqrt(mu / (absA * absA * absA));
        out.meanAnomalyAtEpoch = meanAnomaly;
        out.epochSeconds = epochSeconds;
        out.basisP = basisP;
        out.basisQ = basisQ;
        return true;
    }

    bool evaluate(const KeplerOrbit& orbit, f64 timeSeconds, WorldVec3& outPosition,
                  WorldVec3* outVelocity)
    {
        const f64 e = orbit.eccentricity;

        // A DEFAULT-CONSTRUCTED ORBIT IS NOT AN ORBIT, and it used to be
        // possible to ask one for a velocity and be told a NaN with a
        // straight face. `KeplerOrbit{}` has mu = 0 and a = 0, so the
        // semi-latus rectum below is 0 and the speed factor is sqrt(0/0):
        // the position came back as the origin, which at least looks wrong,
        // but the VELOCITY came back NaN — and the one caller that asks for
        // a velocity is the bubble's rails->dynamic hand-off, which writes
        // it straight into a DynamicBodyComponent. From there the integrator
        // multiplies the NaN by a timestep every tick for ever, the entity's
        // position is NaN, its transform is NaN, and the first visible sign
        // is a mesh that has vanished from the world with nothing in the log.
        //
        // So it refuses, says so, and hands back zeros rather than poison.
        // The caller decides what to do about it; PhysicsSystems declines to
        // convert the entity at all, which leaves it on rails where it was.
        const f64 semiLatus = orbit.semiMajorAxis * (1.0 - e * e); // >0 both conics
        if (!(orbit.mu > 0.0) || !(semiLatus > 0.0) || !std::isfinite(orbit.meanMotion) ||
            !std::isfinite(orbit.meanAnomalyAtEpoch))
        {
            outPosition = WorldVec3{0.0};
            if (outVelocity != nullptr)
            {
                *outVelocity = WorldVec3{0.0};
            }
            // Once. This is called from a per-tick system over every railed
            // entity in the game; a degenerate orbit would otherwise write a
            // line of log per body per tick and bury whatever came next.
            static std::atomic<bool> reported{false};
            if (!reported.exchange(true))
            {
                SW_LOG_ERROR("Kepler",
                             "Refusing to evaluate a degenerate orbit (mu={}, a={}, e={}): "
                             "a default-constructed KeplerOrbit has no velocity to give",
                             orbit.mu, orbit.semiMajorAxis, e);
            }
            return false;
        }

        const f64 meanAnomaly =
            orbit.meanAnomalyAtEpoch + orbit.meanMotion * (timeSeconds - orbit.epochSeconds);

        f64 trueAnomaly = 0.0;
        f64 radius = 0.0;
        if (orbit.isHyperbolic())
        {
            const f64 hyperbolicAnomaly = solveHyperbolicAnomaly(meanAnomaly, e);
            const f64 factor = std::sqrt((e + 1.0) / (e - 1.0));
            trueAnomaly = 2.0 * std::atan(factor * std::tanh(0.5 * hyperbolicAnomaly));
            radius = orbit.semiMajorAxis * (1.0 - e * std::cosh(hyperbolicAnomaly));
        }
        else
        {
            const f64 eccentricAnomaly = solveEllipticAnomaly(meanAnomaly, e);
            const f64 halfE = 0.5 * eccentricAnomaly;
            trueAnomaly = 2.0 * std::atan2(std::sqrt(1.0 + e) * std::sin(halfE),
                                           std::sqrt(1.0 - e) * std::cos(halfE));
            radius = orbit.semiMajorAxis * (1.0 - e * std::cos(eccentricAnomaly));
        }

        const f64 cosNu = std::cos(trueAnomaly);
        const f64 sinNu = std::sin(trueAnomaly);

        outPosition = orbit.basisP * (radius * cosNu) + orbit.basisQ * (radius * sinNu);

        if (outVelocity != nullptr)
        {
            const f64 speedFactor = std::sqrt(orbit.mu / semiLatus);
            *outVelocity = orbit.basisP * (-speedFactor * sinNu) +
                           orbit.basisQ * (speedFactor * (e + cosNu));
        }
        return true;
    }

    f64 timeAtArcFraction(const KeplerOrbit& orbit, f64 t0, f64 t1, f64 fraction)
    {
        const f64 e = orbit.eccentricity;
        if (!(orbit.meanMotion > 0.0) || std::abs(e - 1.0) < kParabolicBand)
        {
            return t0 + (t1 - t0) * fraction; // degenerate: nothing better to do
        }

        const f64 mean0 =
            orbit.meanAnomalyAtEpoch + orbit.meanMotion * (t0 - orbit.epochSeconds);
        const f64 mean1 =
            orbit.meanAnomalyAtEpoch + orbit.meanMotion * (t1 - orbit.epochSeconds);

        f64 anomaly0 = 0.0;
        f64 anomaly1 = 0.0;
        if (orbit.isHyperbolic())
        {
            anomaly0 = solveHyperbolicAnomaly(mean0, e);
            anomaly1 = solveHyperbolicAnomaly(mean1, e);
        }
        else
        {
            // UNWRAPPED eccentric anomaly. The solver answers in [0, 2pi),
            // which would send an arc that crosses periapsis backwards
            // round the ellipse. The whole-revolution count comes from the
            // mean anomaly, which is linear in time and never wraps.
            const f64 turns0 = std::floor(mean0 / kTwoPi);
            const f64 turns1 = std::floor(mean1 / kTwoPi);
            anomaly0 = solveEllipticAnomaly(mean0, e) + turns0 * kTwoPi;
            anomaly1 = solveEllipticAnomaly(mean1, e) + turns1 * kTwoPi;
        }

        const f64 anomaly = anomaly0 + (anomaly1 - anomaly0) * fraction;
        const f64 mean = orbit.isHyperbolic() ? (e * std::sinh(anomaly) - anomaly)
                                              : (anomaly - e * std::sin(anomaly));
        return orbit.epochSeconds + (mean - orbit.meanAnomalyAtEpoch) / orbit.meanMotion;
    }

    f64 period(const KeplerOrbit& orbit)
    {
        SW_ASSERT(!orbit.isHyperbolic(), "Hyperbolic orbits have no period");
        return kTwoPi / orbit.meanMotion;
    }

    f64 periapsis(const KeplerOrbit& orbit)
    {
        return orbit.semiMajorAxis * (1.0 - orbit.eccentricity); // works for a<0, e>1
    }

    f64 apoapsis(const KeplerOrbit& orbit)
    {
        if (orbit.isHyperbolic())
        {
            return std::numeric_limits<f64>::infinity();
        }
        return orbit.semiMajorAxis * (1.0 + orbit.eccentricity);
    }
} // namespace sw::phys::kepler
