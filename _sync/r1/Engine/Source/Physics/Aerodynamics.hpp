#pragma once

// ============================================================================
// Physics/Aerodynamics.hpp
// REAL AERODYNAMICS — tabulated offline, summed at runtime.
//
// The old model was one number per vessel: a ballistic factor, Sigma(Cd*A)
// divided by mass. It could not tell a rocket flying nose-first from the
// same rocket flying sideways, it produced no torque, so no fin ever
// stabilised anything, and every coefficient in it was typed by hand.
//
// This is the replacement, and the shape of it is the whole idea:
//
//   OFFLINE (Tools/AeroForge, once per part)
//       For a few hundred wind directions, a surface solver integrates the
//       pressure and friction over the part's REAL triangulated geometry
//       and records the resulting FORCE and MOMENT. The result is one
//       `.aero.json` file per `.swpart`.
//
//   AT RUNTIME (every physics tick, per part)
//       Look up the flow direction in the part's own frame, read the table,
//       scale by the dynamic pressure and by how much of the part the air
//       can actually SEE, and add it to the vessel's running total.
//
// WHY FORCE AND MOMENT, NOT ACCELERATION. An acceleration depends on the
// mass of the whole vehicle; a force does not. The same fin bolted to a
// probe and to a three-hundred-tonne booster produces the same newtons and
// the same newton-metres for the same airflow — so the table belongs to the
// PART, is generated once, and is valid on every vessel that part is ever
// welded to. The rigid-body solver then does what it is for: sums forces,
// sums moments, and produces the acceleration, the rotation and the
// oscillation for free.
//
// COEFFICIENTS, NOT FORCES, ARE STORED. Every entry is divided by the
// dynamic pressure q = 1/2 rho v^2, which makes the force entries an AREA
// (m^2) and the moment entries a VOLUME (m^3). That is what makes one table
// valid at every speed and every altitude: the shape of the flow around a
// part depends on the DIRECTION it meets the air, and — later — on Mach and
// Reynolds; it does not depend on how thick the air is.
// ============================================================================

#include "Core/Types.hpp"
#include "Math/Math.hpp"
#include "Physics/PhysicsComponents.hpp"

#include <filesystem>
#include <span>
#include <vector>

namespace sw::aero
{
    // ------------------------------------------------------------------------
    // THE AIR
    // ------------------------------------------------------------------------

    /// Density at an altitude above SEA level, exponential profile.
    /// Clamped at and below sea level, zero above the atmosphere's top so
    /// there is no discontinuity to integrate across on the way out.
    [[nodiscard]] f64 density(const phys::AtmosphereComponent& atmosphere, f64 altitude);

    /// Air temperature, linear lapse rate with a stratospheric floor.
    /// It exists for ONE reason — the speed of sound — but that one reason
    /// is what turns a speed into a Mach number, and a Mach number is what
    /// decides whether a shape is cheap or expensive to push through the air.
    [[nodiscard]] f64 temperature(const phys::AtmosphereComponent& atmosphere,
                                  f64 altitude);

    /// a = sqrt(gamma * R * T).
    [[nodiscard]] f64 speedOfSound(const phys::AtmosphereComponent& atmosphere,
                                   f64 altitude);

    /// Dynamic pressure q = 1/2 rho v^2, in pascals. Every tabulated
    /// coefficient is multiplied by this and by nothing else.
    [[nodiscard]] inline f64 dynamicPressure(f64 densityKgM3, f64 speedMps)
    {
        return 0.5 * densityKgM3 * speedMps * speedMps;
    }

    /// COMPRESSIBILITY, as a single multiplier on the whole tabulated force.
    ///
    /// The tables are solved incompressibly. Rather than store a Mach axis
    /// that would multiply every file by six (and that the forge cannot yet
    /// fill honestly), the transonic rise is carried as a curve: flat below
    /// M 0.8, a peak of about 1.6 just past M 1, decaying toward 1.1 by
    /// M 5. When the forge learns to solve per-Mach the tables grow an axis
    /// and this function retires — the runtime call site does not change,
    /// which is the point of keeping it separate.
    [[nodiscard]] f64 machDragFactor(f64 mach);

    /// WIND: the air is not still.
    ///
    /// Deterministic and closed-form — no state, no random numbers, the
    /// same answer on every machine and after every reload, which is what
    /// lets a trajectory prediction and the flight it predicts agree. A
    /// broad eastward jet peaking near 11 km, plus a slow shear that turns
    /// it over the course of an hour. Returns a velocity in the LOCAL
    /// SURFACE frame — add it to the ground's own motion, never to a world
    /// velocity (THE CARRIER-VELOCITY RULE).
    [[nodiscard]] WorldVec3 windVelocity(const phys::AtmosphereComponent& atmosphere,
                                         f64 altitude, const WorldVec3& up,
                                         const WorldVec3& east, f64 timeSeconds);

    // ------------------------------------------------------------------------
    // THE TABLE
    // ------------------------------------------------------------------------

    /// One solved wind direction: what the part does to the air, per pascal
    /// of dynamic pressure.
    struct AeroSample
    {
        /// Force / q, in square metres, in the PART's own frame.
        Vec3 forceM2{0.0f};
        /// Moment / q about the PART ORIGIN, in cubic metres, part frame.
        /// About the origin and not about the part's own centre of mass on
        /// purpose: the vessel that mounts it knows where its own centre of
        /// mass is and where the part sits, and can shift the moment there
        /// in one cross product. A table that had already picked a pivot
        /// would be wrong on every vessel but one.
        Vec3 momentM3{0.0f};
    };

    /// THE SOLVED PART.
    ///
    /// A rectangular grid over the direction the air arrives from, in the
    /// part's own frame:
    ///
    ///   theta  angle from +Z, 0 .. pi over `thetaCount` nodes.
    ///          -Z is the nose of a stack, so theta = 0 is HEAD-ON: the air
    ///          comes at the nose. theta = pi is flying backwards.
    ///   phi    azimuth atan2(y, x), 0 .. 2pi over `phiCount` nodes, the
    ///          last node one step short of a full turn because it wraps.
    ///
    /// Row-major: sample(t, p) = samples[t * phiCount + p].
    struct AeroTable
    {
        u32 partId = 0;
        std::string partName;
        u32 thetaCount = 0;
        u32 phiCount = 0;
        std::vector<AeroSample> samples;
        /// Bookkeeping the forge writes and the game only ever displays:
        /// the biggest silhouette the part has, and the reference length
        /// used for the damping estimate.
        f64 maxAreaM2 = 0.0;
        f64 referenceLengthM = 1.0;

        [[nodiscard]] bool valid() const
        {
            return thetaCount >= 2 && phiCount >= 1 &&
                   samples.size() == static_cast<size_t>(thetaCount) * phiCount;
        }
    };

    /// The part-frame direction the AIR TRAVELS IN, from a part-frame
    /// relative wind. Nose-first flight makes the air move toward +Z.
    [[nodiscard]] inline Vec3 flowDirection(const Vec3& partVelocityInAir)
    {
        const f32 speed = glm::length(partVelocityInAir);
        return (speed > 1.0e-9f) ? Vec3(-partVelocityInAir / speed) : Vec3(0.0f, 0.0f, 1.0f);
    }

    /// Bilinear read of the table for a unit flow direction in part space.
    /// Wraps in phi (it is an angle) and clamps in theta (it is not).
    [[nodiscard]] AeroSample sample(const AeroTable& table, const Vec3& flowDirectionLocal);

    /// Read/write the `.aero.json` sidecar. The on-disk form is flat arrays
    /// of numbers rather than an array of objects: a table is a few thousand
    /// numbers and nobody hand-edits it — the forge writes it, the game
    /// reads it, and keeping it flat keeps the file a tenth of the size.
    [[nodiscard]] bool loadAeroTable(const std::filesystem::path& path, AeroTable& out);
    [[nodiscard]] bool saveAeroTable(const AeroTable& table,
                                     const std::filesystem::path& path);

    /// Every `*.aero.json` in a directory, keyed by part id. Missing files
    /// are not an error: a part without a table falls back to the old
    /// isotropic drag, which is exactly what should happen to a part nobody
    /// has run the forge on yet.
    [[nodiscard]] std::vector<AeroTable> loadAeroTables(
        const std::filesystem::path& directory);

    // ------------------------------------------------------------------------
    // OCCLUSION — what the air can actually SEE
    //
    // A twenty-tank booster is not twenty tanks' worth of drag: nineteen of
    // them are hiding behind the first one. Solving that properly is a
    // wind-tunnel run per VESSEL, which is exactly what this architecture
    // refuses to do at runtime. So the geometry answers a much smaller
    // question — is there anything between this part and the oncoming air?
    // — with a handful of rays against the other parts' boxes.
    //
    // Cheap, and right about the case that matters: shielded parts stop
    // contributing, exposed ones do not, and the answer changes the moment
    // the vessel turns or a stage falls away.
    // ------------------------------------------------------------------------

    /// One part's collision box, posed in the VESSEL frame.
    struct OccluderBox
    {
        Vec3 centre{0.0f};
        Vec3 halfExtents{0.5f};
        Quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
        /// Which part this box belongs to. A part never shadows itself.
        u32 ownerIndex = 0;
    };

    /// Fraction of `boxes[selfIndex]`'s frontal area that the oncoming air
    /// reaches, in [minimum, 1]. `flowDirectionVessel` is the direction the
    /// air TRAVELS in, vessel frame, unit length.
    ///
    /// Never returns zero: a part in a perfect wake still feels base
    /// pressure, and a part whose drag switches off entirely makes a
    /// shielded stage weightless in the airstream — which reads as a bug
    /// long before it reads as physics.
    [[nodiscard]] f32 exposure(std::span<const OccluderBox> boxes, u32 selfIndex,
                               const Vec3& flowDirectionVessel, f32 minimum = 0.05f);

    /// Slab test of a ray against ONE posed box. `outNear` is the entry
    /// distance (negative when the origin is already inside).
    [[nodiscard]] bool rayHitsBox(const OccluderBox& box, const Vec3& origin,
                                  const Vec3& direction, f32 maxDistance, f32& outNear);

    // ------------------------------------------------------------------------
    // TURNING MOMENTS INTO MOTION
    // ------------------------------------------------------------------------

    /// Diagonal inertia of a solid box about its own centre, per axis.
    /// The vessel's tensor is the sum of these plus the parallel-axis term
    /// for each part — good to a few per cent for anything made of stacked
    /// cylinders, which is what a rocket is.
    [[nodiscard]] inline Vec3 boxInertia(f64 massKg, const Vec3& halfExtents)
    {
        const f64 x = 2.0 * static_cast<f64>(halfExtents.x);
        const f64 y = 2.0 * static_cast<f64>(halfExtents.y);
        const f64 z = 2.0 * static_cast<f64>(halfExtents.z);
        const f64 k = massKg / 12.0;
        return Vec3(static_cast<f32>(k * (y * y + z * z)),
                    static_cast<f32>(k * (x * x + z * z)),
                    static_cast<f32>(k * (x * x + y * y)));
    }

    /// AERODYNAMIC DAMPING — why a stable rocket stops wobbling.
    ///
    /// A weathercocking rocket that only ever felt a restoring moment would
    /// swing through the wind and back forever, like a pendulum in vacuum.
    /// What stops it is that the body is also ROTATING: a fin a distance d
    /// behind the centre of mass is being swept sideways at omega*d, which
    /// tilts the air it meets by omega*d/v and produces a force opposing the
    /// rotation. Linearised, that is a moment proportional to -omega:
    ///
    ///     M_damp = -1/2 rho v A d^2 k omega
    ///
    /// with d the distance from the centre of mass to the centre of
    /// pressure — the same lever arm that produced the restoring moment,
    /// which is why a vehicle that is stable is also damped and one that is
    /// not is neither.
    [[nodiscard]] Vec3 dampingMoment(f64 densityKgM3, f64 speedMps, f64 areaM2,
                                     f64 leverArmM, const Vec3& angularVelocity);

    // ------------------------------------------------------------------------
    // THE RESULT, for the flight computer and the HUD
    // ------------------------------------------------------------------------

    /// Written on a vessel root every physics tick it spends in air.
    /// Zeroed in vacuum, so a HUD can read it unconditionally.
    struct AeroStateComponent
    {
        WorldVec3 forceN{0.0};        // world frame, already summed
        Vec3 momentNm{0.0f};          // vessel frame, about the centre of mass
        Vec3 angularAccelRadS2{0.0f}; // vessel frame, moment / inertia
        Vec3 centreOfPressure{0.0f};  // vessel frame
        Vec3 centreOfMass{0.0f};      // vessel frame
        Vec3 inertiaKgM2{1.0f};       // vessel frame, diagonal
        f64 dynamicPressurePa = 0.0;
        f64 densityKgM3 = 0.0;
        f64 machNumber = 0.0;
        f64 airspeedMps = 0.0;
        f64 angleOfAttackRad = 0.0;
        f64 dragN = 0.0; // component opposing the airflow
        f64 liftN = 0.0; // component across it
        u32 inAtmosphere = 0;
    };

    static_assert(std::is_trivially_copyable_v<AeroStateComponent>);
} // namespace sw::aero
