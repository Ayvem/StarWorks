#pragma once

// ============================================================================
// Physics/PhysicsComponents.hpp
// The two mutually exclusive motion regimes of the engine:
//
//  - DynamicBodyComponent : TRULY simulated. Newtonian gravity integrated
//    every Physics tick (50 Hz). Reserved for objects near the player —
//    the BubbleSystem enforces this.
//  - OnRailsComponent     : analytic Kepler orbit. No integration ever; the
//    RailsSystem refreshes the transform at low frequency from the closed-
//    form solution. This is the default state of every distant object.
//
//  - GravitySourceComponent marks celestial bodies that generate gravity.
// ============================================================================

#include "ECS/Entity.hpp"
#include "Physics/Kepler.hpp"

namespace sw::phys
{
    struct DynamicBodyComponent
    {
        WorldVec3 velocity{0.0}; // m/s, world frame
        f64 mass = 1000.0;       // kg (used by thrust; gravity is mass-free)
        /// Effective drag coefficient x area / mass (m^2/kg): atmosphere
        /// deceleration = 0.5 * rho * v^2 * ballistic. ~0.002 for a ship.
        f64 ballisticFactor = 0.002;
        /// Set by SurfaceInteractionSystem while resting on a body surface.
        u32 isGrounded = 0;
    };

    /// WHAT A BODY RESTS ON: an axis-aligned box in the entity's MODEL
    /// space, tight around its collision hull.
    ///
    /// Without one, ground contact has no idea how big the thing landing
    /// is, and the only position it can compute is "put the ORIGIN on the
    /// ground" — which buries every object up to its origin. A rocket
    /// modelled around its centre sinks half its length; the EVA capsule, a
    /// 2 m body centred on itself, sinks exactly 1 m and the player walks
    /// with their waist in the rock.
    ///
    /// A box rather than a radius, on purpose. A bounding SPHERE would
    /// float a 20 m rocket ten metres above the pad, because a rocket's
    /// bounding radius is half its LENGTH. The box knows the difference
    /// between the direction a vehicle is long in and the direction it is
    /// wide in — which is the entire question when it is standing on its
    /// tail.
    struct GroundHullComponent
    {
        Vec3 centre{0.0f};      // model space, metres
        Vec3 halfExtents{0.0f}; // model space, metres
    };

    /// Distance from the entity origin down to the lowest point of the
    /// hull, for a body with this rotation standing under this `up`. The
    /// resting radius is the terrain height plus this.
    ///
    /// The projection is done in MODEL space — a length is the same in
    /// either frame — so it is exact for the box at ANY attitude: a rocket
    /// lying on its side rests on its flank, not on its tail.
    [[nodiscard]] inline f64 groundClearance(const GroundHullComponent& hull,
                                             const Quat& rotation, const Vec3& up)
    {
        const Vec3 down = glm::inverse(rotation) * (-up);
        const f32 clearance = glm::dot(hull.centre, down) +
                              std::abs(down.x) * hull.halfExtents.x +
                              std::abs(down.y) * hull.halfExtents.y +
                              std::abs(down.z) * hull.halfExtents.z;
        // A hull whose box does not enclose its own origin can report a
        // negative clearance; resting "below" the ground is never what the
        // caller means.
        return static_cast<f64>(std::max(0.0f, clearance));
    }

    /// v2 (hierarchical systems): the orbit is PRIMARY-RELATIVE and the
    /// component remembers which body it orbits. The RailsSystem adds the
    /// primary's current world position — so a station riding rails around
    /// Terra follows Terra around the Sun for free.
    struct OnRailsComponent
    {
        KeplerOrbit orbit{};   // primary-relative elements
        ecs::Entity primary{}; // gravity source orbited (null = world origin)
        /// DynamicBody payload preserved across regime conversions: when the
        /// bubble railifies a ship (time warp!) and later releases it, the
        /// ship must come back with ITS mass, not a default.
        f64 dynamicMass = 1000.0;
        f64 dynamicBallisticFactor = 0.002;
    };

    struct GravitySourceComponent
    {
        f64 mu = 0.0;         // GM, m^3/s^2
        f64 bodyRadius = 0.0; // meters — solid surface + altitude rules
        /// Sphere-of-influence radius: inside it, THIS body is the primary
        /// for rails conversions and trajectory patching. The default is
        /// effectively infinite (a lone body owns all of space); real
        /// systems set r_SOI = a * (mu/mu_parent)^(2/5).
        f64 soiRadius = 1.0e300;
        /// Current world-frame velocity of the body, stamped each Physics
        /// tick by whatever moves it (CelestialMotionSystem). Everything
        /// body-relative — atmosphere drag, ground friction, orbit
        /// conversions — measures velocities against this, so physics stays
        /// correct while the planet itself races around its star.
        WorldVec3 worldVelocity{0.0};
        /// Spin of the body: axis * rate (rad/s, world frame). The SURFACE
        /// (and the atmosphere) move at worldVelocity + angularVelocity x r:
        /// ground friction and EVA walking act in that ROTATING frame, so a
        /// landed ship co-rotates with its planet instead of having the
        /// ground slide away underneath it at hundreds of m/s.
        WorldVec3 angularVelocity{0.0};

        // ---- the body's attitude, in f64 ---------------------------------
        // TransformComponent::rotation is an f32 quaternion. That is right
        // for ORIENTING a mesh and ruinous for POSITIONING anything 6,371 km
        // from the axis: f32 carries ~7 digits, so that lever arm is
        // quantised to about 1.2 m — and the error CHANGES as the planet
        // turns, by up to 0.77 m from one frame to the next. Everything
        // anchored to the ground therefore shimmered, and so did the terrain
        // patch, while the camera (a f64 world position) held still.
        //
        // So the spin is kept here at full precision and stamped every tick
        // by whatever turns the body. Position math uses THIS; orientation
        // math can go on using the f32 quaternion, where a 1e-7 rad error is
        // a fraction of a pixel.
        WorldVec3 spinAxis{0.0, 1.0, 0.0};
        f64 spinAngle = 0.0;         // radians at the lane's present time
        f64 spinAnglePrevious = 0.0; // one tick earlier: the render's partner
    };

    /// The body's rotation at a given spin angle, as an exact f64 rotation.
    [[nodiscard]] inline glm::dquat spinRotation(const GravitySourceComponent& source,
                                                 f64 angle)
    {
        const f64 length = glm::length(source.spinAxis);
        if (length < 1.0e-12)
        {
            return glm::dquat{1.0, 0.0, 0.0, 0.0};
        }
        return glm::angleAxis(angle, source.spinAxis / length);
    }

    [[nodiscard]] inline glm::dquat spinRotation(const GravitySourceComponent& source)
    {
        return spinRotation(source, source.spinAngle);
    }

    /// The body's rotation partway through the current tick, for rendering.
    /// The angle is kept wrapped to [0, 2pi), so the two ends of a tick can
    /// straddle the wrap — interpolating them naively would sweep the planet
    /// the long way round in one frame. Take the SHORT arc, exactly as the
    /// f32 slerp beside it does.
    [[nodiscard]] inline glm::dquat spinRotationAt(const GravitySourceComponent& source,
                                                   f64 alpha)
    {
        constexpr f64 kTwoPi = 6.283185307179586;
        f64 delta = source.spinAngle - source.spinAnglePrevious;
        if (delta > kTwoPi * 0.5)
        {
            delta -= kTwoPi;
        }
        else if (delta < -kTwoPi * 0.5)
        {
            delta += kTwoPi;
        }
        return spinRotation(source, source.spinAnglePrevious + delta * alpha);
    }

    /// Rigidly attaches an entity to a celestial body's SURFACE: the local
    /// position is expressed in the body's rotating frame, so anchored
    /// structures (mines, factories, launch pads) co-rotate with the planet
    /// or asteroid they are built on. This — not absolute coordinates — is
    /// what makes surface bases save/load-proof: the body may have rotated
    /// arbitrarily far while the game was closed, the base stays exactly
    /// where it was built.
    struct SurfaceAnchorComponent
    {
        ecs::Entity body{};           // the gravity source this is built on
        WorldVec3 localPosition{0.0}; // body-fixed frame, meters
        /// Orientation in the body-fixed frame (identity = align with the
        /// body, the historical behavior for built structures).
        Quat localRotation{1.0f, 0.0f, 0.0f, 0.0f};
        /// Payload preserved for auto-released anchors (see below).
        f64 dynamicMass = 0.0;
        f64 dynamicBallisticFactor = 0.0;
        /// 1 = this anchor was created by the bubble system for a LANDED
        /// dynamic craft (rails would fling it: a ground state converts to
        /// a degenerate ellipse). Released back to dynamic when the bubble
        /// focus returns. Hand-built structures use 0 and stay anchored.
        u8 autoRelease = 0;
    };

    /// Optional exponential atmosphere around a gravity source.
    struct AtmosphereComponent
    {
        f64 surfaceDensity = 1.225; // kg/m^3 at altitude 0 (Earth-like)
        f64 scaleHeight = 8500.0;   // meters (density /e per scale height)
        f64 topAltitude = 1.4e5;    // no drag above this altitude
    };

    static_assert(std::is_trivially_copyable_v<DynamicBodyComponent>);
    static_assert(std::is_trivially_copyable_v<OnRailsComponent>);
    static_assert(std::is_trivially_copyable_v<GravitySourceComponent>);
    static_assert(std::is_trivially_copyable_v<AtmosphereComponent>);
    static_assert(std::is_trivially_copyable_v<SurfaceAnchorComponent>);
} // namespace sw::phys
