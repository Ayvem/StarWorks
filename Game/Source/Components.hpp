#pragma once

// ============================================================================
// Components.hpp — game-side ECS components.
//
// Spatial components (TransformComponent, PreviousTransformComponent) and
// physics components (DynamicBody, OnRails, GravitySource) are ENGINE types
// since Milestone 6 — aliased here so game code keeps short names.
//
// All components follow the engine rule: trivially copyable, no owning
// members. Meshes are referenced by index into the game's mesh table (an
// asset-ID precursor), never by pointer — save-file friendly by design.
// ============================================================================

#include <Engine.hpp>

namespace game
{
    // Engine spatial/physics components under their game-local names.
    using TransformComponent = sw::TransformComponent;
    using PreviousTransformComponent = sw::PreviousTransformComponent;

    /// Bounding sphere radius in mesh-local units (world radius =
    /// localRadius * uniformScale). Used for frustum culling.
    struct BoundsComponent
    {
        sw::f32 localRadius = 1.0f;
    };

    /// Constant-rate self rotation around a fixed axis. Only applied to
    /// entities that are really simulated (dynamic bodies) or to celestial
    /// bodies — on-rails debris keeps a frozen orientation, invisible at
    /// the distances where rails objects live.
    struct SpinComponent
    {
        sw::Vec3 axis{0.0f, 1.0f, 0.0f};
        sw::f32 radiansPerSecond = 1.0f;
    };

    /// Fixed-mesh renderable (index into the game mesh table).
    struct MeshComponent
    {
        static constexpr sw::u32 kOpaque = 0;
        static constexpr sw::u32 kTransparent = 1;
        /// Cloud deck: the blended pass AND the per-fragment weather path
        /// (Shaders/Clouds.glsl), whose coverage the ground samples for its
        /// shadows. Same size as the old flag — no save migration.
        static constexpr sw::u32 kCloudDeck = 2;
        sw::u32 meshIndex = 0;
        /// Rendered in the blended world pass (atmosphere/cloud shells).
        sw::u32 transparent = kOpaque;
    };

    /// Stability Assist System: torques the craft to hold an attitude.
    struct SasComponent
    {
        static constexpr sw::u32 kOff = 0;
        static constexpr sw::u32 kPrograde = 1;
        static constexpr sw::u32 kRetrograde = 2;
        sw::u32 mode = kOff;
    };

    /// A shell glued to a celestial body's CENTER (atmosphere, clouds) with
    /// its own spin — clouds drift relative to the ground below them.
    struct CloudLayerComponent
    {
        sw::ecs::Entity body{};
        sw::Vec3 spinAxis{0.0f, 1.0f, 0.0f};
        sw::f32 radiansPerSecond = 0.0f;
    };

    /// Celestial body renderable with distance-based level of detail:
    /// meshes[0] is the most detailed sphere, meshes[kLodLevels-1] the
    /// coarsest. The level is chosen from the body's ANGULAR size on screen,
    /// so a moon 384,000 km away costs a few dozen triangles while the
    /// planet you are orbiting gets full geometry. (The Planet module's
    /// quadtree surface will later replace level 0 near the ground.)
    struct CelestialLodComponent
    {
        static constexpr sw::u32 kLodLevels = 5;
        sw::u32 meshIndex[kLodLevels] = {0, 0, 0, 0, 0};
        /// SurfaceStyle id (-1 = flat color): close-orbit rendering swaps
        /// the vertex colors for PER-FRAGMENT procedural shading (M23).
        sw::i32 surfaceStyle = -1;
    };

    /// A controllable vessel. Thrust and attitude are applied by the
    /// ThrustSystem in the Physics lane; the amounts below are per-ship
    /// characteristics (modular construction will later aggregate them
    /// from parts, together with a full inertia tensor).
    struct ShipComponent
    {
        sw::f64 mainThrustNewtons = 4.0e5;   // ~400 kN main engine
        sw::f32 angularAccel = 0.5f;          // rad/s^2 from RCS
        sw::f32 maxAngularSpeed = 0.8f;       // rad/s
        sw::Vec3 angularVelocity{0.0f};       // body frame, integrated state
        /// Throttle limiter [0,1] applied to the main engine (Shift/Ctrl).
        sw::f32 throttle = 1.0f;
    };

    /// Pilot inputs for one physics tick, written by the game layer on the
    /// main thread between simulation advances.
    struct ShipControlsComponent
    {
        sw::f32 thrustAxis = 0.0f;   // -1 (retro) .. +1 (prograde)
        /// SIDESTEP, -1 (left) .. +1 (right). A walker strafes; a rocket has
        /// no such degree of freedom and leaves this at zero.
        sw::f32 strafeAxis = 0.0f;
        sw::Vec3 rotationInput{0.0f}; // pitch (x), yaw (y), roll (z), -1..1
        sw::u32 killRotation = 0;    // bool
        /// Throttle change rate [-1,1] (Shift = +, Ctrl = -).
        sw::f32 throttleDelta = 0.0f;
    };

    /// EVA capsule: a walker. When grounded it moves along the surface
    /// tangent; airborne it falls ballistically like everything else.
    struct CapsuleComponent
    {
        /// Which way the suit FACES, around the local up axis (0 = north,
        /// +pi/2 = east). It is driven by the CAMERA — you always walk where
        /// you are looking — rather than integrated from a turn key: a
        /// third-person walker whose body and view can disagree makes the
        /// player fight two controls to go one direction.
        sw::f32 headingRadians = 0.0f;
        sw::f32 walkSpeed = 4.0f;      // m/s on the ground
        sw::f32 turnSpeed = 1.8f;      // rad/s per unit of mouse look
    };

    /// Marks an entity with a star-map marker (see the M map view): an
    /// octahedron beacon whose scale keeps a constant on-screen size.
    struct MapMarkerComponent
    {
        sw::Vec4 color{1.0f};
    };

    static_assert(std::is_trivially_copyable_v<BoundsComponent>);
    static_assert(std::is_trivially_copyable_v<SpinComponent>);
    static_assert(std::is_trivially_copyable_v<MeshComponent>);
    static_assert(std::is_trivially_copyable_v<CelestialLodComponent>);
    static_assert(std::is_trivially_copyable_v<ShipComponent>);
    static_assert(std::is_trivially_copyable_v<ShipControlsComponent>);
    static_assert(std::is_trivially_copyable_v<CapsuleComponent>);
    static_assert(std::is_trivially_copyable_v<MapMarkerComponent>);
} // namespace game
