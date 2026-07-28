#pragma once

// ============================================================================
// Gameplay/VesselAerodynamics.hpp
// THE OTHER HALF: what the game does with the tables the forge produced.
//
// Every physics tick, for every part-built vessel inside an atmosphere:
//
//   1. work out the air — which body, how high, how dense, how fast the
//      wind is going and therefore how fast the vessel is going THROUGH it;
//   2. turn that into one number, the dynamic pressure q, corrected for the
//      transonic rise;
//   3. for each part, ask its table what it does in this flow direction, in
//      its own frame, and how much of it the air can see past the parts in
//      front of it;
//   4. rotate both answers into the vessel frame, shift the moment to the
//      vessel's centre of mass, and add them up;
//   5. hand the sum to the rigid body: force over mass is an acceleration,
//      moment over inertia is an angular one.
//
// Step 5 is where the whole design pays off. Nothing in this file knows
// what a fin is, or a nose cone, or which way round a rocket ought to fly.
// A vehicle weathercocks because the moments of its own parts add up that
// way, and a vehicle built badly tumbles for exactly the same reason.
// ============================================================================

#include "ECS/System.hpp"
#include "Gameplay/Parts.hpp"
#include "Physics/Aerodynamics.hpp"

#include <unordered_map>

namespace sw::aero
{
    /// The process-wide table registry, filled once at start-up — the same
    /// shape as the part catalogue, and for the same reason: a table is
    /// static data about a part TYPE, and looking it up must not cost an
    /// allocation on a physics tick.
    void setTables(std::vector<AeroTable> tables);
    [[nodiscard]] const AeroTable* findTable(u32 partId);
    [[nodiscard]] usize tableCount();

    /// Loads every `*.aero.json` in a directory into the registry.
    /// Returns how many were read; zero is not fatal — parts without a
    /// table fall back to the old isotropic drag.
    usize loadTables(const std::filesystem::path& directory);

    /// Physics lane, after VesselAssemblySystem (which weighs the vessel and
    /// finds its centre of mass) and before SurfaceInteractionSystem (which
    /// applies the OLD scalar drag to everything this system did not touch).
    class VesselAerodynamicsSystem final : public ecs::System
    {
    public:
        [[nodiscard]] std::string_view name() const override
        {
            return "VesselAerodynamicsSystem";
        }

        [[nodiscard]] ecs::SystemAccess access() const override
        {
            return ecs::SystemAccess{}
                .write<AeroStateComponent>()
                .write<phys::DynamicBodyComponent>()
                .read<TransformComponent>()
                .read<parts::PartComponent>()
                .read<parts::VesselComponent>()
                .read<phys::GravitySourceComponent>()
                .read<phys::AtmosphereComponent>();
        }

        /// The simulation time the wind is evaluated at. Wind is a closed
        /// form in time, so this is the only state it needs — and passing it
        /// in rather than reading a clock is what keeps two runs identical.
        void setTimeSeconds(f64 seconds) { m_timeSeconds = seconds; }

        void update(ecs::World& world, f32 deltaSeconds) override;

        /// How many vessels felt air last tick, for the HUD's system panel.
        [[nodiscard]] u32 lastVesselCount() const { return m_vesselCount; }

    private:
        struct AirBody
        {
            WorldVec3 centre{0.0};
            WorldVec3 velocity{0.0};
            WorldVec3 angularVelocity{0.0};
            f64 radius = 0.0;
            phys::AtmosphereComponent atmosphere{};
        };

        struct PartEntry
        {
            const AeroTable* table = nullptr;
            Vec3 localPosition{0.0f};
            Quat localRotation{1.0f, 0.0f, 0.0f, 0.0f};
        };

        f64 m_timeSeconds = 0.0;
        u32 m_vesselCount = 0;
        std::vector<AirBody> m_bodies;                 // scratch, per tick
        std::vector<OccluderBox> m_boxes;              // scratch, per vessel
        std::vector<PartEntry> m_parts;                // scratch, per vessel
        std::unordered_map<u32, std::vector<parts::HitBox>> m_hullCache;
    };
} // namespace sw::aero
