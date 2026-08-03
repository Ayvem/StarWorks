#pragma once

// ============================================================================
// Space/CelestialIndex.hpp
// A flat, topologically sorted snapshot of the celestial hierarchy, rebuilt
// from the ECS in microseconds (a star system has a handful of bodies).
//
// This is THE analytic query surface for the hierarchy:
//   - world position/velocity of any body at ANY time t (recursive Kepler
//     evaluation up the parent chain — past, present or future);
//   - sphere-of-influence primary for a world position at time t (the KSP
//     rule: the deepest body whose SOI contains the point);
//   - children lists, for encounter scanning during trajectory prediction.
//
// Rebuilding instead of incrementally updating keeps it trivially correct
// across entity creation/destruction and save/load.
// ============================================================================

#include "ECS/Entity.hpp"
#include "Space/SpaceComponents.hpp"

#include <vector>

namespace sw::ecs
{
    class World;
} // namespace sw::ecs

namespace sw::space
{
    class CelestialIndex
    {
    public:
        struct Body
        {
            ecs::Entity entity{};
            i32 parentIndex = -1; // index into bodies(); -1 = root
            f64 mu = 0.0;
            f64 bodyRadius = 0.0;
            f64 soiRadius = 0.0;
            u32 hasOrbit = 0;
            phys::KeplerOrbit orbit{};       // parent-relative
            WorldVec3 staticPosition{0.0};   // world position when !hasOrbit
            char name[CelestialBodyComponent::kNameCapacity] = {};
        };

        /// Snapshots every entity with CelestialBody + Transform +
        /// GravitySource. Bodies whose parent is missing/dead are treated
        /// as static roots at their current transform.
        void rebuild(ecs::World& world);

        [[nodiscard]] usize size() const { return m_bodies.size(); }
        [[nodiscard]] const Body& body(usize index) const { return m_bodies[index]; }
        [[nodiscard]] const std::vector<Body>& bodies() const { return m_bodies; }
        [[nodiscard]] const std::vector<i32>& childrenOf(usize index) const
        {
            return m_children[index];
        }

        /// -1 if the entity is not an indexed celestial body.
        [[nodiscard]] i32 indexOf(ecs::Entity entity) const;

        /// WORLD-frame state of a body at time t (analytic, any t).
        void stateAt(i32 index, f64 timeSeconds, WorldVec3& outPosition,
                     WorldVec3* outVelocity = nullptr) const;
        /// The same, at full clock precision: exact whole seconds plus a
        /// fraction. The per-tick systems that POSITION the world use this;
        /// the map, the HUD and the trajectory predictor use the plain one,
        /// where a millimetre is not a picture anybody can see.
        void stateAtSplit(i32 index, f64 wholeSeconds, f64 fraction,
                          WorldVec3& outPosition,
                          WorldVec3* outVelocity = nullptr) const;

        [[nodiscard]] WorldVec3 positionAt(i32 index, f64 timeSeconds) const;

        /// The body whose sphere of influence rules `worldPosition` at time
        /// t: the CONTAINING body with the smallest SOI (nested SOIs —
        /// moon inside planet inside star — make that the deepest one).
        /// -1 only when the index is empty.
        [[nodiscard]] i32 soiPrimaryAt(const WorldVec3& worldPosition,
                                       f64 timeSeconds) const;

    private:
        std::vector<Body> m_bodies; // parentIndex < own index (topo order)
        std::vector<std::vector<i32>> m_children;
    };
} // namespace sw::space
