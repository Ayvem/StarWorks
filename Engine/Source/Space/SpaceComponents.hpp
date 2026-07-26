#pragma once

// ============================================================================
// Space/SpaceComponents.hpp
// The hierarchical star system: Sun -> planets -> moons.
//
// A CELESTIAL BODY is a gravity source that belongs to the system tree.
// Its PARENT-RELATIVE Kepler orbit lives here (not in OnRailsComponent —
// celestials are moved by the CelestialMotionSystem at Physics rate, with
// proper previous-transform interpolation; generic rails objects are a
// different, cheaper tier). The root star has no parent and no orbit.
//
// Everything is trivially copyable (fixed-size name), so celestial bodies
// serialize through the ordinary snapshot path.
// ============================================================================

#include "ECS/Entity.hpp"
#include "Physics/Kepler.hpp"

#include <cstring>

namespace sw::space
{
    struct CelestialBodyComponent
    {
        static constexpr usize kNameCapacity = 16;

        ecs::Entity parent{};      // null for the root star
        u32 hasOrbit = 0;          // 0: static root; 1: orbit is valid
        phys::KeplerOrbit orbit{}; // PARENT-relative elements
        char name[kNameCapacity] = {};
    };
    static_assert(std::is_trivially_copyable_v<CelestialBodyComponent>);

    /// Builds a celestial body component with a bounded, zero-padded name.
    [[nodiscard]] inline CelestialBodyComponent makeCelestialBody(
        const char* name, ecs::Entity parent = {},
        const phys::KeplerOrbit* orbit = nullptr)
    {
        CelestialBodyComponent body{};
        body.parent = parent;
        if (orbit != nullptr)
        {
            body.hasOrbit = 1;
            body.orbit = *orbit;
        }
        std::strncpy(body.name, name, CelestialBodyComponent::kNameCapacity - 1);
        return body;
    }
} // namespace sw::space
