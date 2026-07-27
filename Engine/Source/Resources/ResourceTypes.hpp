#pragma once

// ============================================================================
// Resources/ResourceTypes.hpp
// The simulated resource catalogue. Every resource has REAL physical
// properties: mass and volume per unit — inventories are volume-limited,
// ships get heavier when loaded, conveyors move real matter. One "unit" is
// a design quantity chosen per resource (see the table in the .cpp).
//
// The registry is static for now; it becomes data-driven (JSON/YAML via
// Assets) once modding/balance iteration demands it — call sites already
// only use the lookup function.
// ============================================================================

#include "Core/Types.hpp"

#include <string_view>

namespace sw::res
{
    enum class Resource : u8
    {
        IronOre = 0,
        CopperOre,
        WaterIce,
        Iron,
        Copper,
        Water,
        Hydrogen,
        Oxygen,
        Fuel,           // dense rocket propellant (1 u = 1 kg)
        ElectricCharge, // 1 u = 1 kJ; near-massless, near-volumeless
        /// A FINISHED VEHICLE, crated. One unit is one rocket, and it is
        /// deliberately enormous: a belt carrying vehicles carries almost
        /// nothing else, which is what a rocket on a conveyor should feel
        /// like.
        ///
        /// Its mass and volume are NOMINAL — one number for every design,
        /// because a unit is a fixed design quantity and a rocket is not.
        /// The conservation that matters is on the METAL: an assembly hall
        /// consumes exactly the bill of materials, and that bill sums to the
        /// finished vessel's dry mass to the last gram (BlueprintTests).
        Vehicle,

        Count
    };

    inline constexpr usize kResourceCount = static_cast<usize>(Resource::Count);

    struct ResourceDef
    {
        std::string_view name;
        f64 massPerUnitKg = 1.0;
        f64 volumePerUnitM3 = 0.001;
    };

    /// Definition lookup; asserts on Resource::Count.
    [[nodiscard]] const ResourceDef& definition(Resource resource);
} // namespace sw::res
