#include "Resources/ResourceTypes.hpp"

#include "Core/Assert.hpp"

#include <array>

namespace sw::res
{
    namespace
    {
        // One unit == 1 kg of matter for solids/liquids; volumes derive from
        // real densities (ore is porous rock ~2.5 t/m^3, iron 7.87 t/m^3,
        // water 1 t/m^3, gases stored compressed at 200 bar).
        constexpr std::array<ResourceDef, kResourceCount> kDefinitions = {{
            {"IronOre", 1.0, 1.0 / 2500.0},
            {"CopperOre", 1.0, 1.0 / 2400.0},
            {"WaterIce", 1.0, 1.0 / 917.0},
            {"Iron", 1.0, 1.0 / 7870.0},
            {"Copper", 1.0, 1.0 / 8960.0},
            {"Water", 1.0, 1.0 / 1000.0},
            {"Hydrogen", 1.0, 1.0 / 17.0},
            {"Oxygen", 1.0, 1.0 / 278.0},
            {"Fuel", 1.0, 1.0 / 820.0},          // RP-1-like density
            // 1 kJ: effectively massless; volume tuned so a battery's
            // inventory volume IS its charge capacity (0.12 m^3 = 800 kJ).
            {"ElectricCharge", 1.0e-9, 1.5e-4},
            // 1 unit = 1 rocket. The mass is a nominal 12 t airframe; the
            // volume is what makes a vehicle belt a vehicle belt.
            {"Vehicle", 12000.0, 60.0},
        }};
    } // namespace

    const ResourceDef& definition(Resource resource)
    {
        const auto index = static_cast<usize>(resource);
        SW_ASSERT(index < kResourceCount, "Invalid resource id {}", index);
        return kDefinitions[index];
    }
} // namespace sw::res
