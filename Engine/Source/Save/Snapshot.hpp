#pragma once

// ============================================================================
// Save/Snapshot.hpp
// Versioned binary snapshot of an ECS World + Simulation clocks.
//
// Identity by NAME, never by runtime id: component columns are written
// under stable strings registered in a Schema, so registration order,
// compiler and build differences never corrupt a save. Loading restores
// entity indices AND generations exactly — every entity reference stored
// inside a component (machine links, anchor bodies...) survives untouched,
// which is why the engine banned pointers in components from day one.
//
// Strict-by-design v1: an unknown component name or a size/version mismatch
// aborts the load with a precise error. Migration shims come later; silent
// data reinterpretation never.
// ============================================================================

#include "Core/Types.hpp"
#include "ECS/Component.hpp"
#include "Serialization/Binary.hpp"

#include <string>
#include <vector>

namespace sw::ecs
{
    class World;
} // namespace sw::ecs

namespace sw::sim
{
    class Simulation;
} // namespace sw::sim

namespace sw::save
{
    /// Maps stable component NAMES to runtime component types.
    /// Register every component that may appear in a saved world.
    class Schema
    {
    public:
        template <typename T>
        void registerComponent(std::string_view name, u32 version = 1)
        {
            m_entries.push_back({std::string(name), ecs::componentTypeId<T>(),
                                 static_cast<u32>(sizeof(T)), version});
        }

        struct Entry
        {
            std::string name;
            ecs::ComponentTypeId typeId;
            u32 size;
            u32 version;
        };

        [[nodiscard]] const Entry* findByName(std::string_view name) const;
        [[nodiscard]] const Entry* findByTypeId(ecs::ComponentTypeId typeId) const;
        [[nodiscard]] const std::vector<Entry>& entries() const { return m_entries; }

    private:
        std::vector<Entry> m_entries;
    };

    /// Components present in the world that the schema cannot name, as
    /// "runtime id N (M bytes)" — empty when the world is saveable.
    ///
    /// WHY THIS IS SEPARATE FROM saveWorld. A component added to a live
    /// entity but never registered does not corrupt a save; it makes saving
    /// IMPOSSIBLE, because saveWorld throws on the first column it cannot
    /// name and writes no file at all. Discovering that when the player
    /// presses F5, hours in, is the worst possible moment — and the log line
    /// they get says only that the save failed.
    ///
    /// So the same question can be asked in advance, cheaply, of a world
    /// that has just been built. See StarWorksGame's startup check.
    [[nodiscard]] std::vector<std::string> unsaveableComponents(const ecs::World& world,
                                                               const Schema& schema);

    /// Serializes every entity/component of the world (columns memcpy'd).
    void saveWorld(const ecs::World& world, const Schema& schema, ser::BinaryWriter& writer);

    /// Restores a world saved with a compatible schema. The world is fully
    /// cleared first. Throws sw::Exception on any incompatibility.
    void loadWorld(ecs::World& world, const Schema& schema, ser::BinaryReader& reader);

    /// Simulation clocks and lane states (matched by lane name).
    void saveSimulation(const sim::Simulation& simulation, ser::BinaryWriter& writer);
    void loadSimulation(sim::Simulation& simulation, ser::BinaryReader& reader);
} // namespace sw::save
