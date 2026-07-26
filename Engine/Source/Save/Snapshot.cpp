#include "Save/Snapshot.hpp"

#include "Core/Log.hpp"
#include "ECS/World.hpp"
#include "Simulation/Simulation.hpp"

#include <unordered_map>

namespace sw::save
{
    namespace
    {
        constexpr u32 kWorldMagic = 0x53575342; // "SWSB"
        constexpr u32 kWorldFormatVersion = 1;
        constexpr u32 kSimMagic = 0x53575349; // "SWSI"
    } // namespace

    const Schema::Entry* Schema::findByName(std::string_view name) const
    {
        for (const Entry& entry : m_entries)
        {
            if (entry.name == name)
            {
                return &entry;
            }
        }
        return nullptr;
    }

    const Schema::Entry* Schema::findByTypeId(ecs::ComponentTypeId typeId) const
    {
        for (const Entry& entry : m_entries)
        {
            if (entry.typeId == typeId)
            {
                return &entry;
            }
        }
        return nullptr;
    }

    void saveWorld(const ecs::World& world, const Schema& schema, ser::BinaryWriter& writer)
    {
        writer.write(kWorldMagic);
        writer.write(kWorldFormatVersion);

        // ---- component table (save-local ids = indices in this table) -------
        writer.write(static_cast<u32>(schema.entries().size()));
        for (const Schema::Entry& entry : schema.entries())
        {
            writer.writeString(entry.name);
            writer.write(entry.version);
            writer.write(entry.size);
        }

        // ---- entity slot table (generations incl. dead slots) ----------------
        const u32 recordCount = world.recordCount();
        writer.write(recordCount);
        for (u32 index = 0; index < recordCount; ++index)
        {
            writer.write(world.slotGeneration(index));
            writer.write(static_cast<u8>(world.isSlotAlive(index) ? 1 : 0));
        }

        // ---- archetype chunks ---------------------------------------------------
        // Count non-empty archetypes first.
        u32 chunkCount = 0;
        for (const ecs::Archetype* archetype : world.archetypeList())
        {
            if (archetype->entityCount() > 0)
            {
                ++chunkCount;
            }
        }
        writer.write(chunkCount);

        for (const ecs::Archetype* archetype : world.archetypeList())
        {
            const u32 entityCount = archetype->entityCount();
            if (entityCount == 0)
            {
                continue;
            }

            // Column list as save-local ids.
            writer.write(archetype->columnCount());
            for (u32 column = 0; column < archetype->columnCount(); ++column)
            {
                const ecs::ComponentInfo& info = archetype->columnInfoAt(column);
                const Schema::Entry* entry = schema.findByTypeId(info.id);
                if (entry == nullptr)
                {
                    SW_THROW("Save schema is missing a component (runtime id {}, size {}) — "
                             "every component present in the world must be registered",
                             info.id, info.size);
                }
                const auto localId = static_cast<u32>(entry - schema.entries().data());
                writer.write(localId);
            }

            writer.write(entityCount);
            for (const ecs::Entity entity : archetype->entities())
            {
                writer.write(entity.index);
                writer.write(entity.generation);
            }
            for (u32 column = 0; column < archetype->columnCount(); ++column)
            {
                const ecs::ComponentInfo& info = archetype->columnInfoAt(column);
                writer.writeBytes(archetype->columnDataAt(column),
                                  static_cast<usize>(entityCount) * info.size);
            }
        }
    }

    void loadWorld(ecs::World& world, const Schema& schema, ser::BinaryReader& reader)
    {
        if (reader.read<u32>() != kWorldMagic)
        {
            SW_THROW("Not a StarWorks world snapshot (bad magic)");
        }
        const u32 formatVersion = reader.read<u32>();
        if (formatVersion != kWorldFormatVersion)
        {
            SW_THROW("Unsupported world snapshot format {} (expected {})", formatVersion,
                     kWorldFormatVersion);
        }

        // ---- component table: map save-local ids to current entries ----------
        const u32 componentCount = reader.read<u32>();
        std::vector<const Schema::Entry*> localToEntry(componentCount, nullptr);
        for (u32 i = 0; i < componentCount; ++i)
        {
            const std::string name = reader.readString();
            const u32 version = reader.read<u32>();
            const u32 size = reader.read<u32>();

            const Schema::Entry* entry = schema.findByName(name);
            if (entry == nullptr)
            {
                SW_THROW("Save contains unknown component '{}' — cannot load", name);
            }
            if (entry->version != version)
            {
                SW_THROW("Component '{}' version mismatch (save {}, engine {})", name,
                         version, entry->version);
            }
            if (entry->size != size)
            {
                SW_THROW("Component '{}' size mismatch (save {}, engine {}) — schema "
                         "version must be bumped on layout changes",
                         name, size, entry->size);
            }
            localToEntry[i] = entry;
        }

        // ---- entity slot table ---------------------------------------------------
        const u32 recordCount = reader.read<u32>();
        std::vector<u32> generations(recordCount);
        for (u32 index = 0; index < recordCount; ++index)
        {
            generations[index] = reader.read<u32>();
            (void)reader.read<u8>(); // alive flag re-derived from chunks
        }

        // ---- archetype chunks -------------------------------------------------------
        world.clearForRestore();
        const u32 chunkCount = reader.read<u32>();
        std::vector<ecs::Entity> entities;
        std::vector<const Schema::Entry*> chunkColumns;

        for (u32 chunk = 0; chunk < chunkCount; ++chunk)
        {
            const u32 columnCount = reader.read<u32>();
            chunkColumns.clear();
            ecs::Signature signature = 0;
            for (u32 column = 0; column < columnCount; ++column)
            {
                const u32 localId = reader.read<u32>();
                if (localId >= localToEntry.size())
                {
                    SW_THROW("Corrupted snapshot: component id {} out of range", localId);
                }
                chunkColumns.push_back(localToEntry[localId]);
                signature |= ecs::Signature{1} << localToEntry[localId]->typeId;
            }

            const u32 entityCount = reader.read<u32>();
            entities.resize(entityCount);
            for (u32 i = 0; i < entityCount; ++i)
            {
                entities[i].index = reader.read<u32>();
                entities[i].generation = reader.read<u32>();
            }

            ecs::Archetype* archetype = world.restoreArchetype(signature);
            const u32 firstRow =
                world.restoreEntitiesIntoArchetype(*archetype, entities);

            // Columns arrive in the SAVED order; find each in the (sorted)
            // restored archetype and memcpy at the right offset.
            for (u32 column = 0; column < columnCount; ++column)
            {
                const Schema::Entry* entry = chunkColumns[column];
                std::byte* destination = archetype->columnData(entry->typeId);
                SW_ASSERT(destination != nullptr, "Restored archetype misses column '{}'",
                          entry->name);
                reader.readBytes(destination + static_cast<usize>(firstRow) * entry->size,
                                 static_cast<usize>(entityCount) * entry->size);
            }
        }

        world.finalizeRestore(generations);
    }

    void saveSimulation(const sim::Simulation& simulation, ser::BinaryWriter& writer)
    {
        writer.write(kSimMagic);
        writer.write(simulation.simulatedSeconds());
        writer.write(simulation.timeScale());
        writer.write(static_cast<u8>(simulation.isPaused() ? 1 : 0));

        writer.write(static_cast<u32>(simulation.laneCount()));
        for (usize i = 0; i < simulation.laneCount(); ++i)
        {
            const sim::SimulationLane& lane =
                const_cast<sim::Simulation&>(simulation).lane(i);
            writer.writeString(lane.name());
            writer.write(lane.tickCount());
            writer.write(lane.accumulatorSeconds());
        }
    }

    void loadSimulation(sim::Simulation& simulation, ser::BinaryReader& reader)
    {
        if (reader.read<u32>() != kSimMagic)
        {
            SW_THROW("Corrupted save: bad simulation section");
        }
        simulation.setSimulatedSeconds(reader.read<f64>());
        simulation.setTimeScale(reader.read<f32>());
        simulation.setPaused(reader.read<u8>() != 0);

        const u32 laneCount = reader.read<u32>();
        for (u32 i = 0; i < laneCount; ++i)
        {
            const std::string name = reader.readString();
            const u64 tickCount = reader.read<u64>();
            const f64 accumulator = reader.read<f64>();

            sim::SimulationLane* lane = simulation.findLane(name);
            if (lane == nullptr)
            {
                SW_THROW("Save references unknown simulation lane '{}'", name);
            }
            lane->restoreState(tickCount, accumulator);
        }
    }
} // namespace sw::save
