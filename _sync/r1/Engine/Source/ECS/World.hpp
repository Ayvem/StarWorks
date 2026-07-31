#pragma once

// ============================================================================
// ECS/World.hpp
// The World owns all entities and archetypes and is the single public API
// of the ECS. Adding/removing a component migrates the entity's row between
// archetypes (memcpy of shared columns); queries iterate the archetypes
// whose signature contains the requested component set.
//
// Threading contract: structural changes (create/destroy/add/remove) are
// NOT thread-safe and must happen on the owning thread; parallel systems
// only read/write component data of disjoint access sets, which the
// SystemScheduler enforces via its stage construction.
// ============================================================================

#include "Core/Assert.hpp"
#include "ECS/Archetype.hpp"
#include "ECS/Component.hpp"
#include "ECS/Entity.hpp"

#include <memory>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace sw::ecs
{
    class World
    {
    public:
        World();

        World(const World&) = delete;
        World& operator=(const World&) = delete;

        // --- entity lifetime -------------------------------------------------
        [[nodiscard]] Entity createEntity();
        void destroyEntity(Entity entity);
        [[nodiscard]] bool isAlive(Entity entity) const;
        [[nodiscard]] u32 aliveCount() const { return m_aliveCount; }

        // --- components --------------------------------------------------------
        template <typename T>
        T& addComponent(Entity entity, const T& value)
        {
            SW_ASSERT(isAlive(entity), "addComponent on dead entity ({})", entity.index);
            EntityRecord& record = m_records[entity.index];
            const Signature bit = componentBit<T>();
            SW_ASSERT((record.archetype->signature() & bit) == 0,
                      "Entity {} already has this component", entity.index);

            Archetype* target =
                getOrCreateArchetype(record.archetype->signature() | bit);
            moveEntityToArchetype(entity, record, *target);

            T* slot = target->componentAt<T>(record.row);
            *slot = value;
            return *slot;
        }

        template <typename T>
        void removeComponent(Entity entity)
        {
            SW_ASSERT(isAlive(entity), "removeComponent on dead entity ({})", entity.index);
            EntityRecord& record = m_records[entity.index];
            const Signature bit = componentBit<T>();
            SW_ASSERT((record.archetype->signature() & bit) != 0,
                      "Entity {} does not have this component", entity.index);

            Archetype* target =
                getOrCreateArchetype(record.archetype->signature() & ~bit);
            moveEntityToArchetype(entity, record, *target);
        }

        template <typename T>
        [[nodiscard]] bool hasComponent(Entity entity) const
        {
            if (!isAlive(entity))
            {
                return false;
            }
            return (m_records[entity.index].archetype->signature() & componentBit<T>()) != 0;
        }

        template <typename T>
        [[nodiscard]] T* tryGetComponent(Entity entity)
        {
            if (!isAlive(entity))
            {
                return nullptr;
            }
            const EntityRecord& record = m_records[entity.index];
            return record.archetype->componentAt<T>(record.row);
        }

        template <typename T>
        [[nodiscard]] T& getComponent(Entity entity)
        {
            T* component = tryGetComponent<T>(entity);
            SW_ASSERT(component != nullptr, "getComponent: entity {} lacks the component",
                      entity.index);
            return *component;
        }

        // --- type-erased component access ------------------------------------
        // For code that learns its component types at RUNTIME rather than at
        // compile time — today that means the network mirror, which is
        // handed a list of component NAMES by the host and has no template
        // parameter to work with. Gameplay code should keep using the typed
        // overloads above; these skip every compile-time check the ECS
        // otherwise gives you.

        [[nodiscard]] bool hasComponentRaw(Entity entity, ComponentTypeId type) const;
        /// Adds the component if it is absent (zero-filled) and returns its
        /// storage. Never null for a live entity and a registered type.
        std::byte* addComponentRaw(Entity entity, ComponentTypeId type);
        void removeComponentRaw(Entity entity, ComponentTypeId type);
        /// Null if the entity is dead or does not carry the component.
        [[nodiscard]] std::byte* tryGetComponentRaw(Entity entity, ComponentTypeId type);

        // --- queries -------------------------------------------------------------
        /// Calls fn(Entity, Ts&...) for every entity having all of Ts.
        /// Iteration order: archetype creation order, then row order —
        /// deterministic for a given sequence of structural operations.
        template <typename... Ts, typename F>
        void forEach(F&& fn)
        {
            const Signature required = makeSignature<Ts...>();
            for (Archetype* archetype : m_archetypeList)
            {
                if ((archetype->signature() & required) != required ||
                    archetype->entityCount() == 0)
                {
                    continue;
                }

                // Resolve column bases once per archetype, then walk rows.
                std::tuple<Ts*...> columns{
                    reinterpret_cast<Ts*>(archetype->columnData(componentTypeId<Ts>()))...};
                const u32 count = archetype->entityCount();
                for (u32 row = 0; row < count; ++row)
                {
                    fn(archetype->entityAt(row), std::get<Ts*>(columns)[row]...);
                }
            }
        }

        /// Number of entities matching a component set (mainly for tests/UI).
        template <typename... Ts>
        [[nodiscard]] u32 count()
        {
            const Signature required = makeSignature<Ts...>();
            u32 total = 0;
            for (const Archetype* archetype : m_archetypeList)
            {
                if ((archetype->signature() & required) == required)
                {
                    total += archetype->entityCount();
                }
            }
            return total;
        }

        // --- serialization support -------------------------------------------
        // These exist for the Save module (Save/Snapshot). They preserve
        // entity indices AND generations so entity references stored inside
        // components survive a save/load round trip. Not for gameplay use.

        [[nodiscard]] const std::vector<Archetype*>& archetypeList() const
        {
            return m_archetypeList;
        }
        [[nodiscard]] u32 recordCount() const { return static_cast<u32>(m_records.size()); }
        [[nodiscard]] bool isSlotAlive(u32 index) const { return m_records[index].alive; }
        [[nodiscard]] u32 slotGeneration(u32 index) const
        {
            return m_records[index].generation;
        }

        /// Empties the world completely (restore starts from a blank slate).
        void clearForRestore();
        /// getOrCreate an archetype for a signature built from mapped ids.
        [[nodiscard]] Archetype* restoreArchetype(Signature signature);
        /// Appends entities (with their EXACT index/generation) to an
        /// archetype; returns the first row used. Column data is memcpy'd
        /// afterwards by the caller through Archetype::columnDataAt.
        u32 restoreEntitiesIntoArchetype(Archetype& archetype, std::span<const Entity> entities);
        /// Rebuilds the free list and alive count; call once after restoring.
        void finalizeRestore(std::span<const u32> allGenerations);

        /// Creates — or revives — the entity carrying EXACTLY this index and
        /// generation, growing the slot table as needed, and returns it. A
        /// slot already alive under the same generation is left untouched;
        /// one alive under a DIFFERENT generation is destroyed first,
        /// because the host recycled that index and the local occupant is
        /// somebody else's row.
        ///
        /// This is the primitive a MIRROR world needs. A network client's
        /// entity indices must match the host's exactly, for the reason the
        /// save file already restores them exactly: entity handles live
        /// INSIDE components — a conveyor names its body, a cable names its
        /// poles, a cloud deck names its planet — and no component tells
        /// anyone which of its fields are handles. Renumber the entities and
        /// every one of those fields quietly points at the wrong thing.
        ///
        /// Cost: O(free slots) for the free-list removal, paid once per
        /// entity that appears, never per tick.
        ///
        /// Throws rather than growing past kMaxMirrorIndex; see there.
        Entity mirrorEntity(Entity entity);

        /// The largest entity index a MIRROR will grow its slot table to.
        ///
        /// THE FAILURE THIS PREVENTS. The index comes off the network. It is
        /// four bytes chosen by whoever sent the snapshot, and the slot table
        /// is grown to hold it: an eight-byte spawn record naming index
        /// 4,294,967,295 asked for a hundred gigabytes. Allocation that size
        /// throws std::bad_alloc, which is NOT an sw::Exception, so the
        /// session's `catch (const Exception&)` did not catch it and one
        /// datagram killed the client.
        ///
        /// WHAT THE CEILING ACTUALLY COSTS, measured on this build by
        /// instrumenting operator new while mirrorEntity grows the table.
        /// The refusal below is on `index > kMaxMirrorIndex`, so the largest
        /// table a snapshot can ask for is kMaxMirrorIndex + 1 = 1,048,577
        /// slots, which is ONE allocation of 25,165,848 bytes — exactly
        /// 24.00 bytes per slot.
        ///
        /// The honest worst case is twice that, and it is worth saying so:
        /// the table is a std::vector and grows geometrically, so a spawn
        /// record walking indices upward reaches capacity 2,097,152 —
        /// measured, a final allocation of 50,331,648 bytes, with the
        /// 25,165,824-byte buffer it is copied from still alive alongside it.
        /// Fifty megabytes is still a budget a hostile spawn record spends
        /// once, and 1,048,576 is more entities than this game has ever had
        /// in one world.
        static constexpr u32 kMaxMirrorIndex = 1u << 20;

    private:
        struct EntityRecord
        {
            u32 generation = 0;
            Archetype* archetype = nullptr;
            u32 row = 0;
            bool alive = false;
        };

        [[nodiscard]] Archetype* getOrCreateArchetype(Signature signature);
        void moveEntityToArchetype(Entity entity, EntityRecord& record, Archetype& target);

        std::vector<EntityRecord> m_records;
        std::vector<u32> m_freeIndices;
        u32 m_aliveCount = 0;

        std::unordered_map<Signature, std::unique_ptr<Archetype>> m_archetypes;
        std::vector<Archetype*> m_archetypeList; // creation order (deterministic iteration)
        Archetype* m_emptyArchetype = nullptr;
    };
} // namespace sw::ecs
