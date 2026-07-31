#include "ECS/World.hpp"

#include "Core/Error.hpp"

namespace sw::ecs
{
    World::World()
    {
        m_emptyArchetype = getOrCreateArchetype(Signature{0});
    }

    Entity World::createEntity()
    {
        u32 index = 0;
        if (!m_freeIndices.empty())
        {
            index = m_freeIndices.back();
            m_freeIndices.pop_back();
        }
        else
        {
            index = static_cast<u32>(m_records.size());
            m_records.emplace_back();
        }

        EntityRecord& record = m_records[index];
        record.alive = true;
        record.archetype = m_emptyArchetype;

        const Entity entity{index, record.generation};
        record.row = m_emptyArchetype->addRow(entity);
        ++m_aliveCount;
        return entity;
    }

    void World::destroyEntity(Entity entity)
    {
        SW_ASSERT(isAlive(entity), "destroyEntity on dead/stale entity ({})", entity.index);
        EntityRecord& record = m_records[entity.index];

        const Entity moved = record.archetype->removeRow(record.row);
        if (!moved.isNull())
        {
            m_records[moved.index].row = record.row;
        }

        record.alive = false;
        record.archetype = nullptr;
        ++record.generation; // stale handles become detectable
        m_freeIndices.push_back(entity.index);
        --m_aliveCount;
    }

    bool World::isAlive(Entity entity) const
    {
        return entity.index < m_records.size() && m_records[entity.index].alive &&
               m_records[entity.index].generation == entity.generation;
    }

    void World::clearForRestore()
    {
        m_records.clear();
        m_freeIndices.clear();
        m_aliveCount = 0;
        m_archetypes.clear();
        m_archetypeList.clear();
        m_emptyArchetype = getOrCreateArchetype(Signature{0});
    }

    Archetype* World::restoreArchetype(Signature signature)
    {
        return getOrCreateArchetype(signature);
    }

    u32 World::restoreEntitiesIntoArchetype(Archetype& archetype,
                                            std::span<const Entity> entities)
    {
        const u32 firstRow = archetype.entityCount();
        for (const Entity entity : entities)
        {
            if (entity.index >= m_records.size())
            {
                m_records.resize(entity.index + 1);
            }
            EntityRecord& record = m_records[entity.index];
            SW_ASSERT(!record.alive, "Restore collision on entity index {}", entity.index);
            record.generation = entity.generation;
            record.archetype = &archetype;
            record.row = archetype.addRow(entity);
            record.alive = true;
        }
        return firstRow;
    }

    void World::finalizeRestore(std::span<const u32> allGenerations)
    {
        // Dead slots keep their saved generation so recycled indices keep
        // invalidating stale handles exactly as before the save.
        if (allGenerations.size() > m_records.size())
        {
            m_records.resize(allGenerations.size());
        }
        m_aliveCount = 0;
        m_freeIndices.clear();
        for (u32 index = 0; index < m_records.size(); ++index)
        {
            if (index < allGenerations.size() && !m_records[index].alive)
            {
                m_records[index].generation = allGenerations[index];
            }
            if (m_records[index].alive)
            {
                ++m_aliveCount;
            }
        }
        // Canonical free-list order: descending, so createEntity() hands out
        // the lowest free index first — deterministic across a round trip.
        for (u32 index = m_records.size(); index-- > 0;)
        {
            if (!m_records[index].alive)
            {
                m_freeIndices.push_back(index);
            }
        }
    }

    bool World::hasComponentRaw(Entity entity, ComponentTypeId type) const
    {
        if (!isAlive(entity))
        {
            return false;
        }
        return (m_records[entity.index].archetype->signature() & (Signature{1} << type)) != 0;
    }

    std::byte* World::addComponentRaw(Entity entity, ComponentTypeId type)
    {
        SW_ASSERT(isAlive(entity), "addComponentRaw on dead entity ({})", entity.index);
        EntityRecord& record = m_records[entity.index];
        const Signature bit = Signature{1} << type;
        if ((record.archetype->signature() & bit) == 0)
        {
            Archetype* target = getOrCreateArchetype(record.archetype->signature() | bit);
            moveEntityToArchetype(entity, record, *target);
        }
        std::byte* column = record.archetype->columnData(type);
        SW_ASSERT(column != nullptr, "addComponentRaw: column {} missing after migration",
                  type);
        return column + static_cast<usize>(record.row) * componentInfo(type).size;
    }

    void World::removeComponentRaw(Entity entity, ComponentTypeId type)
    {
        SW_ASSERT(isAlive(entity), "removeComponentRaw on dead entity ({})", entity.index);
        EntityRecord& record = m_records[entity.index];
        const Signature bit = Signature{1} << type;
        if ((record.archetype->signature() & bit) == 0)
        {
            return;
        }
        Archetype* target = getOrCreateArchetype(record.archetype->signature() & ~bit);
        moveEntityToArchetype(entity, record, *target);
    }

    std::byte* World::tryGetComponentRaw(Entity entity, ComponentTypeId type)
    {
        if (!isAlive(entity))
        {
            return nullptr;
        }
        const EntityRecord& record = m_records[entity.index];
        std::byte* column = record.archetype->columnData(type);
        if (column == nullptr)
        {
            return nullptr;
        }
        return column + static_cast<usize>(record.row) * componentInfo(type).size;
    }

    Entity World::mirrorEntity(Entity entity)
    {
        SW_ASSERT(!entity.isNull(), "mirrorEntity on a null handle");
        if (entity.index > kMaxMirrorIndex)
        {
            // Not an assert: the caller is the network decoder and the value
            // came off a wire anyone can write to, so this is a hostile input
            // to be refused, not a bug in our own code to be trapped. Throwing
            // sw::Exception puts it in the same bucket as every other
            // malformed-snapshot rejection, which the session already drops
            // the datagram for.
            SW_THROW("mirrorEntity asked for index {}, past the {} a mirror will grow to",
                     entity.index, kMaxMirrorIndex);
        }
        if (entity.index >= m_records.size())
        {
            m_records.resize(static_cast<usize>(entity.index) + 1);
        }

        if (m_records[entity.index].alive)
        {
            if (m_records[entity.index].generation == entity.generation)
            {
                return entity; // already mirrored
            }
            // Index recycled on the host: the local occupant is a different
            // entity. Destroying it also frees its row, which is the point —
            // reusing that row would reinterpret one entity's bytes as
            // another's.
            destroyEntity(Entity{entity.index, m_records[entity.index].generation});
        }

        // The slot is dead, so it is somewhere in the free list. Take it out
        // (swap-erase; order of the free list is a policy, not a contract).
        for (usize i = 0; i < m_freeIndices.size(); ++i)
        {
            if (m_freeIndices[i] == entity.index)
            {
                m_freeIndices[i] = m_freeIndices.back();
                m_freeIndices.pop_back();
                break;
            }
        }

        EntityRecord& record = m_records[entity.index];
        record.generation = entity.generation;
        record.alive = true;
        record.archetype = m_emptyArchetype;
        record.row = m_emptyArchetype->addRow(entity);
        ++m_aliveCount;
        return entity;
    }

    Archetype* World::getOrCreateArchetype(Signature signature)
    {
        if (const auto it = m_archetypes.find(signature); it != m_archetypes.end())
        {
            return it->second.get();
        }

        // Collect the ComponentInfo of every bit set in the signature.
        std::vector<ComponentInfo> infos;
        for (u32 id = 0; id < kMaxComponentTypes; ++id)
        {
            if ((signature & (Signature{1} << id)) != 0)
            {
                infos.push_back(componentInfo(id));
            }
        }

        auto archetype = std::make_unique<Archetype>(signature, infos);
        Archetype* pointer = archetype.get();
        m_archetypes.emplace(signature, std::move(archetype));
        m_archetypeList.push_back(pointer);
        return pointer;
    }

    void World::moveEntityToArchetype(Entity entity, EntityRecord& record, Archetype& target)
    {
        Archetype* source = record.archetype;
        const u32 sourceRow = record.row;

        const u32 targetRow = target.addRow(entity);
        target.copyCommonComponents(*source, sourceRow, targetRow);

        const Entity moved = source->removeRow(sourceRow);
        if (!moved.isNull())
        {
            m_records[moved.index].row = sourceRow;
        }

        record.archetype = &target;
        record.row = targetRow;
    }
} // namespace sw::ecs
