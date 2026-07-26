#pragma once

// ============================================================================
// ECS/Archetype.hpp
// An archetype stores every entity that has exactly the same component set,
// as a structure-of-arrays: one contiguous byte column per component type.
// Iteration over an archetype is therefore a linear walk of packed arrays —
// the cache-friendly core of the whole ECS.
//
// Row removal is swap-remove (O(1)): the last row moves into the hole and
// the World patches the moved entity's bookkeeping.
// ============================================================================

#include "Core/Types.hpp"
#include "ECS/Component.hpp"
#include "ECS/Entity.hpp"

#include <span>
#include <vector>

namespace sw::ecs
{
    class Archetype
    {
    public:
        Archetype(Signature signature, std::span<const ComponentInfo> components);

        Archetype(const Archetype&) = delete;
        Archetype& operator=(const Archetype&) = delete;

        [[nodiscard]] Signature signature() const { return m_signature; }
        [[nodiscard]] u32 entityCount() const { return static_cast<u32>(m_entities.size()); }
        [[nodiscard]] Entity entityAt(u32 row) const { return m_entities[row]; }
        [[nodiscard]] std::span<const Entity> entities() const { return m_entities; }

        /// Appends a zero-initialized row for the entity; returns its index.
        u32 addRow(Entity entity);

        /// Swap-removes a row. Returns the entity that was moved into `row`
        /// (so the caller can fix its record), or Entity::null() if the
        /// removed row was the last one.
        Entity removeRow(u32 row);

        /// Base pointer of a component column (nullptr if the type is absent).
        [[nodiscard]] std::byte* columnData(ComponentTypeId type);
        [[nodiscard]] const std::byte* columnData(ComponentTypeId type) const;

        /// Typed element access; the type must be part of the signature.
        template <typename T>
        [[nodiscard]] T* componentAt(u32 row)
        {
            std::byte* data = columnData(componentTypeId<T>());
            return (data != nullptr)
                       ? reinterpret_cast<T*>(data + static_cast<usize>(row) * sizeof(T))
                       : nullptr;
        }

        /// Copies every component type present in BOTH archetypes from
        /// (source, sourceRow) into (*this, destinationRow).
        void copyCommonComponents(const Archetype& source, u32 sourceRow, u32 destinationRow);

        // --- column introspection (serialization support) -------------------
        [[nodiscard]] u32 columnCount() const { return static_cast<u32>(m_columns.size()); }
        [[nodiscard]] const ComponentInfo& columnInfoAt(u32 index) const
        {
            return m_columns[index].info;
        }
        [[nodiscard]] const std::byte* columnDataAt(u32 index) const
        {
            return m_columns[index].data.data();
        }
        [[nodiscard]] std::byte* columnDataAt(u32 index)
        {
            return m_columns[index].data.data();
        }

    private:
        struct Column
        {
            ComponentInfo info{};
            std::vector<std::byte> data; // size == entityCount * info.size
        };

        [[nodiscard]] Column* findColumn(ComponentTypeId type);
        [[nodiscard]] const Column* findColumn(ComponentTypeId type) const;

        Signature m_signature = 0;
        std::vector<Entity> m_entities;
        std::vector<Column> m_columns; // sorted by ComponentTypeId
    };
} // namespace sw::ecs
