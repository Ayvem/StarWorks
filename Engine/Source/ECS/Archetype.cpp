#include "ECS/Archetype.hpp"

#include <algorithm>
#include <cstring>

namespace sw::ecs
{
    Archetype::Archetype(Signature signature, std::span<const ComponentInfo> components)
        : m_signature(signature)
    {
        m_columns.reserve(components.size());
        for (const ComponentInfo& info : components)
        {
            m_columns.push_back({info, {}});
        }
        std::sort(m_columns.begin(), m_columns.end(),
                  [](const Column& a, const Column& b) { return a.info.id < b.info.id; });
    }

    u32 Archetype::addRow(Entity entity)
    {
        const u32 row = static_cast<u32>(m_entities.size());
        m_entities.push_back(entity);
        for (Column& column : m_columns)
        {
            // Value-initialized bytes: new components start zeroed.
            column.data.resize(column.data.size() + column.info.size);
        }
        return row;
    }

    Entity Archetype::removeRow(u32 row)
    {
        const u32 last = static_cast<u32>(m_entities.size()) - 1;
        Entity moved = Entity::null();

        if (row != last)
        {
            for (Column& column : m_columns)
            {
                const usize size = column.info.size;
                std::memcpy(column.data.data() + static_cast<usize>(row) * size,
                            column.data.data() + static_cast<usize>(last) * size, size);
            }
            m_entities[row] = m_entities[last];
            moved = m_entities[row];
        }

        m_entities.pop_back();
        for (Column& column : m_columns)
        {
            column.data.resize(column.data.size() - column.info.size);
        }
        return moved;
    }

    std::byte* Archetype::columnData(ComponentTypeId type)
    {
        Column* column = findColumn(type);
        return (column != nullptr) ? column->data.data() : nullptr;
    }

    const std::byte* Archetype::columnData(ComponentTypeId type) const
    {
        const Column* column = findColumn(type);
        return (column != nullptr) ? column->data.data() : nullptr;
    }

    void Archetype::copyCommonComponents(const Archetype& source, u32 sourceRow,
                                         u32 destinationRow)
    {
        // Both column lists are sorted by type id: single merge pass.
        usize s = 0;
        for (Column& dst : m_columns)
        {
            while (s < source.m_columns.size() && source.m_columns[s].info.id < dst.info.id)
            {
                ++s;
            }
            if (s == source.m_columns.size())
            {
                break;
            }
            const Column& src = source.m_columns[s];
            if (src.info.id == dst.info.id)
            {
                const usize size = dst.info.size;
                std::memcpy(dst.data.data() + static_cast<usize>(destinationRow) * size,
                            src.data.data() + static_cast<usize>(sourceRow) * size, size);
            }
        }
    }

    Archetype::Column* Archetype::findColumn(ComponentTypeId type)
    {
        for (Column& column : m_columns)
        {
            if (column.info.id == type)
            {
                return &column;
            }
        }
        return nullptr;
    }

    const Archetype::Column* Archetype::findColumn(ComponentTypeId type) const
    {
        for (const Column& column : m_columns)
        {
            if (column.info.id == type)
            {
                return &column;
            }
        }
        return nullptr;
    }
} // namespace sw::ecs
