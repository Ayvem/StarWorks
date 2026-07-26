#pragma once

// ============================================================================
// ECS/Entity.hpp
// Entity handle: a 32-bit index + a 32-bit generation.
//
// The generation is bumped every time an index is recycled, so stale handles
// are detected instead of silently pointing at a different entity. Handles
// contain no pointers, which makes them directly serializable — a hard
// requirement of the save system.
// ============================================================================

#include "Core/Types.hpp"

#include <functional>

namespace sw::ecs
{
    struct Entity
    {
        static constexpr u32 kInvalidIndex = 0xFFFFFFFFu;

        u32 index = kInvalidIndex;
        u32 generation = 0;

        [[nodiscard]] static constexpr Entity null() { return {}; }
        [[nodiscard]] constexpr bool isNull() const { return index == kInvalidIndex; }

        [[nodiscard]] constexpr bool operator==(const Entity&) const = default;
    };
} // namespace sw::ecs

template <>
struct std::hash<sw::ecs::Entity>
{
    [[nodiscard]] std::size_t operator()(const sw::ecs::Entity& entity) const noexcept
    {
        return (static_cast<std::size_t>(entity.generation) << 32) | entity.index;
    }
};
