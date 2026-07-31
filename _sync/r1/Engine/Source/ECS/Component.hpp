#pragma once

// ============================================================================
// ECS/Component.hpp
// Component type registration and archetype signatures.
//
// Rules for component types (checked at compile time where possible):
//  - trivially copyable and trivially destructible: rows move with memcpy,
//    columns never run constructors/destructors, and component memory can be
//    serialized as-is. Anything needing ownership semantics (strings,
//    containers) belongs in assets or side tables referenced by ID.
//  - alignment <= 16 bytes (column base storage guarantee).
//
// A signature is a 64-bit mask of component type ids: cheap to compare,
// cheap to hash, and 64 component types is comfortable for this game (the
// limit is a static_assert away from being raised via std::bitset).
// ============================================================================

#include "Core/Assert.hpp"
#include "Core/Types.hpp"

#include <mutex>
#include <type_traits>
#include <vector>

namespace sw::ecs
{
    using ComponentTypeId = u32;
    using Signature = u64;

    inline constexpr u32 kMaxComponentTypes = 64;

    struct ComponentInfo
    {
        ComponentTypeId id = 0;
        u32 size = 0;
        u32 alignment = 0;
    };

    namespace detail
    {
        [[nodiscard]] inline std::vector<ComponentInfo>& componentInfoRegistry()
        {
            static std::vector<ComponentInfo> registry;
            return registry;
        }

        [[nodiscard]] inline ComponentTypeId registerComponentType(u32 size, u32 alignment)
        {
            static std::mutex mutex;
            std::scoped_lock lock(mutex);

            auto& registry = componentInfoRegistry();
            const auto id = static_cast<ComponentTypeId>(registry.size());
            SW_ASSERT(id < kMaxComponentTypes,
                      "Component type limit ({}) exceeded — widen Signature",
                      kMaxComponentTypes);
            registry.push_back({id, size, alignment});
            return id;
        }
    } // namespace detail

    /// Stable (per process) numeric id for a component type. First call
    /// registers the type; thread-safe.
    template <typename T>
    [[nodiscard]] ComponentTypeId componentTypeId()
    {
        static_assert(std::is_trivially_copyable_v<T>,
                      "ECS components must be trivially copyable (use IDs/handles, "
                      "not owning members)");
        static_assert(std::is_trivially_destructible_v<T>,
                      "ECS components must be trivially destructible");
        static_assert(alignof(T) <= 16, "ECS components must not be over-aligned (>16)");
        static_assert(sizeof(T) > 0, "Empty components are not supported yet");

        static const ComponentTypeId id = detail::registerComponentType(
            static_cast<u32>(sizeof(T)), static_cast<u32>(alignof(T)));
        return id;
    }

    /// Info for an already-registered component type id.
    [[nodiscard]] inline const ComponentInfo& componentInfo(ComponentTypeId id)
    {
        return detail::componentInfoRegistry()[id];
    }

    template <typename T>
    [[nodiscard]] Signature componentBit()
    {
        return Signature{1} << componentTypeId<T>();
    }

    /// Signature containing all the given component types.
    template <typename... Ts>
    [[nodiscard]] Signature makeSignature()
    {
        return (Signature{0} | ... | componentBit<Ts>());
    }
} // namespace sw::ecs
