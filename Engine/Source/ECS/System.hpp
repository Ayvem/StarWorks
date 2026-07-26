#pragma once

// ============================================================================
// ECS/System.hpp
// A System is a unit of simulation logic with a declared data-access set.
// The declaration is what makes safe parallelism possible: the scheduler
// only needs the read/write masks, never the code, to know which systems
// may run concurrently.
// ============================================================================

#include "Core/Types.hpp"
#include "ECS/Component.hpp"

#include <string_view>

namespace sw::ecs
{
    class World;

    /// Declared component access of a system, as signature masks.
    struct SystemAccess
    {
        Signature reads = 0;
        Signature writes = 0;

        template <typename T>
        SystemAccess& read()
        {
            reads |= componentBit<T>();
            return *this;
        }

        template <typename T>
        SystemAccess& write()
        {
            writes |= componentBit<T>();
            return *this;
        }

        /// Two systems conflict if either writes what the other touches.
        [[nodiscard]] bool conflictsWith(const SystemAccess& other) const
        {
            return (writes & (other.reads | other.writes)) != 0 ||
                   (other.writes & (reads | writes)) != 0;
        }
    };

    class System
    {
    public:
        virtual ~System() = default;

        [[nodiscard]] virtual std::string_view name() const = 0;
        [[nodiscard]] virtual SystemAccess access() const = 0;
        virtual void update(World& world, f32 deltaSeconds) = 0;
    };
} // namespace sw::ecs
