#pragma once

// ============================================================================
// Core/Types.hpp
// Fundamental fixed-size type aliases used across the whole engine.
// Keeping these short and uniform makes struct layouts, serialization and
// GPU-facing code much easier to audit.
// ============================================================================

#include <cstddef>
#include <cstdint>

namespace sw
{
    using u8  = std::uint8_t;
    using u16 = std::uint16_t;
    using u32 = std::uint32_t;
    using u64 = std::uint64_t;

    using i8  = std::int8_t;
    using i16 = std::int16_t;
    using i32 = std::int32_t;
    using i64 = std::int64_t;

    using f32 = float;
    using f64 = double;

    using usize = std::size_t;

    static_assert(sizeof(f32) == 4, "f32 must be 32 bits");
    static_assert(sizeof(f64) == 8, "f64 must be 64 bits");
} // namespace sw
