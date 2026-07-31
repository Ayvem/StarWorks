#pragma once

// ============================================================================
// Serialization/Binary.hpp
// Bounds-checked binary writer/reader — the byte-level layer every save
// format builds on. Little-endian on every supported platform (x86/ARM);
// values are memcpy'd, never reinterpret-cast, so alignment is never an
// issue. The reader THROWS on any overrun: a truncated or corrupted save
// must fail loudly, never read garbage.
// ============================================================================

#include "Core/Error.hpp"
#include "Core/Types.hpp"

#include <cstring>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace sw::ser
{
    class BinaryWriter
    {
    public:
        template <typename T>
        void write(const T& value)
        {
            static_assert(std::is_trivially_copyable_v<T>,
                          "Only trivially copyable types serialize directly");
            const auto offset = m_bytes.size();
            m_bytes.resize(offset + sizeof(T));
            std::memcpy(m_bytes.data() + offset, &value, sizeof(T));
        }

        void writeBytes(const void* data, usize size)
        {
            const auto offset = m_bytes.size();
            m_bytes.resize(offset + size);
            if (size > 0)
            {
                std::memcpy(m_bytes.data() + offset, data, size);
            }
        }

        void writeString(std::string_view text)
        {
            write(static_cast<u32>(text.size()));
            writeBytes(text.data(), text.size());
        }

        [[nodiscard]] const std::vector<u8>& bytes() const { return m_bytes; }
        [[nodiscard]] usize size() const { return m_bytes.size(); }

    private:
        std::vector<u8> m_bytes;
    };

    class BinaryReader
    {
    public:
        explicit BinaryReader(std::span<const u8> bytes) : m_bytes(bytes) {}

        template <typename T>
        [[nodiscard]] T read()
        {
            static_assert(std::is_trivially_copyable_v<T>,
                          "Only trivially copyable types deserialize directly");
            T value{};
            readBytes(&value, sizeof(T));
            return value;
        }

        void readBytes(void* destination, usize size)
        {
            if (m_cursor + size > m_bytes.size())
            {
                SW_THROW("Corrupted save data: read of {} bytes at offset {} exceeds size {}",
                         size, m_cursor, m_bytes.size());
            }
            if (size > 0)
            {
                std::memcpy(destination, m_bytes.data() + m_cursor, size);
            }
            m_cursor += size;
        }

        [[nodiscard]] std::string readString()
        {
            const u32 length = read<u32>();
            if (length > 1u << 20) // 1 MB name: clearly corrupted
            {
                SW_THROW("Corrupted save data: string length {}", length);
            }
            std::string text(length, '\0');
            readBytes(text.data(), length);
            return text;
        }

        [[nodiscard]] usize remaining() const { return m_bytes.size() - m_cursor; }

    private:
        std::span<const u8> m_bytes;
        usize m_cursor = 0;
    };
} // namespace sw::ser
