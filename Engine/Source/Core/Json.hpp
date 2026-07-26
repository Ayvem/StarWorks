#pragma once

// ============================================================================
// Core/Json.hpp
// A minimal, dependency-free JSON reader/writer.
//
// WHY: parts (and later factories, tech trees...) become DATA FILES that the
// game AND the tools (Part Studio) read and write. JSON is the exchange
// format: human-diffable, hand-editable, and the tool round-trips it. The
// engine rule stands — no third-party parser; this is ~400 lines we own.
//
// Scope: full JSON minus exotic corners we do not need (no \uXXXX surrogate
// pairs — asset files are ASCII by convention; escapes \" \\ \/ \n \t \r \b
// \f are supported). Numbers are f64. Objects preserve INSERTION ORDER so a
// file saved by the tool diffs cleanly against its previous version.
// ============================================================================

#include "Core/Types.hpp"

#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace sw::json
{
    class Value;
    using Array = std::vector<Value>;
    /// Ordered key/value storage (insertion order preserved on save).
    using Object = std::vector<std::pair<std::string, Value>>;

    class Value
    {
    public:
        Value() = default;
        Value(bool boolean) : m_data(boolean) {}
        Value(f64 number) : m_data(number) {}
        Value(i32 number) : m_data(static_cast<f64>(number)) {}
        Value(u32 number) : m_data(static_cast<f64>(number)) {}
        Value(const char* text) : m_data(std::string(text)) {}
        Value(std::string text) : m_data(std::move(text)) {}
        Value(Array array) : m_data(std::move(array)) {}
        Value(Object object) : m_data(std::move(object)) {}

        [[nodiscard]] bool isNull() const { return std::holds_alternative<std::monostate>(m_data); }
        [[nodiscard]] bool isBool() const { return std::holds_alternative<bool>(m_data); }
        [[nodiscard]] bool isNumber() const { return std::holds_alternative<f64>(m_data); }
        [[nodiscard]] bool isString() const { return std::holds_alternative<std::string>(m_data); }
        [[nodiscard]] bool isArray() const { return std::holds_alternative<Array>(m_data); }
        [[nodiscard]] bool isObject() const { return std::holds_alternative<Object>(m_data); }

        [[nodiscard]] bool asBool(bool fallback = false) const
        {
            return isBool() ? std::get<bool>(m_data) : fallback;
        }
        [[nodiscard]] f64 asNumber(f64 fallback = 0.0) const
        {
            return isNumber() ? std::get<f64>(m_data) : fallback;
        }
        [[nodiscard]] const std::string& asString() const
        {
            static const std::string kEmpty;
            return isString() ? std::get<std::string>(m_data) : kEmpty;
        }
        [[nodiscard]] const Array& asArray() const
        {
            static const Array kEmpty;
            return isArray() ? std::get<Array>(m_data) : kEmpty;
        }
        [[nodiscard]] const Object& asObject() const
        {
            static const Object kEmpty;
            return isObject() ? std::get<Object>(m_data) : kEmpty;
        }

        /// Object member lookup; nullptr when absent or not an object.
        [[nodiscard]] const Value* find(std::string_view key) const;

        /// Convenience typed member readers (fallback when missing/mistyped).
        [[nodiscard]] f64 number(std::string_view key, f64 fallback = 0.0) const;
        [[nodiscard]] bool boolean(std::string_view key, bool fallback = false) const;
        [[nodiscard]] const std::string& string(std::string_view key) const;

        /// Object building: sets or replaces `key` (keeps insertion order).
        void set(std::string_view key, Value value);
        /// Array building.
        void push(Value value);

        [[nodiscard]] Array& array() { return std::get<Array>(m_data); }
        [[nodiscard]] Object& object() { return std::get<Object>(m_data); }

        static Value makeObject() { return Value(Object{}); }
        static Value makeArray() { return Value(Array{}); }

    private:
        std::variant<std::monostate, bool, f64, std::string, Array, Object> m_data;
    };

    /// Parses `text`. On failure returns a NULL value and fills `outError`
    /// with "line N: message". An empty outError means success.
    [[nodiscard]] Value parse(std::string_view text, std::string& outError);

    /// Pretty-prints with 2-space indentation and a trailing newline —
    /// the canonical on-disk form (tool saves diff cleanly).
    [[nodiscard]] std::string serialize(const Value& value);
} // namespace sw::json
