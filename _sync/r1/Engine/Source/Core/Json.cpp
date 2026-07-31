#include "Core/Json.hpp"

#include <charconv>
#include <cmath>
#include <cstdio>
#include <format>

namespace sw::json
{
    // ------------------------------------------------------------------ Value
    const Value* Value::find(std::string_view key) const
    {
        if (!isObject())
        {
            return nullptr;
        }
        for (const auto& [name, value] : asObject())
        {
            if (name == key)
            {
                return &value;
            }
        }
        return nullptr;
    }

    f64 Value::number(std::string_view key, f64 fallback) const
    {
        const Value* value = find(key);
        return (value != nullptr) ? value->asNumber(fallback) : fallback;
    }

    bool Value::boolean(std::string_view key, bool fallback) const
    {
        const Value* value = find(key);
        return (value != nullptr) ? value->asBool(fallback) : fallback;
    }

    const std::string& Value::string(std::string_view key) const
    {
        static const std::string kEmpty;
        const Value* value = find(key);
        return (value != nullptr) ? value->asString() : kEmpty;
    }

    void Value::set(std::string_view key, Value value)
    {
        if (!isObject())
        {
            m_data = Object{};
        }
        for (auto& [name, existing] : std::get<Object>(m_data))
        {
            if (name == key)
            {
                existing = std::move(value);
                return;
            }
        }
        std::get<Object>(m_data).emplace_back(std::string(key), std::move(value));
    }

    void Value::push(Value value)
    {
        if (!isArray())
        {
            m_data = Array{};
        }
        std::get<Array>(m_data).push_back(std::move(value));
    }

    // ----------------------------------------------------------------- Parser
    namespace
    {
        struct Parser
        {
            std::string_view text;
            usize cursor = 0;
            u32 line = 1;
            std::string error;

            [[nodiscard]] bool failed() const { return !error.empty(); }

            void fail(std::string_view message)
            {
                if (error.empty())
                {
                    error = std::format("line {}: {}", line, message);
                }
            }

            [[nodiscard]] char peek() const
            {
                return cursor < text.size() ? text[cursor] : '\0';
            }

            char advance()
            {
                const char character = peek();
                ++cursor;
                if (character == '\n')
                {
                    ++line;
                }
                return character;
            }

            void skipWhitespace()
            {
                while (cursor < text.size())
                {
                    const char character = text[cursor];
                    if (character == ' ' || character == '\t' || character == '\r' ||
                        character == '\n')
                    {
                        advance();
                    }
                    else
                    {
                        break;
                    }
                }
            }

            bool expect(char character)
            {
                skipWhitespace();
                if (peek() != character)
                {
                    fail(std::format("expected '{}'", character));
                    return false;
                }
                advance();
                return true;
            }

            Value parseValue(u32 depth)
            {
                if (depth > 64)
                {
                    fail("nesting too deep");
                    return {};
                }
                skipWhitespace();
                const char character = peek();
                switch (character)
                {
                case '{':
                    return parseObject(depth);
                case '[':
                    return parseArray(depth);
                case '"':
                    return Value(parseString());
                case 't':
                    return parseKeyword("true", Value(true));
                case 'f':
                    return parseKeyword("false", Value(false));
                case 'n':
                    return parseKeyword("null", Value{});
                default:
                    if (character == '-' || (character >= '0' && character <= '9'))
                    {
                        return parseNumber();
                    }
                    fail("unexpected character");
                    return {};
                }
            }

            Value parseKeyword(std::string_view keyword, Value result)
            {
                if (text.substr(cursor, keyword.size()) != keyword)
                {
                    fail("invalid literal");
                    return {};
                }
                cursor += keyword.size();
                return result;
            }

            Value parseNumber()
            {
                const usize start = cursor;
                if (peek() == '-')
                {
                    advance();
                }
                while (cursor < text.size())
                {
                    const char character = text[cursor];
                    if ((character >= '0' && character <= '9') || character == '.' ||
                        character == 'e' || character == 'E' || character == '+' ||
                        character == '-')
                    {
                        ++cursor;
                    }
                    else
                    {
                        break;
                    }
                }
                f64 number = 0.0;
                const char* first = text.data() + start;
                const char* last = text.data() + cursor;
                const auto [pointer, errorCode] = std::from_chars(first, last, number);
                if (errorCode != std::errc{} || pointer != last)
                {
                    fail("malformed number");
                    return {};
                }
                return Value(number);
            }

            std::string parseString()
            {
                std::string result;
                if (!expect('"'))
                {
                    return result;
                }
                while (true)
                {
                    if (cursor >= text.size())
                    {
                        fail("unterminated string");
                        return result;
                    }
                    const char character = advance();
                    if (character == '"')
                    {
                        return result;
                    }
                    if (character == '\\')
                    {
                        const char escape = advance();
                        switch (escape)
                        {
                        case '"': result += '"'; break;
                        case '\\': result += '\\'; break;
                        case '/': result += '/'; break;
                        case 'n': result += '\n'; break;
                        case 't': result += '\t'; break;
                        case 'r': result += '\r'; break;
                        case 'b': result += '\b'; break;
                        case 'f': result += '\f'; break;
                        default:
                            fail("unsupported escape");
                            return result;
                        }
                    }
                    else
                    {
                        result += character;
                    }
                }
            }

            Value parseObject(u32 depth)
            {
                Value result = Value::makeObject();
                advance(); // '{'
                skipWhitespace();
                if (peek() == '}')
                {
                    advance();
                    return result;
                }
                while (true)
                {
                    skipWhitespace();
                    std::string key = parseString();
                    if (failed() || !expect(':'))
                    {
                        return result;
                    }
                    Value value = parseValue(depth + 1);
                    if (failed())
                    {
                        return result;
                    }
                    result.object().emplace_back(std::move(key), std::move(value));
                    skipWhitespace();
                    if (peek() == ',')
                    {
                        advance();
                        continue;
                    }
                    expect('}');
                    return result;
                }
            }

            Value parseArray(u32 depth)
            {
                Value result = Value::makeArray();
                advance(); // '['
                skipWhitespace();
                if (peek() == ']')
                {
                    advance();
                    return result;
                }
                while (true)
                {
                    Value value = parseValue(depth + 1);
                    if (failed())
                    {
                        return result;
                    }
                    result.array().push_back(std::move(value));
                    skipWhitespace();
                    if (peek() == ',')
                    {
                        advance();
                        continue;
                    }
                    expect(']');
                    return result;
                }
            }
        };
    } // namespace

    Value parse(std::string_view text, std::string& outError)
    {
        Parser parser{};
        parser.text = text;
        Value result = parser.parseValue(0);
        parser.skipWhitespace();
        if (!parser.failed() && parser.cursor != text.size())
        {
            parser.fail("trailing content");
        }
        outError = parser.error;
        return parser.failed() ? Value{} : result;
    }

    // ------------------------------------------------------------- Serializer
    namespace
    {
        void writeEscaped(std::string& out, const std::string& text)
        {
            out += '"';
            for (const char character : text)
            {
                switch (character)
                {
                case '"': out += "\\\""; break;
                case '\\': out += "\\\\"; break;
                case '\n': out += "\\n"; break;
                case '\t': out += "\\t"; break;
                case '\r': out += "\\r"; break;
                default: out += character; break;
                }
            }
            out += '"';
        }

        void writeNumber(std::string& out, f64 number)
        {
            // Integers print without a decimal point; everything else with
            // enough digits to round-trip f32-grade asset data cleanly.
            if (std::floor(number) == number && std::abs(number) < 1.0e15)
            {
                out += std::format("{}", static_cast<i64>(number));
            }
            else
            {
                out += std::format("{}", number);
            }
        }

        void writeValue(std::string& out, const Value& value, u32 indent)
        {
            const auto pad = [&out](u32 level) { out.append(level * 2, ' '); };

            if (value.isNull()) { out += "null"; return; }
            if (value.isBool()) { out += value.asBool() ? "true" : "false"; return; }
            if (value.isNumber()) { writeNumber(out, value.asNumber()); return; }
            if (value.isString()) { writeEscaped(out, value.asString()); return; }

            if (value.isArray())
            {
                const Array& array = value.asArray();
                if (array.empty())
                {
                    out += "[]";
                    return;
                }
                // Arrays of scalars stay on one line (vectors, colors...).
                bool scalarOnly = true;
                for (const Value& item : array)
                {
                    if (item.isArray() || item.isObject())
                    {
                        scalarOnly = false;
                        break;
                    }
                }
                if (scalarOnly)
                {
                    out += '[';
                    for (usize i = 0; i < array.size(); ++i)
                    {
                        if (i > 0) { out += ", "; }
                        writeValue(out, array[i], indent);
                    }
                    out += ']';
                    return;
                }
                out += "[\n";
                for (usize i = 0; i < array.size(); ++i)
                {
                    pad(indent + 1);
                    writeValue(out, array[i], indent + 1);
                    if (i + 1 < array.size()) { out += ','; }
                    out += '\n';
                }
                pad(indent);
                out += ']';
                return;
            }

            const Object& object = value.asObject();
            if (object.empty())
            {
                out += "{}";
                return;
            }
            out += "{\n";
            for (usize i = 0; i < object.size(); ++i)
            {
                pad(indent + 1);
                writeEscaped(out, object[i].first);
                out += ": ";
                writeValue(out, object[i].second, indent + 1);
                if (i + 1 < object.size()) { out += ','; }
                out += '\n';
            }
            pad(indent);
            out += '}';
        }
    } // namespace

    std::string serialize(const Value& value)
    {
        std::string out;
        writeValue(out, value, 0);
        out += '\n';
        return out;
    }
} // namespace sw::json
