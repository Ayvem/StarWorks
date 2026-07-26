#pragma once

// ============================================================================
// TestFramework.hpp
// Deliberately tiny in-house test harness: registration macro, check
// macros, plain-text report. No third-party dependency, no engine
// dependency beyond Core/Types — tests must be able to break the engine
// without breaking the harness.
// ============================================================================

#include <cstdio>
#include <string>
#include <vector>

namespace sw::test
{
    struct TestCase
    {
        const char* name;
        void (*fn)();
    };

    inline std::vector<TestCase>& registry()
    {
        static std::vector<TestCase> tests;
        return tests;
    }

    inline int& failureCount()
    {
        static int failures = 0;
        return failures;
    }

    struct Registrar
    {
        Registrar(const char* name, void (*fn)()) { registry().push_back({name, fn}); }
    };

    inline int runAll()
    {
        int executed = 0;
        for (const TestCase& test : registry())
        {
            const int failuresBefore = failureCount();
            std::printf("[ RUN  ] %s\n", test.name);
            test.fn();
            ++executed;
            if (failureCount() == failuresBefore)
            {
                std::printf("[  OK  ] %s\n", test.name);
            }
            else
            {
                std::printf("[ FAIL ] %s\n", test.name);
            }
        }
        std::printf("---\n%d test(s), %d failure(s)\n", executed, failureCount());
        return failureCount() == 0 ? 0 : 1;
    }
} // namespace sw::test

#define SW_TEST(name)                                                                \
    static void swTest_##name();                                                     \
    static ::sw::test::Registrar swTestRegistrar_##name(#name, &swTest_##name);      \
    static void swTest_##name()

#define SW_CHECK(expr)                                                               \
    do                                                                               \
    {                                                                                \
        if (!(expr))                                                                 \
        {                                                                            \
            ++::sw::test::failureCount();                                            \
            std::printf("  CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        }                                                                            \
    } while (false)

#define SW_CHECK_EQ(a, b) SW_CHECK((a) == (b))
