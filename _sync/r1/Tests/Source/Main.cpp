// ============================================================================
// Tests/Main.cpp — StarWorksTests entry point.
// Initializes the engine log (tests exercise engine code that logs), runs
// every registered test, and returns non-zero on failure for CTest/CI.
// ============================================================================

#include "TestFramework.hpp"

#include <Core/Log.hpp>

int main()
{
    sw::Log::Config logConfig{};
    logConfig.level = sw::LogLevel::Warn; // keep test output readable
    sw::Log::initialize(logConfig);

    const int result = sw::test::runAll();

    sw::Log::shutdown();
    return result;
}
