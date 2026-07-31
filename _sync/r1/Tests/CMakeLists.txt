# ============================================================================
# StarWorksTests — engine unit tests (in-house harness, CTest integrated).
# Pure CPU: no window, no Vulkan device required, safe on any CI runner.
# ============================================================================

add_executable(StarWorksTests
    Source/CommandBufferTests.cpp
    Source/EcsTests.cpp
    Source/FactoryTests.cpp
    Source/FrustumTests.cpp
    Source/HudOrderTests.cpp
    Source/IndustryTests.cpp
    Source/JsonTests.cpp
    Source/Main.cpp
    Source/NetworkTests.cpp
    Source/PartsTests.cpp
    Source/PhysicsTests.cpp
    Source/PortabilityTests.cpp
    Source/PowerTests.cpp
    # These three load the SHIPPED catalogue, which replaces the global
    # registry — they must link AFTER PartsTests, which checks the
    # built-in fallback.
    Source/HullTests.cpp
    Source/ConstructionTests.cpp
    Source/BlueprintTests.cpp
    Source/AeroTests.cpp
    Source/PrimitivesTests.cpp
    Source/SaveTests.cpp
    Source/ScreenMarkerTests.cpp
    Source/SchedulerTests.cpp
    Source/SimulationTests.cpp
    Source/SpaceTests.cpp
    Source/TerrainTests.cpp
    Source/TestFramework.hpp
)

target_link_libraries(StarWorksTests
    PRIVATE
        StarWorks::Engine
        StarWorks::BuildFlags
)

set_target_properties(StarWorksTests PROPERTIES FOLDER "Tests")

add_test(NAME StarWorksTests COMMAND StarWorksTests)

# The catalog tests validate the SHIPPED part files (nodes on collider
# surfaces, JSON round-trip): mirror the assets next to the test binary.
add_custom_command(TARGET StarWorksTests POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/Assets $<TARGET_FILE_DIR:StarWorksTests>/Assets
    COMMENT "Copying assets next to StarWorksTests"
    VERBATIM
)

get_target_property(SW_TESTS_SOURCES StarWorksTests SOURCES)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/Source PREFIX "Source" FILES ${SW_TESTS_SOURCES})
