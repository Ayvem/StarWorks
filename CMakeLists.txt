# ============================================================================
# StarWorks — root build script
#
# Industrial space simulation game with a bespoke Vulkan engine.
# C++20, CMake >= 3.24 (FetchContent + find_package(Vulkan) with components).
# ============================================================================
cmake_minimum_required(VERSION 3.24)

project(StarWorks
    VERSION 0.1.0
    DESCRIPTION "Industrial space simulation game and bespoke engine"
    LANGUAGES CXX
)

# ----------------------------------------------------------------------------
# Global settings
# ----------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Keep all runtime artifacts together; simplifies running from the build tree.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Folders in Visual Studio solution explorer.
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
endif()

# ----------------------------------------------------------------------------
# Options
# ----------------------------------------------------------------------------
option(SW_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(SW_BUILD_TESTS "Build the StarWorksTests unit test target" ON)

# ----------------------------------------------------------------------------
# Shared build flags (interface target linked by every StarWorks target)
# ----------------------------------------------------------------------------
add_library(StarWorksBuildFlags INTERFACE)
add_library(StarWorks::BuildFlags ALIAS StarWorksBuildFlags)

if(MSVC)
    target_compile_options(StarWorksBuildFlags INTERFACE
        /W4                 # high warning level
        /permissive-        # strict standard conformance
        /Zc:preprocessor    # conforming preprocessor
        /utf-8              # source and execution charset
        /MP                 # parallel compilation
    )
    if(SW_WARNINGS_AS_ERRORS)
        target_compile_options(StarWorksBuildFlags INTERFACE /WX)
    endif()
    target_compile_definitions(StarWorksBuildFlags INTERFACE
        NOMINMAX
        WIN32_LEAN_AND_MEAN
        _CRT_SECURE_NO_WARNINGS
    )
else()
    target_compile_options(StarWorksBuildFlags INTERFACE
        -Wall -Wextra -Wpedantic
    )
    if(SW_WARNINGS_AS_ERRORS)
        target_compile_options(StarWorksBuildFlags INTERFACE -Werror)
    endif()
endif()

# Engine-wide defines. SW_DEBUG drives assertions and default log verbosity.
target_compile_definitions(StarWorksBuildFlags INTERFACE
    $<$<CONFIG:Debug>:SW_DEBUG>
    GLFW_INCLUDE_NONE
    GLM_FORCE_RADIANS
    GLM_FORCE_DEPTH_ZERO_TO_ONE   # Vulkan clip-space depth is [0, 1]
    GLM_ENABLE_EXPERIMENTAL
)

# ----------------------------------------------------------------------------
# Dependencies and helper modules
# ----------------------------------------------------------------------------
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(Dependencies)
include(Shaders)

# ----------------------------------------------------------------------------
# Project targets
# ----------------------------------------------------------------------------
add_subdirectory(Engine)
add_subdirectory(Game)
add_subdirectory(Tools/PartStudio)
add_subdirectory(Tools/AeroForge)

if(SW_BUILD_TESTS)
    enable_testing()
    add_subdirectory(Tests)
endif()

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT StarWorks)
