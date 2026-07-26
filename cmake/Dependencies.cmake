# ============================================================================
# Third-party dependencies.
#
# Policy: the only dependency that must be installed on the machine is the
# Vulkan SDK (headers, loader, glslangValidator). Everything else is fetched
# and built from source by CMake so a fresh clone configures out of the box.
# ============================================================================
include(FetchContent)

# ----------------------------------------------------------------------------
# Vulkan (system SDK)
# ----------------------------------------------------------------------------
find_package(Vulkan REQUIRED)
message(STATUS "StarWorks: Vulkan headers: ${Vulkan_INCLUDE_DIRS}")

# ----------------------------------------------------------------------------
# GLFW 3.4 — windowing / input / surface creation
# ----------------------------------------------------------------------------
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL        OFF CACHE BOOL "" FORCE)

FetchContent_Declare(glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.4
    GIT_SHALLOW    TRUE
)

# ----------------------------------------------------------------------------
# GLM 1.0.1 — mathematics (may be replaced by an in-house library later,
# which is why engine code only touches GLM through Engine/Source/Math/).
# ----------------------------------------------------------------------------
FetchContent_Declare(glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG        1.0.1
    GIT_SHALLOW    TRUE
)

# ----------------------------------------------------------------------------
# VulkanMemoryAllocator 3.1.0 — GPU memory management (AMD/GPUOpen)
# ----------------------------------------------------------------------------
FetchContent_Declare(vma
    GIT_REPOSITORY https://github.com/GPUOpen-LibrariesAndSDKs/VulkanMemoryAllocator.git
    GIT_TAG        v3.1.0
    GIT_SHALLOW    TRUE
)

# ----------------------------------------------------------------------------
# cgltf 1.14 — single-header glTF 2.0 parser (used by Assets/GltfLoader only)
# ----------------------------------------------------------------------------
FetchContent_Declare(cgltf
    GIT_REPOSITORY https://github.com/jkuhlmann/cgltf.git
    GIT_TAG        v1.14
    GIT_SHALLOW    TRUE
)

# cgltf has no top-level CMakeLists.txt, so MakeAvailable only downloads it;
# we then expose its header directory as an interface library.
FetchContent_MakeAvailable(glfw glm vma cgltf)

add_library(cgltf INTERFACE)
target_include_directories(cgltf INTERFACE ${cgltf_SOURCE_DIR})

# Keep third-party projects tidy in IDE solution folders.
foreach(tp_target glfw glm update_mappings)
    if(TARGET ${tp_target})
        set_target_properties(${tp_target} PROPERTIES FOLDER "ThirdParty")
    endif()
endforeach()
