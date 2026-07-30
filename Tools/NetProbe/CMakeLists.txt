# ============================================================================
# NetProbe — a headless host and a headless client, over real UDP sockets,
# measuring what multiplayer actually costs.
#
# The game cannot be run without a display, so every network number in
# docs/Performance.md comes from here: real sockets, real component layouts,
# real encoder. Console only — no window, no Vulkan, no shaders.
# ============================================================================

add_executable(NetProbe
    Source/Main.cpp
)

target_link_libraries(NetProbe
    PRIVATE
        StarWorks::Engine
        StarWorks::BuildFlags
)

set_target_properties(NetProbe PROPERTIES FOLDER "Tools")
set_target_properties(NetProbe PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
