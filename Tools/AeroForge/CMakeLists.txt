# ============================================================================
# AeroForge — the offline wind tunnel that generates .aero.json tables.
#
# Console only: it reads geometry and writes numbers, so it needs no window,
# no Vulkan and no shaders. Run it after editing a part in Part Studio.
# ============================================================================

add_executable(AeroForge
    Source/Main.cpp
)

target_link_libraries(AeroForge
    PRIVATE
        StarWorks::Engine
        StarWorks::BuildFlags
)

set_target_properties(AeroForge PROPERTIES FOLDER "Tools")
set_target_properties(AeroForge PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
