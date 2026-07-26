# ============================================================================
# PartStudio — the .swpart authoring tool, built on the engine.
# ============================================================================

add_executable(PartStudio
    Source/Main.cpp
    Source/PartStudioApp.cpp
    Source/PartStudioApp.hpp
)

target_link_libraries(PartStudio
    PRIVATE
        StarWorks::Engine
        StarWorks::BuildFlags
)

set_target_properties(PartStudio PROPERTIES FOLDER "Tools")
set_target_properties(PartStudio PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY $<TARGET_FILE_DIR:PartStudio>
)

sw_compile_shaders(PartStudio
    SOURCES
        ${CMAKE_SOURCE_DIR}/Shaders/Mesh.vert
        ${CMAKE_SOURCE_DIR}/Shaders/Mesh.frag
        ${CMAKE_SOURCE_DIR}/Shaders/Hud.vert
        ${CMAKE_SOURCE_DIR}/Shaders/Hud.frag
)

# The Studio edits the SAME asset files the game ships: mirror them next
# to the executable (saves write into this mirror; copy back to the source
# Assets/ folder to commit them).
add_custom_command(TARGET PartStudio POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/Assets $<TARGET_FILE_DIR:PartStudio>/Assets
    COMMENT "Copying assets next to PartStudio"
    VERBATIM
)

get_target_property(SW_STUDIO_SOURCES PartStudio SOURCES)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/Source PREFIX "Source" FILES ${SW_STUDIO_SOURCES})
