# ============================================================================
# StarWorks — the game executable.
# ============================================================================

add_executable(StarWorks
    Source/Components.hpp
    Source/Main.cpp
    Source/StarWorksGame.cpp
    Source/StarWorksGame.hpp
    Source/Systems.cpp
    Source/Systems.hpp
)

target_link_libraries(StarWorks
    PRIVATE
        StarWorks::Engine
        StarWorks::BuildFlags
)

set_target_properties(StarWorks PROPERTIES FOLDER "Game")

# Debugging from Visual Studio: run from the executable directory so
# relative asset paths (Shaders/...) resolve.
set_target_properties(StarWorks PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY $<TARGET_FILE_DIR:StarWorks>
)

# Compile the game's shaders and mirror them next to the executable.
sw_compile_shaders(StarWorks
    SOURCES
        ${CMAKE_SOURCE_DIR}/Shaders/Mesh.vert
        ${CMAKE_SOURCE_DIR}/Shaders/Mesh.frag
        ${CMAKE_SOURCE_DIR}/Shaders/Hud.vert
        ${CMAKE_SOURCE_DIR}/Shaders/Hud.frag
)

# Mirror runtime assets next to the executable.
add_custom_command(TARGET StarWorks POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/Assets $<TARGET_FILE_DIR:StarWorks>/Assets
    COMMENT "Copying assets next to StarWorks"
    VERBATIM
)

get_target_property(SW_GAME_SOURCES StarWorks SOURCES)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/Source PREFIX "Source" FILES ${SW_GAME_SOURCES})
