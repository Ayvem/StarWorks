# ============================================================================
# GLSL -> SPIR-V compilation, integrated into the build graph.
#
# Shaders are compiled at build time (never at runtime) and are re-compiled
# automatically when their sources change. Output goes to
#   ${CMAKE_BINARY_DIR}/Shaders/<name>.spv
# and the whole directory is mirrored next to the target executable after
# each build so the runtime can always resolve "Shaders/<name>.spv" relative
# to the binary.
# ============================================================================

# Locate a GLSL compiler. Preference order:
#   1. glslangValidator shipped with the Vulkan SDK (exported by FindVulkan)
#   2. glslangValidator on PATH
#   3. glslc (shaderc) on PATH
if(Vulkan_GLSLANG_VALIDATOR_EXECUTABLE)
    set(SW_SHADER_COMPILER "${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE}")
    set(SW_SHADER_COMPILER_KIND "glslangValidator")
else()
    find_program(SW_GLSLANG_EXE glslangValidator)
    if(SW_GLSLANG_EXE)
        set(SW_SHADER_COMPILER "${SW_GLSLANG_EXE}")
        set(SW_SHADER_COMPILER_KIND "glslangValidator")
    else()
        find_program(SW_GLSLC_EXE glslc)
        if(SW_GLSLC_EXE)
            set(SW_SHADER_COMPILER "${SW_GLSLC_EXE}")
            set(SW_SHADER_COMPILER_KIND "glslc")
        endif()
    endif()
endif()

if(NOT SW_SHADER_COMPILER)
    message(FATAL_ERROR
        "StarWorks: no GLSL compiler found (glslangValidator or glslc). "
        "Install the Vulkan SDK: https://vulkan.lunarg.com/")
endif()
message(STATUS "StarWorks: shader compiler: ${SW_SHADER_COMPILER} (${SW_SHADER_COMPILER_KIND})")

# sw_compile_shaders(<target> SOURCES <file.vert> <file.frag> ...)
#
# Compiles each GLSL source to SPIR-V targeting Vulkan 1.3, creates a
# dedicated custom target and makes <target> depend on it, then mirrors the
# compiled shaders next to <target>'s output binary.
function(sw_compile_shaders target)
    cmake_parse_arguments(ARG "" "" "SOURCES" ${ARGN})

    set(out_dir ${CMAKE_BINARY_DIR}/Shaders)
    file(MAKE_DIRECTORY ${out_dir})

    # ---- shared GLSL modules (M25) -----------------------------------------
    # Noise.glsl / Terrain.glsl / ... are INCLUDED by the entry-point shaders
    # (GL_GOOGLE_include_directive), never compiled on their own: they are the
    # GPU twins of the engine headers and must exist in exactly one place.
    # ${CMAKE_SOURCE_DIR}/Shaders is the include root, and every .glsl module
    # is a build dependency of every compiled shader — editing a module
    # recompiles the shaders that pull it in.
    set(shader_include_dir ${CMAKE_SOURCE_DIR}/Shaders)
    file(GLOB shader_modules CONFIGURE_DEPENDS ${shader_include_dir}/*.glsl)

    set(spv_outputs "")
    foreach(src ${ARG_SOURCES})
        get_filename_component(src_abs ${src} ABSOLUTE)
        get_filename_component(src_name ${src} NAME)
        set(spv ${out_dir}/${src_name}.spv)

        if(SW_SHADER_COMPILER_KIND STREQUAL "glslangValidator")
            set(compile_cmd ${SW_SHADER_COMPILER} -V --target-env vulkan1.3
                            -I${shader_include_dir} -o ${spv} ${src_abs})
        else()
            set(compile_cmd ${SW_SHADER_COMPILER} --target-env=vulkan1.3
                            -I${shader_include_dir} -o ${spv} ${src_abs})
        endif()

        add_custom_command(
            OUTPUT ${spv}
            COMMAND ${compile_cmd}
            DEPENDS ${src_abs} ${shader_modules}
            COMMENT "Compiling shader ${src_name} -> ${src_name}.spv"
            VERBATIM
        )
        list(APPEND spv_outputs ${spv})
    endforeach()

    add_custom_target(${target}_Shaders DEPENDS ${spv_outputs})
    set_target_properties(${target}_Shaders PROPERTIES FOLDER "Shaders")
    add_dependencies(${target} ${target}_Shaders)

    # Mirror compiled shaders next to the executable.
    #
    # THIS HANGS OFF THE SHADER TARGET, NOT THE EXECUTABLE, and the difference
    # is the single most expensive build bug this project has had. As a
    # POST_BUILD on ${target}, the copy ran only when the EXECUTABLE was
    # relinked. Edit a shader and nothing else: glslang recompiles the .spv
    # into ${out_dir}, the link step has nothing to do, the copy never runs,
    # and the game loads the PREVIOUS shader out of bin/Shaders. The build
    # succeeds. Nothing warns. Every capture is of the old code, and the
    # obvious conclusion — "my change did nothing, the theory must be wrong" —
    # is exactly backwards.
    #
    # A custom target is always considered out of date, so this copy runs on
    # every build. Four small files; correctness is not negotiable against
    # that.
    add_custom_command(TARGET ${target}_Shaders POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory
                $<TARGET_FILE_DIR:${target}>/Shaders
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                ${out_dir} $<TARGET_FILE_DIR:${target}>/Shaders
        COMMENT "Copying shaders next to $<TARGET_FILE_NAME:${target}>"
        VERBATIM
    )
endfunction()
