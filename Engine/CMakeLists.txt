# ============================================================================
# StarWorksEngine — static library containing every engine module.
#
# Sources are listed explicitly (no GLOB): the build breaks loudly when a
# file is added or removed, which is exactly what we want on a long-lived
# project.
# ============================================================================

add_library(StarWorksEngine STATIC
    # --- Core ---------------------------------------------------------------
    Source/Core/Application.cpp
    Source/Core/Application.hpp
    Source/Core/Assert.hpp
    Source/Core/Clock.cpp
    Source/Core/Clock.hpp
    Source/Core/Error.hpp
    Source/Core/FileSystem.cpp
    Source/Core/FileSystem.hpp
    Source/Core/Json.cpp
    Source/Core/Json.hpp
    Source/Core/Log.cpp
    Source/Core/Log.hpp
    Source/Core/ThreadPool.cpp
    Source/Core/ThreadPool.hpp
    Source/Core/Types.hpp

    # --- ECS ------------------------------------------------------------------
    Source/ECS/Archetype.cpp
    Source/ECS/Archetype.hpp
    Source/ECS/CommandBuffer.cpp
    Source/ECS/CommandBuffer.hpp
    Source/ECS/Component.hpp
    Source/ECS/Entity.hpp
    Source/ECS/System.hpp
    Source/ECS/SystemScheduler.cpp
    Source/ECS/SystemScheduler.hpp
    Source/ECS/World.cpp
    Source/ECS/World.hpp

    # --- Serialization / Save ------------------------------------------------
    Source/Save/Snapshot.cpp
    Source/Save/Snapshot.hpp
    Source/Serialization/Binary.hpp

    # --- Network --------------------------------------------------------------
    Source/Network/Protocol.cpp
    Source/Network/Protocol.hpp
    Source/Network/Replication.cpp
    Source/Network/Replication.hpp
    Source/Network/Session.cpp
    Source/Network/Timeline.cpp
    Source/Network/Timeline.hpp
    Source/Network/Session.hpp
    Source/Network/Transport.cpp
    Source/Network/Transport.hpp

    # --- Simulation -------------------------------------------------------------
    Source/Simulation/Simulation.cpp
    Source/Simulation/Simulation.hpp

    # --- Platform -----------------------------------------------------------
    Source/Platform/Firewall.cpp
    Source/Platform/Firewall.hpp
    Source/Platform/Window.cpp
    Source/Platform/Window.hpp

    # --- Input ---------------------------------------------------------------
    Source/Input/Input.cpp
    Source/Input/Input.hpp

    # --- Math ----------------------------------------------------------------
    Source/Math/Frustum.cpp
    Source/Math/Frustum.hpp
    Source/Math/Math.hpp
    Source/Math/Noise.hpp

    # --- Planet ----------------------------------------------------------------
    Source/Planet/Deposits.hpp
    Source/Planet/Terrain.hpp

    # --- Resources ---------------------------------------------------------------
    Source/Resources/ResourceTypes.cpp
    Source/Resources/ResourceTypes.hpp

    # --- Factory -----------------------------------------------------------------
    Source/Factory/FactoryComponents.cpp
    Source/Factory/FactoryComponents.hpp
    Source/Factory/FactorySystems.cpp
    Source/Factory/FactorySystems.hpp
    Source/Factory/Conveyor.hpp
    Source/Factory/Power.hpp
    Source/Factory/PowerNetwork.hpp
    Source/Factory/Recipes.cpp
    Source/Factory/Recipes.hpp

    # --- Gameplay (parts & vessels) ----------------------------------------------
    Source/Gameplay/Parts.cpp
    Source/Gameplay/Construction.hpp
    Source/Gameplay/AeroForge.cpp
    Source/Gameplay/AeroForge.hpp
    Source/Gameplay/Blueprint.cpp
    Source/Gameplay/Blueprint.hpp
    Source/Gameplay/PartGeometry.cpp
    Source/Gameplay/PartGeometry.hpp
    Source/Gameplay/Parts.hpp
    Source/Gameplay/VesselAerodynamics.cpp
    Source/Gameplay/VesselAerodynamics.hpp

    # --- Physics ---------------------------------------------------------------
    Source/Physics/Aerodynamics.cpp
    Source/Physics/Aerodynamics.hpp
    Source/Physics/Kepler.cpp
    Source/Physics/Kepler.hpp
    Source/Physics/HullCollision.hpp
    Source/Physics/PhysicsComponents.hpp
    Source/Physics/PhysicsSystems.cpp
    Source/Physics/PhysicsSystems.hpp

    # --- Space (hierarchical star system) --------------------------------------
    Source/Space/CelestialIndex.cpp
    Source/Space/CelestialIndex.hpp
    Source/Space/CelestialSystems.cpp
    Source/Space/CelestialSystems.hpp
    Source/Space/SpaceComponents.hpp
    Source/Space/StarCatalogue.cpp
    Source/Space/StarCatalogue.hpp
    Source/Space/TrajectoryPrediction.cpp
    Source/Space/TrajectoryPrediction.hpp

    # --- Scene ---------------------------------------------------------------
    Source/Scene/Camera.cpp
    Source/Scene/Camera.hpp
    Source/Scene/FreeCameraController.cpp
    Source/Scene/FreeCameraController.hpp
    Source/Scene/TransformComponents.hpp

    # --- Assets ---------------------------------------------------------------
    Source/Assets/CgltfUsage.cpp
    Source/Assets/GltfLoader.cpp
    Source/Assets/GltfLoader.hpp
    Source/Assets/MeshData.hpp
    Source/Assets/PrimitiveFactory.cpp
    Source/Assets/PrimitiveFactory.hpp

    # --- Renderer -------------------------------------------------------------
    Source/Renderer/Mesh.cpp
    Source/Renderer/Mesh.hpp
    Source/Renderer/Renderer.cpp
    Source/Renderer/Renderer.hpp
    Source/Renderer/Vertex.hpp
    Source/Renderer/Vulkan/VmaUsage.cpp
    Source/Renderer/Vulkan/VulkanMemory.cpp
    Source/Renderer/Vulkan/VulkanMemory.hpp
    Source/Renderer/Vulkan/VulkanCommon.cpp
    Source/Renderer/Vulkan/VulkanCommon.hpp
    Source/Renderer/Vulkan/VulkanDevice.cpp
    Source/Renderer/Vulkan/VulkanDevice.hpp
    Source/Renderer/Vulkan/VulkanInstance.cpp
    Source/Renderer/Vulkan/VulkanInstance.hpp
    Source/Renderer/Vulkan/VulkanPipeline.cpp
    Source/Renderer/Vulkan/VulkanPipeline.hpp
    Source/Renderer/Vulkan/VulkanSwapchain.cpp
    Source/Renderer/Vulkan/VulkanSwapchain.hpp

    # --- UI --------------------------------------------------------------------
    Source/UI/HudFont.cpp
    Source/UI/HudFont.hpp
    Source/UI/HudOrder.hpp
    Source/UI/ScreenMarker.hpp

    # --- Public umbrella --------------------------------------------------------
    Source/Engine.hpp
)

add_library(StarWorks::Engine ALIAS StarWorksEngine)

target_include_directories(StarWorksEngine PUBLIC Source)

target_link_libraries(StarWorksEngine
    PUBLIC
        StarWorks::BuildFlags
        Vulkan::Vulkan
        glm::glm
    PRIVATE
        glfw
        GPUOpen::VulkanMemoryAllocator
        cgltf
)

# Sockets. Every other platform has them in libc; Windows keeps them in a
# separate library and will not link UdpSocket without this.
#
# ole32/oleaut32 are the firewall query (INetFwPolicy2 is a COM object) and
# shell32 is ShellExecuteEx, which is what raises the UAC prompt. Listed here
# rather than with #pragma comment(lib) so the code links under any Windows
# toolchain and not only MSVC.
if(WIN32)
    target_link_libraries(StarWorksEngine PRIVATE ws2_32 ole32 oleaut32 shell32)
endif()

set_target_properties(StarWorksEngine PROPERTIES FOLDER "Engine")

# Mirror the source tree in the Visual Studio solution explorer.
get_target_property(SW_ENGINE_SOURCES StarWorksEngine SOURCES)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/Source PREFIX "Source" FILES ${SW_ENGINE_SOURCES})
