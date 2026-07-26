#pragma once

// ============================================================================
// Assets/GltfLoader.hpp
// Minimal glTF 2.0 mesh import (positions, normals, vertex colors, UVs,
// indices) via cgltf. First step of the Assets pipeline: full scene import,
// materials, textures and async streaming follow in later milestones —
// behind this same interface.
// ============================================================================

#include "Assets/MeshData.hpp"

#include <filesystem>

namespace sw
{
    class GltfLoader
    {
    public:
        GltfLoader() = delete;

        /// Loads every primitive of every mesh in the file, merged into a
        /// single MeshData (node transforms applied). Supports .gltf and .glb.
        /// Throws sw::Exception on parse errors or missing position data.
        [[nodiscard]] static MeshData loadMesh(const std::filesystem::path& path);
    };
} // namespace sw
