#include "Assets/GltfLoader.hpp"

#include "Core/Error.hpp"
#include "Core/Log.hpp"

#include <glm/gtc/type_ptr.hpp>

#include <cgltf.h>

#include <cstring>

namespace sw
{
    namespace
    {
        constexpr const char* kLogCat = "Assets";

        /// Reads one float-based accessor element into up to 4 floats.
        void readAccessor(const cgltf_accessor* accessor, cgltf_size index, f32* out,
                          cgltf_size components)
        {
            cgltf_accessor_read_float(accessor, index, out, components);
        }

        void appendPrimitive(MeshData& mesh, const cgltf_primitive& primitive,
                             const Mat4& transform)
        {
            if (primitive.type != cgltf_primitive_type_triangles)
            {
                SW_LOG_WARN(kLogCat, "Skipping non-triangle glTF primitive");
                return;
            }

            const cgltf_accessor* positions = nullptr;
            const cgltf_accessor* normals = nullptr;
            const cgltf_accessor* colors = nullptr;
            const cgltf_accessor* uvs = nullptr;

            for (cgltf_size i = 0; i < primitive.attributes_count; ++i)
            {
                const cgltf_attribute& attribute = primitive.attributes[i];
                if (attribute.index != 0)
                {
                    continue; // only the first set of each attribute type
                }
                switch (attribute.type)
                {
                case cgltf_attribute_type_position: positions = attribute.data; break;
                case cgltf_attribute_type_normal:   normals = attribute.data; break;
                case cgltf_attribute_type_color:    colors = attribute.data; break;
                case cgltf_attribute_type_texcoord: uvs = attribute.data; break;
                default: break;
                }
            }

            if (positions == nullptr)
            {
                SW_THROW("glTF primitive has no POSITION attribute");
            }

            const u32 baseVertex = static_cast<u32>(mesh.vertices.size());
            const Mat3 normalMatrix = glm::inverse(glm::transpose(Mat3(transform)));

            for (cgltf_size i = 0; i < positions->count; ++i)
            {
                Vertex vertex{};

                f32 p[3] = {0, 0, 0};
                readAccessor(positions, i, p, 3);
                vertex.position = Vec3(transform * Vec4(p[0], p[1], p[2], 1.0f));

                if (normals != nullptr)
                {
                    f32 n[3] = {0, 1, 0};
                    readAccessor(normals, i, n, 3);
                    vertex.normal = glm::normalize(normalMatrix * Vec3(n[0], n[1], n[2]));
                }
                if (colors != nullptr)
                {
                    f32 c[4] = {1, 1, 1, 1};
                    readAccessor(colors, i, c, 4);
                    vertex.color = Vec4(c[0], c[1], c[2], c[3]);
                }
                if (uvs != nullptr)
                {
                    f32 t[2] = {0, 0};
                    readAccessor(uvs, i, t, 2);
                    vertex.uv = Vec2(t[0], t[1]);
                }
                mesh.vertices.push_back(vertex);
            }

            if (primitive.indices != nullptr)
            {
                for (cgltf_size i = 0; i < primitive.indices->count; ++i)
                {
                    const u32 index =
                        static_cast<u32>(cgltf_accessor_read_index(primitive.indices, i));
                    mesh.indices.push_back(baseVertex + index);
                }
            }
            else
            {
                for (cgltf_size i = 0; i < positions->count; ++i)
                {
                    mesh.indices.push_back(baseVertex + static_cast<u32>(i));
                }
            }
        }

        void appendNode(MeshData& mesh, const cgltf_node* node)
        {
            if (node->mesh != nullptr)
            {
                f32 matrix[16];
                cgltf_node_transform_world(node, matrix);
                const Mat4 transform = glm::make_mat4(matrix);

                for (cgltf_size i = 0; i < node->mesh->primitives_count; ++i)
                {
                    appendPrimitive(mesh, node->mesh->primitives[i], transform);
                }
            }
            for (cgltf_size i = 0; i < node->children_count; ++i)
            {
                appendNode(mesh, node->children[i]);
            }
        }
    } // namespace

    MeshData GltfLoader::loadMesh(const std::filesystem::path& path)
    {
        const std::string pathUtf8 = path.string();

        cgltf_options options{};
        cgltf_data* data = nullptr;

        cgltf_result result = cgltf_parse_file(&options, pathUtf8.c_str(), &data);
        if (result != cgltf_result_success)
        {
            SW_THROW("Failed to parse glTF '{}' (cgltf error {})", pathUtf8,
                     static_cast<i32>(result));
        }

        result = cgltf_load_buffers(&options, data, pathUtf8.c_str());
        if (result != cgltf_result_success)
        {
            cgltf_free(data);
            SW_THROW("Failed to load glTF buffers for '{}' (cgltf error {})", pathUtf8,
                     static_cast<i32>(result));
        }

        MeshData mesh;
        const cgltf_scene* scene =
            (data->scene != nullptr) ? data->scene
                                     : (data->scenes_count > 0 ? &data->scenes[0] : nullptr);
        if (scene != nullptr)
        {
            for (cgltf_size i = 0; i < scene->nodes_count; ++i)
            {
                appendNode(mesh, scene->nodes[i]);
            }
        }
        else
        {
            // No scene graph: import raw meshes with identity transforms.
            for (cgltf_size m = 0; m < data->meshes_count; ++m)
            {
                for (cgltf_size p = 0; p < data->meshes[m].primitives_count; ++p)
                {
                    appendPrimitive(mesh, data->meshes[m].primitives[p], Mat4{1.0f});
                }
            }
        }

        cgltf_free(data);

        if (mesh.empty())
        {
            SW_THROW("glTF '{}' contains no triangle geometry", pathUtf8);
        }

        SW_LOG_INFO(kLogCat, "Loaded glTF '{}': {} vertices, {} triangles", pathUtf8,
                    mesh.vertices.size(), mesh.indices.size() / 3);
        return mesh;
    }
} // namespace sw
