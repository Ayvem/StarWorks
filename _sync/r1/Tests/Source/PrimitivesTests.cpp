// ============================================================================
// PrimitivesTests.cpp — procedural mesh generation and composition.
// ============================================================================

#include "TestFramework.hpp"

#include <Assets/PrimitiveFactory.hpp>
#include <UI/HudFont.hpp>

using sw::MeshData;
using sw::PrimitiveFactory;

SW_TEST(BoxGeometryCounts)
{
    const MeshData box = PrimitiveFactory::makeBox({1.0f, 2.0f, 3.0f}, {1, 1, 1, 1});
    SW_CHECK_EQ(box.vertices.size(), 24u); // 6 faces * 4 (flat normals)
    SW_CHECK_EQ(box.indices.size(), 36u);  // 6 faces * 2 triangles

    // Extents must match the half sizes on every axis.
    sw::Vec3 maxCorner{0.0f};
    for (const sw::Vertex& vertex : box.vertices)
    {
        maxCorner = glm::max(maxCorner, vertex.position);
    }
    SW_CHECK(maxCorner == sw::Vec3(1.0f, 2.0f, 3.0f));

    // Every normal is unit-length and axis-aligned.
    bool normalsValid = true;
    for (const sw::Vertex& vertex : box.vertices)
    {
        normalsValid &= std::abs(glm::length(vertex.normal) - 1.0f) < 1.0e-6f;
    }
    SW_CHECK(normalsValid);
}

SW_TEST(AppendRebasesIndicesAndTranslates)
{
    MeshData compound = PrimitiveFactory::makeBox({1.0f, 1.0f, 1.0f}, {1, 1, 1, 1});
    const MeshData part = PrimitiveFactory::makeBox({0.5f, 0.5f, 0.5f}, {1, 0, 0, 1});
    PrimitiveFactory::append(compound, part, {10.0f, 0.0f, 0.0f});

    SW_CHECK_EQ(compound.vertices.size(), 48u);
    SW_CHECK_EQ(compound.indices.size(), 72u);

    // Appended indices must all point into the appended vertex range.
    bool rebased = true;
    for (sw::usize i = 36; i < compound.indices.size(); ++i)
    {
        rebased &= compound.indices[i] >= 24 && compound.indices[i] < 48;
    }
    SW_CHECK(rebased);

    // Appended vertices carry the translation.
    bool translated = true;
    for (sw::usize v = 24; v < compound.vertices.size(); ++v)
    {
        translated &= compound.vertices[v].position.x >= 9.0f;
    }
    SW_CHECK(translated);
}

SW_TEST(HudFontGlyphs)
{
    // Every charset glyph must produce quads; unsupported ones are empty.
    const char* charset = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.,-+/%:";
    for (const char* c = charset; *c != '\0'; ++c)
    {
        const MeshData glyph = sw::ui::buildGlyphMesh(*c);
        SW_CHECK(!glyph.empty());
        SW_CHECK(glyph.vertices.size() % 4 == 0); // pixel quads
        SW_CHECK(glyph.indices.size() == glyph.vertices.size() / 4 * 6);
    }
    SW_CHECK(sw::ui::buildGlyphMesh(' ').empty());
    SW_CHECK(sw::ui::buildGlyphMesh('#').empty());

    // Lowercase maps to uppercase.
    SW_CHECK(sw::ui::buildGlyphMesh('a').vertices.size() ==
             sw::ui::buildGlyphMesh('A').vertices.size());
}

SW_TEST(CapsuleGeometry)
{
    const MeshData capsule = PrimitiveFactory::makeCapsule(0.5f, 0.5f, 12, 16, {1, 1, 1, 1});
    SW_CHECK(!capsule.empty());

    // Height spans exactly radius + halfHeight on both sides.
    sw::f32 minY = 1.0e9f;
    sw::f32 maxY = -1.0e9f;
    bool normalsUnit = true;
    for (const sw::Vertex& vertex : capsule.vertices)
    {
        minY = std::min(minY, vertex.position.y);
        maxY = std::max(maxY, vertex.position.y);
        normalsUnit &= std::abs(glm::length(vertex.normal) - 1.0f) < 1.0e-4f;
    }
    SW_CHECK(std::abs(maxY - 1.0f) < 1.0e-5f);
    SW_CHECK(std::abs(minY + 1.0f) < 1.0e-5f);
    SW_CHECK(normalsUnit);
}

SW_TEST(OctahedronGeometry)
{
    const MeshData marker = PrimitiveFactory::makeOctahedron(2.0f, {1, 1, 1, 1});
    SW_CHECK_EQ(marker.vertices.size(), 6u);
    SW_CHECK_EQ(marker.indices.size(), 24u); // 8 triangles

    bool radiiValid = true;
    for (const sw::Vertex& vertex : marker.vertices)
    {
        radiiValid &= std::abs(glm::length(vertex.position) - 2.0f) < 1.0e-6f;
    }
    SW_CHECK(radiiValid);
}
