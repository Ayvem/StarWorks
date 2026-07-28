#include "Gameplay/AeroForge.hpp"

#include "Core/Log.hpp"
#include "Gameplay/PartGeometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sw::aero
{
    namespace
    {
        void appendMesh(const MeshData& mesh, std::vector<SkinTriangle>& out)
        {
            for (usize i = 0; i + 2 < mesh.indices.size(); i += 3)
            {
                const Vertex& v0 = mesh.vertices[mesh.indices[i + 0]];
                const Vertex& v1 = mesh.vertices[mesh.indices[i + 1]];
                const Vertex& v2 = mesh.vertices[mesh.indices[i + 2]];
                const Vec3 edge1 = v1.position - v0.position;
                const Vec3 edge2 = v2.position - v0.position;
                const Vec3 cross = glm::cross(edge1, edge2);
                const f32 length = glm::length(cross);
                if (length < 1.0e-12f)
                {
                    continue; // degenerate: no area, no air
                }
                Vec3 normal = cross / length;
                // The winding is the authority, but a shape generator that
                // got one cap backwards would silently invert its pressure.
                // The shaded normals agree with the outside of the solid, so
                // they arbitrate.
                const Vec3 shaded = v0.normal + v1.normal + v2.normal;
                if (glm::dot(normal, shaded) < 0.0f)
                {
                    normal = -normal;
                }
                out.push_back(SkinTriangle{v0.position, v1.position, v2.position, normal});
            }
        }

        [[nodiscard]] Vec3 perpendicular(const Vec3& axis)
        {
            const Vec3 reference =
                (std::abs(axis.y) < 0.9f) ? Vec3(0.0f, 1.0f, 0.0f) : Vec3(1.0f, 0.0f, 0.0f);
            return glm::normalize(glm::cross(reference, axis));
        }
    } // namespace

    std::vector<SkinTriangle> partSkin(const parts::PartDefinition& definition)
    {
        std::vector<SkinTriangle> skin;
        const MeshData visible = parts::buildPartMesh(definition);
        if (!visible.empty())
        {
            appendMesh(visible, skin);
            return skin;
        }
        for (const parts::PartShape& shape : definition.shapes)
        {
            appendMesh(parts::buildShapeMesh(shape), skin);
        }
        return skin;
    }

    SolvedDirection solveDirection(std::span<const SkinTriangle> skin,
                                   const Vec3& flowDirection, const ForgeSettings& settings)
    {
        SolvedDirection result{};
        if (skin.empty() || settings.resolution < 4)
        {
            return result;
        }
        const Vec3 flow = glm::normalize(flowDirection);
        const Vec3 axisU = perpendicular(flow);
        const Vec3 axisV = glm::cross(flow, axisU);

        // Bounds of the part in the wind's own frame.
        f32 minU = std::numeric_limits<f32>::max();
        f32 maxU = std::numeric_limits<f32>::lowest();
        f32 minV = minU;
        f32 maxV = maxU;
        for (const SkinTriangle& triangle : skin)
        {
            for (const Vec3& point : {triangle.a, triangle.b, triangle.c})
            {
                const f32 u = glm::dot(point, axisU);
                const f32 v = glm::dot(point, axisV);
                minU = std::min(minU, u);
                maxU = std::max(maxU, u);
                minV = std::min(minV, v);
                maxV = std::max(maxV, v);
            }
        }
        // A hair of margin so a face exactly on the boundary still lands
        // inside a pixel rather than half outside the grid.
        const f32 spanU = std::max(maxU - minU, 1.0e-4f) * 1.02f;
        const f32 spanV = std::max(maxV - minV, 1.0e-4f) * 1.02f;
        const f32 centreU = (minU + maxU) * 0.5f;
        const f32 centreV = (minV + maxV) * 0.5f;
        minU = centreU - spanU * 0.5f;
        minV = centreV - spanV * 0.5f;

        const i32 resolution = static_cast<i32>(settings.resolution);
        const f32 pixelU = spanU / static_cast<f32>(resolution);
        const f32 pixelV = spanV / static_cast<f32>(resolution);
        const f64 pixelArea = static_cast<f64>(pixelU) * static_cast<f64>(pixelV);

        // Two depth buffers: the surface the air meets FIRST (pressure) and
        // the one it leaves LAST (base suction). Everything between them is
        // interior and feels nothing, which is the self-occlusion.
        const usize pixelCount = static_cast<usize>(resolution) * resolution;
        constexpr f32 kNoHit = std::numeric_limits<f32>::max();
        std::vector<f32> frontDepth(pixelCount, kNoHit);
        std::vector<f32> backDepth(pixelCount, -kNoHit);
        std::vector<i32> frontTriangle(pixelCount, -1);
        std::vector<i32> backTriangle(pixelCount, -1);

        for (usize index = 0; index < skin.size(); ++index)
        {
            const SkinTriangle& triangle = skin[index];
            const Vec3 p0(glm::dot(triangle.a, axisU), glm::dot(triangle.a, axisV),
                          glm::dot(triangle.a, flow));
            const Vec3 p1(glm::dot(triangle.b, axisU), glm::dot(triangle.b, axisV),
                          glm::dot(triangle.b, flow));
            const Vec3 p2(glm::dot(triangle.c, axisU), glm::dot(triangle.c, axisV),
                          glm::dot(triangle.c, flow));

            const f32 area = (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y);
            if (std::abs(area) < 1.0e-12f)
            {
                continue; // edge-on to the wind: no projected area
            }
            const f32 inverseArea = 1.0f / area;

            const i32 x0 = std::max(0, static_cast<i32>(std::floor(
                                           (std::min({p0.x, p1.x, p2.x}) - minU) / pixelU)));
            const i32 x1 = std::min(resolution - 1,
                                    static_cast<i32>(std::ceil(
                                        (std::max({p0.x, p1.x, p2.x}) - minU) / pixelU)));
            const i32 y0 = std::max(0, static_cast<i32>(std::floor(
                                           (std::min({p0.y, p1.y, p2.y}) - minV) / pixelV)));
            const i32 y1 = std::min(resolution - 1,
                                    static_cast<i32>(std::ceil(
                                        (std::max({p0.y, p1.y, p2.y}) - minV) / pixelV)));

            for (i32 y = y0; y <= y1; ++y)
            {
                const f32 sampleV = minV + (static_cast<f32>(y) + 0.5f) * pixelV;
                for (i32 x = x0; x <= x1; ++x)
                {
                    const f32 sampleU = minU + (static_cast<f32>(x) + 0.5f) * pixelU;
                    const f32 w0 = ((p1.x - sampleU) * (p2.y - sampleV) -
                                    (p2.x - sampleU) * (p1.y - sampleV)) * inverseArea;
                    const f32 w1 = ((p2.x - sampleU) * (p0.y - sampleV) -
                                    (p0.x - sampleU) * (p2.y - sampleV)) * inverseArea;
                    const f32 w2 = 1.0f - w0 - w1;
                    if (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f)
                    {
                        continue;
                    }
                    const f32 depth = w0 * p0.z + w1 * p1.z + w2 * p2.z;
                    const usize pixel = static_cast<usize>(y) * resolution + x;
                    if (depth < frontDepth[pixel])
                    {
                        frontDepth[pixel] = depth;
                        frontTriangle[pixel] = static_cast<i32>(index);
                    }
                    if (depth > backDepth[pixel])
                    {
                        backDepth[pixel] = depth;
                        backTriangle[pixel] = static_cast<i32>(index);
                    }
                }
            }
        }

        // ---- integrate ------------------------------------------------------
        //
        // Both faces obey the same law. For a surface element with outward
        // normal n and projected area dA, the true area is dA / |n.w|, and
        // the pressure coefficient tapers as the square of the same cosine.
        // The two cancel to ONE factor of the cosine, which is why neither
        // term blows up on a surface lying nearly along the flow.
        //
        //   front  c  = -n.w > 0   dF = Cp_max * c  * (-n) * dA
        //   back   cb =  n.w > 0   dF = |Cp_base| * cb *  n  * dA
        //
        // and the moment is the same thing crossed with where it happened.
        Vec3 force(0.0f);
        Vec3 moment(0.0f);
        f64 covered = 0.0;
        const f64 baseCp = std::abs(settings.basePressureCp);

        const auto accumulate = [&](usize pixel, i32 triangleIndex, f32 depth, bool front) {
            if (triangleIndex < 0)
            {
                return;
            }
            const SkinTriangle& triangle = skin[static_cast<usize>(triangleIndex)];
            const f32 cosine = front ? -glm::dot(triangle.normal, flow)
                                     : glm::dot(triangle.normal, flow);
            if (cosine <= 1.0e-4f)
            {
                return;
            }
            const i32 x = static_cast<i32>(pixel % static_cast<usize>(resolution));
            const i32 y = static_cast<i32>(pixel / static_cast<usize>(resolution));
            const Vec3 point = axisU * (minU + (static_cast<f32>(x) + 0.5f) * pixelU) +
                               axisV * (minV + (static_cast<f32>(y) + 0.5f) * pixelV) +
                               flow * depth;

            // THE TWO PRESSURE LAWS, on one axis and in one line.
            //
            //   impact / base   Cp = Cp_max sin^2(d), and its wake mirror.
            //                   Both act along -n on the windward face and
            //                   +n on the leeward one; dividing the true
            //                   area by the same cosine leaves ONE factor of
            //                   it, so neither blows up at grazing.
            //   linear          Cp = k sin(d), tapered off once the flow can
            //                   no longer stay attached. Its area factor
            //                   cancels the cosine completely, which is why
            //                   it survives to shallow angles where the
            //                   impact term has already vanished — and why a
            //                   fin lifts.
            //
            // Both push the same way, so they are one coefficient.
            f64 coefficient = front ? settings.stagnationCp * cosine : baseCp * cosine;
            if (settings.liftingSurface && settings.liftSlope > 0.0 &&
                cosine < settings.separatedSine)
            {
                f64 attachment = 1.0;
                if (cosine > settings.attachedSine)
                {
                    const f64 span =
                        std::max(settings.separatedSine - settings.attachedSine, 1.0e-6);
                    const f64 t = (cosine - settings.attachedSine) / span;
                    attachment = 1.0 - t * t * (3.0 - 2.0 * t); // smooth, flat at both ends
                }
                coefficient += settings.liftSlope * attachment;
            }
            const Vec3 direction = front ? -triangle.normal : triangle.normal;
            Vec3 contribution = direction * static_cast<f32>(coefficient * pixelArea);

            if (front && settings.skinFrictionCf > 0.0)
            {
                // Shear acts along the flow projected into the surface.
                // The wetted area is the projected one divided by the same
                // cosine, clamped: a surface exactly edge-on to the wind is
                // infinitely long in projection and is not infinitely
                // draggy.
                const Vec3 tangent = flow - triangle.normal * glm::dot(flow, triangle.normal);
                const f32 tangentLength = glm::length(tangent);
                if (tangentLength > 1.0e-5f)
                {
                    const f64 wetted = pixelArea / std::max<f64>(cosine, 0.2);
                    contribution += (tangent / tangentLength) *
                                    static_cast<f32>(settings.skinFrictionCf * wetted);
                }
            }
            force += contribution;
            moment += glm::cross(point, contribution);
        };

        for (usize pixel = 0; pixel < pixelCount; ++pixel)
        {
            if (frontTriangle[pixel] >= 0)
            {
                covered += pixelArea;
                accumulate(pixel, frontTriangle[pixel], frontDepth[pixel], true);
            }
            // The rearmost surface, whatever it is. When a pixel crosses
            // only ONE triangle (an unclosed sheet) the same triangle is
            // both buffers' answer — and only one of the two cosines can be
            // positive, so it collects exactly the term it deserves.
            accumulate(pixel, backTriangle[pixel], backDepth[pixel], false);
        }

        result.forceM2 = force;
        result.momentM3 = moment;
        result.projectedAreaM2 = covered;
        return result;
    }

    AeroTable forgePart(const parts::PartDefinition& definition,
                        const ForgeSettings& settings)
    {
        AeroTable table{};
        table.partId = definition.id;
        table.partName = definition.name;
        table.thetaCount = std::max(2u, settings.thetaCount);
        table.phiCount = std::max(1u, settings.phiCount);

        const std::vector<SkinTriangle> skin = partSkin(definition);
        if (skin.empty())
        {
            SW_LOG_WARN("AeroForge", "Part {} ('{}') has no geometry", definition.id,
                        definition.name);
            return table;
        }

        f32 reach = 0.0f;
        Vec3 minimum(std::numeric_limits<f32>::max());
        Vec3 maximum(std::numeric_limits<f32>::lowest());
        for (const SkinTriangle& triangle : skin)
        {
            reach = std::max({reach, glm::length(triangle.a), glm::length(triangle.b),
                              glm::length(triangle.c)});
            for (const Vec3& point : {triangle.a, triangle.b, triangle.c})
            {
                minimum = glm::min(minimum, point);
                maximum = glm::max(maximum, point);
            }
        }
        table.referenceLengthM = std::max(0.1, static_cast<f64>(reach));

        // WING OR BODY, decided once, from the part's own proportions.
        const Vec3 extents = maximum - minimum;
        const f32 thinnest = std::min({extents.x, extents.y, extents.z});
        const f32 longest = std::max({extents.x, extents.y, extents.z});
        ForgeSettings resolved = settings;
        resolved.liftingSurface =
            (longest > 1.0e-4f) &&
            (static_cast<f64>(thinnest / longest) < settings.plateRatio);

        table.samples.resize(static_cast<usize>(table.thetaCount) * table.phiCount);
        const f32 thetaStep = math::kPi / static_cast<f32>(table.thetaCount - 1);
        const f32 phiStep = 2.0f * math::kPi / static_cast<f32>(table.phiCount);

        for (u32 t = 0; t < table.thetaCount; ++t)
        {
            const f32 theta = thetaStep * static_cast<f32>(t);
            for (u32 p = 0; p < table.phiCount; ++p)
            {
                const f32 phi = phiStep * static_cast<f32>(p);
                const Vec3 flow(std::sin(theta) * std::cos(phi),
                                std::sin(theta) * std::sin(phi), std::cos(theta));
                const SolvedDirection solved = solveDirection(skin, flow, resolved);
                AeroSample& out = table.samples[static_cast<usize>(t) * table.phiCount + p];
                out.forceM2 = solved.forceM2;
                out.momentM3 = solved.momentM3;
                table.maxAreaM2 = std::max(table.maxAreaM2, solved.projectedAreaM2);
            }
        }
        return table;
    }

    std::filesystem::path aeroPathFor(const std::filesystem::path& partPath)
    {
        std::filesystem::path out = partPath;
        out.replace_extension();
        out += ".aero.json";
        return out;
    }
} // namespace sw::aero
