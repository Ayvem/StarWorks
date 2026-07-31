// ============================================================================
// GameTerrain.cpp — The walkable terrain patch and the grass field baked onto it.
// Split out of StarWorksGame.cpp; same class, one theme per translation unit.
// ============================================================================

#include "StarWorksGame.hpp"

#include "GameInternal.hpp"
#include "Systems.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <format>
#include <limits>

namespace game
{

    void StarWorksGame::updateTerrainPatch()
    {
        // ---- 1. land a finished build -------------------------------------
        // The job wrote into the pending mesh; nothing on screen referenced
        // it, so the upload is a plain buffer creation with no device idle.
        if (m_terrainJob.load(std::memory_order_acquire) == TerrainJob::Ready)
        {
            // A BUILD THAT PRODUCED NOTHING IS NOT A BUILD TO UPLOAD. The
            // job refuses to hand over a mesh it found a fault in, and a
            // zero-vertex buffer is its own kind of crash — so the landing
            // pad checks rather than assumes, and the patch already on
            // screen simply stays there.
            if (m_terrainPendingMesh.empty())
            {
                m_terrainJob.store(TerrainJob::Idle, std::memory_order_release);
                return;
            }
            if (m_terrainMeshSlots[0] == 0xFFFFFFFFu)
            {
                m_terrainMeshSlots[0] =
                    registerMesh(renderer().createMesh(m_terrainPendingMesh));
                m_terrainMeshSlots[1] =
                    registerMesh(renderer().createMesh(m_terrainPendingMesh));
                m_terrainSlotIndex = 0;
            }
            else
            {
                // DOUBLE BUFFERED: the buffers being destroyed here belong to
                // the build BEFORE last, seconds old and long out of flight.
                // (Replacing the mesh drawn last frame is what used to force
                // a full renderer().waitIdle() — a guaranteed pipeline bubble
                // on every rebuild.)
                m_terrainSlotIndex ^= 1u;
                m_meshes[m_terrainMeshSlots[m_terrainSlotIndex]] =
                    renderer().createMesh(m_terrainPendingMesh);
            }
            m_terrainMeshSlot = m_terrainMeshSlots[m_terrainSlotIndex];
            // KEEP THE GROUND GRID. It is what lets the grass be re-centred
            // without rebuilding the terrain — and, more importantly, what
            // makes the field stand on exactly the surface that is DRAWN.
            // A second sampling of the heightfield would put it up to half a
            // metre out, which on a 0.6 m blade is half the blade.
            m_terrainGridCells = m_terrainPendingCells;
            m_terrainGridEast = m_terrainPendingEast;
            m_terrainGridNorth = m_terrainPendingNorth;
            {
                const sw::usize gridVertices =
                    static_cast<sw::usize>(m_terrainGridCells + 1) *
                    (m_terrainGridCells + 1);
                m_terrainGridVertices.assign(
                    m_terrainPendingMesh.vertices.begin(),
                    m_terrainPendingMesh.vertices.begin() +
                        static_cast<std::ptrdiff_t>(
                            std::min(gridVertices, m_terrainPendingMesh.vertices.size())));
            }
            m_grassCenterDir = sw::Vec3(0.0f); // force the field to re-seed
            m_terrainOriginLocal = m_terrainPendingOrigin;
            m_terrainCenterDir = m_terrainPendingCenterDir;
            m_terrainExtent = m_terrainPendingExtent;
            m_terrainBody = m_terrainPendingBody;
            m_terrainPendingMesh = sw::MeshData{}; // release the CPU copy
            m_terrainJob.store(TerrainJob::Idle, std::memory_order_release);
        }

        const bool wasVisible = m_terrainVisible;
        m_terrainVisible = false;
        const sw::i32 primaryIndex = controlledPrimaryIndex();
        if (primaryIndex < 0)
        {
            return;
        }
        const auto& primary =
            m_celestialIndex.body(static_cast<sw::usize>(primaryIndex));
        const auto* terrain =
            m_world.tryGetComponent<sw::planet::TerrainComponent>(primary.entity);
        const auto* bodyTransform =
            m_world.tryGetComponent<TransformComponent>(primary.entity);
        if (terrain == nullptr || bodyTransform == nullptr)
        {
            return;
        }
        const auto& craft = m_world.getComponent<TransformComponent>(controlledEntity());
        const sw::WorldVec3 radial = craft.position - bodyTransform->position;
        const sw::f64 distance = glm::length(radial);
        const sw::f64 seaAltitude = distance - primary.bodyRadius;
        if (distance <= 1.0 || seaAltitude > 1.2e5)
        {
            // Terrain detail only matters below ~120 km. Above that the
            // globe's own per-fragment surface is what you see, and building
            // a patch of analytic heightfield bought nothing but a hitch.
            return;
        }
        m_terrainVisible = m_terrainMeshSlot != 0xFFFFFFFFu;

        if (m_terrainJob.load(std::memory_order_acquire) != TerrainJob::Idle)
        {
            return; // a build is already in flight; keep showing the current one
        }

        // Patch center: the body-frame direction under the craft. Extent
        // scales with altitude — a wide, coarse patch from high up, a
        // tight, dense one near the ground. That scaling IS the LOD.
        // The body frame, from the body's f64 spin state. An f32 quaternion
        // slides this direction by ~1e-7 rad, which at 6,371 km is 0.6 m of
        // ground — enough to move the patch under your feet every frame.
        const auto* bodySpin =
            m_world.tryGetComponent<sw::phys::GravitySourceComponent>(primary.entity);
        const glm::dquat inverseRotation =
            (bodySpin != nullptr) ? glm::inverse(sw::phys::spinRotation(*bodySpin))
                                  : glm::inverse(glm::dquat(bodyTransform->rotation));
        const sw::Vec3 centerDir =
            sw::Vec3(glm::normalize(inverseRotation * (radial / distance)));
        // HOW HIGH ABOVE THE GROUND — not above the sea.
        //
        // This is the bug that buried the rocket. The patch's resolution is
        // chosen from your altitude, and it was measuring that from SEA
        // LEVEL: standing on an 1,100 m plateau, the game sized the patch
        // for somebody flying at 1,100 m and drew a 6.6 km square in 137 m
        // cells. A mesh that coarse cannot follow a creased fractal, and
        // measured against the collider the drawn ground was off by up to
        // 7.7 m — which is how a 2.4 m rocket ends up half-submerged in a
        // hillside it is, as far as physics is concerned, resting neatly on
        // top of. Terra's terrain reaches 9 km; almost nowhere interesting
        // is at sea level.
        const sw::f64 groundAltitude =
            distance -
            (primary.bodyRadius +
             sw::planet::terrainElevation(*terrain, centerDir));
        // Down to 1.5 km at landing: with 192 cells that is a 15.6 m grid,
        // fine enough for the gullies and benches the 16-octave heightfield
        // carries. (It used to bottom out at 4 km / 125 m cells, which could
        // not show anything smaller than a hill.)
        const sw::f64 extent =
            std::clamp(std::max(groundAltitude, 250.0) * 6.0, 1.5e3, 4.0e5);

        const sw::f64 now = clock().totalSeconds();
        // HOW FAR YOU MAY WALK BEFORE THE PATCH FOLLOWS YOU.
        //
        // Thirty per cent of the extent — 450 m at landing scale — is the
        // right answer for the GROUND, whose vertices are anchored to the
        // planet and therefore identical before and after a re-centre: you
        // cannot see that rebuild happen at all. It is the wrong answer by
        // an order of magnitude for the FIELD standing on it, which only
        // exists within a disc around the patch centre. Walk out of that
        // disc and there is no grass; wait for the patch to re-centre and a
        // whole new field arrives in one frame while the old one leaves in
        // the same one.
        //
        // So a patch that carries plants follows the player at the scale of
        // the field rather than the scale of the terrain. The ground pays
        // for it — a 60 ms rebuild every 40 m instead of every 450 m, which
        // walking is one every ten seconds — and pays it on a worker thread,
        // against a patch already on screen, at most once a second.
        // The grass has its own clock now (see updateGrassField), so the
        // ground is free to go back to the threshold that suits it: its
        // vertices are planet-anchored and a re-centre is invisible.
        const sw::f64 followDistance = extent * 0.30;
        const bool moved =
            static_cast<sw::f64>(glm::distance(centerDir, m_terrainCenterDir)) *
                primary.bodyRadius >
            followDistance;
        const bool rescaled =
            extent > m_terrainExtent * 1.8 || extent < m_terrainExtent * 0.55;
        const bool needRebuild = m_terrainMeshSlot == 0xFFFFFFFFu ||
                                 m_terrainBody != primary.entity || moved || rescaled;
        // Rebuild cadence scales with altitude: near the ground the patch is
        // small and the craft crosses it quickly, high up it spans hundreds
        // of kilometres and nothing moves relative to it.
        const sw::f64 rebuildInterval = std::clamp(seaAltitude / 12000.0, 1.0, 8.0);
        if (!needRebuild ||
            (wasVisible && now - m_lastTerrainRebuildSeconds < rebuildInterval))
        {
            return; // keep showing the current patch
        }
        m_lastTerrainRebuildSeconds = now;

        // The body's palette style (Terra/Luna/Mars) comes from its LOD
        // component — the patch is a close-up of that same world.
        sw::i32 surfaceStyle = 0;
        if (const auto* bodyLod =
                m_world.tryGetComponent<CelestialLodComponent>(primary.entity);
            bodyLod != nullptr && bodyLod->surfaceStyle >= 0)
        {
            surfaceStyle = bodyLod->surfaceStyle;
        }

        // WHERE THE SUN IS, in the body's own frame — because the relief
        // shading below is BAKED, and a baked shadow has to be baked toward
        // something. Terra turns 0.004 degrees in the second between two
        // rebuilds, so a shadow map that is one rebuild old is a shadow map
        // that is right.
        sw::Vec3 sunDirBody{0.0f, 1.0f, 0.0f};
        if (const auto* sunTransform =
                m_world.tryGetComponent<TransformComponent>(m_solEntity))
        {
            const sw::WorldVec3 toSun = sunTransform->position - bodyTransform->position;
            if (glm::length(toSun) > 1.0)
            {
                sunDirBody =
                    sw::Vec3(glm::normalize(inverseRotation * glm::normalize(toSun)));
            }
        }

        // Everything the build needs is captured BY VALUE: the job never
        // touches the world, the renderer or any component.
        const sw::planet::TerrainComponent terrainCopy = *terrain;
        const sw::f64 radius = primary.bodyRadius;
        m_terrainPendingBody = primary.entity;
        m_terrainJob.store(TerrainJob::Running, std::memory_order_release);
        // CELLS ARE NOT A CONSTANT, because what matters is the cell SIZE
        // where somebody is standing. Measured against the collider at
        // landing extent, going from 96 cells to 192 takes the worst gap
        // between the drawn ground and the ground you stand on from 1.25 m
        // down to 0.50 m — and nothing at four hundred kilometres up cares
        // either way, so the big patches keep the cheap grid.
        const sw::u32 cells = (extent <= 2.5e3) ? 192u
                              : (extent <= 2.5e4) ? 128u
                                                  : 96u;
        threadPool().submit([this, terrainCopy, surfaceStyle, centerDir, extent, radius,
                             cells, sunDirBody]() {
            buildTerrainPatch(terrainCopy, surfaceStyle, centerDir, extent, radius,
                              cells, sunDirBody);
            m_terrainJob.store(TerrainJob::Ready, std::memory_order_release);
        });
    }

    void StarWorksGame::buildTerrainPatch(const sw::planet::TerrainComponent& terrain,
                                          sw::i32 surfaceStyle,
                                          const sw::Vec3& centerDir, sw::f64 extent,
                                          sw::f64 radius, sw::u32 cellCount,
                                          const sw::Vec3& sunDirBody)
    {
        // ---- the grid: a tangent plane projected onto the sphere ----------
        const sw::u32 kCells = std::clamp(cellCount, 16u, 256u);
        const sw::u32 kVerts = kCells + 1;
        const sw::Vec3 reference =
            (std::abs(centerDir.y) < 0.99f) ? sw::Vec3{0, 1, 0} : sw::Vec3{1, 0, 0};
        const sw::Vec3 east = glm::normalize(glm::cross(reference, centerDir));
        const sw::Vec3 north = glm::cross(centerDir, east);
        const sw::WorldVec3 origin =
            sw::WorldVec3(centerDir) *
            (radius + sw::planet::terrainElevation(terrain, centerDir));

        // The grid cell size decides how much relief detail is even
        // representable here: sampling sixteen octaves onto a mesh whose
        // cells are kilometres wide only aliases between vertices, and costs
        // the full price of the heightfield at every one of the ~9,400
        // points. Near the ground the cells are 31 m and the count saturates
        // at the body's full stack — which is exactly where it should.
        const sw::f64 cellMetres = 2.0 * extent / kCells;
        // Same convention as the globe LODs: the finest noise frequency a
        // mesh of this spacing can carry, in cycles per radian.
        const sw::f32 patchFrequencyLimit =
            static_cast<sw::f32>(radius / (2.0 * std::max(cellMetres, 1.0e-3)));
        sw::i32 patchOctaves = terrain.reliefOctaves;
        // Octaves are kept until their wavelength drops to a QUARTER of a cell,
        // not twice one. The looser rule saved a couple of samples and cost
        // something worse: collision samples the FULL stack, so the drawn
        // ground sat up to three metres below the ground the lander stood
        // on. At landing extent this now keeps the whole stack — the patch
        // and the collider are the same surface again, to the centimetre.
        while (patchOctaves > 3 &&
               radius / (static_cast<sw::f64>(terrain.frequency) *
                         terrain.reliefFrequency * std::pow(2.07, patchOctaves - 1)) <
                   cellMetres * 0.25)
        {
            --patchOctaves;
        }

        std::vector<sw::WorldVec3> points(kVerts * kVerts);
        std::vector<sw::f32> elevations(kVerts * kVerts);
        /// The SOLID ground height at each vertex — sea level clamped in,
        /// negative sea floor clamped out. This is the surface the relief
        /// shading marches over, and it is the same array the geometry was
        /// built from, so a shadow can never fall on ground that is not
        /// drawn where the shadow says it is.
        std::vector<sw::f32> ground(kVerts * kVerts);
        for (sw::u32 j = 0; j < kVerts; ++j)
        {
            for (sw::u32 i = 0; i < kVerts; ++i)
            {
                const sw::f64 u =
                    (static_cast<sw::f64>(i) / kCells * 2.0 - 1.0) * extent;
                const sw::f64 v =
                    (static_cast<sw::f64>(j) / kCells * 2.0 - 1.0) * extent;
                const sw::WorldVec3 raw = sw::WorldVec3(centerDir) * radius +
                                          sw::WorldVec3(east) * u +
                                          sw::WorldVec3(north) * v;
                const sw::WorldVec3 dir = glm::normalize(raw);
                // SIGNED elevation: the geometry clamps at sea level (the
                // sphere IS the water surface, exactly as collision sees it)
                // but the stored value keeps the negative sea floor, which
                // is what colors deep water.
                const sw::f32 signedElevation = sw::planet::terrainElevationSignedLod(
                    terrain, sw::Vec3(dir), patchOctaves);
                const sw::f64 elevation =
                    (signedElevation > 0.0f) ? static_cast<sw::f64>(signedElevation)
                                             : 0.0;
                points[j * kVerts + i] = dir * (radius + elevation) - origin;
                elevations[j * kVerts + i] = signedElevation;
                ground[j * kVerts + i] = static_cast<sw::f32>(elevation);
            }
        }

        // ---- RELIEF SHADING, baked -----------------------------------------
        //
        // The patch's triangles are 15 m across and its normals are honest,
        // but a lambert term alone leaves rolling ground looking like a
        // painted sheet: nothing casts, nothing pools, and a dip and a rise
        // of the same slope are the same colour. Two terms fix that, and
        // both are computed on the height grid that is already in hand — no
        // extra heightfield evaluations, and no possibility of disagreeing
        // with the surface being drawn.
        //
        //   CAST SHADOW  march toward the sun, one cell at a time, and see
        //                whether the ground ever rises above the ray. This
        //                is what puts a hill's shadow on the valley beside
        //                it, and it is the term that makes a landscape read
        //                as a landscape at low sun.
        //   SKY OCCLUSION  the same march in six directions with no sun in
        //                it: how much of the sky this point can see. Gullies
        //                and the insides of craters darken; ridges do not.
        //
        // Both stay deliberately gentle. This is ground the player has to
        // land on and walk over, not a photograph.
        const sw::f64 cellSpacing = 2.0 * extent / kCells;
        std::vector<sw::f32> shading(kVerts * kVerts, 1.0f);
        {
            const sw::f32 sunUp = glm::dot(sunDirBody, centerDir);
            const sw::Vec3 sunTangent = sunDirBody - centerDir * sunUp;
            const sw::f32 sunTangentLength = glm::length(sunTangent);
            // Direction the shadow ray walks, in GRID cells.
            sw::f32 sunU = 0.0f;
            sw::f32 sunV = 0.0f;
            if (sunTangentLength > 1.0e-5f)
            {
                sunU = glm::dot(sunTangent, east) / sunTangentLength;
                sunV = glm::dot(sunTangent, north) / sunTangentLength;
            }
            // Metres of climb per metre walked toward the sun. A sun on the
            // horizon casts shadows to infinity; cap the slope so the march
            // stays finite and the terminator stays soft.
            const sw::f32 sunSlope =
                (sunTangentLength > 1.0e-5f)
                    ? glm::clamp(sunUp / sunTangentLength, -8.0f, 8.0f)
                    : 8.0f;

            const auto heightAt = [&](sw::i32 i, sw::i32 j) {
                const sw::i32 ci = glm::clamp(i, 0, static_cast<sw::i32>(kCells));
                const sw::i32 cj = glm::clamp(j, 0, static_cast<sw::i32>(kCells));
                return ground[static_cast<sw::usize>(cj) * kVerts + ci];
            };

            constexpr sw::i32 kShadowSteps = 20;
            constexpr sw::i32 kSkySteps = 6;
            // Six azimuths, fixed: enough to tell a hollow from a shoulder,
            // cheap enough to run on every vertex of a 192-cell grid.
            constexpr sw::f32 kSkyDirs[6][2] = {{1.0f, 0.0f},   {0.5f, 0.866f},
                                                {-0.5f, 0.866f}, {-1.0f, 0.0f},
                                                {-0.5f, -0.866f}, {0.5f, -0.866f}};

            for (sw::u32 j = 0; j < kVerts; ++j)
            {
                for (sw::u32 i = 0; i < kVerts; ++i)
                {
                    const sw::usize index = static_cast<sw::usize>(j) * kVerts + i;
                    const sw::f32 base = ground[index];

                    // ---- cast shadow ----------------------------------
                    sw::f32 blocked = 0.0f;
                    if (sunUp > -0.05f)
                    {
                        for (sw::i32 step = 1; step <= kShadowSteps; ++step)
                        {
                            const sw::f32 walk =
                                static_cast<sw::f32>(step) *
                                static_cast<sw::f32>(cellSpacing);
                            const sw::f32 rayHeight = base + walk * sunSlope;
                            const sw::f32 terrainHeight = heightAt(
                                static_cast<sw::i32>(i) +
                                    static_cast<sw::i32>(std::lround(sunU * step)),
                                static_cast<sw::i32>(j) +
                                    static_cast<sw::i32>(std::lround(sunV * step)));
                            // Softened by how far the blocker is: a ridge at
                            // the end of the march throws a vaguer shadow
                            // than the boulder at your feet, which is both
                            // true and what keeps the term from banding.
                            const sw::f32 over = terrainHeight - rayHeight;
                            if (over > 0.0f)
                            {
                                const sw::f32 softness =
                                    2.0f + 0.35f * static_cast<sw::f32>(step) *
                                               static_cast<sw::f32>(cellSpacing);
                                blocked = std::max(blocked,
                                                   glm::clamp(over / softness, 0.0f, 1.0f));
                            }
                        }
                    }
                    else
                    {
                        blocked = 1.0f; // the sun is under this horizon
                    }

                    // ---- sky occlusion ---------------------------------
                    sw::f32 openness = 0.0f;
                    for (const auto& direction : kSkyDirs)
                    {
                        sw::f32 highest = 0.0f; // tangent of the horizon angle
                        for (sw::i32 step = 1; step <= kSkySteps; ++step)
                        {
                            const sw::f32 walk =
                                static_cast<sw::f32>(step) *
                                static_cast<sw::f32>(cellSpacing);
                            const sw::f32 rise =
                                heightAt(static_cast<sw::i32>(i) +
                                             static_cast<sw::i32>(
                                                 std::lround(direction[0] * step)),
                                         static_cast<sw::i32>(j) +
                                             static_cast<sw::i32>(
                                                 std::lround(direction[1] * step))) -
                                base;
                            highest = std::max(highest, rise / walk);
                        }
                        // cos of the horizon angle: 1 = open sky, 0 = a wall.
                        openness += 1.0f / std::sqrt(1.0f + highest * highest);
                    }
                    openness /= 6.0f;

                    // Measured on Terra's roughest ground the term runs
                    // 0.50 .. 1.00; on the launch plain it is flat at 1.00,
                    // because that ground really is flat at fifteen metres
                    // and honest shading of flat ground is no shading. What
                    // makes the plain read is the field standing on it.
                    const sw::f32 sunTerm = 1.0f - 0.55f * blocked;
                    const sw::f32 skyTerm = 0.45f + 0.55f * openness;
                    shading[index] = glm::clamp(sunTerm * skyTerm, 0.22f, 1.0f);
                }
            }
        }

        sw::MeshData mesh;
        // Room for the ground, its rim skirt and a full field of plants, so
        // appending never has to move what is already there. Bounded and
        // stated, because everything below appends to this vector.
        mesh.vertices.reserve(static_cast<sw::usize>(kVerts) * kVerts + 16u * kCells +
                              96000u);
        mesh.vertices.resize(static_cast<sw::usize>(kVerts) * kVerts);
        const auto style = static_cast<SurfaceStyle>(surfaceStyle);
        for (sw::u32 j = 0; j < kVerts; ++j)
        {
            for (sw::u32 i = 0; i < kVerts; ++i)
            {
                const sw::usize index = j * kVerts + i;
                sw::Vertex& vertex = mesh.vertices[index];
                vertex.position = sw::Vec3(points[index]);

                // Finite-difference normal (real slope shading).
                const sw::u32 iPrev = (i > 0) ? i - 1 : i;
                const sw::u32 iNext = (i < kCells) ? i + 1 : i;
                const sw::u32 jPrev = (j > 0) ? j - 1 : j;
                const sw::u32 jNext = (j < kCells) ? j + 1 : j;
                const sw::Vec3 dx =
                    sw::Vec3(points[j * kVerts + iNext] - points[j * kVerts + iPrev]);
                const sw::Vec3 dy =
                    sw::Vec3(points[jNext * kVerts + i] - points[jPrev * kVerts + i]);
                vertex.normal = glm::normalize(glm::cross(dx, dy));

                // Palette: the SAME function the globe LODs and the fragment
                // shader use — walking down from orbit never crosses a color
                // seam. The slope comes straight from the finite-difference
                // normal we just built (tan of the angle to the local
                // vertical), so cliffs are bare rock here too.
                const sw::Vec3 vertexDir =
                    sw::Vec3(glm::normalize(origin + sw::WorldVec3(vertex.position)));
                const sw::f32 cosine =
                    glm::clamp(glm::dot(vertex.normal, vertexDir), 0.05f, 1.0f);
                const sw::f32 slope =
                    std::sqrt(std::max(0.0f, 1.0f - cosine * cosine)) / cosine;
                // The patch's own Nyquist limit: metre-scale cells resolve
                // every frequency in the palette, so nothing is faded here.
                colorizeSurfaceVertex(vertex, style, vertexDir, elevations[index],
                                      slope, terrain, patchFrequencyLimit);
                // The baked relief term multiplies the ALBEDO, so it stacks
                // with the shader's own lambert instead of replacing it: a
                // slope facing the sun is bright, a slope facing the sun
                // from inside somebody else's shadow is not.
                const sw::f32 relief = shading[index];
                vertex.color.r *= relief;
                vertex.color.g *= relief;
                vertex.color.b *= relief;
            }
        }
        mesh.indices.reserve(kCells * kCells * 6);
        for (sw::u32 j = 0; j < kCells; ++j)
        {
            for (sw::u32 i = 0; i < kCells; ++i)
            {
                const sw::u32 a = j * kVerts + i;
                const sw::u32 b = a + 1;
                const sw::u32 c = a + kVerts;
                const sw::u32 d = c + 1;
                // CCW seen from OUTSIDE the planet (+up): front faces out.
                mesh.indices.insert(mesh.indices.end(), {a, b, c, b, d, c});
            }
        }

        // ---- THE RIM SKIRT --------------------------------------------------
        //
        // The patch is a sheet laid over the globe, and the globe is a second
        // ground surface underneath it — 133 km between vertices at this
        // level of detail, so within a few kilometres of the camera it is
        // one enormous flat triangle. It has to stay: beyond the patch's
        // 1.5 km rim it IS the horizon. But it should never be SEEN, and at
        // the rim it was: the sheet simply stopped, and the eye followed the
        // cut straight down onto the surface below.
        //
        // A skirt closes it. One ring of quads dropped from the border
        // vertices, darkened like a cut bank, costing 4 x kCells triangles —
        // half a per cent of the patch. Nothing else about the second
        // surface needs changing, because front-to-back batching plus the
        // early depth test already reject every one of its hidden fragments
        // before it evaluates a noise octave.
        {
            const sw::f32 skirtDrop =
                static_cast<sw::f32>(std::max(200.0, extent * 0.35));
            const auto addSkirt = [&](sw::u32 i, sw::u32 j, sw::u32 iNext, sw::u32 jNext) {
                const sw::usize a = static_cast<sw::usize>(j) * kVerts + i;
                const sw::usize b = static_cast<sw::usize>(jNext) * kVerts + iNext;
                const sw::u32 first = static_cast<sw::u32>(mesh.vertices.size());
                const sw::Vec3 down = -centerDir * skirtDrop;

                // BY VALUE, and this is not a style preference. Reading the
                // two rim vertices through REFERENCES into `mesh.vertices`
                // and then pushing onto that same vector is a use-after-free
                // the moment the push reallocates — which it does on the very
                // first quad, because the vector was sized exactly to the
                // ground grid. It cost a crash the instant a patch with a
                // skirt was built, and it is the reason the grass appeared to
                // be at fault: the grass is simply what the same build
                // produces next.
                const sw::Vertex topA = mesh.vertices[a];
                const sw::Vertex topB = mesh.vertices[b];

                // The rim's outward normal: along the edge, turned a quarter
                // turn about the local vertical. Degenerate edges (two rim
                // vertices at the same place) would normalise a zero vector
                // into NaN and hand the GPU a mesh full of them, so the
                // fallback is stated rather than hoped for.
                sw::Vec3 along = topB.position - topA.position;
                if (glm::dot(along, along) < 1.0e-12f)
                {
                    along = east;
                }
                const sw::Vec3 outward = glm::cross(centerDir, glm::normalize(along));
                const sw::Vec3 normal = (glm::dot(outward, outward) > 1.0e-12f)
                                            ? glm::normalize(outward)
                                            : centerDir;

                for (sw::u32 corner = 0; corner < 4; ++corner)
                {
                    const sw::Vertex& source = (corner % 2 == 0) ? topA : topB;
                    sw::Vertex vertex = source;
                    if (corner >= 2)
                    {
                        vertex.position += down;
                        vertex.color = sw::Vec4(sw::Vec3(source.color) * 0.45f, 1.0f);
                    }
                    vertex.normal = normal;
                    mesh.vertices.push_back(vertex);
                }
                // Both windings: which side of the rim faces the camera
                // depends on which edge of the patch it is.
                mesh.indices.insert(mesh.indices.end(),
                                    {first, first + 2, first + 1, first + 1, first + 2,
                                     first + 3, first, first + 1, first + 2, first + 1,
                                     first + 3, first + 2});
            };
            for (sw::u32 i = 0; i < kCells; ++i)
            {
                addSkirt(i, 0, i + 1, 0);
                addSkirt(i, kCells, i + 1, kCells);
                addSkirt(0, i, 0, i + 1);
                addSkirt(kCells, i, kCells, i + 1);
            }
        }

        // ---- THE GUARD ------------------------------------------------------
        //
        // Everything above APPENDS to one vertex vector, and a geometry bug
        // in an append is invisible to the compiler and silent at runtime
        // until a driver chokes on it. One pass over the finished mesh turns
        // that whole class of fault into a log line and a patch that is
        // simply not shown: 0.3 ms against a 60 ms build, which is nothing
        // for never handing the GPU a NaN.
        {
            sw::usize bad = 0;
            for (const sw::Vertex& vertex : mesh.vertices)
            {
                if (!std::isfinite(vertex.position.x) || !std::isfinite(vertex.position.y) ||
                    !std::isfinite(vertex.position.z) || !std::isfinite(vertex.normal.x) ||
                    !std::isfinite(vertex.normal.y) || !std::isfinite(vertex.normal.z))
                {
                    bad += 1;
                }
            }
            const sw::u32 vertexCount = static_cast<sw::u32>(mesh.vertices.size());
            for (const sw::u32 index : mesh.indices)
            {
                if (index >= vertexCount)
                {
                    bad += 1;
                    break;
                }
            }
            if (bad != 0)
            {
                SW_LOG_ERROR("Terrain",
                             "Patch build produced {} bad vertices or an out-of-range "
                             "index ({} vertices, {} indices) - discarding",
                             bad, vertexCount, mesh.indices.size());
                return; // the previous patch keeps being shown
            }
        }

        m_terrainPendingMesh = std::move(mesh);
        m_terrainPendingOrigin = origin;
        m_terrainPendingCenterDir = centerDir;
        m_terrainPendingExtent = extent;
        m_terrainPendingCells = kCells;
        m_terrainPendingEast = east;
        m_terrainPendingNorth = north;
    }

    // ------------------------------------------------------------------------
    // THE GRASS FIELD
    //
    // Its own geometry, on its own clock, cut into chunks that go to the GPU
    // one per frame. It reads the ground grid the terrain patch already
    // produced, so every blade stands on exactly the surface that is drawn —
    // and so re-centring the field costs nothing on the heightfield.
    // ------------------------------------------------------------------------
    void StarWorksGame::buildGrassField(const std::vector<sw::Vertex>& groundGrid,
                                        sw::u32 cellCount, const sw::Vec3& centerDir,
                                        const sw::Vec3& east, const sw::Vec3& north,
                                        sw::f64 extent, sw::f64 radius,
                                        const sw::Vec3& fieldDir)
    {
        for (sw::MeshData& chunk : m_grassPending)
        {
            chunk.vertices.clear();
            chunk.indices.clear();
            chunk.vertices.reserve(16000);
            chunk.indices.reserve(48000);
        }
        const sw::u32 verts = cellCount + 1;
        if (groundGrid.size() < static_cast<sw::usize>(verts) * verts)
        {
            return;
        }

        const auto hash01 = [](sw::i64 a, sw::i64 b, sw::u32 salt) {
            sw::u64 h = static_cast<sw::u64>(a) * 0x9E3779B97F4A7C15ull;
            h ^= static_cast<sw::u64>(b) * 0xC2B2AE3D27D4EB4Full;
            h ^= static_cast<sw::u64>(salt) * 0x165667B19E3779F9ull;
            h ^= h >> 29;
            h *= 0xBF58476D1CE4E5B9ull;
            h ^= h >> 32;
            return static_cast<sw::f32>(h & 0xFFFFFFull) / 16777215.0f;
        };

        // Where the PLAYER is, in the patch's own tangent chart. The field
        // follows them; the chart does not move.
        const sw::f64 fieldU = static_cast<sw::f64>(glm::dot(fieldDir, east)) * radius;
        const sw::f64 fieldV = static_cast<sw::f64>(glm::dot(fieldDir, north)) * radius;

        // The lattice is anchored to the PLANET: absolute plate-carrée metres,
        // so a tuft keeps its cell — and therefore its jitter, its height and
        // its colour — no matter which patch or which field it lands in.
        const sw::f64 latitude =
            std::asin(glm::clamp(static_cast<sw::f64>(centerDir.y), -1.0, 1.0));
        const sw::f64 longitude = std::atan2(static_cast<sw::f64>(centerDir.x),
                                             static_cast<sw::f64>(centerDir.z));
        const sw::f64 anchorU = longitude * radius * std::cos(latitude) + fieldU;
        const sw::f64 anchorV = latitude * radius + fieldV;

        // ---- THE FIELD'S FOUR KNOBS, together and named -----------------------
        //
        // They interact, so changing one alone gives a result nobody expects:
        // the spacing decides how many cells exist, the density pair how many
        // of those cells grow anything, and the height pair how big what grows
        // is. And height is multiplied by density further down, so thinning
        // the field also shortens it — which is why both were reduced by less
        // than the final effect suggests.
        //
        // Measured over the planted disc, ground held fully green (the
        // densest case there is): 8 834 tufts / 26 502 blades before,
        // 3 717 / 11 151 after — 42 % of the tufts, 0.042 down to 0.018 per
        // square metre. Mean blade 0.224 m instead of 0.434 m, tallest 0.61 m
        // instead of 1.38 m: the 1.75 m suit now stands nearly eight times
        // the mean blade rather than four, and nothing reaches its waist.
        constexpr sw::f64 kSpacing = 1.9;          // was 1.5
        constexpr sw::f32 kDensityFloor = 0.24f;   // was 0.35
        constexpr sw::f32 kDensitySpread = 0.62f;  // was 0.90
        constexpr sw::f32 kHeightFloor = 0.13f;    // was 0.22
        constexpr sw::f32 kHeightSpread = 0.30f;   // was 0.55

        const sw::f64 plantRadius = std::min(extent * 0.20, 260.0);
        const sw::f64 plantFull = 30.0;  // full density inside this
        const sw::f64 plantFade = 0.82;  // height fades over the last 18 %

        // Bilinear read of the ground the patch drew. Positions, colours and
        // normals all come from the same four vertices, so a blade stands on
        // the surface, is lit like it, and is coloured by it.
        const auto sampleGrid = [&](sw::f64 u, sw::f64 v, sw::Vec3& outPoint,
                                    sw::Vec4& outColor, sw::Vec3& outNormal) {
            const sw::f64 fx = (u / extent * 0.5 + 0.5) * cellCount;
            const sw::f64 fy = (v / extent * 0.5 + 0.5) * cellCount;
            if (fx < 0.0 || fy < 0.0 || fx >= cellCount || fy >= cellCount)
            {
                return false;
            }
            const sw::u32 i = static_cast<sw::u32>(fx);
            const sw::u32 j = static_cast<sw::u32>(fy);
            const sw::f32 a = static_cast<sw::f32>(fx - i);
            const sw::f32 b = static_cast<sw::f32>(fy - j);
            const sw::usize i00 = static_cast<sw::usize>(j) * verts + i;
            const sw::usize i10 = i00 + 1;
            const sw::usize i01 = i00 + verts;
            const sw::usize i11 = i01 + 1;
            const auto mix2 = [a, b](auto p00, auto p10, auto p01, auto p11) {
                return (p00 * (1.0f - a) + p10 * a) * (1.0f - b) +
                       (p01 * (1.0f - a) + p11 * a) * b;
            };
            outPoint = mix2(groundGrid[i00].position, groundGrid[i10].position,
                            groundGrid[i01].position, groundGrid[i11].position);
            outColor = mix2(groundGrid[i00].color, groundGrid[i10].color,
                            groundGrid[i01].color, groundGrid[i11].color);
            outNormal = glm::normalize(mix2(groundGrid[i00].normal, groundGrid[i10].normal,
                                            groundGrid[i01].normal, groundGrid[i11].normal));
            return true;
        };

        const sw::i64 uFirst =
            static_cast<sw::i64>(std::floor((anchorU - plantRadius) / kSpacing));
        const sw::i64 uLast =
            static_cast<sw::i64>(std::ceil((anchorU + plantRadius) / kSpacing));
        const sw::i64 vFirst =
            static_cast<sw::i64>(std::floor((anchorV - plantRadius) / kSpacing));
        const sw::i64 vLast =
            static_cast<sw::i64>(std::ceil((anchorV + plantRadius) / kSpacing));

        for (sw::i64 kv = vFirst; kv <= vLast; ++kv)
        {
            for (sw::i64 ku = uFirst; ku <= uLast; ++ku)
            {
                // Cell -> patch-local metres, jittered by a function OF THE
                // CELL, so the jitter travels with the planet and not with
                // the field.
                const sw::f32 jitterU = hash01(ku, kv, 11u) - 0.5f;
                const sw::f32 jitterV = hash01(ku, kv, 23u) - 0.5f;
                const sw::f64 localU = static_cast<sw::f64>(ku) * kSpacing - anchorU +
                                       static_cast<sw::f64>(jitterU) * kSpacing * 0.85;
                const sw::f64 localV = static_cast<sw::f64>(kv) * kSpacing - anchorV +
                                       static_cast<sw::f64>(jitterV) * kSpacing * 0.85;
                const sw::f64 distanceSquared = localU * localU + localV * localV;
                if (distanceSquared > plantRadius * plantRadius)
                {
                    continue;
                }
                // ...and back into the patch's chart, which is offset from the
                // field's by where the player stands in it.
                const sw::f64 u = localU + fieldU;
                const sw::f64 v = localV + fieldV;

                sw::Vec3 base{0.0f};
                sw::Vec4 groundColor{0.0f};
                sw::Vec3 groundNormal{0.0f};
                if (!sampleGrid(u, v, base, groundColor, groundNormal))
                {
                    continue;
                }
                // A cliff face is bare, and it is bare because it is a cliff.
                if (glm::dot(groundNormal, centerDir) < 0.88f)
                {
                    continue;
                }
                // WHAT THE GROUND ITSELF SAYS. Green ground grows things; rock,
                // ice, dune and open water do not — read straight off the
                // palette, so a plant can never appear on a colour that would
                // not support it and no biome table has to be kept in step
                // with the one the terrain already has.
                const sw::f32 green =
                    groundColor.g - 0.5f * (groundColor.r + groundColor.b);
                sw::f32 density = sw::math::smoothstepf(-0.005f, 0.045f, green);
                density *= kDensityFloor + kDensitySpread * hash01(ku / 13, kv / 13, 91u);
                // Thinned as an inverse power rather than a ramp: cover within
                // thirty metres, texture beyond, and a total that stays near
                // six thousand tufts however far the field is asked to reach.
                const sw::f64 distance = std::sqrt(distanceSquared);
                if (distance > plantFull)
                {
                    density *= static_cast<sw::f32>(std::pow(plantFull / distance, 1.5));
                }
                if (hash01(ku, kv, 57u) > density)
                {
                    continue;
                }
                // Fade LATE, so the band sits beyond anything a re-centre can
                // move a visible tuft across.
                const sw::f32 edge =
                    1.0f - sw::math::smoothstepf(static_cast<sw::f32>(plantFade), 1.0f,
                                                 static_cast<sw::f32>(distance /
                                                                      plantRadius));
                if (edge <= 0.02f)
                {
                    continue;
                }
                const sw::f32 height = (kHeightFloor + kHeightSpread *
                                        hash01(ku, kv, 131u)) *
                                       edge * (0.6f + density);
                if (height < 0.06f)
                {
                    continue;
                }
                const sw::f32 width = height * (0.28f + 0.16f * hash01(ku, kv, 77u));

                const sw::Vec3 leaf =
                    glm::mix(sw::Vec3(groundColor), sw::Vec3(0.20f, 0.34f, 0.12f),
                             0.55f + 0.25f * hash01(ku, kv, 197u));
                const sw::Vec4 rootColor{leaf * 0.55f, 1.0f};
                const sw::Vec4 tipColor{leaf * (1.05f + 0.25f * hash01(ku, kv, 211u)),
                                        1.0f};

                // WHICH CHUNK. By hash, so the six of them are the same size
                // and each is spread over the whole field — the set is only
                // ever shown complete, so this is about balancing the six
                // uploads, not about what appears first.
                const sw::u32 chunkIndex =
                    std::min(static_cast<sw::u32>(hash01(ku, kv, 777u) *
                                                  static_cast<sw::f32>(kGrassChunks)),
                             kGrassChunks - 1u);
                sw::MeshData& chunk = m_grassPending[chunkIndex];

                for (sw::u32 blade = 0; blade < 3; ++blade)
                {
                    const sw::f32 yaw =
                        (hash01(ku, kv, 300u + blade) + static_cast<sw::f32>(blade)) *
                        2.0943951f;
                    const sw::Vec3 lean = east * std::cos(yaw) + north * std::sin(yaw);
                    const sw::Vec3 across = glm::normalize(glm::cross(centerDir, lean));
                    const sw::f32 bend =
                        height * (0.25f + 0.35f * hash01(ku, kv, 400u + blade));

                    const sw::Vec3 root = base;
                    const sw::Vec3 tip = root + centerDir * height + lean * bend;
                    const sw::Vec3 normal =
                        glm::normalize(centerDir * 0.72f + across * 0.28f);

                    const sw::u32 first = static_cast<sw::u32>(chunk.vertices.size());
                    const sw::Vec3 offsets[4] = {root - across * (width * 0.5f),
                                                 root + across * (width * 0.5f),
                                                 tip - across * (width * 0.12f),
                                                 tip + across * (width * 0.12f)};
                    for (sw::u32 corner = 0; corner < 4; ++corner)
                    {
                        sw::Vertex vertex{};
                        vertex.position = offsets[corner];
                        vertex.normal = normal;
                        vertex.color = (corner < 2) ? rootColor : tipColor;
                        vertex.uv = {0.05f, 0.15f}; // matte: leaves do not shine
                        chunk.vertices.push_back(vertex);
                    }
                    // Both windings, so a blade is never invisible from the
                    // side the culler happens to be looking from.
                    chunk.indices.insert(chunk.indices.end(),
                                         {first, first + 1, first + 2, first + 1,
                                          first + 3, first + 2, first + 2, first + 1,
                                          first, first + 2, first + 3, first + 1});
                }
            }
        }
    }

    void StarWorksGame::updateGrassField()
    {
        // ---- 1. land one chunk per frame ------------------------------------
        //
        // ONE. `uploadToBuffer` submits its copy and then waits on a fence,
        // which drains whatever the graphics queue is holding — so every
        // upload costs up to a frame of GPU work whatever its size. Doing six
        // small ones on six frames turns one visible spike into six frames
        // nobody notices, and the field on screen never flickers because the
        // OLD set keeps drawing until the last new chunk has landed.
        if (m_grassJob.load(std::memory_order_acquire) == TerrainJob::Ready)
        {
            const sw::u32 target = m_grassSet ^ 1u;
            if (m_grassUploadCursor < kGrassChunks)
            {
                const sw::u32 index = m_grassUploadCursor;
                m_grassChunkValid[target][index] = !m_grassPending[index].empty();
                if (m_grassChunkValid[target][index])
                {
                    if (m_grassSlots[target][index] == 0xFFFFFFFFu)
                    {
                        m_grassSlots[target][index] =
                            registerMesh(renderer().createMesh(m_grassPending[index]));
                    }
                    else
                    {
                        m_meshes[m_grassSlots[target][index]] =
                            renderer().createMesh(m_grassPending[index]);
                    }
                }
                m_grassUploadCursor += 1;
                return; // one upload per frame, and not one more
            }
            // Every chunk has landed: show the new field and release the CPU
            // copies.
            m_grassSet = target;
            m_grassLiveCount = kGrassChunks;
            m_grassCenterDir = m_grassPendingCenterDir;
            m_grassOriginLocal = m_grassPendingOriginLocal;
            m_grassBody = m_terrainBody;
            for (sw::MeshData& chunk : m_grassPending)
            {
                chunk = sw::MeshData{};
            }
            m_grassJob.store(TerrainJob::Idle, std::memory_order_release);
            return;
        }
        if (m_grassJob.load(std::memory_order_acquire) != TerrainJob::Idle)
        {
            return; // a field is being seeded
        }

        // ---- 2. does the field need to move? --------------------------------
        if (m_terrainGridVertices.empty() || m_terrainGridCells == 0 ||
            m_terrainExtent > 2.5e3)
        {
            m_grassLiveCount = 0; // too high up for a field to mean anything
            return;
        }
        const sw::i32 primaryIndex = controlledPrimaryIndex();
        if (primaryIndex < 0 || m_terrainBody.isNull())
        {
            return;
        }
        const auto& primary =
            m_celestialIndex.body(static_cast<sw::usize>(primaryIndex));
        const auto* bodyTransform =
            m_world.tryGetComponent<TransformComponent>(m_terrainBody);
        const auto* bodySpin =
            m_world.tryGetComponent<sw::phys::GravitySourceComponent>(m_terrainBody);
        if (bodyTransform == nullptr)
        {
            return;
        }
        const auto& craft = m_world.getComponent<TransformComponent>(controlledEntity());
        const sw::WorldVec3 radial = craft.position - bodyTransform->position;
        const sw::f64 distance = glm::length(radial);
        if (distance <= 1.0)
        {
            return;
        }
        const glm::dquat inverseRotation =
            (bodySpin != nullptr) ? glm::inverse(sw::phys::spinRotation(*bodySpin))
                                  : glm::inverse(glm::dquat(bodyTransform->rotation));
        const sw::Vec3 here =
            sw::Vec3(glm::normalize(inverseRotation * (radial / distance)));

        // FORTY METRES, the scale of the field rather than of the terrain.
        // Any further and a walking player leaves the grass behind.
        const sw::f64 travelled =
            static_cast<sw::f64>(glm::distance(here, m_grassCenterDir)) *
            primary.bodyRadius;
        if (m_grassLiveCount != 0 && travelled < 40.0)
        {
            return;
        }

        // ---- 3. seed it -----------------------------------------------------
        // The grid is copied into the job rather than shared: 1.8 MB and a
        // fifth of a millisecond buys the guarantee that a terrain rebuild
        // landing mid-seed cannot pull the ground out from under it.
        const std::vector<sw::Vertex> grid = m_terrainGridVertices;
        const sw::u32 cells = m_terrainGridCells;
        const sw::Vec3 centerDir = m_terrainCenterDir;
        const sw::Vec3 east = m_terrainGridEast;
        const sw::Vec3 north = m_terrainGridNorth;
        const sw::f64 extent = m_terrainExtent;
        const sw::f64 radius = primary.bodyRadius;
        m_grassPendingCenterDir = here;
        m_grassPendingOriginLocal = m_terrainOriginLocal;
        m_grassUploadCursor = 0;
        m_grassJob.store(TerrainJob::Running, std::memory_order_release);
        threadPool().submit([this, grid, cells, centerDir, east, north, extent, radius,
                             here]() {
            buildGrassField(grid, cells, centerDir, east, north, extent, radius, here);
            m_grassJob.store(TerrainJob::Ready, std::memory_order_release);
        });
    }
} // namespace game
