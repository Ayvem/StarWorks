// ============================================================================
// GameScene.cpp — World construction: celestial bodies, LOD sets, the initial outpost.
// Split out of StarWorksGame.cpp; same class, one theme per translation unit.
// ============================================================================

#include "StarWorksGame.hpp"

#include "GameInternal.hpp"
#include "Systems.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <format>
#include <limits>

namespace game
{

    CelestialLodComponent StarWorksGame::makeSphereLodSet(const sw::Vec4& color,
                                                          sw::i32 surfaceStyle,
                                                          sw::f64 bodyRadiusMeters)
    {
        // RELIEF SHADING (M22): the globe's vertex normals are tilted by
        // the gradient of the SAME analytic heightfield physics collides
        // with — mountain ranges catch the light and throw shadow flanks
        // from orbit, and they are exactly where the terrain patch will
        // put them when you land. Ocean stays flat (elevation clamps to 0,
        // gradient vanishes) and keeps its mirror specular.
        // A landable style (0-13) gets its preset — the SAME one collision
        // will sample — and relief if the caller told us how big the body
        // is. Gas styles (>= 20) have no ground: their vertex colours come
        // from gasGiantAlbedo in colorizeSurfaceVertex, and there is no
        // elevation to shade.
        sw::planet::TerrainComponent terrain{};
        const sw::f64 bodyRadius = bodyRadiusMeters;
        bool hasRelief = false;
        if (surfaceStyle >= 0 && !isGasStyle(surfaceStyle))
        {
            terrain = sw::planet::terrainPreset(surfaceStyle);
            hasRelief = bodyRadius > 0.0;
        }

        CelestialLodComponent lod{};
        lod.surfaceStyle = surfaceStyle;
        for (sw::u32 level = 0; level < CelestialLodComponent::kLodLevels; ++level)
        {
            sw::MeshData sphere = sw::PrimitiveFactory::makeUvSphere(
                1.0f, kLodRings[level], kLodSegments[level], color);
            if (surfaceStyle < 0)
            {
                lod.meshIndex[level] = registerMesh(renderer().createMesh(sphere));
                continue;
            }

            // ONE pass per vertex (M25): the elevation is sampled once and
            // feeds BOTH the palette and the relief normal. The v2
            // heightfield costs ~22 noise samples per point — paying it
            // twice per vertex, on five LODs, would be a visible hitch at
            // world build time.
            //
            // Only the two closest LODs carry relief normals: farther ones
            // subtend a handful of pixels and the sampling would be wasted.
            // Slopes are physically tiny (9 km over thousands of km), so an
            // exaggeration factor makes them read without moving a vertex
            // (silhouette and collision stay exact) — but it must WHISPER.
            // The M22 value was 220, tuned when these normals were the only
            // relief there was; against the per-fragment path (x5.5, under
            // 1.6 R) that made the mid-range band look crumpled into
            // ten-kilometre mountains that then ironed themselves flat on
            // approach. A planet seen from space is nearly smooth: the
            // vertex gradients are sampled over a ~25 km arc (already
            // smoothed, so a little more than 5.5 compensates), the far of
            // the two relief LODs gets less again, and the tilt is capped
            // at 45 degrees exactly like the shader's kMaxNormalTilt — the
            // saturated foil look WAS the old cap-less x220.
            const sw::f32 kSlopeExaggeration = (level == 0) ? 22.0f : 9.0f;
            constexpr sw::f32 kMaxNormalTilt = 1.0f; // tangent: 45 deg
            const sw::f32 epsilon = 0.004f; // ~25 km sampling arc on Terra
            const bool reliefNormals = hasRelief && level < 2;
            // A 25 km arc cannot resolve a 2 km ridge: sampling the full
            // octave stack here would only alias between vertices. Level 0
            // takes six relief octaves, level 1 five, the rest four.
            const sw::i32 reliefOctavesForLevel =
                std::min(terrain.reliefOctaves,
                         (level == 0) ? 6 : ((level == 1) ? 5 : 4));
            const auto style = static_cast<SurfaceStyle>(surfaceStyle);
            // Nyquist for this tessellation: rings / 2pi. Nothing finer than
            // this may reach the palette, or the mesh turns a texture into
            // noise (see colorizeSurfaceVertex).
            const sw::f32 frequencyLimit =
                static_cast<sw::f32>(kLodRings[level]) / 6.2831853f;
            for (sw::Vertex& vertex : sphere.vertices)
            {
                const sw::Vec3 dir = glm::normalize(vertex.position);
                // Far LODs sample fewer relief octaves — exactly the LOD the
                // shader applies by screen footprint, so the vertex path and
                // the fragment path meet in the middle instead of popping.
                const sw::f32 elevation =
                    hasRelief ? sw::planet::terrainElevationSignedLod(
                                    terrain, dir, reliefOctavesForLevel)
                              : 0.0f;

                sw::f32 slope = 0.0f;
                sw::f32 slopeA = 0.0f;
                sw::f32 slopeB = 0.0f;
                sw::Vec3 tangentA{0.0f};
                sw::Vec3 tangentB{0.0f};
                if (reliefNormals && elevation > 0.0f)
                {
                    const sw::Vec3 reference = std::abs(dir.y) < 0.95f
                                                   ? sw::Vec3{0, 1, 0}
                                                   : sw::Vec3{1, 0, 0};
                    tangentA = glm::normalize(glm::cross(reference, dir));
                    tangentB = glm::cross(dir, tangentA);
                    slopeA = (sw::planet::terrainElevationSignedLod(
                                  terrain, glm::normalize(dir + tangentA * epsilon),
                                  reliefOctavesForLevel) -
                              elevation) /
                             static_cast<sw::f32>(bodyRadius * epsilon);
                    slopeB = (sw::planet::terrainElevationSignedLod(
                                  terrain, glm::normalize(dir + tangentB * epsilon),
                                  reliefOctavesForLevel) -
                              elevation) /
                             static_cast<sw::f32>(bodyRadius * epsilon);
                    slope = glm::length(sw::Vec2{slopeA, slopeB});
                }

                colorizeSurfaceVertex(vertex, style, dir, elevation, slope, terrain,
                                      frequencyLimit);

                if (reliefNormals && elevation > 0.0f)
                {
                    sw::Vec3 tilt = (tangentA * slopeA + tangentB * slopeB) *
                                    kSlopeExaggeration;
                    const sw::f32 tiltLength = glm::length(tilt);
                    if (tiltLength > kMaxNormalTilt)
                    {
                        tilt *= kMaxNormalTilt / tiltLength;
                    }
                    vertex.normal = glm::normalize(dir - tilt);
                }
            }
            lod.meshIndex[level] = registerMesh(renderer().createMesh(sphere));
        }
        return lod;
    }

    void StarWorksGame::buildScene()
    {
        // SW_CLOCK=<seconds> WINDS THE SIMULATION CLOCK FORWARD, BEFORE THE
        // WORLD IS BUILT — and the "before" is the whole hook.
        //
        // It exists because the fault it reproduces cannot be reached any
        // other way from a capture: everything analytic (every planet's
        // position, every rail, every planet's spin) is a function of
        // ABSOLUTE simulated time, and a crossing to Proxima Centauri costs
        // about 3e11 seconds of it. Setting the clock from inside the frame
        // loop instead was tried first and measured 12 km of wobble that was
        // pure artefact: the planet rotates to a new attitude under a walker
        // still carrying the velocity it had at t = 0, and what the probe
        // then reports is a ballistic hop, not a precision problem.
        if (const char* clockSpec = std::getenv("SW_CLOCK"))
        {
            const sw::f64 seconds = std::strtod(clockSpec, nullptr);
            m_simulation.setSimulatedSeconds(seconds);
            SW_LOG_INFO("Game", "SW_CLOCK: simulated time set to {:.6g} s before build",
                        seconds);
        }

        // ---- meshes -------------------------------------------------------------
        // Sol's colors exceed 1.0 slightly: paired with the emissive tint it
        // reads as a glowing star, not a lit rock.
        const CelestialLodComponent solLod =
            makeSphereLodSet({1.0f, 0.92f, 0.72f, 1.0f});
        const CelestialLodComponent terraLod =
            makeSphereLodSet({0.21f, 0.33f, 0.48f, 1.0f},
                             static_cast<sw::i32>(SurfaceStyle::Terra), kTerraRadius);
        const CelestialLodComponent lunaLod =
            makeSphereLodSet({0.42f, 0.41f, 0.43f, 1.0f},
                             static_cast<sw::i32>(SurfaceStyle::Luna), kLunaRadius);
        const CelestialLodComponent marsLod =
            makeSphereLodSet({0.62f, 0.32f, 0.18f, 1.0f},
                             static_cast<sw::i32>(SurfaceStyle::Mars), kMarsRadius);

        // Environment meshes: the fixed star dome, Terra's atmosphere veil
        // and its drifting cloud shell.
        m_starfieldMeshIndex =
            registerMesh(renderer().createMesh(buildStarfieldMesh()));
        // THE SUN'S GLARE, in three layers, because one disc cannot be both a
        // blinding core and a glow that reaches across a quarter of the sky.
        // Radiances are in grade units, where 4.0 is already pure white — the
        // core's fifteen is deliberately far past clipping so the white region
        // has an EDGE that stays put as the disc's apparent size changes.
        //
        // THE THREE PROFILES ARE ONE PROFILE, and they were tuned together
        // against a measured background star rather than by eye. From Saturn
        // the brightest star in the frame runs 170 / 137 / 115 / 82 / 59 / 43
        // sRGB at 0 / 4 / 6 / 8 / 15 / 26 pixels from its centre and dies at
        // forty. Whatever the sun does it has to beat that at EVERY radius, or
        // it reads as the smaller object however bright its centre pixel is —
        // which is exactly what a peak of 255 inside a four-pixel dot did.
        // These three give 255 / 255 / 223 / 207 / 150 / 82 out to eighty-five.
        //
        // AND THE BLEND HAPPENS IN LINEAR LIGHT, which is worth forty percent
        // and is invisible from the shader. The swapchain is B8G8R8A8_SRGB, so
        // the hardware un-encodes both sides before mixing them: an alpha of
        // 0.25 does NOT put a quarter of the layer's sRGB value on screen, it
        // puts a quarter of its LIGHT there, which is a much larger number
        // once re-encoded. Modelled the naive way these profiles predicted 116
        // where the renderer measured 170 — so the first attempt at a tail was
        // tuned to a curve that was already half again too bright, and the sun
        // came out as a smooth cream BALL with an edge on it rather than as a
        // glare. Any future tuning of these numbers has to go through
        // srgb->linear, mix, linear->srgb or it will make the same mistake.
        // NEUTRAL, AND THE COLOUR COMES FROM THE STAR.
        //
        // These three carried Sol's warm ramp — (1, 0.98, 0.95), (1, 0.80,
        // 0.52), (1, 0.68, 0.38) — baked into their vertices, and the star's
        // own hue was multiplied ON TOP of it. For Sol that is a no-op, since
        // Sol's hue is neutral by construction. For everything else it was a
        // filter: Sirius's (0.51, 0.66, 1.00) times the aureole's (1, 0.68,
        // 0.38) is (0.51, 0.45, 0.38), which is ORANGE. The bluest star in the
        // sky was being drawn through a sunset.
        //
        // So the meshes are white and the ramp moved into the tint, where it
        // is expressed as a TEMPERATURE FACTOR rather than a colour: the halo
        // of a star is drawn as the blackbody of a cooler one. Sol's ramp is
        // reproduced to within four hundredths in blue (that is where the
        // factors in collectStarVisual come from), and every other star's
        // glare now reddens outward FROM ITS OWN COLOUR instead of toward
        // Sol's.
        m_sunCoreMeshIndex = registerMesh(renderer().createMesh(
            buildGlareDiscMesh({1.0f, 1.0f, 1.0f}, 34.0f, 1.00f, 3.2f, 0.52f)));
        m_sunHaloMeshIndex = registerMesh(renderer().createMesh(
            buildGlareDiscMesh({1.0f, 1.0f, 1.0f}, 3.80f, 1.00f, 3.0f, 0.16f,
                               0.10f, 6.0f)));
        m_sunAureoleMeshIndex = registerMesh(renderer().createMesh(
            buildGlareDiscMesh({1.0f, 1.0f, 1.0f}, 3.20f, 0.66f, 4.5f, 0.0f,
                               0.24f, 14.0f)));
        // Soft round billboard for plasma/exhaust + a lens-flare ghost disc.
        m_particleGlowMeshIndex = registerMesh(renderer().createMesh(
            buildGlowDiscMesh({1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}, 2.0f)));
        // HUD-path disc: straight alpha (center 1 -> rim 0), tint modulates.
        m_flareMeshIndex = registerMesh(renderer().createMesh(
            buildGlowDiscMesh({1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}, 1.0f, 0.0f)));
        const sw::u32 atmosphereMeshId =
            registerMesh(renderer().createMesh(buildAtmosphereShellMesh()));
        const sw::u32 cloudMeshId =
            registerMesh(renderer().createMesh(buildCloudShellMesh()));

        // The F2 overlay's box: a UNIT cube (half extent 0.5), scaled to
        // each hull box. White, so the tint alone decides how it reads.
        m_hullBoxMeshIndex = registerMesh(renderer().createMesh(
            sw::PrimitiveFactory::makeCube(1.0f, {1.0f, 1.0f, 1.0f, 1.0f})));
        // Part meshes, indexed by catalog id (small ids: direct table).
        for (const sw::parts::PartDefinition& definition : sw::parts::catalog())
        {
            // THE BODY, and then one mesh per thing that moves. A part with no
            // animations gets exactly what it always got: one welded mesh, one
            // draw item, nothing changed. A part with a solar array gets two,
            // and the second is drawn with an extra transform in front of it.
            m_partMeshIds[definition.id] = registerMesh(
                renderer().createMesh(sw::parts::buildPartMeshGroup(definition, -1)));
            for (sw::u32 group = 0; group < definition.animations.size(); ++group)
            {
                const sw::MeshData mesh =
                    sw::parts::buildPartMeshGroup(definition, static_cast<sw::i32>(group));
                if (mesh.indices.empty())
                {
                    continue; // an animation that only gates, moving nothing
                }
                m_partGroupMeshIds[partGroupKey(definition.id, group)] =
                    registerMesh(renderer().createMesh(mesh));
            }
        }
        // The BELT and its CARGO are ordinary parts. Their mesh slots and
        // their metrics are read from the catalogue rather than hard-coded,
        // so redrawing CV-1 in Part Studio — longer, wider, taller deck —
        // changes every belt in the world without touching this file.
        if (const auto* belt =
                sw::parts::findDefinition(sw::parts::kBuildingConveyor))
        {
            m_conveyorMeshIndex = m_partMeshIds.at(belt->id);
            constexpr sw::f32 kHuge = 1.0e9f;
            sw::Vec3 low{kHuge, kHuge, kHuge};
            sw::Vec3 high{-kHuge, -kHuge, -kHuge};
            sw::parts::expandPartHullBounds(*belt, sw::Vec3{0.0f},
                                                sw::Quat{1.0f, 0.0f, 0.0f, 0.0f}, low,
                                                high);
            if (low.z <= high.z)
            {
                m_conveyorSegmentM = std::max(0.1f, high.z - low.z);
                m_conveyorDeckHeightM = high.y; // cargo rides on the deck
            }
            SW_LOG_INFO("Game", "Conveyor part: {:.2f} m segment, deck at {:.2f} m",
                        m_conveyorSegmentM, m_conveyorDeckHeightM);
        }
        if (const auto* crate =
                sw::parts::findDefinition(sw::parts::kPropConveyorCrate))
        {
            m_cargoMeshIndex = m_partMeshIds.at(crate->id);
        }
        // ...and the CRADLE a finished rocket rides in. Same contract, a
        // different prop: the belt out of the VAB is meant to look like a
        // different kind of traffic, because it is.
        if (const auto* cradle =
                sw::parts::findDefinition(sw::parts::kPropVehicleCradle))
        {
            m_vehicleCargoMeshIndex = m_partMeshIds.at(cradle->id);
        }
        // The CABLE is the same story: one authored span, repeated along the
        // curve. Its length comes off its own collider, so a thicker or
        // longer CW-1 redrawn in Part Studio re-wires the whole base.
        if (const auto* wire = sw::parts::findDefinition(sw::parts::kBuildingCable))
        {
            m_cableMeshIndex = m_partMeshIds.at(wire->id);
            constexpr sw::f32 kHuge = 1.0e9f;
            sw::Vec3 low{kHuge, kHuge, kHuge};
            sw::Vec3 high{-kHuge, -kHuge, -kHuge};
            sw::parts::expandPartHullBounds(*wire, sw::Vec3{0.0f},
                                                sw::Quat{1.0f, 0.0f, 0.0f, 0.0f}, low,
                                                high);
            if (low.z <= high.z)
            {
                m_cableSegmentM = std::max(0.1f, high.z - low.z);
            }
            SW_LOG_INFO("Game", "Cable part: {:.2f} m span segment", m_cableSegmentM);
        }

        // THE PLAYER IS A PART. First person or not, you are visible to
        // yourself in the map, to a future second player, and in every
        // screenshot taken from the ship — and a capsule primitive said
        // "placeholder" in all of them. EV-1 is an ordinary .swpart prop, so
        // the suit is redrawn in Part Studio like everything else, and its
        // ground hull comes from its own hitbox rather than from a constant
        // in this file that could drift from the model.
        if (const auto* suit = sw::parts::findDefinition(sw::parts::kPropEvaSuit))
        {
            m_capsuleMeshIndex = m_partMeshIds.at(suit->id);
            constexpr sw::f32 kHuge = 1.0e9f;
            sw::Vec3 low{kHuge, kHuge, kHuge};
            sw::Vec3 high{-kHuge, -kHuge, -kHuge};
            sw::parts::expandPartHullBounds(*suit, sw::Vec3{0.0f},
                                            sw::Quat{1.0f, 0.0f, 0.0f, 0.0f}, low, high);
            if (low.y <= high.y)
            {
                m_capsuleHull.centre = (low + high) * 0.5f;
                m_capsuleHull.halfExtents = (high - low) * 0.5f;
            }
            SW_LOG_INFO("Game", "EVA suit hull: centre {:.2f} half {:.2f} x {:.2f}",
                        m_capsuleHull.centre.y, m_capsuleHull.halfExtents.x,
                        m_capsuleHull.halfExtents.y);
        }
        else
        {
            m_capsuleMeshIndex = registerMesh(renderer().createMesh(
                sw::PrimitiveFactory::makeCapsule(0.5f, 0.5f, 12, 16,
                                                  {0.9f, 0.6f, 0.2f, 1.0f})));
        }
        m_markerMeshIndex = registerMesh(renderer().createMesh(buildMarkerMesh()));
        // The trajectory line's own segment: a unit box, stretched along +Z
        // between two samples of a conic and thickened with distance so it
        // stays one pixel wide however far out the map is zoomed.
        m_orbitLineMeshIndex = registerMesh(renderer().createMesh(
            sw::PrimitiveFactory::makeBox({0.5f, 0.5f, 0.5f}, {1.0f, 1.0f, 1.0f, 1.0f})));

        // Sun position and eclipse occluders are camera-relative and set
        // every frame in onRender.

        auto snapshotOf = [](const TransformComponent& transform) {
            return PreviousTransformComponent{transform.position, transform.rotation};
        };

        // ================= THE HIERARCHY: Sol -> Terra/Mars -> Luna ==============
        // Parent-relative Kepler elements, real values. Initial world
        // positions are the analytic evaluation at t=0 — identical to what
        // the CelestialMotionSystem will compute on the first tick.

        // ---- Sol: the static root ------------------------------------------------
        {
            const sw::ecs::Entity e = m_world.createEntity();
            TransformComponent transform{}; // world origin
            transform.uniformScale = static_cast<sw::f32>(kSolRadius);
            m_world.addComponent(e, transform);
            m_world.addComponent(e, snapshotOf(transform));
            m_world.addComponent(e, BoundsComponent{1.0f});
            m_world.addComponent(e, solLod);
            m_world.addComponent(e, SpinComponent{{0.0f, 1.0f, 0.0f}, 2.9e-6f});
            sw::phys::GravitySourceComponent gravity{kMuSol, kSolRadius};
            gravity.angularVelocity = {0.0, 2.9e-6, 0.0};
            m_world.addComponent(e, gravity);
            m_world.addComponent(e, sw::space::makeCelestialBody("SOL"));
            m_world.addComponent(e, MapMarkerComponent{{1.0f, 0.85f, 0.3f, 1.0f}});
            m_solEntity = e;
        }

        // ---- and the thirty-five other suns --------------------------------
        buildCatalogueStars();

        // ---- Terra: SOLID SURFACE + ATMOSPHERE, on rails around Sol --------------
        const sw::phys::KeplerOrbit terraOrbit = sw::phys::kepler::fromElements(
            kMuSol, kTerraSma, 0.0167, 0.0, 0.0, 0.0, /*M0=*/0.0, /*epoch=*/0.0);
        sw::WorldVec3 terraPos0{};
        sw::phys::kepler::evaluate(terraOrbit, 0.0, terraPos0);
        {
            const sw::ecs::Entity e = m_world.createEntity();
            TransformComponent transform{};
            transform.position = terraPos0;
            transform.uniformScale = static_cast<sw::f32>(kTerraRadius);
            m_world.addComponent(e, transform);
            m_world.addComponent(e, snapshotOf(transform));
            m_world.addComponent(e, BoundsComponent{1.0f});
            m_world.addComponent(e, terraLod);
            m_world.addComponent(e, SpinComponent{{0.0f, 1.0f, 0.0f}, 7.2921e-5f});
            sw::phys::GravitySourceComponent gravity{kMuTerra, kTerraRadius};
            gravity.soiRadius = kTerraSoi;
            gravity.angularVelocity = kTerraAngularVelocity; // matches the Spin
            m_world.addComponent(e, gravity);
            m_world.addComponent(e, sw::phys::AtmosphereComponent{1.225, 8500.0, 1.4e5});
            m_world.addComponent(e, presetTerra()); // REAL ground: collision + visuals
            // Geology, analytic like the ground itself: nothing about a
            // deposit is stored, so a survey cannot lie and a save cannot
            // move the ore.
            m_world.addComponent(e, sw::planet::depositsTerra());
            m_world.addComponent(e, sw::space::makeCelestialBody("TERRA", m_solEntity,
                                                                 &terraOrbit));
            m_world.addComponent(e, MapMarkerComponent{{0.35f, 0.65f, 1.0f, 1.0f}});
            m_terraEntity = e;
        }
        const sw::ecs::Entity terraEntity = m_terraEntity;

        // ---- Terra's VISIBLE atmosphere + cloud deck (transparent shells) --------
        {
            const sw::ecs::Entity e = m_world.createEntity();
            TransformComponent transform{};
            transform.position = terraPos0;
            // THE SHELL IS THE CANVAS, and it has to be big enough for
            // everything Shaders/Atmosphere.glsl draws — not just the 80 km
            // air column it marches. Anything the shader emits above this
            // mesh has no fragment to be drawn into and is cut by the shell's
            // own silhouette, which reads as a hard-edged circular window.
            //
            // F19 sized it at 1.0130 (83 km) for the air alone, and that cost
            // two things: the nightglow had to be moved 25 km DOWN from its
            // measured altitude to stay under the mesh, and aurorae could not
            // be drawn at all, since they live at 90-400 km. F20 raises it to
            // 350 km, which puts the whole of both inside. The number is not
            // free — the shell is a transparent full-disc pass, and this one
            // covers 8% more sky than the old one at the same distance — but
            // it is bounded: the marched air still ends at 80 km, so the
            // extra fragments cost a sphere test and a discard unless the
            // aurora is actually in front of them.
            transform.uniformScale = static_cast<sw::f32>(kTerraRadius * 1.0550); // ~350 km
            m_world.addComponent(e, transform);
            m_world.addComponent(e, snapshotOf(transform));
            m_world.addComponent(e, BoundsComponent{1.0f});
            MeshComponent mesh{atmosphereMeshId};
            mesh.transparent = 1;
            m_world.addComponent(e, mesh);
            m_world.addComponent(e, CloudLayerComponent{terraEntity, {0, 1, 0}, 0.0f});
        }
        {
            const sw::ecs::Entity e = m_world.createEntity();
            TransformComponent transform{};
            transform.position = terraPos0;
            transform.uniformScale = static_cast<sw::f32>(kTerraRadius * 1.005); // ~32 km
            m_world.addComponent(e, transform);
            m_world.addComponent(e, snapshotOf(transform));
            m_world.addComponent(e, BoundsComponent{1.0f});
            MeshComponent mesh{cloudMeshId};
            mesh.transparent = MeshComponent::kCloudDeck;
            m_world.addComponent(e, mesh);
            // M28: the shell is GLUED to Terra's rotation and the drift moved
            // into the shader (Clouds.glsl). That is what lets the ground
            // path reproduce the deck exactly from the world clock and put
            // each shadow under the cloud that casts it.
            m_world.addComponent(e, CloudLayerComponent{terraEntity, {0, 1, 0}, 0.0f});
        }

        // ---- Luna: around TERRA (5.14 deg inclination, real) ---------------------
        const sw::phys::KeplerOrbit lunaOrbit = sw::phys::kepler::fromElements(
            kMuTerra, kLunaSma, 0.0549, 0.0897, 0.0, 0.0, /*M0=*/0.6, /*epoch=*/0.0);
        {
            const sw::ecs::Entity e = m_world.createEntity();
            TransformComponent transform{};
            sw::WorldVec3 lunaRel{};
            sw::phys::kepler::evaluate(lunaOrbit, 0.0, lunaRel);
            transform.position = terraPos0 + lunaRel;
            transform.uniformScale = static_cast<sw::f32>(kLunaRadius);
            m_world.addComponent(e, transform);
            m_world.addComponent(e, snapshotOf(transform));
            m_world.addComponent(e, BoundsComponent{1.0f});
            m_world.addComponent(e, lunaLod);
            m_world.addComponent(e, SpinComponent{{0.0f, 1.0f, 0.0f}, 2.66e-6f});
            sw::phys::GravitySourceComponent gravity{kMuLuna, kLunaRadius};
            gravity.soiRadius = kLunaSoi;
            gravity.angularVelocity = {0.0, 2.66e-6, 0.0};
            m_world.addComponent(e, gravity);
            m_world.addComponent(e, presetLuna());
            m_world.addComponent(e, sw::planet::depositsLuna()); // polar ice = propellant
            m_world.addComponent(e, sw::space::makeCelestialBody("LUNA", terraEntity,
                                                                 &lunaOrbit));
            m_world.addComponent(e, MapMarkerComponent{{0.75f, 0.75f, 0.78f, 1.0f}});
        }

        // ---- Mars: second planet, red and far -------------------------------------
        const sw::phys::KeplerOrbit marsOrbit = sw::phys::kepler::fromElements(
            kMuSol, kMarsSma, 0.0934, 0.0323, 0.0, 0.0, /*M0=*/2.0, /*epoch=*/0.0);
        {
            const sw::ecs::Entity e = m_world.createEntity();
            TransformComponent transform{};
            sw::phys::kepler::evaluate(marsOrbit, 0.0, transform.position);
            transform.uniformScale = static_cast<sw::f32>(kMarsRadius);
            m_world.addComponent(e, transform);
            m_world.addComponent(e, snapshotOf(transform));
            m_world.addComponent(e, BoundsComponent{1.0f});
            m_world.addComponent(e, marsLod);
            m_world.addComponent(e, SpinComponent{{0.0f, 1.0f, 0.0f}, 7.088e-5f});
            sw::phys::GravitySourceComponent gravity{kMuMars, kMarsRadius};
            gravity.soiRadius = kMarsSoi;
            gravity.angularVelocity = {0.0, 7.088e-5, 0.0};
            m_world.addComponent(e, gravity);
            m_world.addComponent(e, presetMars());
            m_world.addComponent(e, sw::planet::depositsMars());
            m_world.addComponent(e, sw::space::makeCelestialBody("MARS", m_solEntity,
                                                                 &marsOrbit));
            m_world.addComponent(e, MapMarkerComponent{{1.0f, 0.45f, 0.25f, 1.0f}});
        }

        // ================= THE REST OF THE SOLAR SYSTEM ===========================
        // One table, one loop. Every number is real (radii, GM, semi-major
        // axes, eccentricities, inclinations, spins); SOI radii are
        // r = a * (mu/mu_parent)^(2/5) like the hand-built bodies above.
        // Moons are tidally locked: their spin rate is COMPUTED from their
        // orbit (n = sqrt(mu_parent / a^3)) rather than typed, so it cannot
        // disagree.
        {
            struct MoonDef
            {
                const char* name;
                sw::f64 radius, mu, sma, ecc, incRad, m0, soi;
                sw::i32 style;
                sw::Vec4 mapColor;
                sw::planet::DepositComponent deposits;
            };
            struct PlanetDef
            {
                const char* name;
                sw::f64 radius, mu, sma, ecc, incRad, m0, soi;
                sw::f64 spinRate; // rad/s, about `spinAxis`
                sw::Vec3 spinAxis;
                sw::i32 style;
                sw::Vec4 mapColor;
                bool landable;
                sw::phys::AtmosphereComponent atmosphere; // topAltitude 0 = none
                sw::planet::DepositComponent deposits;    // used when landable
                bool rings;
                std::vector<MoonDef> moons;
            };

            const auto deposit = [](sw::u32 seed, sw::f32 metal, sw::f32 ice,
                                    sw::f32 polarBias) {
                sw::planet::DepositComponent d{};
                d.seed = seed;
                d.frequency = 10.0f;
                d.metalRichness = metal;
                d.iceRichness = ice;
                d.icePolarBias = polarBias;
                return d;
            };
            const sw::planet::DepositComponent noDeposits{};

            const std::vector<PlanetDef> planets = {
                // ---- Mercury: airless metal world, slow 58.6-day spin ----------
                {"MERCURY", kMercuryRadius, kMuMercury, kMercurySma, 0.2056, 0.1222,
                 3.8, kMercurySoi, 1.24e-6, {0.0f, 1.0f, 0.0f},
                 static_cast<sw::i32>(SurfaceStyle::Mercury),
                 {0.72f, 0.68f, 0.62f, 1.0f}, true, {0.0, 0.0, 0.0},
                 deposit(0x31E0u, 0.72f, 0.15f, 0.98f), false, {}},
                // ---- Venus: retrograde spin (axis flipped), crushing air -------
                // Not landable YET: its visible surface is the cloud deck
                // (style 24, the gas family), and the ground below is a
                // milestone of its own.
                {"VENUS", kVenusRadius, kMuVenus, kVenusSma, 0.0068, 0.0592, 1.2,
                 kVenusSoi, 2.99e-7, {0.0f, -1.0f, 0.0f},
                 static_cast<sw::i32>(SurfaceStyle::Venus),
                 {0.92f, 0.82f, 0.55f, 1.0f}, false, {65.0, 15900.0, 3.5e5},
                 noDeposits, false, {}},
                // ---- Jupiter and the Galilean moons ----------------------------
                {"JUPITER", kJupiterRadius, kMuJupiter, kJupiterSma, 0.0489, 0.0227,
                 0.6, kJupiterSoi, 1.7585e-4, {0.0546f, 0.9985f, 0.0f},
                 static_cast<sw::i32>(SurfaceStyle::Jupiter),
                 {0.85f, 0.68f, 0.45f, 1.0f}, false, {0.16, 27000.0, 5.0e6},
                 noDeposits, false,
                 {{"IO", 1.8216e6, 5.96e12, 4.217e8, 0.0041, 0.0007, 0.0, 7.8e6,
                   static_cast<sw::i32>(SurfaceStyle::Io),
                   {0.90f, 0.80f, 0.35f, 1.0f}, deposit(0x10AAu, 0.80f, 0.0f, 0.0f)},
                  {"EUROPA", 1.5608e6, 3.20e12, 6.709e8, 0.009, 0.0082, 1.1, 9.7e6,
                   static_cast<sw::i32>(SurfaceStyle::Europa),
                   {0.80f, 0.84f, 0.90f, 1.0f}, deposit(0xE0AAu, 0.10f, 0.92f, 0.15f)},
                  {"GANYMEDE", 2.6341e6, 9.888e12, 1.0704e9, 0.0013, 0.0035, 2.4,
                   2.44e7, static_cast<sw::i32>(SurfaceStyle::Ganymede),
                   {0.62f, 0.60f, 0.58f, 1.0f}, deposit(0x6A9Eu, 0.28f, 0.70f, 0.3f)},
                  {"CALLISTO", 2.4103e6, 7.179e12, 1.8827e9, 0.0074, 0.0034, 4.0,
                   3.77e7, static_cast<sw::i32>(SurfaceStyle::Callisto),
                   {0.45f, 0.42f, 0.40f, 1.0f}, deposit(0xCA77u, 0.32f, 0.60f, 0.3f)}}},
                // ---- Saturn: the rings ride a shell entity below ---------------
                {"SATURN", kSaturnRadius, kMuSaturn, kSaturnSma, 0.0565, 0.0434, 2.7,
                 kSaturnSoi, 1.637e-4, {0.4497f, 0.8931f, 0.0f},
                 static_cast<sw::i32>(SurfaceStyle::Saturn),
                 {0.88f, 0.78f, 0.58f, 1.0f}, false, {0.19, 59500.0, 1.0e7},
                 noDeposits, true,
                 {{"ENCELADUS", 2.521e5, 7.21e9, 2.3795e8, 0.0047, 0.0002, 0.4,
                   4.87e5, static_cast<sw::i32>(SurfaceStyle::Enceladus),
                   {0.92f, 0.95f, 1.0f, 1.0f}, deposit(0xE7CEu, 0.05f, 0.95f, 0.1f)},
                  {"RHEA", 7.638e5, 1.539e11, 5.2704e8, 0.0013, 0.0060, 1.9, 3.65e6,
                   static_cast<sw::i32>(SurfaceStyle::Rhea),
                   {0.70f, 0.70f, 0.70f, 1.0f}, deposit(0x8EAAu, 0.20f, 0.75f, 0.2f)},
                  {"TITAN", 2.5747e6, 8.978e12, 1.2219e9, 0.0288, 0.0061, 5.1,
                   4.33e7, static_cast<sw::i32>(SurfaceStyle::Titan),
                   {0.85f, 0.62f, 0.30f, 1.0f}, deposit(0x717Au, 0.22f, 0.70f, 0.4f)}}},
                // ---- Uranus: rolling on its side (97.8 deg) --------------------
                {"URANUS", kUranusRadius, kMuUranus, kUranusSma, 0.0457, 0.0135, 5.4,
                 kUranusSoi, 1.012e-4, {0.9908f, -0.1352f, 0.0f},
                 static_cast<sw::i32>(SurfaceStyle::Uranus),
                 {0.60f, 0.82f, 0.85f, 1.0f}, false, {0.42, 27700.0, 4.0e6},
                 noDeposits, false,
                 {{"TITANIA", 7.884e5, 2.28e11, 4.363e8, 0.0011, 0.0017, 0.9, 7.55e6,
                   static_cast<sw::i32>(SurfaceStyle::Titania),
                   {0.66f, 0.64f, 0.62f, 1.0f}, deposit(0x717Bu, 0.30f, 0.60f, 0.3f)},
                  {"OBERON", 7.614e5, 1.92e11, 5.835e8, 0.0014, 0.0012, 3.3, 9.4e6,
                   static_cast<sw::i32>(SurfaceStyle::Oberon),
                   {0.64f, 0.58f, 0.53f, 1.0f}, deposit(0x0BE0u, 0.32f, 0.55f, 0.3f)}}},
                // ---- Neptune and its captured, backwards moon ------------------
                {"NEPTUNE", kNeptuneRadius, kMuNeptune, kNeptuneSma, 0.0113, 0.0309,
                 4.7, kNeptuneSoi, 1.083e-4, {0.4744f, 0.8803f, 0.0f},
                 static_cast<sw::i32>(SurfaceStyle::Neptune),
                 {0.35f, 0.50f, 0.90f, 1.0f}, false, {0.45, 20000.0, 4.0e6},
                 noDeposits, false,
                 // Triton's orbit is RETROGRADE (i = 157 deg): the one moon
                 // in the sky that rises where the others set.
                 {{"TRITON", 1.3534e6, 1.428e12, 3.5476e8, 0.000016, 2.7402, 2.2,
                   1.20e7, static_cast<sw::i32>(SurfaceStyle::Triton),
                   {0.82f, 0.75f, 0.72f, 1.0f}, deposit(0x7217u, 0.15f, 0.85f, 0.3f)}}},
            };

            // Mars's moons, added to the planet built above: two captured
            // asteroids so small (11 and 6 km) that their SOI would sit
            // INSIDE their own rock — so they get no gravity of their own
            // and no ground: scenery on rails, exactly what they are until
            // a docking-scale visit makes them a milestone.
            const std::vector<MoonDef> marsMoons = {
                {"PHOBOS", 1.1267e4, 7.11e5, 9.376e6, 0.0151, 0.0192, 0.8, 0.0, -1,
                 {0.55f, 0.50f, 0.46f, 1.0f}, noDeposits},
                {"DEIMOS", 6.2e3, 9.85e4, 2.346e7, 0.00033, 0.0164, 3.9, 0.0, -1,
                 {0.58f, 0.54f, 0.50f, 1.0f}, noDeposits},
            };

            const auto buildMoon = [&](const MoonDef& moon, sw::ecs::Entity parent,
                                       const sw::WorldVec3& parentPos, sw::f64 parentMu) {
                const sw::phys::KeplerOrbit orbit = sw::phys::kepler::fromElements(
                    parentMu, moon.sma, moon.ecc, moon.incRad, 0.0, 0.0, moon.m0, 0.0);
                const sw::ecs::Entity e = m_world.createEntity();
                TransformComponent transform{};
                sw::WorldVec3 rel{};
                sw::phys::kepler::evaluate(orbit, 0.0, rel);
                transform.position = parentPos + rel;
                transform.uniformScale = static_cast<sw::f32>(moon.radius);
                m_world.addComponent(e, transform);
                m_world.addComponent(e, snapshotOf(transform));
                m_world.addComponent(e, BoundsComponent{1.0f});
                m_world.addComponent(
                    e, makeSphereLodSet({0.55f, 0.53f, 0.50f, 1.0f}, moon.style,
                                        moon.radius));
                // Tidally locked: one turn per orbit, from the orbit itself.
                const sw::f64 lockedRate =
                    std::sqrt(parentMu / (moon.sma * moon.sma * moon.sma));
                m_world.addComponent(
                    e, SpinComponent{{0.0f, 1.0f, 0.0f},
                                     static_cast<sw::f32>(lockedRate)});
                if (moon.soi > 0.0)
                {
                    sw::phys::GravitySourceComponent gravity{moon.mu, moon.radius};
                    gravity.soiRadius = moon.soi;
                    gravity.angularVelocity = {0.0, lockedRate, 0.0};
                    m_world.addComponent(e, gravity);
                    m_world.addComponent(e, sw::planet::terrainPreset(moon.style));
                    m_world.addComponent(e, moon.deposits);
                }
                m_world.addComponent(
                    e, sw::space::makeCelestialBody(moon.name, parent, &orbit));
                m_world.addComponent(e, MapMarkerComponent{moon.mapColor});
            };

            for (const PlanetDef& planet : planets)
            {
                const sw::phys::KeplerOrbit orbit = sw::phys::kepler::fromElements(
                    kMuSol, planet.sma, planet.ecc, planet.incRad, 0.0, 0.0,
                    planet.m0, 0.0);
                const sw::ecs::Entity e = m_world.createEntity();
                TransformComponent transform{};
                sw::phys::kepler::evaluate(orbit, 0.0, transform.position);
                transform.uniformScale = static_cast<sw::f32>(planet.radius);
                const sw::WorldVec3 planetPos = transform.position;
                m_world.addComponent(e, transform);
                m_world.addComponent(e, snapshotOf(transform));
                m_world.addComponent(e, BoundsComponent{1.0f});
                m_world.addComponent(
                    e, makeSphereLodSet({0.6f, 0.6f, 0.6f, 1.0f}, planet.style,
                                        planet.landable ? planet.radius : 0.0));
                m_world.addComponent(
                    e, SpinComponent{planet.spinAxis,
                                     static_cast<sw::f32>(planet.spinRate)});
                sw::phys::GravitySourceComponent gravity{planet.mu, planet.radius};
                gravity.soiRadius = planet.soi;
                gravity.angularVelocity =
                    sw::WorldVec3(planet.spinAxis) * planet.spinRate;
                m_world.addComponent(e, gravity);
                if (planet.atmosphere.topAltitude > 0.0)
                {
                    m_world.addComponent(e, planet.atmosphere);
                }
                if (planet.landable)
                {
                    m_world.addComponent(e, sw::planet::terrainPreset(planet.style));
                    m_world.addComponent(e, planet.deposits);
                }
                m_world.addComponent(
                    e, sw::space::makeCelestialBody(planet.name, m_solEntity, &orbit));
                m_world.addComponent(e, MapMarkerComponent{planet.mapColor});

                if (planet.rings)
                {
                    // The rings: a transparent annulus glued to the planet.
                    const sw::u32 ringMeshId = registerMesh(
                        renderer().createMesh(buildRingMesh(planet.spinAxis)));
                    const sw::ecs::Entity ring = m_world.createEntity();
                    TransformComponent ringTransform{};
                    ringTransform.position = planetPos;
                    ringTransform.uniformScale =
                        static_cast<sw::f32>(planet.radius);
                    m_world.addComponent(ring, ringTransform);
                    m_world.addComponent(ring, snapshotOf(ringTransform));
                    m_world.addComponent(ring, BoundsComponent{2.3f});
                    MeshComponent ringMesh{ringMeshId};
                    ringMesh.transparent = MeshComponent::kLitTransparent;
                    m_world.addComponent(ring, ringMesh);
                    m_world.addComponent(
                        ring, CloudLayerComponent{e, planet.spinAxis, 0.0f});
                }

                for (const MoonDef& moon : planet.moons)
                {
                    buildMoon(moon, e, planetPos, planet.mu);
                }
            }

            // Mars's moons attach to the Mars entity built above this table.
            {
                sw::ecs::Entity marsEntity{};
                sw::WorldVec3 marsPos{};
                m_world.forEach<sw::space::CelestialBodyComponent,
                                TransformComponent>(
                    [&](sw::ecs::Entity entity,
                        sw::space::CelestialBodyComponent& body,
                        TransformComponent& bodyTransform) {
                        if (std::string_view(body.name) == "MARS")
                        {
                            marsEntity = entity;
                            marsPos = bodyTransform.position;
                        }
                    });
                if (!marsEntity.isNull())
                {
                    for (const MoonDef& moon : marsMoons)
                    {
                        buildMoon(moon, marsEntity, marsPos, kMuMars);
                    }
                }
            }
        }

        // ============ EVERYTHING ELSE LIVES IN TERRA'S SOI ========================
        // Spawns are TERRA-relative: dynamic bodies add Terra's world
        // position and orbital velocity; rails objects simply reference
        // Terra as their primary (relative elements unchanged).
        sw::WorldVec3 terraVel0{};
        {
            sw::WorldVec3 unused{};
            sw::phys::kepler::evaluate(terraOrbit, 0.0, unused, &terraVel0);
        }
        const sw::WorldVec3 stationCenter =
            terraPos0 + sw::WorldVec3{0.0, 0.0, kStationOrbitRadius};

        // ---- GROUND OUTPOST: the first FACTORY SITE, built from data ------------
        // F1 turns the old hard-coded mining rig into what the player will
        // build in F2: a hub that owns a site, and buildings that are
        // .swpart definitions running .swrecipe recipes. Nothing below
        // hard-codes a rate, a power figure or a chain — it names catalogue
        // ids and lets the data say what they mean.
        //
        // Anchored in Terra's rotating frame: the site rides the planet's
        // rotation and survives save/load at its exact construction site.
        {
            const sw::planet::TerrainComponent terrain = presetTerra();
            const sw::planet::DepositComponent deposits = sw::planet::depositsTerra();

            // ---- the SURVEY: where the ore actually is ---------------------
            // The old rig sat at the equator on +Z because that was a
            // convenient number. A mine's yield is now the analytic deposit
            // density under its feet, so the scene does what a player will
            // do in F2: scan the neighbourhood and site the mine on the best
            // ground it finds. Deterministic — the same world always builds
            // the same outpost.
            const sw::Vec3 siteUp = terraStartSite();

            // Local tangent frame of the site: buildings are laid out in
            // metres on this plane, which is exactly the frame F2's ground
            // build mode will place them in.
            const sw::Vec3 siteEast = glm::normalize(
                glm::cross(sw::Vec3{0.0f, 1.0f, 0.0f}, siteUp));
            const sw::Vec3 siteNorth = glm::cross(siteUp, siteEast);
            // Model space is Y-up; the anchor's local rotation stands the
            // building on the local vertical.
            const sw::Quat standUp = standUpFor(siteUp);

            sw::ecs::Entity hubEntity{};
            // Spawns one building from its catalogue definition: geometry,
            // power, storage and siting rules all come from the .swpart.
            auto spawnBuilding = [&](sw::u32 definitionId, sw::f32 eastMetres,
                                     sw::f32 northMetres, sw::u32 recipeId,
                                     const sw::Vec4& marker, sw::f32 yawRadians = 0.0f) {
                // Offsets are metres on the tangent plane, re-normalised so
                // every building sits on the sphere — and then handed to the
                // very same placement the player's build cursor uses.
                const sw::Vec3 direction = glm::normalize(
                    siteUp + siteEast * (eastMetres / static_cast<sw::f32>(kTerraRadius)) +
                    siteNorth * (northMetres / static_cast<sw::f32>(kTerraRadius)));
                return placeBuilding(definitionId, terraEntity, direction, yawRadians,
                                     recipeId, hubEntity, marker);
            };

            // The hub defines the site; everything else points back at it.
            hubEntity = spawnBuilding(sw::parts::kBuildingHub, 0.0f, 0.0f, 0u, {});
            if (!hubEntity.isNull())
            {
                sw::factory::SiteComponent site{};
                std::snprintf(site.name, sizeof(site.name), "%s", "TERRA ALPHA");
                site.body = terraEntity;
                m_world.addComponent(hubEntity, site);
                m_world.getComponent<sw::factory::BuildingComponent>(hubEntity).site =
                    hubEntity;
            }

            // Each machine is turned so its conveyor mouth faces the belt it
            // feeds: the miner ships south to the smelter, the smelter ships
            // west to the silo. The port directions come from the .swpart,
            // so re-authoring a mouth in Part Studio re-aims the machine.
            const auto* minerPart = sw::parts::findDefinition(sw::parts::kBuildingMiner);
            const auto* refineryPart =
                sw::parts::findDefinition(sw::parts::kBuildingRefinery);
            const auto* storagePart =
                sw::parts::findDefinition(sw::parts::kBuildingStorage);
            auto portDirection = [](const sw::parts::PartDefinition* definition,
                                    sw::parts::NodeType type) {
                const sw::parts::AttachNode* node =
                    (definition != nullptr) ? sw::parts::findConveyorNode(*definition, type)
                                            : nullptr;
                return (node != nullptr) ? node->direction : sw::Vec3{0.0f, 0.0f, 1.0f};
            };

            const sw::ecs::Entity minerEntity = spawnBuilding(
                sw::parts::kBuildingMiner, 34.0f, 0.0f, sw::factory::kRecipeMineIronOre,
                {},
                yawToFace(siteUp, portDirection(minerPart, sw::parts::NodeType::ConveyorOut),
                          -siteNorth));
            const sw::ecs::Entity refineryEntity = spawnBuilding(
                sw::parts::kBuildingRefinery, 34.0f, -30.0f,
                sw::factory::kRecipeSmeltIron, {},
                yawToFace(siteUp, portDirection(refineryPart, sw::parts::NodeType::ConveyorIn),
                          siteNorth));
            const sw::ecs::Entity storageEntity = spawnBuilding(
                sw::parts::kBuildingStorage, 0.0f, -30.0f, 0u, {},
                yawToFace(siteUp, portDirection(storagePart, sw::parts::NodeType::ConveyorIn),
                          siteEast));
            const sw::ecs::Entity solarEntity =
                spawnBuilding(sw::parts::kBuildingSolarFarm, -34.0f, -15.0f, 0u, {});

            // F3: the BANK. A site with panels and no storage is a site that
            // stops every sunset, so the starting outpost is delivered with
            // one — half charged, which makes the first night a decision
            // (run the smelter now, or keep the charge?) instead of a
            // scripted blackout.
            const sw::ecs::Entity batteryEntity =
                spawnBuilding(sw::parts::kBuildingBatteryBank, -34.0f, 14.0f, 0u, {});
            if (!batteryEntity.isNull())
            {
                sw::factory::inventoryAdd(
                    m_world.getComponent<sw::factory::InventoryComponent>(batteryEntity),
                    sw::res::Resource::ElectricCharge, 500000.0); // 500 MJ
            }

            // The BEACON: the site is at a surveyed spot on a 6,371 km
            // sphere, and nothing else here can be seen from the air. Its
            // 25 m lit mast finds you on the ground; the pointer it puts on
            // the map and on the HUD finds you from orbit.
            const sw::ecs::Entity beaconEntity =
                spawnBuilding(sw::parts::kBuildingBeacon, -14.0f, 22.0f, 0u,
                              {1.0f, 0.78f, 0.28f, 1.0f});
            if (!beaconEntity.isNull())
            {
                sw::factory::BeaconComponent beacon{};
                std::snprintf(beacon.label, sizeof(beacon.label), "%s", "TERRA ALPHA");
                beacon.rangeM = 1.0e6;    // 1000 km: visible from low orbit
                beacon.nearRangeM = 500.0; // ...and out of the way once you land
                m_world.addComponent(beaconEntity, beacon);
            }

            // ---- F5: THE VAB AND THE PAD ----------------------------------
            // The far end of the yard, and the reason the near end exists.
            // The hall faces the pad, the pad faces the hall, and the belt
            // between them carries finished rockets — the one belt on this
            // planet whose cargo is a vehicle.
            //
            // The pad stands where new vessels have always appeared (120 m
            // east of the hub), so it is now a REAL building at the place
            // that used to be a computed guess.
            const auto* vabPart = sw::parts::findDefinition(sw::parts::kBuildingVab);
            const auto* padPart =
                sw::parts::findDefinition(sw::parts::kBuildingLaunchPad);
            const sw::ecs::Entity vabEntity = spawnBuilding(
                sw::parts::kBuildingVab, 75.0f, 0.0f, 0u, {},
                yawToFace(siteUp, portDirection(vabPart, sw::parts::NodeType::ConveyorOut),
                          siteEast));
            const sw::ecs::Entity padEntity = spawnBuilding(
                sw::parts::kBuildingLaunchPad, 120.0f, 0.0f, 0u,
                {0.4f, 0.9f, 0.92f, 1.0f},
                yawToFace(siteUp, portDirection(padPart, sw::parts::NodeType::ConveyorIn),
                          -siteEast));
            if (!vabEntity.isNull())
            {
                // Enough metal for a first hull, and NO order standing. The
                // outpost is stocked, not started: the player designs
                // something in the hangar and orders it, which is the whole
                // loop this milestone exists to make mandatory. Seeding an
                // order here would just re-create the starting rocket that
                // was deleted a hundred lines above.
                auto& bin =
                    m_world.getComponent<sw::factory::InventoryComponent>(vabEntity);
                sw::factory::inventoryAdd(bin, sw::res::Resource::Iron, 3000.0);
                sw::factory::inventoryAdd(bin, sw::res::Resource::Copper, 500.0);
            }
            if (!padEntity.isNull())
            {
                // ...and fuel on the pad, so the first rocket rolls out with
                // something in its tanks.
                sw::factory::inventoryAdd(
                    m_world.getComponent<sw::factory::InventoryComponent>(padEntity),
                    sw::res::Resource::Fuel, 16000.0);
            }

            // ---- the BELTS, laid by the very tool the player uses ---------
            // Two clicks' worth of work: pick the machine that ships, pick
            // the one that receives, and `planBelt` produces the run. The
            // starting outpost gets no shortcut — if the tool could not lay
            // this belt, neither could the scene.
            auto layBelt = [&](sw::ecs::Entity from, sw::ecs::Entity to) {
                std::vector<BeltTile> tiles;
                const sw::build::Verdict verdict = planBelt(terraEntity, from, to, tiles);
                if (verdict != sw::build::Verdict::Ok)
                {
                    SW_LOG_WARN("Game", "Starting belt refused: {}",
                                sw::build::verdictText(verdict));
                }
                for (const BeltTile& tile : tiles)
                {
                    placeBuilding(sw::parts::kBuildingConveyor, terraEntity,
                                  tile.direction, tile.yawRadians, 0u, hubEntity, {});
                }
            };
            layBelt(minerEntity, refineryEntity);
            layBelt(refineryEntity, storageEntity);
            layBelt(vabEntity, padEntity);

            // ...and now derive what those rows of segments actually connect.
            rebuildConveyorNetwork();

            // ---- the GRID, wired the way the player would wire it --------
            // One PL-1 in the middle of the yard, and a single span from it
            // to every machine. That is not decoration: without the pole the
            // outpost could not be one grid at all, because a building takes
            // exactly one cable and seven of them cannot form a chain.
            // The starting base gets no exemption from the rule it teaches.
            const sw::ecs::Entity poleEntity =
                spawnBuilding(sw::parts::kBuildingPowerPole, 10.0f, -14.0f, 0u, {});
            // A second pole out by the pad: a span is 120 m at most, and the
            // launch complex is further from the yard than that rule allows
            // in one hop. Poles are how a grid crosses ground, which is the
            // whole reason they exist.
            const sw::ecs::Entity farPoleEntity =
                spawnBuilding(sw::parts::kBuildingPowerPole, 97.0f, -16.0f, 0u, {});
            rebuildPowerNetwork(); // number the nodes before asking about them
            if (!poleEntity.isNull() && !farPoleEntity.isNull())
            {
                layCable(terraEntity, poleEntity, farPoleEntity);
            }
            if (!farPoleEntity.isNull())
            {
                for (const sw::ecs::Entity end : {vabEntity, padEntity})
                {
                    if (end.isNull())
                    {
                        continue;
                    }
                    sw::WorldVec3 from{};
                    sw::WorldVec3 to{};
                    if (planCable(farPoleEntity, end, from, to) !=
                        sw::factory::CableVerdict::Ok)
                    {
                        SW_LOG_WARN("Game", "Launch complex cable refused");
                        continue;
                    }
                    layCable(terraEntity, farPoleEntity, end);
                }
            }
            if (!poleEntity.isNull())
            {
                for (const sw::ecs::Entity end :
                     {hubEntity, minerEntity, refineryEntity, storageEntity, solarEntity,
                      batteryEntity, beaconEntity})
                {
                    if (end.isNull())
                    {
                        continue;
                    }
                    sw::WorldVec3 from{};
                    sw::WorldVec3 to{};
                    const sw::factory::CableVerdict verdict =
                        planCable(poleEntity, end, from, to);
                    if (verdict != sw::factory::CableVerdict::Ok)
                    {
                        SW_LOG_WARN("Game", "Starting cable refused: {}",
                                    sw::factory::cableVerdictText(verdict));
                        continue;
                    }
                    layCable(terraEntity, poleEntity, end);
                }
            }

            if (!hubEntity.isNull())
            {
                auto& site = m_world.getComponent<sw::factory::SiteComponent>(hubEntity);
                m_world.forEach<sw::factory::BuildingComponent,
                                sw::factory::PowerComponent>(
                    [&](sw::ecs::Entity, sw::factory::BuildingComponent& building,
                        sw::factory::PowerComponent& power) {
                        if (building.site != hubEntity)
                        {
                            return;
                        }
                        site.producedKw += power.producedKw;
                        site.consumedKw += power.consumedKw;
                        ++site.buildingCount;
                    });
                SW_LOG_INFO("Game",
                            "Site '{}': {} buildings, {:.0f} kW produced / {:.0f} kW "
                            "demanded, iron grade {:.2f}",
                            site.name, site.buildingCount, site.producedKw,
                            site.consumedKw,
                            !minerEntity.isNull()
                                ? m_world
                                      .getComponent<sw::factory::BuildingComponent>(
                                          minerEntity)
                                      .groundDensity
                                : 0.0f);
            }
        }

        // ---- THE PLAYER: on foot, at the base --------------------------------
        //
        // There is no starting rocket any more, and that is the point. A
        // vessel exists in this world only because a VAB was given a design
        // and the iron and copper to build it; handing the player one at
        // start-up made the entire assembly line decorative.
        //
        // So the player starts as a suit standing on the pad apron at TERRA
        // ALPHA. On foot is the NORMAL state now — piloting is something you
        // board, not something you begin in.
        {
            const sw::ecs::Entity e = m_world.createEntity();

            // Ten metres north of the hub, on the ground, in the same tangent
            // frame every building was laid out in.
            const sw::Vec3 siteUp = terraStartSite();
            const sw::Vec3 siteEast =
                glm::normalize(glm::cross(sw::Vec3{0.0f, 1.0f, 0.0f}, siteUp));
            const sw::Vec3 siteNorth = glm::cross(siteUp, siteEast);
            const sw::Vec3 standDirection = glm::normalize(
                siteUp + siteNorth * (14.0f / static_cast<sw::f32>(kTerraRadius)));

            sw::f64 elevation = 0.0;
            if (const auto* terrain =
                    m_world.tryGetComponent<sw::planet::TerrainComponent>(terraEntity))
            {
                elevation = sw::planet::terrainElevation(*terrain, standDirection);
            }

            TransformComponent transform{};
            transform.position = terraPos0 + sw::WorldVec3(standDirection) *
                                                 (kTerraRadius + elevation + 2.0);
            transform.rotation = standUpFor(standDirection);
            m_world.addComponent(
                e, PreviousTransformComponent{transform.position, transform.rotation});
            m_world.addComponent(e, transform);
            m_world.addComponent(e, BoundsComponent{1.4f});
            m_world.addComponent(e, MeshComponent{m_capsuleMeshIndex});
            m_world.addComponent(e, MapMarkerComponent{{1.0f, 0.8f, 0.2f, 1.0f}});
            m_world.addComponent(e, m_capsuleHull);
            if (const auto* suit = sw::parts::findDefinition(sw::parts::kPropEvaSuit))
            {
                sw::phys::HullComponent hull{};
                if (hullFor(*suit, hull))
                {
                    m_world.addComponent(e, hull);
                    m_world.addComponent(e, sw::phys::HullMoverComponent{});
                }
            }
            m_world.addComponent(e, CapsuleComponent{});
            m_world.addComponent(e, ShipControlsComponent{});

            // THE CARRIER VELOCITY. A body standing on Terra is not still: it
            // is doing Terra's orbit plus Terra's spin at this latitude, some
            // 30 km/s and 465 m/s of it. Start it at rest in the world frame
            // and the ground leaves at half a kilometre a second.
            sw::WorldVec3 spin{0.0};
            if (const auto* gravity =
                    m_world.tryGetComponent<sw::phys::GravitySourceComponent>(terraEntity))
            {
                spin = glm::cross(gravity->angularVelocity,
                                  transform.position - terraPos0);
            }
            sw::phys::DynamicBodyComponent body{};
            body.velocity = terraVel0 + spin;
            body.mass = 120.0;
            body.ballisticFactor = 0.01;
            m_world.addComponent(e, body);

            m_capsuleEntity = e;
            m_evaMode = true;
        }

        // ---- F15: the ENDURANCE, parked at Saturn ----------------------------
        buildEndurance();
    }

    // ------------------------------------------------------------------------
    // F15 — THE ENDURANCE
    //
    // The ring ship from Interstellar, built the only honest way this engine
    // has: as PARTS. Ten .swpart definitions and one programmatic blueprint
    // that places thirty-five of them — the ship is an ASSEMBLY, exactly
    // what the hangar would produce, so the hangar can load it, its parts
    // carry real collision hulls, and the simulation bubble wakes it into a
    // genuine dynamic vessel the moment a player takes the controls.
    //
    // THE FILM'S OWN PARTS LIST, which is not the one guesswork produces:
    // twelve modules of five kinds — four PROPULSION (three plasma engines
    // apiece, twelve nozzles in all), four CARGO pods (detachable; they are
    // meant to come off and be the ground base), two HABITATS, one CRYO bay
    // and one COMMAND module — strung on twelve short TUNNELS, because the
    // modules do not touch. Inside the ring sits the CORE DOCKING HUB, six
    // ports for the support craft, held on six SPOKES. The ring is what you
    // recognise; the core is what you dock with.
    //
    // The geometry is one paragraph of trigonometry. Module centres sit on
    // the apothem a = 29.4 m of a regular dodecagon; the outer faces then
    // land at 32.0 m, so the ring measures the film's 64 m across. Each
    // polygon VERTEX is exactly a*tan(15 deg) = 7.88 m along the tangent
    // from its module's centre, which leaves 1.68 m of half-tunnel between
    // a 12.4 m module and the next — the joint is arithmetic, not a fudge
    // factor. The six spokes reach from the hub's 3.4 m skin to the inner
    // faces at 27.0 m, and land on every second module, which is the only
    // spacing that divides both six and twelve.
    //
    // VESSEL FRAME: the ring axis is Z and the nose is -Z, because -Z is
    // where this engine's thrust points and the Endurance is meant to be
    // FLOWN. That single choice is why the twelve nozzles, authored on each
    // propulsion module's own aft face, all end up firing out the ship's
    // tail without a special case.
    // ------------------------------------------------------------------------
    void StarWorksGame::buildEndurance()
    {
        // The shipped catalogue only: the built-in fallback has no 200-range
        // ids, and a blueprint of missing definitions would dereference null
        // in instantiateBlueprint. Refusing to build is the correct answer.
        if (sw::parts::findDefinition(sw::parts::kPartEnduranceCoreHub) == nullptr)
        {
            SW_LOG_WARN("Game", "ENDURANCE: catalogue has no 200-range parts, skipped");
            return;
        }

        // Saturn, by name — the same lookup the Mars moons use. The planet
        // table owns the entity; this function only borrows it.
        sw::ecs::Entity saturnEntity{};
        sw::WorldVec3 saturnPos{};
        m_world.forEach<sw::space::CelestialBodyComponent, TransformComponent>(
            [&](sw::ecs::Entity entity, sw::space::CelestialBodyComponent& body,
                TransformComponent& bodyTransform) {
                if (std::string_view(body.name) == "SATURN")
                {
                    saturnEntity = entity;
                    saturnPos = bodyTransform.position;
                }
            });
        if (saturnEntity.isNull())
        {
            SW_LOG_WARN("Game", "ENDURANCE: no SATURN in the scene, skipped");
            return;
        }

        // ---- the blueprint ---------------------------------------------------
        constexpr sw::f32 kApothem = 29.4f;      // module centres
        constexpr sw::f32 kModuleHalfLen = 6.2f; // 12.4 m modules
        constexpr sw::f32 kModuleHalfRad = 2.4f; // radial half thickness
        constexpr sw::f32 kCargoHalfRad = 2.6f;  // the pods are chunkier
        constexpr sw::f32 kHubRadius = 3.4f;
        constexpr sw::f32 kSpokeHalfLen = 11.8f;
        constexpr sw::u32 kModuleCount = 12;
        // tan(15 deg) and 1/cos(15 deg): where the polygon vertices are.
        const sw::f32 kTanHalf = std::tan(glm::pi<sw::f32>() / kModuleCount);
        const sw::f32 kSecHalf = 1.0f / std::cos(glm::pi<sw::f32>() / kModuleCount);

        // NODES BY NAME, never by index. A blueprint that says "node 2" is a
        // blueprint that silently re-plumbs itself the day somebody inserts
        // a port in Part Studio; this one says "dock" and fails loudly.
        const auto nodeIndex = [](sw::u32 definitionId, std::string_view name) -> sw::u8 {
            if (const auto* definition = sw::parts::findDefinition(definitionId))
            {
                for (sw::usize i = 0; i < definition->nodes.size(); ++i)
                {
                    if (definition->nodes[i].name == name)
                    {
                        return static_cast<sw::u8>(i);
                    }
                }
                SW_LOG_WARN("Game", "ENDURANCE: part {} has no node '{}'", definitionId,
                            name);
            }
            return 0;
        };

        std::vector<BlueprintPart> blueprint;
        blueprint.reserve(35);

        // Basis-to-quaternion, same convention as the pad spawn: the columns
        // are the images of the part's local X/Y/Z axes in the vessel frame.
        const auto orient = [](const sw::Vec3& x, const sw::Vec3& y,
                               const sw::Vec3& z) {
            return glm::quat_cast(sw::Mat3{x, y, z});
        };
        const sw::Vec3 axis{0.0f, 0.0f, 1.0f}; // ring axis; the nose is -Z

        // 0: THE CORE HUB, at the centre, its own axis already the ring's.
        {
            BlueprintPart hub{};
            hub.definitionId = sw::parts::kPartEnduranceCoreHub;
            blueprint.push_back(hub);
        }

        // WHICH MODULE GOES WHERE. Four propulsion at 90 degrees is the only
        // arrangement whose thrust passes through the centre of mass; the
        // four cargo pods likewise, so the ring stays balanced as they leave.
        // The habitats sit opposite each other and so do the command module
        // and the cryo bay, which is what keeps a spinning ring from
        // wobbling about an axis it was not built to turn on.
        const auto moduleKind = [](sw::u32 index) -> sw::u32 {
            switch (index)
            {
            case 0:
            case 3:
            case 6:
            case 9:
                return sw::parts::kPartEnduranceEngine;
            case 1:
            case 4:
            case 7:
            case 10:
                return sw::parts::kPartEnduranceCargo;
            case 2:
            case 8:
                return sw::parts::kPartEnduranceHabitat;
            case 5:
                return sw::parts::kPartEnduranceCommand;
            default:
                return sw::parts::kPartEnduranceCryo;
            }
        };

        // The ring, module and tunnel alternating. Each module's local frame
        // is X = outward radial, Y = ring axis, Z = its own long axis — and
        // the ring axis maps to -Z of the vessel, which is what turns every
        // module's authored aft face into the ship's aft face.
        //
        // The chain is a TREE, not a loop: tunnel 11 hangs off module 11 and
        // its far end merely abuts module 0 without a joint. A cycle is not
        // expressible as parent links, and pretending otherwise would put a
        // second parent on a part that can only have one.
        std::vector<sw::i32> moduleIndex(kModuleCount, -1);
        for (sw::u32 i = 0; i < kModuleCount; ++i)
        {
            const sw::f32 theta =
                static_cast<sw::f32>(i) * (glm::two_pi<sw::f32>() / kModuleCount);
            const sw::Vec3 radial{std::cos(theta), std::sin(theta), 0.0f};
            const sw::Vec3 tangent{-std::sin(theta), std::cos(theta), 0.0f};

            BlueprintPart module{};
            module.definitionId = moduleKind(i);
            module.localPosition = radial * kApothem;
            module.localRotation = orient(radial, -axis, tangent);
            if (i > 0)
            {
                module.parentIndex = static_cast<sw::i32>(blueprint.size()) - 1;
                module.parentPoint = nodeIndex(sw::parts::kPartEnduranceTunnel, "ringB");
                module.childPoint = nodeIndex(module.definitionId, "ringA");
            }
            else
            {
                // Module 0 is the root of the ring, hung off the hub through
                // the spoke placed for it below — see the spoke loop.
                module.parentIndex = -1;
            }
            moduleIndex[i] = static_cast<sw::i32>(blueprint.size());
            blueprint.push_back(module);

            // ...and the tunnel that leaves it, sitting on the vertex.
            const sw::f32 vertexTheta =
                theta + glm::pi<sw::f32>() / static_cast<sw::f32>(kModuleCount);
            const sw::Vec3 vertexRadial{std::cos(vertexTheta), std::sin(vertexTheta),
                                        0.0f};
            const sw::Vec3 vertexTangent{-std::sin(vertexTheta), std::cos(vertexTheta),
                                         0.0f};
            BlueprintPart tunnel{};
            tunnel.definitionId = sw::parts::kPartEnduranceTunnel;
            tunnel.localPosition = vertexRadial * (kApothem * kSecHalf);
            tunnel.localRotation = orient(vertexRadial, -axis, vertexTangent);
            tunnel.parentIndex = moduleIndex[i];
            tunnel.parentPoint = nodeIndex(module.definitionId, "ringB");
            tunnel.childPoint = nodeIndex(sw::parts::kPartEnduranceTunnel, "ringA");
            blueprint.push_back(tunnel);
        }

        // THE JOINT IS ARITHMETIC, and checked against the tunnel actually
        // shipped rather than trusted to two numbers agreeing in two files:
        // half a module plus half a tunnel must reach the polygon vertex at
        // a*tan(15 deg), or the ring does not close. Re-author EN-4 longer
        // in Part Studio and this says so instead of leaving twelve gaps.
        {
            const sw::f32 needed = kApothem * kTanHalf - kModuleHalfLen;
            sw::f32 authored = needed;
            if (const auto* tunnel =
                    sw::parts::findDefinition(sw::parts::kPartEnduranceTunnel))
            {
                for (const sw::parts::HitBox& box : sw::parts::effectiveHull(*tunnel))
                {
                    authored = std::abs(box.halfExtents.z);
                }
            }
            if (std::abs(authored - needed) > 0.05f)
            {
                SW_LOG_WARN("Game",
                            "ENDURANCE: the tunnel is {:.2f} m half-long and the "
                            "ring needs {:.2f} m — the joints will not meet",
                            authored, needed);
            }
        }

        // THE SIX SPOKES, hub skin to module inner face, on every second
        // module — the only spacing that divides both six and twelve. Their
        // local +Z runs OUTWARD, so node "hub" faces the core and "ring"
        // faces the module it carries.
        for (sw::u32 k = 0; k < 6; ++k)
        {
            const sw::u32 module = k * 2;
            const sw::f32 theta =
                static_cast<sw::f32>(module) * (glm::two_pi<sw::f32>() / kModuleCount);
            const sw::Vec3 radial{std::cos(theta), std::sin(theta), 0.0f};
            const sw::Vec3 tangent{-std::sin(theta), std::cos(theta), 0.0f};
            BlueprintPart spoke{};
            spoke.definitionId = sw::parts::kPartEnduranceSpoke;
            spoke.localPosition = radial * (kHubRadius + kSpokeHalfLen);
            spoke.localRotation = orient(axis, -tangent, radial);
            spoke.parentIndex = 0; // the hub
            spoke.parentPoint = nodeIndex(sw::parts::kPartEnduranceCoreHub,
                                          "spoke" + std::to_string(k));
            spoke.childPoint = nodeIndex(sw::parts::kPartEnduranceSpoke, "hub");
            blueprint.push_back(spoke);
        }
        // ...and module 0 hangs off its own spoke, which is what ties the
        // ring to the core in the part TREE. The other five spokes meet
        // their modules geometrically without a second joint, for the same
        // reason the twelfth tunnel does: one parent each.
        blueprint[static_cast<sw::usize>(moduleIndex[0])].parentIndex =
            static_cast<sw::i32>(blueprint.size()) - 6;
        blueprint[static_cast<sw::usize>(moduleIndex[0])].parentPoint =
            nodeIndex(sw::parts::kPartEnduranceSpoke, "ring");
        blueprint[static_cast<sw::usize>(moduleIndex[0])].childPoint =
            nodeIndex(sw::parts::kPartEnduranceEngine, "spoke");

        // The four support craft, belly (or roof) against the outer rim of
        // the four cargo pods, noses forward along the ring axis. Rangers
        // opposite each other, Landers likewise — four masses at 90 degrees
        // on a ring that spins is not a detail, it is the difference between
        // artificial gravity and a wobble.
        //
        // A Ranger sits on its BELLY (its floor port, the one the film docks
        // with); a Lander hangs by its ROOF, because its underside is all
        // legs and lift engines and they have to face out.
        struct CraftMount
        {
            sw::u32 module;      // which cargo pod carries it
            sw::u32 definition;  // Ranger or Lander
            sw::f32 standOff;    // its own dock node's distance from centre
            bool roof;           // true = mounted by its roof (+Y local)
        };
        const CraftMount craft[] = {
            {1, sw::parts::kPartEnduranceRanger, 0.85f, false},
            {7, sw::parts::kPartEnduranceRanger, 0.85f, false},
            {4, sw::parts::kPartEnduranceLander, 1.5f, true},
            {10, sw::parts::kPartEnduranceLander, 1.5f, true},
        };
        for (const CraftMount& mount : craft)
        {
            const sw::f32 theta = static_cast<sw::f32>(mount.module) *
                                  (glm::two_pi<sw::f32>() / kModuleCount);
            const sw::Vec3 radial{std::cos(theta), std::sin(theta), 0.0f};
            const sw::Vec3 tangent{-std::sin(theta), std::cos(theta), 0.0f};
            BlueprintPart part{};
            part.definitionId = mount.definition;
            part.localPosition = radial * (kApothem + kCargoHalfRad + mount.standOff);
            // Nose (local -Z) forward, so local Z is the ring axis in both
            // cases; the mounting face is what differs: belly-down puts
            // local +Y outward, roof-down puts it inward.
            part.localRotation = mount.roof ? orient(tangent, -radial, axis)
                                            : orient(-tangent, radial, axis);
            part.parentIndex = moduleIndex[mount.module];
            part.parentPoint = nodeIndex(sw::parts::kPartEnduranceCargo, "dock");
            part.childPoint = nodeIndex(mount.definition, "dock");
            blueprint.push_back(part);
        }

        // The honest mass: the sum of what the catalogue says the parts
        // weigh, so the rails->dynamic hand-off wakes the ship at the mass
        // VesselAssemblySystem will immediately recompute from those same
        // parts.
        sw::f64 dryMassKg = 0.0;
        for (const BlueprintPart& bp : blueprint)
        {
            if (const auto* definition = sw::parts::findDefinition(bp.definitionId))
            {
                dryMassKg += definition->dryMassKg;
            }
        }

        // ---- AND IT HAS TO BALANCE ------------------------------------------
        //
        // Every comment above claims it does — four propulsion modules at
        // ninety degrees "so the thrust passes through the centre of mass",
        // habitats opposite each other, Rangers and Landers likewise, "which
        // is what keeps a spinning ring from wobbling about an axis it was
        // not built to turn on". None of that was ever checked, and it was
        // not true: a 20 t command pod hung opposite a 22 t cryo bay, which
        // put the balance point 11.7 cm off the axle. Thrust through a point
        // that is not the balance point is a lever — 88 kN on 11.7 cm is
        // 10.3 kN m — so the ship yawed under acceleration with all four
        // engines lit, and the pilot who reported it was right.
        //
        // The arithmetic is four lines and it is the same shape as the tunnel
        // check above: state the property, measure it, and say so rather than
        // trusting two files to agree. It is the RADIAL offset that matters;
        // along the axle the balance point may sit wherever the design puts
        // it, because that is the direction the engines push.
        {
            sw::Vec3 moment{0.0f};
            sw::f32 mass = 0.0f;
            for (const BlueprintPart& bp : blueprint)
            {
                if (const auto* definition = sw::parts::findDefinition(bp.definitionId))
                {
                    const auto m = static_cast<sw::f32>(definition->dryMassKg);
                    moment += bp.localPosition * m;
                    mass += m;
                }
            }
            const sw::Vec3 centre = (mass > 0.0f) ? (moment / mass) : sw::Vec3{0.0f};
            const sw::f32 offAxis = glm::length(sw::Vec3{centre.x, centre.y, 0.0f});
            if (offAxis > 0.02f)
            {
                SW_LOG_WARN("Game",
                            "ENDURANCE: the balance point is {:.3f} m off the ring "
                            "axis ({:.3f}, {:.3f}) — thrust will twist it by "
                            "{:.0f} N m",
                            offAxis, centre.x, centre.y,
                            offAxis * 4.0f *
                                static_cast<sw::f32>(
                                    sw::parts::findDefinition(
                                        sw::parts::kPartEnduranceEngine)
                                        ->thrustNewtons));
            }
        }

        // ---- the vessel root, ON RAILS around Saturn -------------------------
        // a = 4.0e8 m: 6.9 Saturn radii — clear of the rings (2.27 R), inside
        // Rhea, deep inside the SOI. Low eccentricity, near the moon plane.
        const sw::phys::KeplerOrbit orbit = sw::phys::kepler::fromElements(
            kMuSaturn, 4.0e8, 0.01, 0.05, 0.0, 0.0, 1.3, 0.0);
        sw::WorldVec3 relative{};
        sw::WorldVec3 relativeVelocity{};
        sw::phys::kepler::evaluate(orbit, 0.0, relative, &relativeVelocity);

        const sw::ecs::Entity root = m_world.createEntity();
        TransformComponent transform{};
        transform.position = saturnPos + relative;
        // Attitude belongs to the spin from here on: RailsSpinSystem holds
        // the ring's own axis along the SpinComponent's world axis and turns
        // it about that, which lays the ring flat in Saturn's orbital plane.
        m_world.addComponent(root, transform);
        m_world.addComponent(root, PreviousTransformComponent{transform.position,
                                                              transform.rotation});
        m_world.addComponent(root, BoundsComponent{0.1f});
        m_world.addComponent(root, MapMarkerComponent{{0.92f, 0.90f, 0.85f, 1.0f}});
        m_world.addComponent(root, sw::parts::VesselComponent{});
        // ---- and it is a SHIP, not scenery ------------------------------
        // Everything a pilot needs, on the root, from the first frame of a
        // new world: `P` boards the nearest vessel and this is one. The
        // angular figures are a 500-tonne ring's, not a rocket's — it turns
        // like something 64 m across, which is the point of flying it.
        ShipComponent ship{};
        ship.mainThrustNewtons = 0.0; // part-built: thrust comes from EN-2
        ship.angularAccel = 0.05f;
        ship.maxAngularSpeed = 0.2f;
        m_world.addComponent(root, ship);
        m_world.addComponent(root, ShipControlsComponent{});
        m_world.addComponent(root, SasComponent{});
        // The air's answer, refreshed every tick. Its PRESENCE is also the
        // switch that turns the old isotropic drag off for this vessel: a
        // part-built craft is flown by the tables AeroForge solved for it.
        m_world.addComponent(root, sw::aero::AeroStateComponent{});
        // 5.6 rpm — the film's own figure, and the one that puts a bit under
        // 1 g on the floor of a 32 m ring: w^2 r = 0.5864^2 x 32 = 11 m/s^2.
        constexpr sw::f32 kRingRpm = 5.6f;
        m_world.addComponent(
            root, SpinComponent{{0.0f, 1.0f, 0.0f},
                                kRingRpm * glm::two_pi<sw::f32>() / 60.0f});
        sw::phys::OnRailsComponent rails{};
        rails.orbit = orbit;
        rails.primary = saturnEntity;
        rails.dynamicMass = dryMassKg;
        rails.dynamicBallisticFactor = 0.002;
        m_world.addComponent(root, rails);

        // instantiateBlueprint reads the HANGAR's working list; borrow it
        // for the build and hand it back untouched. In buildScene that list
        // is empty, but the swap keeps this function honest wherever it is
        // called from.
        const sw::usize partCount = blueprint.size();
        std::vector<BlueprintPart> parked = std::move(m_blueprint);
        m_blueprint = std::move(blueprint);
        // The return is deliberately dropped: handed an existing root,
        // instantiateBlueprint populates it and gives the same entity back,
        // and this one is already in hand.
        static_cast<void>(instantiateBlueprint(root, {}));
        m_blueprint = std::move(parked);

        // ---- FUEL AND CHARGE, put in the tanks that were built empty -------
        // instantiateBlueprint fills only parts whose TYPE is FuelTank —
        // the rule that makes a tank a tank. The Endurance has none: its
        // propellant lives in the propulsion modules and its joules in the
        // habitats, so the ship is fuelled here, explicitly, exactly the way
        // the starting outpost's battery bank is charged.
        sw::f64 fuelKg = 0.0;
        sw::f64 chargeKj = 0.0;
        m_world.forEach<sw::parts::PartComponent, sw::factory::InventoryComponent>(
            [&](sw::ecs::Entity, sw::parts::PartComponent& part,
                sw::factory::InventoryComponent& inventory) {
                if (part.vessel != root)
                {
                    return;
                }
                const auto* definition = sw::parts::findDefinition(part.definitionId);
                if (definition == nullptr)
                {
                    return;
                }
                const sw::res::Resource resource = definition->capacities[0].resource;
                if (resource == sw::res::Resource::Count)
                {
                    return;
                }
                const sw::f64 units = definition->capacities[0].units;
                sw::factory::inventoryAdd(inventory, resource, units);
                if (resource == sw::res::Resource::Fuel)
                {
                    fuelKg += units;
                }
                else if (resource == sw::res::Resource::ElectricCharge)
                {
                    chargeKj += units;
                }
            });

        SW_LOG_INFO("Game",
                    "ENDURANCE: {} parts, {:.0f} t dry + {:.0f} t fuel, {:.0f} MJ, "
                    "{:.1f} m ring at {:.1f} rpm, on rails around SATURN",
                    partCount, dryMassKg / 1000.0, fuelKg / 1000.0, chargeKj / 1000.0,
                    2.0f * (kApothem + kModuleHalfRad), kRingRpm);
    }
} // namespace game
