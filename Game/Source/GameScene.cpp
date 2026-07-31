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
                                                          sw::i32 surfaceStyle)
    {
        // RELIEF SHADING (M22): the globe's vertex normals are tilted by
        // the gradient of the SAME analytic heightfield physics collides
        // with — mountain ranges catch the light and throw shadow flanks
        // from orbit, and they are exactly where the terrain patch will
        // put them when you land. Ocean stays flat (elevation clamps to 0,
        // gradient vanishes) and keeps its mirror specular.
        sw::planet::TerrainComponent terrain{};
        sw::f64 bodyRadius = 0.0;
        bool hasRelief = false;
        if (surfaceStyle >= 0)
        {
            switch (static_cast<SurfaceStyle>(surfaceStyle))
            {
            case SurfaceStyle::Terra:
                terrain = presetTerra();
                bodyRadius = kTerraRadius;
                hasRelief = true;
                break;
            case SurfaceStyle::Luna:
                terrain = presetLuna();
                bodyRadius = kLunaRadius;
                hasRelief = true;
                break;
            case SurfaceStyle::Mars:
                terrain = presetMars();
                bodyRadius = kMarsRadius;
                hasRelief = true;
                break;
            }
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
            // Slopes are physically tiny (9 km over thousands of km); an
            // exaggeration factor makes them READ from orbit without moving
            // a single vertex (silhouette and collision stay exact).
            constexpr sw::f32 kSlopeExaggeration = 220.0f;
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
                    vertex.normal = glm::normalize(
                        dir - (tangentA * slopeA + tangentB * slopeB) *
                                  kSlopeExaggeration);
                }
            }
            lod.meshIndex[level] = registerMesh(renderer().createMesh(sphere));
        }
        return lod;
    }

    void StarWorksGame::buildScene()
    {
        // ---- meshes -------------------------------------------------------------
        // Sol's colors exceed 1.0 slightly: paired with the emissive tint it
        // reads as a glowing star, not a lit rock.
        const CelestialLodComponent solLod =
            makeSphereLodSet({1.0f, 0.92f, 0.72f, 1.0f});
        const CelestialLodComponent terraLod = makeSphereLodSet(
            {0.21f, 0.33f, 0.48f, 1.0f}, static_cast<sw::i32>(SurfaceStyle::Terra));
        const CelestialLodComponent lunaLod = makeSphereLodSet(
            {0.42f, 0.41f, 0.43f, 1.0f}, static_cast<sw::i32>(SurfaceStyle::Luna));
        const CelestialLodComponent marsLod = makeSphereLodSet(
            {0.62f, 0.32f, 0.18f, 1.0f}, static_cast<sw::i32>(SurfaceStyle::Mars));

        // Environment meshes: the fixed star dome, Terra's atmosphere veil
        // and its drifting cloud shell.
        m_starfieldMeshIndex =
            registerMesh(renderer().createMesh(buildStarfieldMesh()));
        m_sunHaloMeshIndex = registerMesh(renderer().createMesh(buildGlowDiscMesh(
            {1.0f, 0.86f, 0.62f}, {1.0f, 0.5f, 0.22f}, 1.55f)));
        m_sunCoreMeshIndex = registerMesh(renderer().createMesh(buildGlowDiscMesh(
            {1.0f, 0.99f, 0.94f}, {1.0f, 0.86f, 0.55f}, 2.0f)));
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
            m_partMeshIds[definition.id] = registerMesh(
                renderer().createMesh(sw::parts::buildPartMesh(definition)));
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
            // Encloses the 80 km air column Shaders/Atmosphere.glsl marches
            // (a shell smaller than the model would clip the limb).
            transform.uniformScale = static_cast<sw::f32>(kTerraRadius * 1.0130); // ~83 km
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
    }
} // namespace game
