// ============================================================================
// PartsTests.cpp — the part system: catalog integrity, vessel aggregation
// (mass/thrust/drag from parts + carried resources) and rigid attachment.
// ============================================================================

#include <cstring>
#include "TestFramework.hpp"

#include <Core/FileSystem.hpp>
#include <ECS/World.hpp>
#include <Gameplay/PartGeometry.hpp>
#include <Gameplay/Parts.hpp>

#include <cmath>
#include <filesystem>

using namespace sw;
using namespace sw::parts;

SW_TEST(PartCatalogCoversEveryTypeWithSaneData)
{
    // Every one of the 9 part types exists, ids are unique and stable,
    // and every definition exposes the full property sheet.
    bool typeSeen[static_cast<usize>(PartType::Count)] = {};
    u32 seenIds[64] = {};
    usize idCount = 0;
    for (const PartDefinition& definition : catalog())
    {
        typeSeen[static_cast<usize>(definition.type)] = true;
        for (usize i = 0; i < idCount; ++i)
        {
            SW_CHECK(seenIds[i] != definition.id); // unique ids
        }
        seenIds[idCount++] = definition.id;

        SW_CHECK(definition.dryMassKg > 0.0);
        SW_CHECK(definition.costCredits > 0.0);
        SW_CHECK(definition.volumeM3 > 0.0);
        SW_CHECK(definition.crashToleranceMps > 0.0);
        SW_CHECK(definition.breakingForceN > 0.0);
        SW_CHECK(definition.dragCoefficientArea > 0.0);
        SW_CHECK(!definition.nodes.empty());  // every part connects
        SW_CHECK(!definition.shapes.empty()); // every part has geometry
        SW_CHECK(!definition.name.empty());
        SW_CHECK(findDefinition(definition.id) == &definition);
    }
    for (usize type = 0; type < static_cast<usize>(PartType::Count); ++type)
    {
        SW_CHECK(typeSeen[type]);
    }
    // Function-specific data.
    SW_CHECK(findDefinition(kPartEngineVector)->thrustNewtons > 0.0);
    SW_CHECK(findDefinition(kPartEngineVector)->specificImpulseS > 0.0);
    SW_CHECK(findDefinition(kPartSolarWing)->chargeRateKw > 0.0);
    SW_CHECK(findDefinition(kPartFinBasic)->liftCoefficient > 0.0);
    SW_CHECK(findDefinition(kPartFuelTankMedium)->capacities[0].resource ==
             res::Resource::Fuel);
}

SW_TEST(VesselAssemblyAggregatesPartsAndCargo)
{
    ecs::World world;

    // Vessel root.
    const ecs::Entity root = world.createEntity();
    world.addComponent(root, TransformComponent{});
    world.addComponent(root, VesselComponent{});
    world.addComponent(root, phys::DynamicBodyComponent{{0.0, 0.0, 0.0}, 1.0});

    auto addPart = [&world, root](u32 definitionId, const Vec3& localPosition) {
        const ecs::Entity part = world.createEntity();
        world.addComponent(part, TransformComponent{});
        world.addComponent(part, PreviousTransformComponent{});
        PartComponent component{};
        component.definitionId = definitionId;
        component.vessel = root;
        component.localPosition = localPosition;
        world.addComponent(part, component);
        return part;
    };
    addPart(kPartCoreStructural, {0.0f, 0.0f, -3.0f});
    const ecs::Entity tank = addPart(kPartFuelTankMedium, {0.0f, 0.0f, 0.0f});
    addPart(kPartEngineVector, {0.0f, 0.0f, 3.0f});

    factory::InventoryComponent inventory{};
    inventory.volumeCapacityM3 = 21.0;
    factory::inventoryAdd(inventory, res::Resource::Fuel, 16000.0);
    world.addComponent(tank, inventory);

    VesselAssemblySystem assembly;
    assembly.update(world, 0.02f);

    const auto& vessel = world.getComponent<VesselComponent>(root);
    const f64 expectedDry = findDefinition(kPartCoreStructural)->dryMassKg +
                            findDefinition(kPartFuelTankMedium)->dryMassKg +
                            findDefinition(kPartEngineVector)->dryMassKg;
    SW_CHECK_EQ(vessel.partCount, 3u);
    SW_CHECK(std::abs(vessel.dryMassKg - expectedDry) < 1.0e-6);
    // Total = dry + 16 t of fuel (1 unit = 1 kg), pushed into the body.
    SW_CHECK(std::abs(vessel.totalMassKg - (expectedDry + 16000.0)) < 1.0e-6);
    const auto& body = world.getComponent<phys::DynamicBodyComponent>(root);
    SW_CHECK(std::abs(body.mass - vessel.totalMassKg) < 1.0e-6);
    SW_CHECK(std::abs(vessel.maxThrustNewtons - 4.0e5) < 1.0);
    // Mass flow = F / (Isp * g0).
    SW_CHECK(std::abs(vessel.maxMassFlowKgps - 4.0e5 / (345.0 * 9.80665)) < 1.0e-6);
    SW_CHECK(body.ballisticFactor > 0.0);

    // Burn half the fuel: the vessel gets lighter on the next pass — the
    // rocket equation emerges from cargo mass alone.
    factory::inventoryRemove(world.getComponent<factory::InventoryComponent>(tank),
                             res::Resource::Fuel, 8000.0);
    assembly.update(world, 0.02f);
    SW_CHECK(std::abs(world.getComponent<VesselComponent>(root).totalMassKg -
                      (expectedDry + 8000.0)) < 1.0e-6);
}

SW_TEST(DecouplingSplitsTheVesselAlongTheJoint)
{
    ecs::World world;

    const ecs::Entity root = world.createEntity();
    TransformComponent rootTransform{};
    rootTransform.position = {1.0e6, 0.0, 0.0};
    world.addComponent(root, rootTransform);
    world.addComponent(root, PreviousTransformComponent{});
    world.addComponent(root, VesselComponent{});
    world.addComponent(root, phys::DynamicBodyComponent{{100.0, 0.0, 0.0}, 1.0});

    auto addPart = [&](u32 definitionId, f32 z) {
        const ecs::Entity part = world.createEntity();
        world.addComponent(part, TransformComponent{});
        world.addComponent(part, PreviousTransformComponent{});
        PartComponent component{};
        component.definitionId = definitionId;
        component.vessel = root;
        component.localPosition = {0.0f, 0.0f, z};
        world.addComponent(part, component);
        return part;
    };
    // core -- decoupler -- tank -- engine  (nose to tail)
    const ecs::Entity core = addPart(kPartCoreStructural, -4.0f);
    const ecs::Entity decoupler = addPart(kPartDecouplerFlat, -2.0f);
    const ecs::Entity tank = addPart(kPartFuelTankMedium, 0.5f);
    const ecs::Entity engine = addPart(kPartEngineVector, 3.5f);
    connectParts(world, core, decoupler, 1, 0, JointType::Stack, 2.5e5, 2.5e5);
    connectParts(world, decoupler, tank, 1, 0, JointType::Stack, 2.5e5, 2.5e5);
    connectParts(world, tank, engine, 1, 0, JointType::Stack, 4.0e5, 4.0e5);

    const ecs::Entity separated = decoupleAt(world, decoupler);
    SW_CHECK(!separated.isNull());

    // Core + decoupler stay; tank + engine ride the new vessel, which
    // inherited the velocity plus a separation shove.
    SW_CHECK(world.getComponent<PartComponent>(core).vessel == root);
    SW_CHECK(world.getComponent<PartComponent>(decoupler).vessel == root);
    SW_CHECK(world.getComponent<PartComponent>(tank).vessel == separated);
    SW_CHECK(world.getComponent<PartComponent>(engine).vessel == separated);
    const auto& body = world.getComponent<phys::DynamicBodyComponent>(separated);
    SW_CHECK(glm::length(body.velocity - WorldVec3{100.0, 0.0, 0.0}) > 0.5);
    // The decoupler-tank joint is gone: only 2 joints remain.
    u32 jointCount = 0;
    world.forEach<JointComponent>([&](ecs::Entity, JointComponent&) { ++jointCount; });
    SW_CHECK_EQ(jointCount, 2u);
}

SW_TEST(DockingMergesVesselsAndReroutesParts)
{
    ecs::World world;

    auto makeVessel = [&](const WorldVec3& position, const Quat& rotation) {
        const ecs::Entity vessel = world.createEntity();
        TransformComponent transform{};
        transform.position = position;
        transform.rotation = rotation;
        world.addComponent(vessel, transform);
        world.addComponent(vessel, PreviousTransformComponent{});
        world.addComponent(vessel, VesselComponent{});
        world.addComponent(vessel, phys::DynamicBodyComponent{{0.0, 0.0, 0.0}, 1.0});
        return vessel;
    };
    auto addPort = [&](ecs::Entity vessel, f32 z) {
        const ecs::Entity part = world.createEntity();
        world.addComponent(part, TransformComponent{});
        world.addComponent(part, PreviousTransformComponent{});
        PartComponent component{};
        component.definitionId = kPartDockingRing;
        component.vessel = vessel;
        component.localPosition = {0.0f, 0.0f, z};
        world.addComponent(part, component);
        return part;
    };

    const ecs::Entity vesselA = makeVessel({0.0, 0.0, 0.0}, {1, 0, 0, 0});
    const ecs::Entity vesselB = makeVessel({0.0, 0.0, -3.0}, {1, 0, 0, 0});
    const ecs::Entity portA = addPort(vesselA, -1.0f);
    const ecs::Entity portB = addPort(vesselB, 1.0f);

    // Attachment pass gives both ports their WORLD poses before docking.
    PartAttachmentSystem attachment;
    attachment.update(world, 0.02f);

    SW_CHECK(dockVessels(world, portA, portB));
    // B's port now belongs to vessel A, re-localized into A's frame
    // (B root was at z=-3, its port at local +1 -> world -2 -> A-local -2).
    const auto& merged = world.getComponent<PartComponent>(portB);
    SW_CHECK(merged.vessel == vesselA);
    SW_CHECK(std::abs(merged.localPosition.z - (-2.0f)) < 1.0e-3f);
    SW_CHECK(!world.isAlive(vesselB)); // absorbed root destroyed
    // A docking joint now links the two ports.
    u32 dockingJoints = 0;
    world.forEach<JointComponent>([&](ecs::Entity, JointComponent& joint) {
        if (joint.type == JointType::Docking &&
            ((joint.partA == portA && joint.partB == portB) ||
             (joint.partA == portB && joint.partB == portA)))
        {
            ++dockingJoints;
        }
    });
    SW_CHECK_EQ(dockingJoints, 1u);
}

SW_TEST(PartsRideTheirVesselRigidly)
{
    ecs::World world;

    const ecs::Entity root = world.createEntity();
    TransformComponent rootTransform{};
    rootTransform.position = {1000.0, 2000.0, 3000.0};
    rootTransform.rotation =
        glm::angleAxis(1.2f, glm::normalize(Vec3{0.3f, 1.0f, 0.2f}));
    world.addComponent(root, rootTransform);
    world.addComponent(root, PreviousTransformComponent{{900.0, 2000.0, 3000.0},
                                                        rootTransform.rotation});

    const ecs::Entity part = world.createEntity();
    world.addComponent(part, TransformComponent{});
    world.addComponent(part, PreviousTransformComponent{});
    PartComponent component{};
    component.definitionId = kPartFuelTankMedium;
    component.vessel = root;
    component.localPosition = {0.0f, 0.0f, 4.0f};
    world.addComponent(part, component);

    PartAttachmentSystem attachment;
    attachment.update(world, 0.02f);

    const auto& transform = world.getComponent<TransformComponent>(part);
    const WorldVec3 expected =
        rootTransform.position +
        WorldVec3(rootTransform.rotation * Vec3{0.0f, 0.0f, 4.0f});
    SW_CHECK(glm::length(transform.position - expected) < 1.0e-4);
    // Previous derived from the vessel's previous: lockstep interpolation.
    const auto& previous = world.getComponent<PreviousTransformComponent>(part);
    SW_CHECK(std::abs(previous.position.x -
                      (900.0 + (expected.x - 1000.0))) < 1.0e-4);
}

SW_TEST(ShippedCatalogLoadsWithNodesOnColliderSurfaces)
{
    // Load the SHIPPED .swpart files (copied next to the test binary) and
    // verify the property the whole VAB depends on: every attach node
    // sits ON the collider surface of its part — never inside, never
    // floating away (this was the pre-M19 bug).
    const std::filesystem::path directory =
        FileSystem::executableDirectory() / "Assets" / "Parts";
    SW_CHECK(parts::loadCatalog(directory));
    // The catalogue now holds two families in one id space (F1): rocket
    // parts, and BUILDINGS carrying an industrial block. Count them apart —
    // the VAB palette filters on exactly this predicate.
    // THREE families now, and the split is what the two palettes filter on:
    // vessel parts go in the VAB, buildings in the F build menu, and PROPS
    // (conveyor cargo) in neither — the game places those itself.
    usize rocketParts = 0;
    usize buildings = 0;
    usize props = 0;
    for (const parts::PartDefinition& definition : parts::catalog())
    {
        if (parts::isProp(definition)) { props += 1; }
        else if (parts::isBuilding(definition)) { buildings += 1; }
        else { rocketParts += 1; }
        // Exactly one family each: the predicates must not overlap.
        SW_CHECK(parts::isVesselPart(definition) !=
                 (parts::isBuilding(definition) || parts::isProp(definition)));
    }
    // + the ten ENDURANCE pieces (F15): five module kinds, the connecting
    // tunnel, the core hub and its spoke, and the two support craft — all
    // ordinary vessel parts.
    SW_CHECK_EQ(rocketParts, static_cast<usize>(19));
    // + CV-1, BT-1, PL-1, CW-1, and F5's VB-1 hall and LP-1 pad
    SW_CHECK_EQ(buildings, static_cast<usize>(12));
    // CR-1 crate, EV-1 suit, CR-2 vehicle cradle
    SW_CHECK_EQ(props, static_cast<usize>(3));

    // CONVEYOR PORTS. Every machine that takes part in a chain declares its
    // mouths on its geometry, and their DIRECTION is the contract: goods
    // leave through an out and arrive at an in.
    struct PortExpectation
    {
        u32 id;
        bool wantsIn;
        bool wantsOut;
    };
    const PortExpectation expectations[] = {
        {parts::kBuildingMiner, false, true},     // digs, ships out
        {parts::kBuildingRefinery, true, true},   // takes ore, ships metal
        {parts::kBuildingStorage, true, false},   // the end of a chain
        {parts::kBuildingHub, true, false},
        {parts::kBuildingConveyor, true, true},   // a belt is a chain link
    };
    for (const PortExpectation& expectation : expectations)
    {
        const parts::PartDefinition* definition =
            parts::findDefinition(expectation.id);
        SW_CHECK(definition != nullptr);
        if (definition == nullptr) { continue; }
        SW_CHECK_EQ(parts::findConveyorNode(*definition, parts::NodeType::ConveyorIn) !=
                        nullptr,
                    expectation.wantsIn);
        SW_CHECK_EQ(parts::findConveyorNode(*definition, parts::NodeType::ConveyorOut) !=
                        nullptr,
                    expectation.wantsOut);
    }

    // The type name survives a round trip through the .swpart JSON — the
    // whole point of the port being data is that Part Studio writes it.
    for (const parts::NodeType type :
         {parts::NodeType::Stack, parts::NodeType::Radial, parts::NodeType::ConveyorIn,
          parts::NodeType::ConveyorOut, parts::NodeType::Power})
    {
        parts::NodeType parsed = parts::NodeType::Count;
        SW_CHECK(parts::nodeTypeFromName(parts::nodeTypeName(type), parsed));
        SW_CHECK_EQ(parsed, type);
    }

    // The BC-1 beacon: a building whose product is being found. It must be
    // tall enough to be seen from the ground and cheap enough in power to
    // leave running, and it must be exempt from the ore rule (a beacon is
    // planted where you want to come back to, not where the iron is).
    const parts::PartDefinition* beacon = parts::findDefinition(parts::kBuildingBeacon);
    SW_CHECK(beacon != nullptr);
    if (beacon != nullptr)
    {
        SW_CHECK(parts::isBuilding(*beacon));
        SW_CHECK(beacon->building.category == factory::BuildingCategory::Beacon);
        SW_CHECK_EQ(beacon->building.minOreDensity, 0.0);
        SW_CHECK(beacon->building.powerKw < 0.0);   // it draws, never produces
        SW_CHECK(parts::partBoundsRadius(*beacon) > 10.0f); // a mast, not a box
    }

    for (const parts::PartDefinition& definition : parts::catalog())
    {
        bool anyCollider = false;
        for (const parts::PartShape& shape : definition.shapes)
        {
            anyCollider = anyCollider || shape.collider;
        }
        SW_CHECK(anyCollider); // every part has a real collision hull
        SW_CHECK(parts::partBoundsRadius(definition) > 0.1f);

        for (const parts::AttachNode& node : definition.nodes)
        {
            // Cast from OUTSIDE the part back toward the node along its
            // outward direction: the collider surface must be within
            // 15 cm of the authored node position.
            const Vec3 origin = node.position + node.direction * 3.0f;
            parts::PartRayHit hit{};
            const bool wasHit =
                parts::raycastPart(definition, origin, -node.direction, 6.0f, hit);
            SW_CHECK(wasHit);
            if (wasHit)
            {
                const Vec3 surfacePoint = origin - node.direction * hit.t;
                SW_CHECK(glm::length(surfacePoint - node.position) < 0.15f);
            }
        }
    }
}

SW_TEST(PartFilesRoundTripThroughSaveAndLoad)
{
    const parts::PartDefinition* tank = parts::findDefinition(parts::kPartFuelTankMedium);
    SW_CHECK(tank != nullptr);

    const std::filesystem::path temp =
        std::filesystem::temp_directory_path() / "sw_roundtrip.swpart";
    SW_CHECK(parts::savePartFile(*tank, temp));
    parts::PartDefinition loaded{};
    SW_CHECK(parts::loadPartFile(temp, loaded));
    std::filesystem::remove(temp);

    SW_CHECK_EQ(loaded.id, tank->id);
    SW_CHECK(loaded.name == tank->name);
    SW_CHECK(std::abs(loaded.dryMassKg - tank->dryMassKg) < 1.0e-9);
    SW_CHECK(loaded.capacities[0].resource == tank->capacities[0].resource);
    SW_CHECK(std::abs(loaded.capacities[0].units - tank->capacities[0].units) < 1.0e-9);
    SW_CHECK_EQ(loaded.shapes.size(), tank->shapes.size());
    SW_CHECK_EQ(loaded.nodes.size(), tank->nodes.size());
    for (usize i = 0; i < loaded.shapes.size(); ++i)
    {
        SW_CHECK(loaded.shapes[i].kind == tank->shapes[i].kind);
        SW_CHECK(glm::length(loaded.shapes[i].size - tank->shapes[i].size) < 1.0e-4f);
        SW_CHECK(glm::length(loaded.shapes[i].color - tank->shapes[i].color) < 1.0e-4f);
        SW_CHECK(loaded.shapes[i].collider == tank->shapes[i].collider);
    }
    for (usize i = 0; i < loaded.nodes.size(); ++i)
    {
        SW_CHECK(loaded.nodes[i].name == tank->nodes[i].name);
        SW_CHECK(glm::length(loaded.nodes[i].position - tank->nodes[i].position) < 1.0e-4f);
        SW_CHECK(loaded.nodes[i].type == tank->nodes[i].type);
    }
}

SW_TEST(PartGeometryRaycastAndOverlap)
{
    // A hand-built part: one collider cylinder (r=1, half length 2).
    parts::PartDefinition cylinder{};
    cylinder.id = 1000;
    parts::PartShape shape{};
    shape.kind = parts::ShapeKind::Cylinder;
    shape.size = {1.0f, 2.0f, 0.0f};
    shape.collider = true;
    cylinder.shapes.push_back(shape);

    // Side hit: from +X toward the axis strikes the wall at x = 1.
    parts::PartRayHit hit{};
    SW_CHECK(parts::raycastPart(cylinder, {5.0f, 0.0f, 0.5f}, {-1.0f, 0.0f, 0.0f},
                                100.0f, hit));
    SW_CHECK(std::abs(hit.t - 4.0f) < 1.0e-3f);
    SW_CHECK(glm::length(hit.normal - Vec3{1.0f, 0.0f, 0.0f}) < 1.0e-3f);
    // Cap hit from below.
    SW_CHECK(parts::raycastPart(cylinder, {0.2f, 0.0f, 6.0f}, {0.0f, 0.0f, -1.0f},
                                100.0f, hit));
    SW_CHECK(std::abs(hit.t - 4.0f) < 1.0e-3f);
    SW_CHECK(hit.normal.z > 0.9f);
    // Miss past the side.
    SW_CHECK(!parts::raycastPart(cylinder, {5.0f, 1.5f, 0.0f}, {-1.0f, 0.0f, 0.0f},
                                 100.0f, hit));

    // Overlap: two identical cylinders stacked flush along Z do NOT
    // overlap (margin), truly interpenetrating ones DO, rotation matters.
    const Quat identity{1.0f, 0.0f, 0.0f, 0.0f};
    SW_CHECK(!parts::partsOverlap(cylinder, {0.0f, 0.0f, 0.0f}, identity, cylinder,
                                  {0.0f, 0.0f, 4.0f}, identity, 0.04f));
    SW_CHECK(parts::partsOverlap(cylinder, {0.0f, 0.0f, 0.0f}, identity, cylinder,
                                 {0.0f, 0.0f, 2.5f}, identity, 0.04f));
    // A long thin box rotated 90 deg about X now spans Y instead of Z.
    parts::PartDefinition stick{};
    stick.id = 1001;
    parts::PartShape stickShape{};
    stickShape.kind = parts::ShapeKind::Box;
    stickShape.size = {0.2f, 0.2f, 3.0f};
    stickShape.collider = true;
    stick.shapes.push_back(stickShape);
    const Quat aboutX = glm::angleAxis(glm::radians(90.0f), Vec3{1.0f, 0.0f, 0.0f});
    // Unrotated at y=2: the stick (z-long) beside the cylinder -> overlap in Z.
    SW_CHECK(parts::partsOverlap(cylinder, {0.0f, 0.0f, 0.0f}, identity, stick,
                                 {0.0f, 1.1f, 0.0f}, identity, 0.04f));
    // Rotated to lie along Y and pushed up: clears the cylinder.
    SW_CHECK(!parts::partsOverlap(cylinder, {0.0f, 0.0f, 0.0f}, identity, stick,
                                  {0.0f, 4.5f, 0.0f}, aboutX, 0.04f));

    // Mesh generation: visible-only, per-shape colors, sane normals.
    parts::PartDefinition meshed = cylinder;
    meshed.shapes[0].visible = true;
    parts::PartShape hidden{};
    hidden.kind = parts::ShapeKind::Box;
    hidden.visible = false;
    hidden.collider = true;
    meshed.shapes.push_back(hidden);
    const MeshData mesh = parts::buildPartMesh(meshed);
    SW_CHECK(!mesh.empty());
    for (const Vertex& vertex : mesh.vertices)
    {
        SW_CHECK(std::abs(glm::length(vertex.normal) - 1.0f) < 1.0e-2f);
    }
}

// ============================================================================
// F15 — THE ENDURANCE, CHECKED AS A CONTRACT
//
// The ring ship is assembled by arithmetic in GameScene, and arithmetic
// against data authored in another file is exactly the pair that drifts:
// widen a module in Part Studio, and twelve joints open up in a way no
// existing test would notice. So the numbers the blueprint depends on are
// asserted here, against the SHIPPED catalogue — the ring's closure, the
// spoke's reach, the names of every node the blueprint asks for by name,
// and the propellant and power that make the thing flyable rather than
// scenery.
// ============================================================================
SW_TEST(EnduranceCatalogueDescribesAFlyableRing)
{
    const std::filesystem::path directory =
        FileSystem::executableDirectory() / "Assets" / "Parts";
    SW_CHECK(parts::loadCatalog(directory));

    const u32 ids[] = {
        parts::kPartEnduranceHabitat, parts::kPartEnduranceEngine,
        parts::kPartEnduranceCommand, parts::kPartEnduranceTunnel,
        parts::kPartEnduranceRanger,  parts::kPartEnduranceLander,
        parts::kPartEnduranceCargo,   parts::kPartEnduranceCryo,
        parts::kPartEnduranceCoreHub, parts::kPartEnduranceSpoke};
    for (const u32 id : ids)
    {
        const parts::PartDefinition* definition = parts::findDefinition(id);
        SW_CHECK(definition != nullptr);
        if (definition == nullptr) { continue; }
        // Every piece is a VESSEL part (no industrial block, not a prop) and
        // carries a collision hull: you can bump into all of it.
        SW_CHECK(parts::isVesselPart(*definition));
        SW_CHECK(!parts::effectiveHull(*definition).empty());
    }

    // ---- the ring closes ---------------------------------------------------
    // Module centres on the apothem a = 29.4 m of a regular dodecagon; the
    // vertex is a*tan(15 deg) along the tangent, and what is left over
    // between a 12.4 m module and it is exactly half a tunnel.
    constexpr f32 kApothem = 29.4f;
    constexpr f32 kModuleHalfLen = 6.2f;
    constexpr f32 kModuleHalfRad = 2.4f;
    const f32 vertex = kApothem * std::tan(glm::pi<f32>() / 12.0f);
    const auto halfExtent = [](u32 id, int axis) {
        const parts::PartDefinition* definition = parts::findDefinition(id);
        if (definition == nullptr) { return 0.0f; }
        f32 out = 0.0f;
        for (const parts::HitBox& box : parts::effectiveHull(*definition))
        {
            out = std::max(out, std::abs(box.halfExtents[axis]));
        }
        return out;
    };
    SW_CHECK(std::abs(halfExtent(parts::kPartEnduranceTunnel, 2) -
                      (vertex - kModuleHalfLen)) < 0.05f);
    // ...and every module is the length and thickness that arithmetic used.
    for (const u32 id : {parts::kPartEnduranceHabitat, parts::kPartEnduranceEngine,
                         parts::kPartEnduranceCommand, parts::kPartEnduranceCryo})
    {
        SW_CHECK(std::abs(halfExtent(id, 2) - kModuleHalfLen) < 0.01f);
        SW_CHECK(std::abs(halfExtent(id, 0) - kModuleHalfRad) < 0.01f);
    }
    // 64 metres across, which is the film's number and the one the whole
    // silhouette rests on.
    SW_CHECK(std::abs(2.0f * (kApothem + kModuleHalfRad) - 64.0f) < 1.0f);

    // ---- the spokes reach exactly from the core to the ring ----------------
    // hub skin + a whole spoke must land on the module's inner face, or the
    // six struts inside the ring are six struts that miss it.
    const f32 hubRadius = halfExtent(parts::kPartEnduranceCoreHub, 0);
    const f32 spokeHalf = halfExtent(parts::kPartEnduranceSpoke, 2);
    SW_CHECK(std::abs((hubRadius + 2.0f * spokeHalf) -
                      (kApothem - kModuleHalfRad)) < 0.05f);

    // ---- every node the blueprint asks for, by name ------------------------
    const auto hasNode = [](u32 id, std::string_view name) {
        const parts::PartDefinition* definition = parts::findDefinition(id);
        if (definition == nullptr) { return false; }
        for (const parts::AttachNode& node : definition->nodes)
        {
            if (node.name == name) { return true; }
        }
        return false;
    };
    for (const u32 id : {parts::kPartEnduranceHabitat, parts::kPartEnduranceEngine,
                         parts::kPartEnduranceCommand, parts::kPartEnduranceCryo,
                         parts::kPartEnduranceCargo})
    {
        SW_CHECK(hasNode(id, "ringA"));
        SW_CHECK(hasNode(id, "ringB"));
        SW_CHECK(hasNode(id, "spoke"));
    }
    SW_CHECK(hasNode(parts::kPartEnduranceCargo, "dock"));
    SW_CHECK(hasNode(parts::kPartEnduranceRanger, "dock"));
    SW_CHECK(hasNode(parts::kPartEnduranceLander, "dock"));
    SW_CHECK(hasNode(parts::kPartEnduranceSpoke, "hub"));
    SW_CHECK(hasNode(parts::kPartEnduranceSpoke, "ring"));
    // The core hub's six radial berths and its six docking ports: the
    // "mounts six support craft at one time" the design claims.
    for (const char* name : {"spoke0", "spoke1", "spoke2", "spoke3", "spoke4",
                             "spoke5"})
    {
        SW_CHECK(hasNode(parts::kPartEnduranceCoreHub, name));
    }
    usize hubPorts = 0;
    for (const char* name : {"dockF0", "dockF1", "dockF2", "dockA0", "dockA1",
                             "dockA2"})
    {
        hubPorts += hasNode(parts::kPartEnduranceCoreHub, name) ? 1u : 0u;
    }
    SW_CHECK_EQ(hubPorts, static_cast<usize>(6));

    // ---- it can actually fly ----------------------------------------------
    // Twelve plasma engines over four modules, propellant in those same
    // modules, and joules and photovoltaics in the habitats. Without these
    // the Endurance is a very large ornament.
    const parts::PartDefinition* propulsion =
        parts::findDefinition(parts::kPartEnduranceEngine);
    SW_CHECK(propulsion != nullptr);
    if (propulsion != nullptr)
    {
        SW_CHECK(propulsion->thrustNewtons > 0.0);
        SW_CHECK(propulsion->specificImpulseS > 1000.0); // plasma, not chemical
        SW_CHECK(propulsion->capacities[0].resource == res::Resource::Fuel);
        SW_CHECK(propulsion->capacities[0].units > 0.0);
    }
    const parts::PartDefinition* habitat =
        parts::findDefinition(parts::kPartEnduranceHabitat);
    SW_CHECK(habitat != nullptr);
    if (habitat != nullptr)
    {
        // Battery-typed: SolarChargeSystem only pays a vessel that has one.
        SW_CHECK(habitat->type == parts::PartType::Battery);
        SW_CHECK(habitat->chargeRateKw > 0.0);
        SW_CHECK(habitat->capacities[0].resource == res::Resource::ElectricCharge);
    }

    // ---- and the air has an answer for every piece -------------------------
    // AeroForge solved a table for each one; a part flown from its geometry
    // rather than from a single guessed Cd*A is the whole point of F6.
    for (const u32 id : ids)
    {
        const parts::PartDefinition* definition = parts::findDefinition(id);
        if (definition == nullptr) { continue; }
        SW_CHECK(definition->dragCoefficientArea > 0.0); // the fallback still exists
    }
}

// ============================================================================
// THE HINGE, RECOVERED FROM TWO POSES
//
// Animations are authored by moving the thing: the panel is posed once closed
// and once open, and the file holds those two poses and nothing else. Playing
// that back the obvious way — slerp the rotation, lerp the position — is
// correct at both ENDS and wrong in the middle, and wrong in the one way that
// is most visible: a panel swinging ninety degrees about a mount at its root
// has its centre cut the corner, so it shrinks toward the hub and springs back
// out. It reads as a telescope, not a hinge.
//
// Chasles' theorem says every rigid motion is a rotation about some axis
// through some point plus a slide along that axis. These tests set up motions
// whose hinge is known by construction and check that it comes back.
// ============================================================================
SW_TEST(AHingeIsRecoveredFromThePosesItProduced)
{
    using namespace sw::parts;

    // A panel two metres long, hinged at the origin, swung 90 degrees about
    // +Y. Its centre starts at (2,0,0) and must end at (0,0,-2) having
    // travelled an ARC of radius 2 — never nearer the hinge, never further.
    const sw::Vec3 startPosition{2.0f, 0.0f, 0.0f};
    const sw::Quat startRotation{1.0f, 0.0f, 0.0f, 0.0f};
    const sw::Quat turn = glm::angleAxis(1.5707963f, sw::Vec3{0.0f, 1.0f, 0.0f});
    const sw::Vec3 endPosition = turn * startPosition;
    const sw::Quat endRotation = turn * startRotation;

    const HingeMotion hinge =
        hingeBetween(startPosition, startRotation, endPosition, endRotation);
    SW_CHECK(std::abs(hinge.angleRadians - 1.5707963f) < 1.0e-4f);
    SW_CHECK(std::abs(std::abs(hinge.axis.y) - 1.0f) < 1.0e-4f);
    // The pivot is the mount, and it is at the origin.
    SW_CHECK(glm::length(hinge.pivot) < 1.0e-3f);
    SW_CHECK(glm::length(hinge.slide) < 1.0e-3f);

    // Now the property that matters: the radius is constant along the whole
    // travel. A lerp would give 1.414 at the midpoint against the true 2.0 —
    // thirty per cent of the panel's own length, swallowed and spat back out.
    for (sw::i32 step = 0; step <= 10; ++step)
    {
        const sw::f32 phase = static_cast<sw::f32>(step) / 10.0f;
        sw::Vec3 position{};
        sw::Quat rotation{};
        poseAlongHinge(hinge, startPosition, startRotation, phase, position, rotation);
        SW_CHECK(std::abs(glm::length(position) - 2.0f) < 1.0e-3f);
    }
    // ...and both ends land exactly where they were authored.
    sw::Vec3 atZero{};
    sw::Quat rotationAtZero{};
    poseAlongHinge(hinge, startPosition, startRotation, 0.0f, atZero, rotationAtZero);
    SW_CHECK(glm::length(atZero - startPosition) < 1.0e-4f);
    sw::Vec3 atOne{};
    sw::Quat rotationAtOne{};
    poseAlongHinge(hinge, startPosition, startRotation, 1.0f, atOne, rotationAtOne);
    SW_CHECK(glm::length(atOne - endPosition) < 1.0e-3f);
    SW_CHECK(std::abs(glm::dot(rotationAtOne, endRotation)) > 0.9999f);
}

SW_TEST(AHingeOffTheOriginAndAPureSlideBothSurvive)
{
    using namespace sw::parts;

    // A hinge at (0, 0, -0.5) — a bay door on the nose end of a part — turning
    // 70 degrees about +X. Nothing about this is symmetric, which is the
    // point: the pivot solve has to find a point that is not the origin.
    const sw::Vec3 pivot{0.0f, 0.0f, -0.5f};
    const sw::Vec3 startPosition{0.0f, 0.4f, -0.5f};
    const sw::Quat startRotation =
        glm::angleAxis(0.3f, glm::normalize(sw::Vec3{1.0f, 1.0f, 0.0f}));
    const sw::Quat turn = glm::angleAxis(1.2217305f, sw::Vec3{1.0f, 0.0f, 0.0f});
    const sw::Vec3 endPosition = pivot + turn * (startPosition - pivot);
    const sw::Quat endRotation = turn * startRotation;

    const HingeMotion hinge =
        hingeBetween(startPosition, startRotation, endPosition, endRotation);
    SW_CHECK(std::abs(hinge.angleRadians - 1.2217305f) < 1.0e-4f);
    // Only the component PERPENDICULAR to the axis is determined — sliding the
    // pivot along the hinge changes no point's path — so that is what is
    // checked, and checking the whole vector would be checking an arbitrary
    // choice rather than a fact.
    const sw::Vec3 pivotError = hinge.pivot - pivot;
    const sw::Vec3 perpendicular =
        pivotError - hinge.axis * glm::dot(pivotError, hinge.axis);
    SW_CHECK(glm::length(perpendicular) < 1.0e-3f);
    // The radius about the true pivot holds all the way round.
    for (sw::i32 step = 0; step <= 8; ++step)
    {
        sw::Vec3 position{};
        sw::Quat rotation{};
        poseAlongHinge(hinge, startPosition, startRotation,
                       static_cast<sw::f32>(step) / 8.0f, position, rotation);
        SW_CHECK(std::abs(glm::length(position - pivot) - 0.4f) < 2.0e-3f);
    }

    // A LANDING GEAR SLIDES, and a slide has no pivot at all: (I - R) is the
    // zero matrix and the solve that finds a hinge would divide by it. It has
    // to come back as a straight line instead.
    const sw::Vec3 outPosition{0.0f, -1.2f, 0.0f};
    const HingeMotion slide = hingeBetween(sw::Vec3{0.0f}, startRotation, outPosition,
                                           startRotation);
    SW_CHECK(slide.angleRadians < 1.0e-4f);
    SW_CHECK(glm::length(slide.slide - outPosition) < 1.0e-4f);
    sw::Vec3 halfway{};
    sw::Quat unchanged{};
    poseAlongHinge(slide, sw::Vec3{0.0f}, startRotation, 0.5f, halfway, unchanged);
    SW_CHECK(glm::length(halfway - outPosition * 0.5f) < 1.0e-4f);
    SW_CHECK(std::abs(glm::dot(unchanged, startRotation)) > 0.9999f);
}

SW_TEST(AnimationsSurviveTheRoundTripAndTheOldFilesStillLoad)
{
    using namespace sw::parts;
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() / "sw_anim_roundtrip";
    std::filesystem::create_directories(directory);
    const std::filesystem::path path = directory / "anim.swpart";

    PartDefinition written{};
    written.id = 9001;
    written.type = PartType::SolarPanel;
    written.name = "TEST WING";
    written.chargeRateKw = 7.5;
    PartShape body{};
    body.position = {0.0f, 0.0f, 0.0f};
    written.shapes.push_back(body);
    PartShape array{};
    array.position = {-1.0f, 0.0f, 0.0f};
    array.rotationDeg = {0.0f, 90.0f, 0.0f};
    array.animation = 0;
    array.endPosition = {-2.0f, 0.25f, 0.0f};
    array.endRotationDeg = {0.0f, 0.0f, 15.0f};
    array.endEmissive = 0.75f;
    written.shapes.push_back(array);
    PartAnimation animation{};
    std::strncpy(animation.name, "SOLAR ARRAY", PartAnimation::kNameCapacity - 1);
    animation.trigger = AnimationTrigger::Toggle;
    animation.verbs = AnimationVerbs::OpenClose;
    animation.gates = AnimationGates::Power;
    animation.durationSeconds = 4.5f;
    animation.startsOpen = true;
    written.animations.push_back(animation);

    SW_CHECK(savePartFile(written, path));
    PartDefinition read{};
    SW_CHECK(loadPartFile(path, read));
    SW_CHECK_EQ(read.animations.size(), static_cast<sw::usize>(1));
    SW_CHECK_EQ(std::string(read.animations[0].name), std::string("SOLAR ARRAY"));
    SW_CHECK(read.animations[0].gates == AnimationGates::Power);
    SW_CHECK(read.animations[0].startsOpen);
    SW_CHECK(std::abs(read.animations[0].durationSeconds - 4.5f) < 1.0e-6f);
    SW_CHECK_EQ(read.shapes[0].animation, -1);
    SW_CHECK_EQ(read.shapes[1].animation, 0);
    SW_CHECK(glm::length(read.shapes[1].endPosition - array.endPosition) < 1.0e-6f);
    SW_CHECK(glm::length(read.shapes[1].endRotationDeg - array.endRotationDeg) < 1.0e-4f);
    SW_CHECK(std::abs(read.shapes[1].endEmissive - 0.75f) < 1.0e-6f);

    // THE STATIC SHAPE WROTE NO ANIMATION KEYS AT ALL. Thirty-four shipped
    // parts animate nothing, and emitting four dead keys on every shape of
    // every one of them would put a thousand lines of noise into the next
    // diff of an unrelated edit.
    const std::vector<sw::u8> bytes = sw::FileSystem::readBinaryFile(path);
    const std::string text(bytes.begin(), bytes.end());
    SW_CHECK_EQ(text.find("endPosition"), text.rfind("endPosition")); // exactly one

    // AND A FILE FROM BEFORE ANY OF THIS still loads, with every shape static.
    PartDefinition legacy{};
    legacy.id = 9002;
    legacy.name = "OLD";
    legacy.shapes.push_back(body);
    const std::filesystem::path legacyPath = directory / "legacy.swpart";
    SW_CHECK(savePartFile(legacy, legacyPath));
    PartDefinition reloaded{};
    SW_CHECK(loadPartFile(legacyPath, reloaded));
    SW_CHECK(reloaded.animations.empty());
    SW_CHECK_EQ(reloaded.shapes[0].animation, -1);

    // A shape pointing at an animation that is not there is corrected on load
    // rather than becoming an invisible group nobody can find.
    PartDefinition dangling = legacy;
    dangling.id = 9003;
    dangling.shapes[0].animation = 3;
    const std::filesystem::path danglingPath = directory / "dangling.swpart";
    SW_CHECK(savePartFile(dangling, danglingPath));
    PartDefinition fixed{};
    SW_CHECK(loadPartFile(danglingPath, fixed));
    SW_CHECK_EQ(fixed.shapes[0].animation, -1);

    std::filesystem::remove_all(directory);
}

SW_TEST(AStowedPanelMakesNoPowerAndAShutEngineMakesNoThrust)
{
    using namespace sw::parts;
    PartDefinition panel{};
    panel.chargeRateKw = 4.0;
    PartAnimation deploy{};
    deploy.gates = AnimationGates::Power;
    panel.animations.push_back(deploy);

    PartAnimationComponent state{};
    state.count = 1;

    // Stowed: nothing. Half open: half of it — a panel caught mid-travel
    // really is presenting half its area, and a rule that waited for exactly
    // 1 would put the whole effect in the last instant of a four-second
    // deployment.
    state.phase[0] = 0.0f;
    SW_CHECK(animationGate(panel, &state, AnimationGates::Power) < 1.0e-6);
    state.phase[0] = 0.5f;
    SW_CHECK(std::abs(animationGate(panel, &state, AnimationGates::Power) - 0.5) < 1.0e-6);
    state.phase[0] = 1.0f;
    SW_CHECK(std::abs(animationGate(panel, &state, AnimationGates::Power) - 1.0) < 1.0e-6);
    // The gate is per KIND: a power gate says nothing about thrust.
    SW_CHECK(std::abs(animationGate(panel, &state, AnimationGates::Thrust) - 1.0) < 1.0e-6);

    // AND A PART WITH NO ANIMATION STATE WORKS, which is the default that
    // matters: getting it backwards would have switched off every engine in
    // the game the day animations were added.
    SW_CHECK(std::abs(animationGate(panel, nullptr, AnimationGates::Power) - 1.0) < 1.0e-6);
    PartDefinition plain{};
    SW_CHECK(std::abs(animationGate(plain, &state, AnimationGates::Power) - 1.0) < 1.0e-6);
}

// ============================================================================
// PICKING A PART TO OPERATE
//
// "Le click est possible mais extremement difficile." It was, and the exact
// collider raycast was wrong twice over. Once because the colliders sit at the
// shapes' REST poses, so a solar wing drawn swung out has its collider still
// folded against the hull — clicking the panel you can SEE misses, and
// clicking empty space beside the tank hits. And once because exact is the
// wrong ambition: a wing seen edge-on from twenty metres is two pixels of
// collider, and a switch that demands two pixels is a switch nobody can work.
//
// What replaced it is one ray and the part's bounding sphere, which already
// covers the deployed pose. This pins the tolerance that buys, in degrees,
// because degrees are what a hand actually has to hit.
// ============================================================================
SW_TEST(APartIsPickedByTheRayThatPassesThroughIt)
{
    using namespace sw::parts;
    const sw::Vec3 forward{0.0f, 0.0f, -1.0f};

    // Dead centre, twenty metres away, a part with a 1.5 m bounding sphere.
    const sw::Vec3 toPart{0.0f, 0.0f, -20.0f};
    SW_CHECK(std::abs(rayEntersSphere(toPart, forward, 1.5f) - 18.5f) < 1.0e-4f);

    // HOW FAR OFF THE HAND MAY BE. asin(1.5/20) is 4.3 degrees; anything
    // inside that must hit and anything outside must miss. Four degrees is a
    // target a person can hit without aiming; the old exact test on an
    // edge-on panel was a twentieth of one.
    const auto rayAtDegrees = [&](sw::f32 degrees) {
        const sw::f32 radians = degrees * 0.017453292f;
        return glm::normalize(sw::Vec3{std::sin(radians), 0.0f, -std::cos(radians)});
    };
    SW_CHECK(rayEntersSphere(toPart, rayAtDegrees(0.0f), 1.5f) >= 0.0f);
    SW_CHECK(rayEntersSphere(toPart, rayAtDegrees(3.0f), 1.5f) >= 0.0f);
    SW_CHECK(rayEntersSphere(toPart, rayAtDegrees(4.0f), 1.5f) >= 0.0f);
    SW_CHECK(rayEntersSphere(toPart, rayAtDegrees(5.0f), 1.5f) < 0.0f);

    // BEHIND THE EYE IS NOT IN THE WAY. A part directly astern shares the
    // ray's LINE and none of its direction, and a test that forgot the sign
    // would let the pilot work switches by looking away from them.
    SW_CHECK(rayEntersSphere(sw::Vec3{0.0f, 0.0f, 20.0f}, forward, 1.5f) < 0.0f);
    // ...but a part the eye is INSIDE is at zero, not behind: standing in the
    // middle of a cargo bay still counts as pointing at it.
    SW_CHECK_EQ(rayEntersSphere(sw::Vec3{0.3f, 0.0f, 0.2f}, forward, 1.5f), 0.0f);

    // NEAREST WINS, so a part in front of another shadows it exactly as it
    // looks like it should. This is the whole of the depth rule.
    const sw::f32 near = rayEntersSphere(sw::Vec3{0.0f, 0.0f, -8.0f}, forward, 1.0f);
    const sw::f32 far = rayEntersSphere(sw::Vec3{0.0f, 0.0f, -40.0f}, forward, 1.0f);
    SW_CHECK(near >= 0.0f && far >= 0.0f && near < far);
}

SW_TEST(ADeployedWingIsClickableWhereItIsDrawn)
{
    using namespace sw::parts;
    // The bug in one assertion, on a part built here so the check runs
    // whichever catalogue is loaded. A wing's array is authored FOLDED and
    // swings out to x = -1.55; the bounding sphere the pick uses has to cover
    // both poses, or the half of the animation the pilot spends looking at the
    // deployed panel is exactly the half that cannot be clicked.
    PartDefinition wing{};
    wing.id = 9100;
    wing.type = PartType::SolarPanel;
    PartShape mount{};
    mount.size = {0.10f, 0.09f, 0.09f};
    wing.shapes.push_back(mount);
    PartShape array{};
    array.position = {-0.20f, -1.35f, 0.0f}; // stowed alongside the hull
    array.rotationDeg = {0.0f, 0.0f, 90.0f};
    array.size = {1.40f, 0.02f, 0.55f};
    array.animation = 0;
    array.endPosition = {-1.55f, 0.0f, 0.0f}; // deployed, straight out
    array.endRotationDeg = {0.0f, 0.0f, 0.0f};
    wing.shapes.push_back(array);
    wing.animations.push_back(PartAnimation{});

    const f32 radius = partBoundsRadius(wing);
    // Both poses inside the sphere, with their own half-extents.
    SW_CHECK(glm::length(array.position) + 1.4f <= radius + 1.0e-4f);
    SW_CHECK(glm::length(array.endPosition) + 1.4f <= radius + 1.0e-4f);

    // ...and the deployed pose is the one that sets it. Take the animation
    // away and the sphere shrinks, which is precisely the regression this
    // guards: a radius measured on the stowed silhouette alone.
    PartDefinition stowedOnly = wing;
    stowedOnly.shapes[1].animation = -1;
    SW_CHECK(partBoundsRadius(stowedOnly) < radius);

    // The tolerance that buys at a working distance: a couple of degrees.
    const sw::Vec3 forward{0.0f, 0.0f, -1.0f};
    const sw::Vec3 toPart{0.0f, 0.0f, -25.0f};
    SW_CHECK(rayEntersSphere(toPart, forward, radius) >= 0.0f);
    const sw::f32 wobble = 3.0f * 0.017453292f;
    SW_CHECK(rayEntersSphere(toPart,
                             glm::normalize(sw::Vec3{std::sin(wobble), 0.0f,
                                                     -std::cos(wobble)}),
                             radius) >= 0.0f);
}

// ============================================================================
// THE SHIP THE PILOT IS ACTUALLY IN
//
// "Cela ne marche toujours pas ni pour le moteur ni pour le panneau solaire."
// Three rounds of fixing the pick had all been aimed at the wrong thing. The
// menu, the ray, the click-versus-drag rule were by then all correct; what was
// missing is that the animations had been authored on the CATALOGUE parts —
// the SP-2 wing and the V-400 engine, which a player meets in the hangar — and
// the Endurance the game starts you in is built from none of them. Every part
// within reach of the cursor had an empty `animations` list, the pick
// (correctly) refuses those, and so a feature that worked perfectly could not
// be reached from inside the game.
//
// The measurement that ended it was blunt: "35 part entities, 0 with
// animations". This test is that measurement, kept.
// ============================================================================
SW_TEST(TheEnduranceModulesArePartsThePilotCanOperate)
{
    using namespace sw::parts;
    SW_CHECK(loadCatalog(FileSystem::executableDirectory() / "Assets" / "Parts"));

    // The habitat's arrays: a toggle the pilot works by hand, gating the
    // module's own charge rate, with real geometry bound to it. A gate with
    // nothing moving is a switch that does nothing visible; geometry with no
    // gate is scenery.
    const PartDefinition* habitat = findDefinition(kPartEnduranceHabitat);
    SW_CHECK(habitat != nullptr);
    SW_CHECK(!habitat->animations.empty());
    SW_CHECK(habitat->animations[0].trigger == AnimationTrigger::Toggle);
    SW_CHECK(habitat->animations[0].gates == AnimationGates::Power);
    SW_CHECK(habitat->chargeRateKw > 0.0);
    usize movingShapes = 0;
    for (const PartShape& shape : habitat->shapes)
    {
        movingShapes += (shape.animation == 0) ? 1u : 0u;
    }
    SW_CHECK(movingShapes >= 2); // one array each side of the module

    // The propulsion module: a hand switch that gates thrust, and a second
    // animation the THROTTLE drives — the pilot never presses that one, so it
    // must not be a toggle or it would appear in the menu as a dead button.
    const PartDefinition* engine = findDefinition(kPartEnduranceEngine);
    SW_CHECK(engine != nullptr);
    SW_CHECK_EQ(engine->animations.size(), static_cast<usize>(2));
    SW_CHECK(engine->animations[0].trigger == AnimationTrigger::Toggle);
    SW_CHECK(engine->animations[0].gates == AnimationGates::Thrust);
    SW_CHECK(engine->thrustNewtons > 0.0);
    SW_CHECK(engine->animations[1].trigger == AnimationTrigger::Throttle);
    // ...and it lights something up, from dark: a nozzle authored bright at
    // rest is an engine that looks lit while it is shut down.
    usize glows = 0;
    for (const PartShape& shape : engine->shapes)
    {
        if (shape.animation == 1)
        {
            ++glows;
            SW_CHECK(shape.endEmissive > shape.emissive);
            SW_CHECK(shape.emissive < 0.25f);
        }
    }
    SW_CHECK_EQ(glows, static_cast<usize>(3)); // three nozzles per module

    // AND THE CLICK TARGET COVERS THE DEPLOYED POSE. The pick tests a bounding
    // sphere; if that sphere were measured on the stowed silhouette the arrays
    // would stand two metres outside anything a cursor could hit.
    const f32 reach = partBoundsRadius(*habitat);
    for (const PartShape& shape : habitat->shapes)
    {
        if (shape.animation >= 0)
        {
            SW_CHECK(glm::length(shape.endPosition) <= reach + 1.0e-4f);
        }
    }

    // Every animated part in the catalogue, checked the same way: a shape may
    // only point at an animation that exists, and no part may carry more than
    // the runtime component can hold.
    usize animatedParts = 0;
    for (const PartDefinition& definition : catalog())
    {
        if (definition.animations.empty())
        {
            continue;
        }
        ++animatedParts;
        SW_CHECK(definition.animations.size() <= kMaxPartAnimations);
        for (const PartShape& shape : definition.shapes)
        {
            SW_CHECK(shape.animation < static_cast<i32>(definition.animations.size()));
        }
    }
    // The SP-2 wing and V-400 engine from the hangar, plus the two Endurance
    // modules that were the whole point of this test.
    SW_CHECK_EQ(animatedParts, static_cast<usize>(4));
}
