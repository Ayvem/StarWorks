// ============================================================================
// PartsTests.cpp — the part system: catalog integrity, vessel aggregation
// (mass/thrust/drag from parts + carried resources) and rigid attachment.
// ============================================================================

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
    usize rocketParts = 0;
    usize buildings = 0;
    for (const parts::PartDefinition& definition : parts::catalog())
    {
        (parts::isBuilding(definition) ? buildings : rocketParts) += 1;
    }
    SW_CHECK_EQ(rocketParts, static_cast<usize>(9));
    SW_CHECK_EQ(buildings, static_cast<usize>(6));

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
