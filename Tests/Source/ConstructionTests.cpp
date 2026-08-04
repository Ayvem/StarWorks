// ============================================================================
// ConstructionTests.cpp — F2's contract: where a building may stand.
//
// Three callers have to agree to the metre on this — the ghost the player
// aims with, the placement that commits it, and the scene builder that lays
// the starting outpost — so the answer lives in one tested function rather
// than three plausible ones.
// ============================================================================

#include "TestFramework.hpp"

#include <Core/FileSystem.hpp>
#include <ECS/World.hpp>
#include <Factory/Conveyor.hpp>
#include <Gameplay/Construction.hpp>
#include <Gameplay/PartGeometry.hpp>
#include <Gameplay/Parts.hpp>

#include <glm/gtx/quaternion.hpp> // glm::rotation, for the radial glue pose

#include <cmath>
#include <span>
#include <vector>

using namespace sw;

namespace
{
    /// A place on Terra with the character we want to test against.
    Vec3 findGround(const planet::TerrainComponent& terrain, f64 minElevation,
                    f32 minSlope, f32 maxSlope)
    {
        constexpr f64 kRadius = 6.371e6;
        for (u32 i = 0; i < 20000; ++i)
        {
            const f32 a = static_cast<f32>(i) * 0.0137f;
            const f32 b = static_cast<f32>(i) * 0.0071f;
            const Vec3 direction = glm::normalize(
                Vec3{std::cos(a) * std::cos(b), std::sin(b), std::sin(a) * std::cos(b)});
            if (planet::terrainElevation(terrain, direction) <= minElevation)
            {
                continue;
            }
            const f32 slope = planet::terrainLocalSlope(terrain, direction, kRadius, 8.0f);
            if (slope >= minSlope && slope <= maxSlope)
            {
                return direction;
            }
        }
        return Vec3{0.0f, 0.0f, 1.0f};
    }
} // namespace

SW_TEST(BuildRaycastLandsOnTheSurfaceItAimsAt)
{
    const planet::TerrainComponent terrain = planet::presetTerra();
    constexpr f64 kRadius = 6.371e6;
    // GENTLE ground: looking along a cliff can carry a ray for kilometres,
    // which is a fine thing for it to do but not what this test measures.
    const Vec3 ground = findGround(terrain, 200.0, 0.0f, 0.05f);
    const f64 elevation = planet::terrainElevation(terrain, ground);

    // Stand 2 m above the ground and look STRAIGHT DOWN: the hit must be the
    // ground under your feet, to the centimetre.
    {
        const WorldVec3 eye = WorldVec3(ground) * (kRadius + elevation + 2.0);
        WorldVec3 hit{};
        SW_CHECK(build::raycastTerrain(terrain, kRadius, eye, -WorldVec3(ground), 50.0,
                                       hit));
        const Vec3 hitDirection = Vec3(glm::normalize(hit));
        const f64 hitGround =
            kRadius + planet::terrainElevation(terrain, hitDirection);
        SW_CHECK(std::abs(glm::length(hit) - hitGround) < 0.01);
        SW_CHECK(build::groundDistance(hitDirection, ground, kRadius) < 0.01);
    }

    // Look ALONG the surface from eye height: the ray must still land on the
    // ground, ahead of you, within the range it was given.
    {
        const WorldVec3 eye = WorldVec3(ground) * (kRadius + elevation + 1.7);
        const Vec3 east = glm::normalize(glm::cross(Vec3{0.0f, 1.0f, 0.0f}, ground));
        // 25 degrees below the horizon.
        const Vec3 aim = glm::normalize(east - ground * std::tan(0.436f));
        WorldVec3 hit{};
        SW_CHECK(build::raycastTerrain(terrain, kRadius, eye, WorldVec3(aim), 60.0, hit));
        const Vec3 hitDirection = Vec3(glm::normalize(hit));
        SW_CHECK(std::abs(glm::length(hit) -
                          (kRadius + planet::terrainElevation(terrain, hitDirection))) <
                 0.05);
        const f64 reach = build::groundDistance(hitDirection, ground, kRadius);
        SW_CHECK(reach > 0.5 && reach < 60.0);
    }

    // Look UP and you hit nothing, however long you look.
    {
        const WorldVec3 eye = WorldVec3(ground) * (kRadius + elevation + 1.7);
        WorldVec3 hit{};
        SW_CHECK(!build::raycastTerrain(terrain, kRadius, eye, WorldVec3(ground), 5000.0,
                                        hit));
    }
}

SW_TEST(PlacementRulesComeFromTheSwpartAndRefuseForAReason)
{
    SW_CHECK(parts::loadCatalog(FileSystem::executableDirectory() / "Assets" / "Parts"));

    const planet::TerrainComponent terrain = planet::presetTerra();
    const planet::DepositComponent deposits = planet::depositsTerra();
    constexpr f64 kRadius = 6.371e6;

    const parts::PartDefinition* miner = parts::findDefinition(parts::kBuildingMiner);
    const parts::PartDefinition* solar =
        parts::findDefinition(parts::kBuildingSolarFarm);
    const parts::PartDefinition* rocket =
        parts::findDefinition(parts::kPartFuelTankMedium);
    SW_CHECK(miner != nullptr && solar != nullptr && rocket != nullptr);

    // A rocket part is not a building, and says so rather than crashing.
    SW_CHECK_EQ(build::validatePlacement(terrain, deposits, kRadius, *rocket,
                                         Vec3{0.0f, 0.0f, 1.0f}, {}),
                build::Verdict::NoDefinition);

    // THE SEA. +Z on Terra is open ocean — the same spot that used to hold
    // the whole outpost.
    SW_CHECK_EQ(build::validatePlacement(terrain, deposits, kRadius, *solar,
                                         Vec3{0.0f, 0.0f, 1.0f}, {}),
                build::Verdict::Underwater);

    // A CLIFF. The SL-1 solar field accepts 0.12; find ground well past that.
    const Vec3 cliff = findGround(terrain, 200.0, 0.35f, 1.0e9f);
    SW_CHECK_EQ(build::validatePlacement(terrain, deposits, kRadius, *solar, cliff, {}),
                build::Verdict::TooSteep);

    // ORE. The site the survey founds carries 0.85; the miner asks 0.08.
    f32 grade = 0.0f;
    const Vec3 site = planet::surveyEquatorialSite(
        terrain, deposits, res::Resource::IronOre, kRadius, grade);
    SW_CHECK(grade > miner->building.minOreDensity);
    SW_CHECK_EQ(build::validatePlacement(terrain, deposits, kRadius, *miner, site, {}),
                build::Verdict::Ok);

    // ...AND NOWHERE IS A DEAD END. This used to hunt for barren ground and
    // check that the miner was refused on it; there is no barren ground any
    // more, and that is the point. Copper gates the electronics and ice gates
    // the fuel and the air, so a landing site without either cannot start a
    // colony at all — through no decision the player made, and with no cure
    // available to somebody who cannot yet build anything.
    //
    // Every rock now carries a tenth of both. Forty thousand directions over
    // the whole sphere, and not one of them refuses a mine for want of ore.
    u32 refusedForOre = 0;
    f32 poorest = 1.0f;
    f32 richest = 0.0f;
    for (u32 i = 0; i < 40000; ++i)
    {
        const f32 a = static_cast<f32>(i) * 0.0137f;
        const f32 b = static_cast<f32>(i) * 0.0071f;
        const Vec3 candidate = glm::normalize(
            Vec3{std::cos(a) * std::cos(b), std::sin(b), std::sin(a) * std::cos(b)});
        f32 density = 0.0f;
        planet::bestDeposit(deposits, candidate, density);
        poorest = std::min(poorest, density);
        richest = std::max(richest, density);
        if (density < miner->building.minOreDensity)
        {
            ++refusedForOre;
        }
    }
    SW_CHECK_EQ(refusedForOre, static_cast<u32>(0));
    SW_CHECK(poorest >= deposits.baselineDensity - 1.0e-5f);

    // AND SITING STILL MATTERS, which is the half a floor could have
    // destroyed. Yield is the nominal rate times the density underfoot, so a
    // mine on baseline ground runs at a fraction of one on a real deposit:
    // expansion stopped being a requirement and became an optimisation.
    SW_CHECK(richest > poorest * 5.0f);

    // OVERLAP. A building already standing on the spot blocks it; step off by
    // more than the two radii and it is free again.
    const f32 minerRadius = build::footprintRadius(miner->building);
    const std::vector<build::Footprint> occupied = {{site, minerRadius}};
    SW_CHECK_EQ(build::validatePlacement(terrain, deposits, kRadius, *miner, site,
                                         occupied),
                build::Verdict::Overlapping);

    // BELTS ARE EXEMPT. A row of 2 m segments overlaps itself by the circle
    // measure, and a belt has to reach a mouth that is inside its machine's
    // own circle — so the overlap rule does not apply to them, in either
    // direction. Without this you cannot lay a belt at all.
    const parts::PartDefinition* segment =
        parts::findDefinition(parts::kBuildingConveyor);
    SW_CHECK(segment != nullptr);
    if (segment != nullptr)
    {
        SW_CHECK_EQ(build::validatePlacement(terrain, deposits, kRadius, *segment, site,
                                             occupied),
                    build::Verdict::Ok);
    }

    // The clearance really is the sum of the half-diagonals, not a guess.
    // (This is the check that caught groundDistance measuring every pair of
    // buildings as zero metres apart: at these separations an f32 dot product
    // is exactly 1.0.)
    const Vec3 east = glm::normalize(glm::cross(Vec3{0.0f, 1.0f, 0.0f}, site));
    const f64 clearance = static_cast<f64>(minerRadius) * 2.0;
    for (const f64 offset : {clearance * 0.9, clearance * 1.15})
    {
        const Vec3 shifted = glm::normalize(
            site + east * static_cast<f32>(offset / kRadius));
        const build::Verdict verdict = build::validatePlacement(
            terrain, deposits, kRadius, *miner, shifted, occupied);
        if (offset < clearance)
        {
            SW_CHECK_EQ(verdict, build::Verdict::Overlapping);
        }
        else
        {
            SW_CHECK(verdict != build::Verdict::Overlapping);
        }
    }
}

// THE CONVEYOR NETWORK IS DERIVED, NOT DECLARED.
//
// The player places belt segments one at a time, like any other building.
// What makes a ROW of them a working link is that their mouths meet — a
// question about where things ARE. So the graph is retraced after every
// build and every demolition, and these are the cases it has to get right:
// a run that connects, a run with a hole in it, a stub that leads nowhere,
// and a ring that must not become an infinite loop.
SW_TEST(ConveyorChainsAreTracedFromPortsThatMeet)
{
    // A straight run along X: machine, three belts, machine. Each belt is
    // 2 m long, so its in port is 1 m behind its centre and its out port 1 m
    // ahead — consecutive centres 2 m apart make the mouths coincide.
    auto belt = [](f64 x, u32 index) {
        factory::PortNode node{};
        node.entity = {index, 0};
        node.isBelt = true;
        node.centre = WorldVec3{x, 0.0, 0.0};
        node.inPorts[0] = WorldVec3{x - 1.0, 0.0, 0.0};
        node.outPorts[0] = WorldVec3{x + 1.0, 0.0, 0.0};
        node.inCount = 1;
        node.outCount = 1;
        return node;
    };
    auto machine = [](f64 x, u32 index, bool hasIn, bool hasOut) {
        factory::PortNode node{};
        node.entity = {index, 0};
        node.centre = WorldVec3{x, 0.0, 0.0};
        node.inPorts[0] = WorldVec3{x, 0.0, 0.0};
        node.outPorts[0] = WorldVec3{x, 0.0, 0.0};
        node.inCount = hasIn ? 1u : 0u;
        node.outCount = hasOut ? 1u : 0u;
        return node;
    };

    {
        // 0: mine at x=0 (out) -> belts at 1, 3, 5 -> 4: smelter at x=6 (in)
        const std::vector<factory::PortNode> nodes = {
            machine(0.0, 10, false, true), belt(1.0, 11), belt(3.0, 12), belt(5.0, 13),
            machine(6.0, 14, true, false)};
        const std::vector<factory::Chain> chains = factory::traceConveyorChains(nodes, factory::kConveyorPortSnapM);
        SW_CHECK_EQ(chains.size(), static_cast<usize>(1));
        SW_CHECK_EQ(chains[0].source, 0u);
        SW_CHECK_EQ(chains[0].destination, 4u);
        SW_CHECK_EQ(chains[0].belts.size(), static_cast<usize>(3));
        // In TRAVEL order, which is what the cargo path depends on.
        SW_CHECK_EQ(chains[0].belts[0], 1u);
        SW_CHECK_EQ(chains[0].belts[1], 2u);
        SW_CHECK_EQ(chains[0].belts[2], 3u);
    }

    {
        // A HOLE: demolish the middle belt and the run must stop existing.
        // Not "carry on across the gap" — that is what makes demolition read
        // as demolition, and it is why the snap distance has to stay under
        // one segment's length. (It did not, at first: a 2.5 m snap happily
        // bridged the 2 m hole a missing tile leaves.)
        const std::vector<factory::PortNode> nodes = {
            machine(0.0, 10, false, true), belt(1.0, 11), belt(5.0, 13),
            machine(6.0, 14, true, false)};
        SW_CHECK_EQ(factory::traceConveyorChains(nodes, factory::kConveyorPortSnapM).size(),
                    static_cast<usize>(0));
    }

    {
        // A STUB: belts that lead nowhere are not a chain.
        const std::vector<factory::PortNode> nodes = {machine(0.0, 10, false, true),
                                                      belt(1.0, 11), belt(3.0, 12)};
        SW_CHECK_EQ(factory::traceConveyorChains(nodes, factory::kConveyorPortSnapM).size(),
                    static_cast<usize>(0));
    }

    {
        // A RING of belts feeding each other is a legal thing to build. It
        // must terminate, and it must not produce a chain.
        std::vector<factory::PortNode> nodes = {machine(0.0, 10, false, true), belt(1.0, 11),
                                                belt(3.0, 12), belt(5.0, 13)};
        nodes[3].outPorts[0] = nodes[1].inPorts[0]; // last belt loops back to the first
        const std::vector<factory::Chain> chains = factory::traceConveyorChains(nodes, factory::kConveyorPortSnapM);
        SW_CHECK_EQ(chains.size(), static_cast<usize>(0));
    }

    {
        // TWO independent runs are two chains, and neither steals the other's
        // belts.
        std::vector<factory::PortNode> nodes = {
            machine(0.0, 10, false, true), belt(1.0, 11), machine(2.0, 12, true, false),
            machine(100.0, 13, false, true), belt(101.0, 14),
            machine(102.0, 15, true, false)};
        const std::vector<factory::Chain> chains = factory::traceConveyorChains(nodes, factory::kConveyorPortSnapM);
        SW_CHECK_EQ(chains.size(), static_cast<usize>(2));
        SW_CHECK_EQ(chains[0].belts.size(), static_cast<usize>(1));
        SW_CHECK_EQ(chains[1].belts.size(), static_cast<usize>(1));
        SW_CHECK(chains[0].belts[0] != chains[1].belts[0]);
    }

    {
        // A machine wired DIRECTLY to another — no belts at all — is still a
        // chain. Two machines set side by side should just work.
        const std::vector<factory::PortNode> nodes = {machine(0.0, 10, false, true),
                                                      machine(0.5, 11, true, false)};
        const std::vector<factory::Chain> chains = factory::traceConveyorChains(nodes, factory::kConveyorPortSnapM);
        SW_CHECK_EQ(chains.size(), static_cast<usize>(1));
        SW_CHECK(chains[0].belts.empty());
    }
}

// ============================================================================
// F49 — WHERE A RADIAL PART MAY GO
//
// The complaint was "radial placement barely works, and symmetry 3 is never
// green". It was not a symmetry bug: a radial part was tested for overlap
// against the very part it was being bolted to, and a round tank's hull is a
// SQUARE prism circumscribing it — so a battery was inside its parent's
// corner at every azimuth except the four compass points. Measured on the
// shipped catalogue: 28% of azimuths legal at 1, 2 and 4, and none at all at
// 3, 6 and 8, which put a copy at exactly 120°, 60° and 45°.
// ============================================================================

SW_TEST(ARadialPartMayBeGluedAllTheWayRoundItsParent)
{
    SW_CHECK(parts::loadCatalog(FileSystem::executableDirectory() / "Assets" / "Parts"));
    const parts::PartDefinition* tank =
        parts::findDefinition(parts::kPartFuelTankMedium);
    const parts::PartDefinition* core =
        parts::findDefinition(parts::kPartCoreStructural);
    const parts::PartDefinition* engine =
        parts::findDefinition(parts::kPartEngineVector);
    const parts::PartDefinition* battery =
        parts::findDefinition(parts::kPartBatteryPack);
    SW_CHECK(tank != nullptr && core != nullptr && engine != nullptr &&
             battery != nullptr);
    if (battery == nullptr || tank == nullptr) { return; }

    // The rocket every player builds first: core, tank, engine.
    constexpr usize kTankIndex = 1;
    std::vector<parts::PlacementPiece> placed{
        {core, Vec3{0.0f, 0.0f, -3.4f}, Quat{1, 0, 0, 0}},
        {tank, Vec3{0.0f}, Quat{1, 0, 0, 0}},
        {engine, Vec3{0.0f, 0.0f, 3.2f}, Quat{1, 0, 0, 0}}};

    i32 radialNode = -1;
    for (u8 c = 0; c < static_cast<u8>(battery->nodes.size()); ++c)
    {
        if (battery->nodes[c].type == parts::NodeType::Radial) { radialNode = c; break; }
    }
    SW_CHECK(radialNode >= 0);
    if (radialNode < 0) { return; }

    // Where the editor would put the part for a click at this azimuth and
    // height: on the tank's surface, its radial node facing into the hull.
    const auto glue = [&](f32 azimuth, f32 height) {
        const Vec3 normal{std::cos(azimuth), std::sin(azimuth), 0.0f};
        const Vec3 hit = normal * tank->shapes[0].size.x + Vec3{0.0f, 0.0f, height};
        const Quat rotation = glm::rotation(
            glm::normalize(battery->nodes[static_cast<usize>(radialNode)].direction),
            -normal);
        return parts::PlacementPiece{
            battery,
            hit - rotation * battery->nodes[static_cast<usize>(radialNode)].position,
            rotation};
    };

    for (const u32 symmetry : {1u, 2u, 3u, 4u, 6u, 8u})
    {
        for (i32 degrees = 0; degrees < 360; degrees += 5)
        {
            for (const f32 height : {-1.6f, -0.8f, 0.0f, 0.8f, 1.6f})
            {
                const parts::PlacementPiece piece =
                    glue(static_cast<f32>(degrees) * 3.14159265f / 180.0f, height);
                std::vector<parts::PlacementPiece> candidates;
                for (u32 k = 0; k < symmetry; ++k)
                {
                    candidates.push_back(parts::symmetryClone(piece, k, symmetry));
                }
                const i32 parent = static_cast<i32>(kTankIndex);
                SW_CHECK(!parts::placementCollides(candidates, placed,
                                                   std::span{&parent, 1}, 0.05f));
            }
        }
    }

    // A RING OF BOOSTERS, which is the shape the complaint was really about.
    // Three tanks strapped round a core at 120 degrees do not touch — the
    // gaps are metres wide — but their hulls are SQUARE prisms circumscribing
    // them, and two squares at 120 degrees intersect at the corners where the
    // metal never does. Nothing but the geometry changed here: a hull whose x
    // and y match, on a part built out of round primitives, is read as the
    // cylinder it is.
    {
        const parts::PartDefinition* booster =
            parts::findDefinition(parts::kPartFuelTankMedium);
        std::vector<parts::PlacementPiece> ring;
        for (i32 k = 0; k < 3; ++k)
        {
            const f32 azimuth = static_cast<f32>(k) * 2.0943951f;
            const Vec3 outward{std::cos(azimuth), std::sin(azimuth), 0.0f};
            // Rim to rim with a hand's width to spare, the way a decoupler
            // holds one.
            ring.push_back({booster, outward * 2.7f, Quat{1, 0, 0, 0}});
        }
        for (usize i = 0; i < ring.size(); ++i)
        {
            for (usize j = i + 1; j < ring.size(); ++j)
            {
                SW_CHECK(!parts::placementCollides(std::span{&ring[i], 1},
                                                   std::span{&ring[j], 1},
                                                   std::span<const i32>{}, 0.05f));
            }
            // ...and against the core they are strapped to, which is the
            // placement the editor actually judges.
            SW_CHECK(!parts::placementCollides(std::span{&ring[i], 1},
                                               std::span<const parts::PlacementPiece>{placed},
                                               std::span<const i32>{}, 0.05f));
        }
        // Round does not mean permissive: slide one in until the tubes really
        // do overlap and it is refused again.
        const parts::PlacementPiece jammed{booster, Vec3{2.0f, 0.0f, 0.0f},
                                           Quat{1, 0, 0, 0}};
        SW_CHECK(parts::placementCollides(std::span{&jammed, 1},
                                          std::span<const parts::PlacementPiece>{placed},
                                          std::span<const i32>{}, 0.05f));
    }

    // AND THE EXCLUSION IS EXACTLY ONE PART WIDE. The same spot, judged
    // without naming a parent, still reads as a collision — that is the
    // square hull the fix is working around, and it has not gone anywhere.
    const parts::PlacementPiece diagonal = glue(0.7f, 0.0f);
    SW_CHECK(parts::placementCollides(std::span{&diagonal, 1},
                                      std::span<const parts::PlacementPiece>{placed},
                                      std::span<const i32>{}, 0.05f));

    // ...and a part already standing where you are aiming is still in the
    // way, parent or no parent: shielding the parent must not shield the
    // design.
    placed.push_back(glue(0.0f, 0.0f));
    const parts::PlacementPiece onTop = glue(0.02f, 0.05f);
    const i32 tankIndex = static_cast<i32>(kTankIndex);
    SW_CHECK(parts::placementCollides(std::span{&onTop, 1},
                                      std::span<const parts::PlacementPiece>{placed},
                                      std::span{&tankIndex, 1}, 0.05f));
    // One step round the tank and it fits again.
    const parts::PlacementPiece clear = glue(1.0f, 0.0f);
    SW_CHECK(!parts::placementCollides(std::span{&clear, 1},
                                       std::span<const parts::PlacementPiece>{placed},
                                       std::span{&tankIndex, 1}, 0.05f));
}

SW_TEST(ARadialDecouplerDropsItsBoosterSideways)
{
    SW_CHECK(parts::loadCatalog(FileSystem::executableDirectory() / "Assets" / "Parts"));
    ecs::World world;

    const ecs::Entity root = world.createEntity();
    TransformComponent rootTransform{};
    rootTransform.position = {1.0e6, 0.0, 0.0};
    world.addComponent(root, rootTransform);
    world.addComponent(root, PreviousTransformComponent{});
    world.addComponent(root, parts::VesselComponent{});
    world.addComponent(root, phys::DynamicBodyComponent{{100.0, 0.0, 0.0}, 1.0});

    const auto addPart = [&](u32 definitionId, const Vec3& position,
                             const Quat& rotation = Quat{1.0f, 0.0f, 0.0f, 0.0f}) {
        const ecs::Entity part = world.createEntity();
        TransformComponent transform{};
        transform.position = rootTransform.position + WorldVec3(position);
        transform.rotation = rotation;
        world.addComponent(part, transform);
        world.addComponent(part, PreviousTransformComponent{});
        parts::PartComponent component{};
        component.definitionId = definitionId;
        component.vessel = root;
        component.localPosition = position;
        component.localRotation = rotation;
        world.addComponent(part, component);
        return part;
    };

    // A core with one booster strapped to its +X flank on a TR-2. The
    // decoupler's radial node looks -X, into the core.
    const ecs::Entity core = addPart(parts::kPartFuelTankMedium, {0.0f, 0.0f, 0.0f});
    const ecs::Entity tr2 = addPart(parts::kPartDecouplerRadial, {1.2f, 0.0f, 0.0f});
    const ecs::Entity booster = addPart(parts::kPartFuelTankMedium, {3.0f, 0.0f, 0.0f});
    const ecs::Entity boosterEngine =
        addPart(parts::kPartEngineVector, {3.0f, 0.0f, 3.2f});
    // Surface joints, exactly as the editor makes them: 255 on the parent,
    // and on the child the radial node that FACES the surface — node 3 of a
    // tank looks -X, which is the one a part glued to something on its -X
    // side would present.
    parts::connectParts(world, core, tr2, 255, 0, parts::JointType::Radial, 1.8e5, 1.8e5);
    parts::connectParts(world, tr2, booster, 255, 3, parts::JointType::Radial, 1.8e5, 1.8e5);
    parts::connectParts(world, booster, boosterEngine, 1, 0, parts::JointType::Stack, 4.0e5, 4.0e5);

    const ecs::Entity dropped = parts::decoupleAt(world, tr2);
    SW_CHECK(!dropped.isNull());
    if (dropped.isNull()) { return; }
    // THE BOOSTER LEAVES AND THE CORE DOES NOT — the old rule chose by height
    // along the stack, and a booster beside its parent has no height to speak
    // of, so it was a coin toss which side of the joint flew away.
    SW_CHECK(world.getComponent<parts::PartComponent>(core).vessel == root);
    SW_CHECK(world.getComponent<parts::PartComponent>(tr2).vessel == root);
    SW_CHECK(world.getComponent<parts::PartComponent>(booster).vessel == dropped);
    SW_CHECK(world.getComponent<parts::PartComponent>(boosterEngine).vessel == dropped);

    // ...and it goes OUT, along the joint that broke, not down the stack: a
    // booster shoved tail-ward grinds along the core it was bolted to.
    const auto& body = world.getComponent<phys::DynamicBodyComponent>(dropped);
    const WorldVec3 pushed = body.velocity - WorldVec3{100.0, 0.0, 0.0};
    SW_CHECK(pushed.x > 1.0);
    SW_CHECK(std::abs(pushed.z) < 0.1);
}

