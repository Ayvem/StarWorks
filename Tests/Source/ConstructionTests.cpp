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
#include <Factory/Conveyor.hpp>
#include <Gameplay/Construction.hpp>

#include <cmath>
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

    // ORE. The site the survey founds carries 0.85; the miner asks 0.15.
    f32 grade = 0.0f;
    const Vec3 site = planet::surveyEquatorialSite(
        terrain, deposits, res::Resource::IronOre, kRadius, grade);
    SW_CHECK(grade > miner->building.minOreDensity);
    SW_CHECK_EQ(build::validatePlacement(terrain, deposits, kRadius, *miner, site, {}),
                build::Verdict::Ok);

    // Barren but otherwise fine ground refuses the MINER and accepts the
    // solar field — the rule is per-building, straight off the .swpart.
    Vec3 barren = site;
    bool foundBarren = false;
    for (u32 i = 0; i < 40000 && !foundBarren; ++i)
    {
        const f32 a = static_cast<f32>(i) * 0.0137f;
        const f32 b = static_cast<f32>(i) * 0.0071f;
        const Vec3 candidate = glm::normalize(
            Vec3{std::cos(a) * std::cos(b), std::sin(b), std::sin(a) * std::cos(b)});
        f32 density = 0.0f;
        planet::bestDeposit(deposits, candidate, density);
        if (density < miner->building.minOreDensity &&
            planet::terrainElevation(terrain, candidate) > 40.0 &&
            planet::terrainLocalSlope(terrain, candidate, kRadius, 10.0f) < 0.10f)
        {
            barren = candidate;
            foundBarren = true;
        }
    }
    SW_CHECK(foundBarren);
    SW_CHECK_EQ(build::validatePlacement(terrain, deposits, kRadius, *miner, barren, {}),
                build::Verdict::NotEnoughOre);
    SW_CHECK_EQ(build::validatePlacement(terrain, deposits, kRadius, *solar, barren, {}),
                build::Verdict::Ok);

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
