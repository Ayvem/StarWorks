// ============================================================================
// HullTests.cpp — a part's HITBOX, and a machine's several MOUTHS.
//
// Two things a part used to only be able to have one of.
//
//   1. THE HULL. What a part looks like and what it bumps into were the same
//      list of primitives: the shapes marked `collider`. That conflates two
//      jobs — geometry wants cones and forty segments, collision wants as few
//      boxes as will do — and it meant you could not fix a hull without
//      redrawing the model. A part now AUTHORS its hull as axis-aligned
//      boxes, and a part that declares none still falls back to the old
//      behaviour, so every .swpart written before this loads unchanged.
//
//   2. THE MOUTHS. One conveyor-in and one conveyor-out per machine meant an
//      electrolyser's hydrogen and oxygen were stuck sharing a single belt.
//      Ports are a list now, and their ORDER is the contract: out mouth i
//      ships the recipe's product i.
// ============================================================================

#include "TestFramework.hpp"

#include <Core/FileSystem.hpp>
#include <Factory/Conveyor.hpp>
#include <Gameplay/PartGeometry.hpp>
#include <Gameplay/Parts.hpp>
#include <ECS/World.hpp>
#include <Physics/HullCollision.hpp>
#include <Physics/PhysicsSystems.hpp>
#include <Scene/TransformComponents.hpp>

#include <cmath>
#include <filesystem>

using namespace sw;
using namespace sw::parts;

namespace
{
    /// A part whose GEOMETRY and whose HULL deliberately disagree: a slim
    /// visible bar, and a hull box twice as wide. Nothing may quietly use one
    /// where the other was meant.
    [[nodiscard]] PartDefinition mismatchedPart()
    {
        PartDefinition part{};
        part.id = 9001;
        part.name = "TEST BAR";
        PartShape bar{};
        bar.kind = ShapeKind::Box;
        bar.size = {0.5f, 0.1f, 0.1f};
        bar.visible = true;
        bar.collider = true;
        part.shapes.push_back(bar);
        return part;
    }

    [[nodiscard]] Vec3 hullSize(const PartDefinition& part)
    {
        constexpr f32 kHuge = 1.0e9f;
        Vec3 low{kHuge, kHuge, kHuge};
        Vec3 high{-kHuge, -kHuge, -kHuge};
        expandPartHullBounds(part, Vec3{0.0f}, Quat{1.0f, 0.0f, 0.0f, 0.0f}, low, high);
        return high - low;
    }
} // namespace

SW_TEST(TheAuthoredHullWinsOverTheGeometry)
{
    PartDefinition part = mismatchedPart();

    // No hitboxes: the collider shapes ARE the hull, exactly as before.
    SW_CHECK(!hasHitbox(part));
    SW_CHECK(glm::length(hullSize(part) - Vec3{1.0f, 0.2f, 0.2f}) < 1.0e-5f);

    // Declare one, and it is what answers — even where it disagrees with
    // every triangle the part draws. That disagreement is the whole point:
    // a hull you cannot change without redrawing the model is not authored.
    part.hitboxes.push_back({Vec3{0.0f, 1.0f, 0.0f}, Vec3{2.0f, 0.5f, 0.5f}});
    SW_CHECK(hasHitbox(part));
    SW_CHECK(glm::length(hullSize(part) - Vec3{4.0f, 1.0f, 1.0f}) < 1.0e-5f);

    // ...and the culling sphere grew with it, or the part would vanish while
    // it was still being stood on.
    SW_CHECK(partBoundsRadius(part) > glm::length(Vec3{0.0f, 1.0f, 0.0f}));

    // A SECOND box unions in rather than replacing: a part is not one box.
    part.hitboxes.push_back({Vec3{0.0f, -3.0f, 0.0f}, Vec3{0.25f, 0.5f, 0.25f}});
    const Vec3 size = hullSize(part);
    SW_CHECK(std::abs(size.x - 4.0f) < 1.0e-5f);  // widest box still rules X
    SW_CHECK(std::abs(size.y - 5.0f) < 1.0e-5f);  // -3.5 .. +1.5
}

SW_TEST(TheHullFollowsThePartWhenThePartIsTurned)
{
    PartDefinition part = mismatchedPart();
    part.hitboxes.push_back({Vec3{0.0f}, Vec3{2.0f, 0.5f, 0.5f}});

    // A quarter turn about Y swaps the box's X and Z extents. The hull is
    // axis-aligned in the PART's frame, not the world's — otherwise a rocket
    // lying on its side would rest on the box it has standing up.
    constexpr f32 kHuge = 1.0e9f;
    Vec3 low{kHuge, kHuge, kHuge};
    Vec3 high{-kHuge, -kHuge, -kHuge};
    expandPartHullBounds(part, Vec3{0.0f},
                         glm::angleAxis(1.5707963f, Vec3{0.0f, 1.0f, 0.0f}), low, high);
    const Vec3 size = high - low;
    SW_CHECK(std::abs(size.x - 1.0f) < 1.0e-4f);
    SW_CHECK(std::abs(size.z - 4.0f) < 1.0e-4f);

    // ...and it translates with the part, rather than sitting at the origin.
    low = Vec3{kHuge};
    high = Vec3{-kHuge};
    expandPartHullBounds(part, Vec3{10.0f, 0.0f, 0.0f}, Quat{1.0f, 0.0f, 0.0f, 0.0f},
                         low, high);
    SW_CHECK(std::abs(low.x - 8.0f) < 1.0e-5f);
    SW_CHECK(std::abs(high.x - 12.0f) < 1.0e-5f);
}

SW_TEST(FittingAHullFromCollidersBoundsEveryRotatedShape)
{
    PartDefinition part{};
    part.id = 9002;
    part.name = "TEST WEDGE";
    PartShape bar{};
    bar.kind = ShapeKind::Box;
    bar.size = {1.0f, 0.1f, 0.1f};
    bar.rotationDeg = {0.0f, 0.0f, 45.0f}; // turned in the XY plane
    bar.visible = true;
    bar.collider = true;
    part.shapes.push_back(bar);
    // A render-only greeble must NOT enter the hull: that is the whole point
    // of the collider flag, and FIT must respect it.
    PartShape greeble{};
    greeble.kind = ShapeKind::Box;
    greeble.position = {0.0f, 8.0f, 0.0f};
    greeble.size = {0.05f, 0.05f, 0.05f};
    greeble.visible = true;
    greeble.collider = false;
    part.shapes.push_back(greeble);

    const std::vector<HitBox> fitted = hitboxesFromColliders(part);
    SW_CHECK_EQ(fitted.size(), static_cast<usize>(1));
    // An AABB around a 45-degree bar is wider than the bar: 1.0*cos45 +
    // 0.1*sin45 = 0.778 on each of X and Y. Bounding a rotated box is what
    // makes the fitted hull a starting point to TIGHTEN rather than a final
    // answer, and the number says so honestly.
    SW_CHECK(std::abs(fitted[0].halfExtents.x - 0.7778f) < 1.0e-3f);
    SW_CHECK(std::abs(fitted[0].halfExtents.y - 0.7778f) < 1.0e-3f);
    SW_CHECK(std::abs(fitted[0].halfExtents.z - 0.1f) < 1.0e-5f);
}

SW_TEST(TwoPartsCollideByTheirHullsNotTheirTriangles)
{
    PartDefinition slim = mismatchedPart();   // visible half-extent 0.5 on X
    PartDefinition wide = mismatchedPart();
    wide.id = 9003;
    wide.hitboxes.push_back({Vec3{0.0f}, Vec3{3.0f, 0.5f, 0.5f}});

    const Quat none{1.0f, 0.0f, 0.0f, 0.0f};
    // 2 m apart: the two BARS miss by a metre, but `wide` declares a hull
    // that reaches 3 m, so they touch. A VAB that validated against the
    // triangles would let the player build a rocket that cannot exist.
    SW_CHECK(partsOverlap(slim, Vec3{0.0f}, none, wide, Vec3{2.0f, 0.0f, 0.0f}, none,
                          0.01f));
    // ...and far enough apart, they do not.
    SW_CHECK(!partsOverlap(slim, Vec3{0.0f}, none, wide, Vec3{6.0f, 0.0f, 0.0f}, none,
                           0.01f));
}

SW_TEST(EveryShippedPartResolvesToASolidHullExceptTheOnesYouWalkThrough)
{
    SW_CHECK(loadCatalog(FileSystem::executableDirectory() / "Assets" / "Parts"));
    usize authored = 0;
    usize derived = 0;
    for (const PartDefinition& definition : catalog())
    {
        // The contract is not "every file authors a hull" — that is the
        // artist's job and still in progress — it is that every part
        // RESOLVES to one, authored or derived, so nothing is accidentally
        // intangible while the hulls are being drawn.
        const std::vector<HitBox> hull = effectiveHull(definition);
        SW_CHECK(!hull.empty());
        (hasHitbox(definition) ? authored : derived) += 1;
        for (const HitBox& box : hull)
        {
            // A zero-thickness hull is a hull nothing can rest on and
            // nothing can be stopped by.
            SW_CHECK(box.halfExtents.x > 0.0f);
            SW_CHECK(box.halfExtents.y > 0.0f);
            SW_CHECK(box.halfExtents.z > 0.0f);
        }
        const Vec3 size = hullSize(definition);
        SW_CHECK(size.x > 0.0f && size.y > 0.0f && size.z > 0.0f);
    }
    SW_CHECK(authored > 0); // the authored hulls really are being read

    // ...and the two things you WALK THROUGH are excluded by CATEGORY, not
    // by whether someone remembered to leave their hitboxes out. A belt you
    // could not step over would make a factory floor an assault course.
    const PartDefinition* belt = findDefinition(kBuildingConveyor);
    const PartDefinition* cable = findDefinition(kBuildingCable);
    SW_CHECK(belt != nullptr && cable != nullptr);
    if (belt != nullptr && cable != nullptr)
    {
        SW_CHECK(!isSolid(*belt));
        SW_CHECK(!isSolid(*cable));
        // They still HAVE hitboxes — the build validator and the renderer
        // want them. Solidity is a separate question from shape.
        SW_CHECK(!effectiveHull(*belt).empty());
    }
    for (const u32 id : {kBuildingRefinery, kBuildingMiner, kBuildingStorage,
                         kBuildingPowerPole, kPartFuelTankMedium, kPropEvaSuit})
    {
        const PartDefinition* definition = findDefinition(id);
        SW_CHECK(definition != nullptr);
        if (definition != nullptr)
        {
            SW_CHECK(isSolid(*definition));
        }
    }

    // THE PLAYER IS A PART, and its hull is what ground contact stands on,
    // so it must be a plausible body rather than a stray default.
    const PartDefinition* suit = findDefinition(kPropEvaSuit);
    SW_CHECK(suit != nullptr);
    if (suit != nullptr)
    {
        SW_CHECK(isProp(*suit)); // never in the VAB palette
        const Vec3 size = hullSize(*suit);
        SW_CHECK(size.y > 1.0f && size.y < 5.0f);
        SW_CHECK(size.x < 3.0f && size.z < 3.0f);
        // The origin sits INSIDE the body, not on top of it: the suit
        // reaches below its own transform, which is what stops the player
        // walking with their waist in the rock.
        SW_CHECK(suit->hitboxes[0].center.y - suit->hitboxes[0].halfExtents.y < -0.5f);
    }
}

SW_TEST(HullsSurviveTheJsonRoundTrip)
{
    PartDefinition part = mismatchedPart();
    part.hitboxes.push_back({Vec3{0.25f, -1.5f, 0.75f}, Vec3{2.0f, 0.5f, 0.125f}});
    part.hitboxes.push_back({Vec3{0.0f, 3.0f, 0.0f}, Vec3{0.5f, 0.5f, 0.5f}});

    const std::filesystem::path temp =
        std::filesystem::temp_directory_path() / "sw_hull_roundtrip.swpart";
    SW_CHECK(savePartFile(part, temp));
    PartDefinition loaded{};
    SW_CHECK(loadPartFile(temp, loaded));
    std::filesystem::remove(temp);

    SW_CHECK_EQ(loaded.hitboxes.size(), part.hitboxes.size());
    for (usize i = 0; i < loaded.hitboxes.size(); ++i)
    {
        SW_CHECK(glm::length(loaded.hitboxes[i].center - part.hitboxes[i].center) <
                 1.0e-4f);
        SW_CHECK(glm::length(loaded.hitboxes[i].halfExtents -
                             part.hitboxes[i].halfExtents) < 1.0e-4f);
    }

    // A part with NO hull writes no `hitboxes` key at all, so a file that
    // never had one does not grow a field it does not use.
    PartDefinition plain = mismatchedPart();
    SW_CHECK(savePartFile(plain, temp));
    PartDefinition reloaded{};
    SW_CHECK(loadPartFile(temp, reloaded));
    std::filesystem::remove(temp);
    SW_CHECK(reloaded.hitboxes.empty());
}

// ---------------------------------------------------------------------------
// SEVERAL MOUTHS
// ---------------------------------------------------------------------------
SW_TEST(TheRefineryHasTwoMouthsEachWayInAuthoredOrder)
{
    SW_CHECK(loadCatalog(FileSystem::executableDirectory() / "Assets" / "Parts"));
    const PartDefinition* refinery = findDefinition(kBuildingRefinery);
    SW_CHECK(refinery != nullptr);
    if (refinery == nullptr)
    {
        return;
    }

    const std::vector<const AttachNode*> ins =
        conveyorNodes(*refinery, NodeType::ConveyorIn);
    const std::vector<const AttachNode*> outs =
        conveyorNodes(*refinery, NodeType::ConveyorOut);
    SW_CHECK_EQ(ins.size(), static_cast<usize>(2));
    SW_CHECK_EQ(outs.size(), static_cast<usize>(2));
    SW_CHECK_EQ(conveyorNodeCount(*refinery, NodeType::ConveyorIn), 2u);

    // The mouths are in DIFFERENT places — two ports at the same spot would
    // be one port with extra steps, and the belt tool picks by proximity.
    SW_CHECK(glm::length(ins[0]->position - ins[1]->position) > 1.0f);
    SW_CHECK(glm::length(outs[0]->position - outs[1]->position) > 1.0f);

    // ...and `findConveyorNode` still answers with the FIRST, so every
    // caller written before ports were a list keeps its old behaviour.
    SW_CHECK(findConveyorNode(*refinery, NodeType::ConveyorIn) == ins[0]);
    SW_CHECK(findConveyorNode(*refinery, NodeType::ConveyorOut) == outs[0]);

    // Every mouth is on the hull, not floating beside the machine.
    for (const AttachNode* node : {ins[0], ins[1], outs[0], outs[1]})
    {
        const Vec3 origin = node->position + node->direction * 3.0f;
        PartRayHit hit{};
        SW_CHECK(raycastPart(*refinery, origin, -node->direction, 6.0f, hit));
        SW_CHECK(glm::length((origin - node->direction * hit.t) - node->position) < 0.15f);
    }
}

SW_TEST(TwoMouthsCarryTwoProductsDownTwoBelts)
{
    using factory::PortNode;

    // An electrolyser with two out mouths, and two silos, each fed by its
    // own one-tile run. This is the layout the second port pair exists for:
    // hydrogen one way, oxygen the other.
    auto node = [](u32 index) {
        PortNode n{};
        n.entity = {index, 0};
        return n;
    };

    std::vector<PortNode> nodes;
    // 0: the machine. Out mouth 0 at -X, out mouth 1 at +X.
    PortNode machine = node(10);
    machine.centre = WorldVec3{0.0, 0.0, 0.0};
    machine.outPorts[0] = WorldVec3{-8.0, 0.0, 0.0};
    machine.outPorts[1] = WorldVec3{8.0, 0.0, 0.0};
    machine.outCount = 2;
    nodes.push_back(machine);

    // 1, 2: one belt tile on each side (2 m long, mouths 1 m from centre).
    PortNode left = node(11);
    left.isBelt = true;
    left.centre = WorldVec3{-9.0, 0.0, 0.0};
    left.inPorts[0] = WorldVec3{-8.0, 0.0, 0.0};
    left.outPorts[0] = WorldVec3{-10.0, 0.0, 0.0};
    left.inCount = 1;
    left.outCount = 1;
    nodes.push_back(left);

    PortNode right = node(12);
    right.isBelt = true;
    right.centre = WorldVec3{9.0, 0.0, 0.0};
    right.inPorts[0] = WorldVec3{8.0, 0.0, 0.0};
    right.outPorts[0] = WorldVec3{10.0, 0.0, 0.0};
    right.inCount = 1;
    right.outCount = 1;
    nodes.push_back(right);

    // 3, 4: the silos.
    PortNode siloA = node(13);
    siloA.centre = WorldVec3{-11.0, 0.0, 0.0};
    siloA.inPorts[0] = WorldVec3{-10.0, 0.0, 0.0};
    siloA.inCount = 1;
    nodes.push_back(siloA);

    PortNode siloB = node(14);
    siloB.centre = WorldVec3{11.0, 0.0, 0.0};
    siloB.inPorts[0] = WorldVec3{10.0, 0.0, 0.0};
    siloB.inCount = 1;
    nodes.push_back(siloB);

    const std::vector<factory::Chain> chains = factory::traceConveyorChains(nodes, 0.4);
    SW_CHECK_EQ(chains.size(), static_cast<usize>(2));

    // TWO chains, from the SAME machine, by DIFFERENT mouths — which is what
    // lets the game route product 0 down one and product 1 down the other.
    bool sawPortZero = false;
    bool sawPortOne = false;
    for (const factory::Chain& chain : chains)
    {
        SW_CHECK_EQ(chain.source, 0u);
        SW_CHECK_EQ(chain.belts.size(), static_cast<usize>(1));
        sawPortZero = sawPortZero || chain.sourcePort == 0;
        sawPortOne = sawPortOne || chain.sourcePort == 1;
        SW_CHECK(chain.destination == 3u || chain.destination == 4u);
    }
    SW_CHECK(sawPortZero);
    SW_CHECK(sawPortOne);
}

SW_TEST(AnInMouthTakesOneBeltAndTheSecondRunFindsTheOther)
{
    using factory::PortNode;

    // Two machines shipping into ONE machine that has two in mouths, both
    // within snap of both belts. Without claiming, both runs would pile onto
    // whichever mouth happened to be nearest and the second would be a link
    // into an occupied hole.
    std::vector<PortNode> nodes;
    for (u32 side = 0; side < 2; ++side)
    {
        PortNode source{};
        source.entity = {10 + side, 0};
        source.centre = WorldVec3{-4.0, static_cast<f64>(side), 0.0};
        source.outPorts[0] = WorldVec3{-1.0, static_cast<f64>(side) * 0.1, 0.0};
        source.outCount = 1;
        nodes.push_back(source);
    }
    PortNode sink{};
    sink.entity = {20, 0};
    sink.centre = WorldVec3{2.0, 0.0, 0.0};
    sink.inPorts[0] = WorldVec3{-1.0, 0.0, 0.0};
    sink.inPorts[1] = WorldVec3{-1.0, 0.1, 0.0};
    sink.inCount = 2;
    nodes.push_back(sink);

    const std::vector<factory::Chain> chains = factory::traceConveyorChains(nodes, 1.0);
    SW_CHECK_EQ(chains.size(), static_cast<usize>(2));
    SW_CHECK(chains[0].destinationPort != chains[1].destinationPort);
    SW_CHECK(chains[0].source != chains[1].source);
}

// ---------------------------------------------------------------------------
// SOLID OBJECTS
//
// The arithmetic that decides where the player ends up when they walk into a
// refinery. It is pure, so it can be pinned exactly rather than eyeballed in
// a running game — which is the only reason it ever gets pinned at all.
// ---------------------------------------------------------------------------
SW_TEST(TheShortestPushIsTheOneThatGetsYouOutOfAWall)
{
    using namespace sw::phys;
    const Quat none{1.0f, 0.0f, 0.0f, 0.0f};

    // A player-sized box overlapping a wide, thin wall by 10 cm on Z.
    const Obb player = makeObb(Vec3{0.0f, 0.0f, 0.0f}, Vec3{0.4f, 0.9f, 0.4f}, none);
    const Obb wall = makeObb(Vec3{0.0f, 0.0f, 0.7f}, Vec3{5.0f, 3.0f, 0.4f}, none);

    Vec3 axis{0.0f};
    f32 depth = 0.0f;
    SW_CHECK(obbPenetration(player, wall, axis, depth));
    // 0.4 + 0.4 - 0.7 = 0.1 m of overlap, and the way out is BACKWARDS along
    // Z. Pushing along the deepest axis instead would send a player standing
    // beside a wall out through its roof — 3 m up rather than 10 cm back.
    SW_CHECK(std::abs(depth - 0.1f) < 1.0e-5f);
    SW_CHECK(glm::length(axis - Vec3{0.0f, 0.0f, -1.0f}) < 1.0e-5f);

    // Applying it separates them, and only just: a push that overshot would
    // pop the player away from every surface they brushed.
    const Obb moved = makeObb(player.centre + axis * depth, player.halfExtents, none);
    SW_CHECK(!obbOverlap(moved, wall));
    const Obb short_ = makeObb(player.centre + axis * (depth * 0.9f),
                               player.halfExtents, none);
    SW_CHECK(obbOverlap(short_, wall));
}

SW_TEST(EdgeToEdgeOverlapIsCaughtByTheCrossAxes)
{
    using namespace sw::phys;
    // Two long thin boxes crossing at 45 degrees, offset so that NEITHER
    // box's own three face axes separate them — only a cross product does.
    // Dropping the nine cross axes is the classic separating-axis shortcut
    // and the classic separating-axis bug: it reads as the player being able
    // to stand inside the corner of a diagonal building.
    const Obb a = makeObb(Vec3{0.0f}, Vec3{2.0f, 0.1f, 0.1f},
                          Quat{1.0f, 0.0f, 0.0f, 0.0f});
    const Obb b = makeObb(Vec3{0.0f, 0.0f, 0.0f}, Vec3{2.0f, 0.1f, 0.1f},
                          glm::angleAxis(0.7853982f, Vec3{0.0f, 1.0f, 0.0f}));
    SW_CHECK(obbOverlap(a, b));

    // ...and slid apart along their common normal, they part.
    const Obb high = makeObb(Vec3{0.0f, 0.5f, 0.0f}, Vec3{2.0f, 0.1f, 0.1f},
                             glm::angleAxis(0.7853982f, Vec3{0.0f, 1.0f, 0.0f}));
    SW_CHECK(!obbOverlap(a, high));
}

SW_TEST(ARayFindsTheNearFaceOfWhatYouAreLookingAt)
{
    using namespace sw::phys;
    const Quat none{1.0f, 0.0f, 0.0f, 0.0f};
    const Obb machine = makeObb(Vec3{0.0f, 0.0f, 10.0f}, Vec3{4.0f, 3.0f, 2.0f}, none);

    // Looking straight at it from the origin: the hit is the NEAR face at
    // 8 m, not the centre at 10 — which is the whole reason the E panel
    // asks a ray rather than a distance. A machine you are touching can
    // have its centre well out of reach.
    f32 t = 0.0f;
    Vec3 normal{};
    SW_CHECK(rayObb(Vec3{0.0f}, Vec3{0.0f, 0.0f, 1.0f}, machine, 50.0f, t, normal));
    SW_CHECK(std::abs(t - 8.0f) < 1.0e-4f);
    SW_CHECK(glm::length(normal - Vec3{0.0f, 0.0f, -1.0f}) < 1.0e-4f);

    // BEHIND you is not in front of you, however close it is. The old rule
    // took the nearest centre and would happily have opened this one.
    SW_CHECK(!rayObb(Vec3{0.0f}, Vec3{0.0f, 0.0f, -1.0f}, machine, 50.0f, t, normal));
    // ...and neither is "in front, but out of reach".
    SW_CHECK(!rayObb(Vec3{0.0f}, Vec3{0.0f, 0.0f, 1.0f}, machine, 5.0f, t, normal));
    // Off to one side, the ray misses even at the right distance.
    SW_CHECK(!rayObb(Vec3{20.0f, 0.0f, 0.0f}, Vec3{0.0f, 0.0f, 1.0f}, machine, 50.0f, t,
                     normal));
}

SW_TEST(TheBroadPhaseRejectsWithOneComparison)
{
    using namespace sw::phys;
    // A hull's radius must CONTAIN every one of its boxes, or the broad
    // phase would reject a pair that really touches — a false negative you
    // would experience as walking through a wall now and then.
    const PartDefinition* refinery = findDefinition(kBuildingRefinery);
    SW_CHECK(refinery != nullptr);
    if (refinery == nullptr)
    {
        return;
    }
    f32 radius = 0.0f;
    for (const HitBox& box : effectiveHull(*refinery))
    {
        radius = std::max(radius, obbRadius(box.center, box.halfExtents));
    }
    constexpr f32 kHuge = 1.0e9f;
    Vec3 low{kHuge, kHuge, kHuge};
    Vec3 high{-kHuge, -kHuge, -kHuge};
    expandPartHullBounds(*refinery, Vec3{0.0f}, Quat{1.0f, 0.0f, 0.0f, 0.0f}, low, high);
    SW_CHECK(radius >= glm::length(low) - 1.0e-3f);
    SW_CHECK(radius >= glm::length(high) - 1.0e-3f);
}

SW_TEST(TheWalkerIsPushedOutOfBuildingsAndWalksOverBelts)
{
    using namespace sw::phys;
    ecs::World world;

    // A hull built the way the game builds one, from a real definition.
    auto solidFrom = [](const PartDefinition& definition) {
        HullComponent hull{};
        for (const HitBox& box : effectiveHull(definition))
        {
            if (hull.count >= kMaxHullBoxes) { break; }
            hull.boxes[hull.count++] = {box.center, glm::abs(box.halfExtents)};
            hull.radius = std::max(hull.radius, obbRadius(box.center, box.halfExtents));
        }
        return hull;
    };
    auto spawn = [&](const PartDefinition& definition, const WorldVec3& position) {
        const ecs::Entity entity = world.createEntity();
        TransformComponent transform{};
        transform.position = position;
        world.addComponent(entity, transform);
        world.addComponent(entity, solidFrom(definition));
        return entity;
    };

    const PartDefinition* refinery = findDefinition(kBuildingRefinery);
    const PartDefinition* suit = findDefinition(kPropEvaSuit);
    SW_CHECK(refinery != nullptr && suit != nullptr);
    if (refinery == nullptr || suit == nullptr)
    {
        return;
    }

    // The refinery at the origin; the player pressed into its +Z face,
    // which is the shortest way back out — so the push is along -Z and the
    // velocity it kills is the one that was carrying them in.
    spawn(*refinery, WorldVec3{0.0, 0.0, 0.0});
    const ecs::Entity player = spawn(*suit, WorldVec3{0.0, 0.0, 5.5});
    world.addComponent(player, HullMoverComponent{});
    DynamicBodyComponent body{};
    body.velocity = WorldVec3{0.0, 0.0, 5.0}; // still walking in
    body.mass = 120.0;
    world.addComponent(player, body);

    HullCollisionSystem collision;
    // A handful of ticks: the push is capped per tick on purpose, so a body
    // that starts buried walks out rather than being flung across the map.
    for (u32 tick = 0; tick < 12; ++tick)
    {
        collision.update(world, 0.02f);
    }

    const WorldVec3 out = world.getComponent<TransformComponent>(player).position;
    SW_CHECK(out.z > 5.5); // it moved, and outward — away from the machine
    // ...and it is genuinely clear of the machine now.
    Obb playerBox = makeObb(Vec3(out), suit->hitboxes[0].halfExtents,
                            Quat{1.0f, 0.0f, 0.0f, 0.0f});
    playerBox.centre += suit->hitboxes[0].center;
    bool stillInside = false;
    for (const HitBox& box : effectiveHull(*refinery))
    {
        stillInside = stillInside ||
                      obbOverlap(playerBox, makeObb(box.center, box.halfExtents,
                                                    Quat{1.0f, 0.0f, 0.0f, 0.0f}));
    }
    SW_CHECK(!stillInside);
    // THE VELOCITY IS UNTOUCHED. The first version removed "the component
    // heading into the wall" and launched the player a hundred metres,
    // because a world velocity on a planet is mostly CARRIER motion —
    // 30 km/s of orbit and 465 m/s of spin — and taking a bite out of it
    // is a kilometres-per-second impulse. Position-only resolution is the
    // whole job; the walker sets its own tangential velocity every tick.
    const WorldVec3 kept = world.getComponent<DynamicBodyComponent>(player).velocity;
    SW_CHECK(glm::length(kept - WorldVec3{0.0, 0.0, 5.0}) < 1.0e-12);

    // ---- and a BELT is not an obstacle -------------------------------
    // A conveyor deck carries hitboxes (the renderer and the build
    // validator want them) but is not SOLID, so the game gives it no hull
    // at all and a player standing on one is left exactly where they are.
    const PartDefinition* belt = findDefinition(kBuildingConveyor);
    SW_CHECK(belt != nullptr && !isSolid(*belt));

    ecs::World flat;
    const ecs::Entity walker = flat.createEntity();
    TransformComponent transform{};
    flat.addComponent(walker, transform);
    flat.addComponent(walker, solidFrom(*suit));
    flat.addComponent(walker, HullMoverComponent{});
    HullCollisionSystem alone;
    alone.update(flat, 0.02f);
    SW_CHECK(glm::length(flat.getComponent<TransformComponent>(walker).position) == 0.0);
    SW_CHECK_EQ(alone.lastNarrowPairs(), 0u); // nothing to test against
}

SW_TEST(WalkingIntoAWallDoesNotFireYouOffAPlanet)
{
    using namespace sw::phys;
    ecs::World world;

    // Two boxes overlapping by 10 cm, and a mover carrying the velocity a
    // body ACTUALLY has when it is standing on Terra: thirty kilometres a
    // second of orbit around the Sun, plus the planet's own rotation.
    // Projecting THAT onto a wall normal and subtracting it is what threw
    // the player a hundred metres, so this is the regression.
    const WorldVec3 carrier{29780.0, 0.0, 465.0};

    auto box = [&](const WorldVec3& position) {
        const ecs::Entity entity = world.createEntity();
        TransformComponent transform{};
        transform.position = position;
        world.addComponent(entity, transform);
        HullComponent hull{};
        hull.boxes[0] = {Vec3{0.0f}, Vec3{1.0f, 1.0f, 1.0f}};
        hull.count = 1;
        hull.radius = obbRadius(Vec3{0.0f}, Vec3{1.0f, 1.0f, 1.0f});
        world.addComponent(entity, hull);
        return entity;
    };

    box(WorldVec3{0.0, 0.0, 0.0});
    const ecs::Entity walker = box(WorldVec3{0.0, 0.0, 1.9});
    world.addComponent(walker, HullMoverComponent{});
    DynamicBodyComponent body{};
    body.velocity = carrier;
    body.mass = 120.0;
    world.addComponent(walker, body);

    HullCollisionSystem collision;
    collision.update(world, 0.02f);

    // Pushed clear by exactly the overlap — 10 cm, not a hundred metres.
    const WorldVec3 out = world.getComponent<TransformComponent>(walker).position;
    SW_CHECK(std::abs(out.z - 2.0) < 1.0e-5);
    SW_CHECK(std::abs(out.x) < 1.0e-9 && std::abs(out.y) < 1.0e-9);

    // ...and the carrier velocity came through untouched. A collision on a
    // moving planet must never reason about a world velocity.
    SW_CHECK(glm::length(
                 world.getComponent<DynamicBodyComponent>(walker).velocity - carrier) <
             1.0e-12);

    // A second tick moves nothing: once separated, there is nothing to fix,
    // which is what stops a resolved contact from becoming a rocket motor.
    collision.update(world, 0.02f);
    SW_CHECK(glm::length(world.getComponent<TransformComponent>(walker).position - out) <
             1.0e-12);
}
