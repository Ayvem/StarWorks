// ============================================================================
// FairingTests.cpp — F48: the shell the player draws.
//
// A fairing is the first part in this game whose SHAPE is not authored, so it
// is also the first whose numbers cannot be checked against a file. They are
// checked against the textbook instead: a cone made of flat panels has a drag
// coefficient this file works out from its half-angle, and the code has to
// land on it without ever having been told what a cone is.
// ============================================================================

#include "TestFramework.hpp"

#include <Core/FileSystem.hpp>
#include <ECS/World.hpp>
#include <Gameplay/Blueprint.hpp>
#include <Gameplay/Fairing.hpp>
#include <Gameplay/PartGeometry.hpp>
#include <Gameplay/Parts.hpp>
#include <Gameplay/VesselAerodynamics.hpp>
#include <Physics/Aerodynamics.hpp>
#include <Scene/TransformComponents.hpp>

#include <glm/gtx/quaternion.hpp> // glm::rotation, for the attitude fixture

#include <cmath>
#include <filesystem>

using namespace sw;
using namespace sw::parts;

namespace
{
    constexpr f64 kPi = 3.14159265358979323846;

    /// A cone of base radius `radius` and height `height`, cut into `sides`
    /// flat panels: ring 0 is the base's rim, ring 1 is the closed nose.
    [[nodiscard]] FairingComponent cone(f32 radius, f32 height, u32 sides = 24)
    {
        FairingComponent fairing{};
        fairing.rings[0] = {0.0f, radius};
        fairing.rings[1] = {height, 0.0f};
        fairing.ringCount = 2;
        fairing.sides = sides;
        fairing.closed = 1;
        return fairing;
    }

    /// A plain drum: base rim, then the same radius higher up, then a nose
    /// cap of the given length. The shape a payload actually flies under.
    [[nodiscard]] FairingComponent shroud(f32 radius, f32 barrel, f32 nose,
                                          u32 sides = 16)
    {
        FairingComponent fairing{};
        fairing.rings[0] = {0.0f, radius};
        fairing.rings[1] = {barrel, radius};
        fairing.rings[2] = {barrel + nose, 0.0f};
        fairing.ringCount = 3;
        fairing.sides = sides;
        fairing.closed = 1;
        return fairing;
    }
} // namespace

// ---------------------------------------------------------------------------
// THE PROFILE
// ---------------------------------------------------------------------------

SW_TEST(AProfileIsAStraightLineBetweenTheRingsTheCursorPlaced)
{
    const FairingComponent shell = shroud(1.25f, 3.0f, 2.0f);

    // On a ring, at the ring's radius; between two, on the line joining them.
    SW_CHECK(std::abs(fairingRadiusAt(shell, 0.0f) - 1.25f) < 1.0e-5f);
    SW_CHECK(std::abs(fairingRadiusAt(shell, 1.5f) - 1.25f) < 1.0e-5f);
    SW_CHECK(std::abs(fairingRadiusAt(shell, 3.0f) - 1.25f) < 1.0e-5f);
    SW_CHECK(std::abs(fairingRadiusAt(shell, 4.0f) - 0.625f) < 1.0e-5f);
    SW_CHECK(std::abs(fairingRadiusAt(shell, 5.0f)) < 1.0e-5f);
    // Outside the profile there is no shell, and saying "the nearest radius"
    // there would shield a payload standing above the nose.
    SW_CHECK(fairingRadiusAt(shell, -0.1f) == 0.0f);
    SW_CHECK(fairingRadiusAt(shell, 5.1f) == 0.0f);

    // A DRAFT IS NOT A SHELL. One ring is the base's own rim and nothing else.
    FairingComponent draft{};
    draft.rings[0] = {0.0f, 1.25f};
    draft.ringCount = 1;
    SW_CHECK(fairingRadiusAt(draft, 0.0f) == 0.0f);
}

SW_TEST(AFairingEnclosesWhatIsInsideItAndNothingElse)
{
    const FairingComponent shell = shroud(1.25f, 3.0f, 2.0f);
    // -Z is up the stack: a payload one metre above the mounting face sits at
    // z = -1, which is the sign error this test exists to catch.
    SW_CHECK(fairingEncloses(shell, Vec3{0.0f, 0.0f, -1.0f}));
    SW_CHECK(fairingEncloses(shell, Vec3{1.0f, 0.0f, -2.0f}));
    SW_CHECK(fairingEncloses(shell, Vec3{0.0f, 0.4f, -4.0f})); // inside the nose
    // Through the wall, above the nose, below the base: all outside.
    SW_CHECK(!fairingEncloses(shell, Vec3{1.3f, 0.0f, -2.0f}));
    SW_CHECK(!fairingEncloses(shell, Vec3{0.0f, 0.9f, -4.0f}));
    SW_CHECK(!fairingEncloses(shell, Vec3{0.0f, 0.0f, -5.5f}));
    SW_CHECK(!fairingEncloses(shell, Vec3{0.0f, 0.0f, 0.5f}));
    // The point is taken WHOLE and in the shell's own frame — a point on the
    // far side of the axis is as enclosed as one on the near side.
    SW_CHECK(fairingEncloses(shell, Vec3{-1.0f, 0.0f, -2.0f}));

    // AN OPEN DRAFT SHIELDS NOTHING, and neither does one already thrown away.
    FairingComponent open = shell;
    open.closed = 0;
    SW_CHECK(!fairingEncloses(open, Vec3{0.0f, 0.0f, -1.0f}));
    FairingComponent gone = shell;
    gone.jettisoned = 1;
    SW_CHECK(!fairingEncloses(gone, Vec3{0.0f, 0.0f, -1.0f}));
}

SW_TEST(AShellWeighsWhatItsSurfaceIsMadeOf)
{
    // A cylinder's lateral area is 2 pi r h, a cone's is pi r l — the two
    // cases the sum of truncated cones has to reduce to.
    FairingComponent drum{};
    drum.rings[0] = {0.0f, 1.0f};
    drum.rings[1] = {2.0f, 1.0f};
    drum.ringCount = 2;
    drum.closed = 1;
    SW_CHECK(std::abs(fairingAreaM2(drum) - 2.0 * kPi * 1.0 * 2.0) < 1.0e-6);

    const FairingComponent nose = cone(1.0f, 1.0f);
    SW_CHECK(std::abs(fairingAreaM2(nose) - kPi * 1.0 * std::sqrt(2.0)) < 1.0e-6);

    // ...and the mass is that area in aluminium. A three-metre shroud over a
    // two-metre payload lands where a real one does: a hundred-odd kilograms.
    const FairingComponent real = shroud(1.25f, 3.0f, 2.0f);
    SW_CHECK(std::abs(fairingMassKg(real) -
                      fairingAreaM2(real) * kFairingMassPerM2) < 1.0e-9);
    SW_CHECK(fairingMassKg(real) > 300.0);
    SW_CHECK(fairingMassKg(real) < 700.0);
    // A draft has no surface, so it has no mass to quote while it is drawn.
    FairingComponent draft{};
    draft.rings[0] = {0.0f, 1.25f};
    draft.ringCount = 1;
    SW_CHECK(fairingMassKg(draft) == 0.0);
}

// ---------------------------------------------------------------------------
// THE DRAG OF A STACK OF FLAT PLATES
//
// Newtonian impact theory over a cone of half-angle a, with the same three
// terms the offline forge integrates per pixel:
//
//   Cd = sin^2(a)                        <- impact, referred to pi r^2
//      + Cf cos(a) / sin(a)              <- skin friction along the slant
//
// which is 1 for a flat plate, 1/2 for a 45-degree cone and a tenth for a
// three-to-one one. Nothing in Fairing.hpp knows any of that: it adds up
// panels. If the two agree, the panels are the right shape and wound the
// right way round.
// ---------------------------------------------------------------------------

namespace
{
    /// The shell's force per unit q, along the flow, for air arriving nose-on.
    /// Flow is the direction the AIR travels in the part's frame, so +Z is a
    /// vessel flying -Z first: nose into the wind.
    [[nodiscard]] f64 axialDragCoefficient(const FairingComponent& fairing, f32 radius)
    {
        Vec3 force{0.0f};
        Vec3 moment{0.0f};
        fairingAero(fairing, Vec3{0.0f, 0.0f, 1.0f}, force, moment);
        return static_cast<f64>(force.z) / (kPi * static_cast<f64>(radius * radius));
    }

    [[nodiscard]] f64 coneDragCoefficient(f64 halfAngleRadians)
    {
        const f64 s = std::sin(halfAngleRadians);
        const f64 c = std::cos(halfAngleRadians);
        return kFairingStagnationCp * s * s + kFairingSkinCf * c / s;
    }
} // namespace

SW_TEST(AFairingsDragIsTheTextbookConeItIsShapedLike)
{
    // The lathe cuts a POLYGON, so both the wetted area and the panel tilt
    // fall a little short of the smooth cone this compares against: with 24
    // sides that is under two per cent, and five is a comfortable roof that
    // still catches a wrong constant or a flipped normal.
    struct Case
    {
        f32 radius;
        f32 height;
    };
    const Case cases[] = {{1.0f, 0.02f}, {1.0f, 1.0f}, {1.0f, 3.0f}, {1.6f, 4.0f}};
    for (const Case& shape : cases)
    {
        const f64 halfAngle = std::atan2(static_cast<f64>(shape.radius),
                                         static_cast<f64>(shape.height));
        const f64 expected = coneDragCoefficient(halfAngle);
        const f64 measured = axialDragCoefficient(cone(shape.radius, shape.height),
                                                  shape.radius);
        SW_CHECK(std::abs(measured - expected) < expected * 0.05);
    }

    // AND THE ORDERING IS THE WHOLE POINT OF DRAWING ONE: the flat-ish cap is
    // a plate, the long one is a needle.
    SW_CHECK(axialDragCoefficient(cone(1.0f, 0.02f), 1.0f) > 0.9);
    SW_CHECK(axialDragCoefficient(cone(1.0f, 4.0f), 1.0f) < 0.12);
    SW_CHECK(axialDragCoefficient(cone(1.0f, 0.02f), 1.0f) >
             axialDragCoefficient(cone(1.0f, 1.0f), 1.0f));
    SW_CHECK(axialDragCoefficient(cone(1.0f, 1.0f), 1.0f) >
             axialDragCoefficient(cone(1.0f, 4.0f), 1.0f));
}

SW_TEST(ABarrelInAxialFlowIsAllSkinAndNoPressure)
{
    // A cylinder's panels are edge-on to the wind: every one of them is in
    // base pressure, those cancel round the axis, and what is left is the
    // friction of the air sliding along the wall.
    FairingComponent drum{};
    drum.rings[0] = {0.0f, 1.0f};
    drum.rings[1] = {3.0f, 1.0f};
    drum.ringCount = 2;
    drum.sides = 24;
    drum.closed = 1;

    Vec3 force{0.0f};
    Vec3 moment{0.0f};
    fairingAero(drum, Vec3{0.0f, 0.0f, 1.0f}, force, moment);
    const f64 area = fairingAreaM2(drum);
    SW_CHECK(std::abs(static_cast<f64>(force.z) - kFairingSkinCf * area) <
             kFairingSkinCf * area * 0.02);
    // Axisymmetric shape, axial flow: no side force and no moment. A sign
    // error on one panel's normal shows up here and nowhere else.
    SW_CHECK(std::abs(force.x) < 1.0e-4f);
    SW_CHECK(std::abs(force.y) < 1.0e-4f);
    SW_CHECK(glm::length(moment) < 1.0e-3f);

    // Side-on, the same drum is a wall: the force turns with the wind and is
    // an order of magnitude bigger than the friction it made head-on.
    fairingAero(drum, Vec3{1.0f, 0.0f, 0.0f}, force, moment);
    SW_CHECK(force.x > 0.0f);
    SW_CHECK(static_cast<f64>(force.x) > kFairingSkinCf * area * 10.0);
    SW_CHECK(std::abs(force.z) < force.x * 0.05f);
}

SW_TEST(ADraftAndAJettisonedShellHaveNoDragAtAll)
{
    Vec3 force{1.0f};
    Vec3 moment{1.0f};

    FairingComponent open = cone(1.0f, 2.0f);
    open.closed = 0;
    fairingAero(open, Vec3{0.0f, 0.0f, 1.0f}, force, moment);
    SW_CHECK(glm::length(force) == 0.0f);
    SW_CHECK(glm::length(moment) == 0.0f);

    FairingComponent gone = cone(1.0f, 2.0f);
    gone.jettisoned = 1;
    fairingAero(gone, Vec3{0.0f, 0.0f, 1.0f}, force, moment);
    SW_CHECK(glm::length(force) == 0.0f);

    // Still air is not a direction, and must not become a NaN one.
    fairingAero(cone(1.0f, 2.0f), Vec3{0.0f}, force, moment);
    SW_CHECK(glm::length(force) == 0.0f);
    SW_CHECK(std::isfinite(force.x) && std::isfinite(moment.z));
}

// ---------------------------------------------------------------------------
// THE MESH
// ---------------------------------------------------------------------------

SW_TEST(TheLathedShellIsTheSurfaceItsAreaSaysItIs)
{
    const FairingComponent shell = shroud(1.25f, 3.0f, 2.0f, 12);
    const MeshData mesh = buildFairingMesh(shell, Vec4{1.0f});
    SW_CHECK(!mesh.vertices.empty());
    SW_CHECK(mesh.indices.size() % 3 == 0);

    // Sum the triangles. Two bands of twelve panels is a polygon inscribed in
    // the profile, so it comes in just under the smooth area — never over.
    f64 area = 0.0;
    for (usize i = 0; i + 2 < mesh.indices.size(); i += 3)
    {
        const Vec3 a = mesh.vertices[mesh.indices[i]].position;
        const Vec3 b = mesh.vertices[mesh.indices[i + 1]].position;
        const Vec3 c = mesh.vertices[mesh.indices[i + 2]].position;
        area += 0.5 * static_cast<f64>(glm::length(glm::cross(b - a, c - a)));
    }
    const f64 smooth = fairingAreaM2(shell);
    SW_CHECK(area < smooth * 1.001);
    SW_CHECK(area > smooth * 0.94);

    // EVERY NORMAL POINTS OUT. Inside-out panels are invisible from outside
    // and look like a hole in the rocket.
    for (const Vertex& vertex : mesh.vertices)
    {
        const Vec2 radial{vertex.position.x, vertex.position.y};
        if (glm::length(radial) < 1.0e-4f)
        {
            continue; // the tip, where "outward" has no meaning
        }
        SW_CHECK(glm::dot(Vec2{vertex.normal.x, vertex.normal.y}, radial) > 0.0f);
    }

    // ...and the panels the jettison throws away are that same surface, cut
    // into as many pieces as the shell has sides.
    f64 panelArea = 0.0;
    for (u32 side = 0; side < shell.sides; ++side)
    {
        const MeshData panel = buildFairingPanelMesh(shell, side, Vec4{1.0f});
        SW_CHECK(!panel.indices.empty());
        for (usize i = 0; i + 2 < panel.indices.size(); i += 3)
        {
            const Vec3 a = panel.vertices[panel.indices[i]].position;
            const Vec3 b = panel.vertices[panel.indices[i + 1]].position;
            const Vec3 c = panel.vertices[panel.indices[i + 2]].position;
            panelArea += 0.5 * static_cast<f64>(glm::length(glm::cross(b - a, c - a)));
        }
    }
    SW_CHECK(std::abs(panelArea - area) < area * 1.0e-3);

    // A draft has no surface yet: drawing one is not a crash and not a mesh.
    FairingComponent draft{};
    draft.rings[0] = {0.0f, 1.25f};
    draft.ringCount = 1;
    SW_CHECK(buildFairingMesh(draft, Vec4{1.0f}).indices.empty());
}

// ---------------------------------------------------------------------------
// WHAT IT IS FOR: the payload feels no wind
// ---------------------------------------------------------------------------

SW_TEST(APayloadInsideAShroudFeelsNoWindAtAll)
{
    const std::filesystem::path assets =
        FileSystem::executableDirectory() / "Assets" / "Parts";
    SW_CHECK(parts::loadCatalog(assets));
    SW_CHECK(aero::loadTables(assets) >= 9);

    constexpr f64 kTerraRadius = 6.371e6;

    // The same rocket four ways: a fairing base carrying a solar wing, with
    // the shell closed or still a draft, and with the wing there or not. The
    // wing is the part to put inside — it is the flimsiest thing in the
    // catalogue and the one a real shroud is bought for.
    //
    // The measurement is a DIFFERENCE rather than a comparison between two
    // shapes: adding the payload under a closed shell must change the drag by
    // nothing at all, and adding it under an open one must change it.
    const auto dragWithShell = [&](bool closed, bool withPayload) {
        ecs::World world;
        parts::VesselAssemblySystem assembly;
        aero::VesselAerodynamicsSystem aerodynamics;

        const ecs::Entity terra = world.createEntity();
        world.addComponent(terra, TransformComponent{});
        phys::GravitySourceComponent gravity{};
        gravity.mu = 3.986004418e14;
        gravity.bodyRadius = kTerraRadius;
        world.addComponent(terra, gravity);
        world.addComponent(terra, phys::AtmosphereComponent{});

        const ecs::Entity root = world.createEntity();
        world.addComponent(root, TransformComponent{});
        world.addComponent(root, parts::VesselComponent{});
        world.addComponent(root, phys::DynamicBodyComponent{});
        world.addComponent(root, aero::AeroStateComponent{});

        const ecs::Entity base = world.createEntity();
        parts::PartComponent basePart{};
        basePart.definitionId = parts::kPartFairingBase;
        basePart.vessel = root;
        basePart.localPosition = Vec3{0.0f};
        world.addComponent(base, basePart);
        world.addComponent(base, TransformComponent{});
        FairingComponent shell = shroud(1.25f, 2.0f, 1.5f);
        shell.closed = closed ? 1u : 0u;
        world.addComponent(base, shell);

        if (withPayload)
        {
            const ecs::Entity payload = world.createEntity();
            parts::PartComponent wing{};
            wing.definitionId = parts::kPartSolarWing;
            wing.vessel = root;
            wing.localPosition = Vec3{0.0f, 0.0f, -1.5f}; // inside the barrel
            world.addComponent(payload, wing);
            world.addComponent(payload, TransformComponent{});
        }

        TransformComponent& transform = world.getComponent<TransformComponent>(root);
        phys::DynamicBodyComponent& body =
            world.getComponent<phys::DynamicBodyComponent>(root);
        const WorldVec3 up{0.0, 1.0, 0.0};
        transform.position = up * (kTerraRadius + 6000.0);
        body.velocity = up * 260.0;
        transform.rotation = glm::rotation(Vec3(0, 0, -1), Vec3(up));

        assembly.update(world, 0.02f);
        aerodynamics.update(world, 0.02f);
        return world.getComponent<aero::AeroStateComponent>(root).dragN;
    };

    const f64 sealedWith = dragWithShell(true, true);
    const f64 sealedWithout = dragWithShell(true, false);
    const f64 openWith = dragWithShell(false, true);
    const f64 openWithout = dragWithShell(false, false);

    SW_CHECK(sealedWith > 0.0); // the shell itself is in the wind
    SW_CHECK(openWith > 0.0);
    // NOT A REDUCED EXPOSURE — NOTHING. A part in another part's wake still
    // feels base pressure, and `exposure()` deliberately never returns zero
    // for it; a part inside a sealed shroud is a different fact.
    SW_CHECK(std::abs(sealedWith - sealedWithout) < sealedWithout * 1.0e-9);
    // ...and the shielding is the SHELL's doing, not the payload being
    // aerodynamically negligible: with the profile still an open draft, the
    // same wing in the same place changes the answer.
    SW_CHECK(openWith - openWithout > 1000.0);
}

// ---------------------------------------------------------------------------
// A DRAWN SHAPE IS PART OF THE DESIGN
// ---------------------------------------------------------------------------

SW_TEST(AShellSurvivesTheDesignFileItWasDrawnOn)
{
    ShipBlueprint design{};
    design.name = "SHROUDED";
    BlueprintPartRecord base{};
    base.definitionId = kPartFairingBase;
    base.fairing = shroud(1.25f, 2.4f, 1.8f, 10);
    design.parts.push_back(base);
    BlueprintPartRecord plain{};
    plain.definitionId = kPartCoreStructural;
    plain.localPosition = Vec3{0.0f, 0.0f, 0.9f};
    plain.parentIndex = 0;
    design.parts.push_back(plain);

    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "sw_test_fairing.swship";
    SW_CHECK(saveBlueprintFile(design, path));

    ShipBlueprint loaded{};
    SW_CHECK(loadBlueprintFile(path, loaded));
    SW_CHECK(loaded.parts.size() == 2);
    if (loaded.parts.size() != 2) { return; }
    const FairingComponent& back = loaded.parts[0].fairing;
    SW_CHECK(back.ringCount == base.fairing.ringCount);
    SW_CHECK(back.sides == base.fairing.sides);
    SW_CHECK(back.closed == base.fairing.closed);
    for (u32 i = 0; i < back.ringCount; ++i)
    {
        SW_CHECK(glm::length(back.rings[i] - base.fairing.rings[i]) < 1.0e-5f);
    }
    // The shape is the mass and the drag, so a profile that came back rounded
    // would quietly reprice the rocket.
    SW_CHECK(std::abs(fairingMassKg(back) - fairingMassKg(base.fairing)) < 1.0e-6);
    // ...and a part that is not a fairing carries no profile through the file.
    SW_CHECK(loaded.parts[1].fairing.ringCount == 0);

    std::error_code error{};
    std::filesystem::remove(path, error);
}
