// ============================================================================
// SaveTests.cpp — binary layer, world snapshot round trip, post-load
// determinism of the factory simulation, and surface anchoring.
// ============================================================================

#include "TestFramework.hpp"

#include <ECS/World.hpp>
#include <Factory/FactorySystems.hpp>
#include <Physics/PhysicsSystems.hpp>
#include <Save/Snapshot.hpp>
#include <Scene/TransformComponents.hpp>
#include <Simulation/Simulation.hpp>

#include <cmath>

using namespace sw;
using sw::res::Resource;

namespace
{
    struct RefHolder
    {
        ecs::Entity target{};
        int value = 0;
    };

    save::Schema makeTestSchema()
    {
        save::Schema schema;
        schema.registerComponent<TransformComponent>("sw.Transform", 1);
        schema.registerComponent<PreviousTransformComponent>("sw.PreviousTransform", 1);
        schema.registerComponent<phys::SurfaceAnchorComponent>("phys.SurfaceAnchor", 1);
        schema.registerComponent<phys::GravitySourceComponent>("phys.GravitySource", 1);
        schema.registerComponent<factory::InventoryComponent>("factory.Inventory", 1);
        schema.registerComponent<factory::MinerComponent>("factory.Miner", 1);
        schema.registerComponent<factory::RefineryComponent>("factory.Refinery", 1);
        schema.registerComponent<factory::ItemLinkComponent>("factory.ItemLink", 1);
        schema.registerComponent<RefHolder>("test.RefHolder", 1);
        return schema;
    }
} // namespace

SW_TEST(BinaryWriterReaderRoundTripAndBounds)
{
    ser::BinaryWriter writer;
    writer.write<u32>(0xC0FFEE01);
    writer.write<f64>(3.14159);
    writer.writeString("StarWorks");
    writer.write<ecs::Entity>({42, 7});

    ser::BinaryReader reader(writer.bytes());
    SW_CHECK_EQ(reader.read<u32>(), 0xC0FFEE01u);
    SW_CHECK_EQ(reader.read<f64>(), 3.14159);
    SW_CHECK(reader.readString() == "StarWorks");
    const auto entity = reader.read<ecs::Entity>();
    SW_CHECK(entity.index == 42 && entity.generation == 7);
    SW_CHECK_EQ(reader.remaining(), 0u);

    // Overrun must throw, not read garbage.
    bool threw = false;
    try
    {
        (void)reader.read<u32>();
    }
    catch (const Exception&)
    {
        threw = true;
    }
    SW_CHECK(threw);
}

SW_TEST(WorldSnapshotPreservesEntitiesGenerationsAndReferences)
{
    const save::Schema schema = makeTestSchema();

    ecs::World world;
    // Force generation churn so the snapshot must preserve generations.
    const ecs::Entity dead1 = world.createEntity();
    const ecs::Entity dead2 = world.createEntity();
    world.destroyEntity(dead1);
    world.destroyEntity(dead2);

    const ecs::Entity target = world.createEntity(); // recycled index, gen>0
    {
        TransformComponent transform{};
        transform.position = {1.0e6, -2.0e6, 3.0e6};
        world.addComponent(target, transform);
    }
    const ecs::Entity holder = world.createEntity();
    world.addComponent(holder, RefHolder{target, 1234});

    // ---- save -> fresh world -> verify ------------------------------------
    ser::BinaryWriter writer;
    save::saveWorld(world, schema, writer);

    ecs::World loaded;
    ser::BinaryReader reader(writer.bytes());
    save::loadWorld(loaded, schema, reader);

    SW_CHECK_EQ(loaded.aliveCount(), 2u);
    SW_CHECK(loaded.isAlive(target)); // same index AND generation
    SW_CHECK(loaded.isAlive(holder));
    SW_CHECK(!loaded.isAlive(dead1)); // stale handles stay stale

    const auto& restoredRef = loaded.getComponent<RefHolder>(holder);
    SW_CHECK(restoredRef.value == 1234);
    SW_CHECK(restoredRef.target == target);
    // The reference must actually resolve in the loaded world.
    SW_CHECK(loaded.isAlive(restoredRef.target));
    SW_CHECK(std::abs(loaded.getComponent<TransformComponent>(restoredRef.target).position.z -
                      3.0e6) < 1.0e-9);

    // New entities after load must not collide with restored ones.
    const ecs::Entity fresh = loaded.createEntity();
    SW_CHECK(loaded.isAlive(fresh));
    SW_CHECK(loaded.isAlive(target));
    SW_CHECK(loaded.aliveCount() == 3u);
}

SW_TEST(FactorySimulationIsDeterministicAcrossSaveLoad)
{
    const save::Schema schema = makeTestSchema();

    // Build a mining->refining->hauling chain, run it, save it MID-RUN.
    auto buildChain = [](ecs::World& world, ecs::Entity& outDepot) {
        const ecs::Entity rock = world.createEntity();
        factory::InventoryComponent hopper{};
        hopper.volumeCapacityM3 = 10.0;
        world.addComponent(rock, hopper);
        world.addComponent(rock, factory::MinerComponent{Resource::IronOre, 2.0, 0.0});

        const ecs::Entity refinery = world.createEntity();
        factory::InventoryComponent tanks{};
        tanks.volumeCapacityM3 = 10.0;
        world.addComponent(refinery, tanks);
        world.addComponent(refinery, factory::RefineryComponent{
                                         Resource::IronOre, Resource::Iron, 1.0, 0.9, 0.0});
        world.addComponent(refinery,
                           factory::ItemLinkComponent{rock, Resource::IronOre, 1.5});

        const ecs::Entity depot = world.createEntity();
        factory::InventoryComponent silo{};
        silo.volumeCapacityM3 = 10.0;
        world.addComponent(depot, silo);
        world.addComponent(depot, factory::ItemLinkComponent{refinery, Resource::Iron, 1.0});
        outDepot = depot;
    };
    auto makeSim = [] {
        auto simulation = std::make_unique<sim::Simulation>(
            std::vector<sim::LaneConfig>{{"Logistics", 10.0f, 8}, {"Automation", 5.0f, 8}});
        simulation->findLane("Automation")
            ->scheduler()
            .addSystem(std::make_unique<factory::MinerSystem>());
        simulation->findLane("Automation")
            ->scheduler()
            .addSystem(std::make_unique<factory::RefinerySystem>());
        simulation->findLane("Logistics")
            ->scheduler()
            .addSystem(std::make_unique<factory::TransferSystem>());
        return simulation;
    };

    ecs::World original;
    ecs::Entity depot{};
    buildChain(original, depot);
    auto simulationA = makeSim();

    for (int i = 0; i < 100; ++i) // 12.5 s, mid-production
    {
        simulationA->advance(original, 0.125f, nullptr);
    }

    // Snapshot world + simulation clocks.
    ser::BinaryWriter writer;
    save::saveWorld(original, schema, writer);
    save::saveSimulation(*simulationA, writer);

    ecs::World restored;
    auto simulationB = makeSim();
    ser::BinaryReader reader(writer.bytes());
    save::loadWorld(restored, schema, reader);
    save::loadSimulation(*simulationB, reader);

    // Run BOTH for the same further time with identical slicing.
    for (int i = 0; i < 100; ++i)
    {
        simulationA->advance(original, 0.125f, nullptr);
        simulationB->advance(restored, 0.125f, nullptr);
    }

    // Bitwise-equal factory outcome: same iron in the depot, same totals.
    const f64 ironA = factory::inventoryCount(
        original.getComponent<factory::InventoryComponent>(depot), Resource::Iron);
    const f64 ironB = factory::inventoryCount(
        restored.getComponent<factory::InventoryComponent>(depot), Resource::Iron);
    SW_CHECK(ironA > 5.0);   // production really happened
    SW_CHECK(ironA == ironB); // EXACT determinism, not approximate
}

SW_TEST(SurfaceAnchorCoRotatesAndSurvivesSnapshot)
{
    const save::Schema schema = makeTestSchema();

    // A planet whose spin is held in f64 on its GravitySource, the way
    // CelestialSpinSystem stamps it. The f32 quaternion on the transform is
    // kept in sync — it still orients the mesh — but it is deliberately NOT
    // what positions a base a planet radius from the axis.
    ecs::World world;
    const ecs::Entity planet = world.createEntity();
    {
        world.addComponent(planet, TransformComponent{});
        phys::GravitySourceComponent source{1.0e14, 1.0e6};
        source.spinAxis = WorldVec3{0.0, 1.0, 0.0};
        world.addComponent(planet, source);
    }
    const ecs::Entity base = world.createEntity();
    {
        world.addComponent(base, TransformComponent{});
        world.addComponent(base, PreviousTransformComponent{});
        world.addComponent(base, phys::SurfaceAnchorComponent{planet, {0.0, 0.0, 1.0e6}});
    }

    phys::SurfaceAnchorSystem anchorSystem;
    auto spinPlanet = [&](ecs::World& w, f64 angle) {
        auto& source = w.getComponent<phys::GravitySourceComponent>(planet);
        source.spinAnglePrevious = source.spinAngle;
        source.spinAngle = angle;
        w.getComponent<TransformComponent>(planet).rotation =
            glm::angleAxis(static_cast<f32>(angle), Vec3{0.0f, 1.0f, 0.0f});
    };

    // A quarter turn about +Y: the base follows to the rotated surface
    // point — and lands within a MILLIMETRE of the exact answer. The old
    // f32-quaternion path could only promise half a metre here, which is
    // precisely why everything anchored to the ground used to shimmer.
    spinPlanet(world, math::kHalfPi);
    anchorSystem.update(world, 0.02f);

    const WorldVec3 expected =
        glm::angleAxis(static_cast<f64>(math::kHalfPi), glm::dvec3{0.0, 1.0, 0.0}) *
        WorldVec3{0.0, 0.0, 1.0e6};
    const WorldVec3 actual = world.getComponent<TransformComponent>(base).position;
    SW_CHECK(glm::length(actual - expected) < 1.0e-3);

    // THE JITTER ITSELF. Turn the planet at Terra's real rate, one 50 Hz
    // tick at a time, and watch how far the base wanders off the exact
    // circle from frame to frame. With an f32 rotation this excursion
    // reached 0.77 m — a visible shimmer on every building at the site.
    constexpr f64 kRate = 7.2921e-5;
    f64 worstStep = 0.0;
    f64 previousError = 0.0;
    for (i32 tick = 0; tick < 200; ++tick)
    {
        const f64 angle = std::fmod(kRate * (1.0e6 + tick * 0.02), 6.283185307179586);
        spinPlanet(world, angle);
        anchorSystem.update(world, 0.02f);
        const WorldVec3 exact = glm::angleAxis(angle, glm::dvec3{0.0, 1.0, 0.0}) *
                                WorldVec3{0.0, 0.0, 1.0e6};
        const f64 error = glm::length(
            world.getComponent<TransformComponent>(base).position - exact);
        if (tick > 0)
        {
            worstStep = std::max(worstStep, std::abs(error - previousError));
        }
        previousError = error;
    }
    SW_CHECK(worstStep < 1.0e-6); // micrometres, not decimetres

    // Snapshot and make sure the ANCHOR (not the world position) is what
    // persists: rotate further after load, the base keeps following.
    ser::BinaryWriter writer;
    save::saveWorld(world, schema, writer);
    ecs::World loaded;
    ser::BinaryReader reader(writer.bytes());
    save::loadWorld(loaded, schema, reader);

    spinPlanet(loaded, static_cast<f64>(math::kPi));
    anchorSystem.update(loaded, 0.02f);

    const WorldVec3 expectedAfter =
        glm::angleAxis(static_cast<f64>(math::kPi), glm::dvec3{0.0, 1.0, 0.0}) *
        WorldVec3{0.0, 0.0, 1.0e6};
    SW_CHECK(glm::length(loaded.getComponent<TransformComponent>(base).position -
                         expectedAfter) < 1.0e-3);
}

// ----------------------------------------------------------------------------
// A component in the world but not in the schema
// ----------------------------------------------------------------------------

SW_TEST(AnUnregisteredComponentIsNamedBeforeItCanBreakASave)
{
    // THE BUG THIS EXISTS FOR. AssemblyComponent and VehicleQueueComponent
    // were added to every assembly hall — so to the starting outpost, on the
    // first frame of a new game — and neither was registered in the game's
    // save schema. saveWorld throws on the first column it cannot name and
    // writes nothing, so NO save could be made from the very first minute,
    // and the only evidence was one "Save failed" line in the log.
    //
    // The cure is not a bigger try/catch, it is asking the question EARLY.
    ecs::World world;
    save::Schema schema = makeTestSchema();

    // A clean world first: the check must be silent when there is nothing
    // wrong, or nobody will believe it when there is.
    const ecs::Entity known = world.createEntity();
    world.addComponent(known, TransformComponent{});
    world.addComponent(known, factory::InventoryComponent{});
    SW_CHECK(save::unsaveableComponents(world, schema).empty());
    {
        ser::BinaryWriter writer;
        save::saveWorld(world, schema, writer);
        SW_CHECK(writer.size() > 0);
    }

    // Now the situation the game was in. VehicleQueueComponent stands in for
    // "a real component someone forgot"; nothing about the test depends on
    // which one it is.
    const ecs::Entity hall = world.createEntity();
    world.addComponent(hall, TransformComponent{});
    world.addComponent(hall, factory::VehicleQueueComponent{});

    const std::vector<std::string> missing = save::unsaveableComponents(world, schema);
    // EXACTLY ONE, and it must not sweep up the components that ARE
    // registered on the same archetype — a report that names everything
    // names nothing.
    SW_CHECK_EQ(missing.size(), usize{1});
    if (!missing.empty())
    {
        SW_CHECK(missing[0].find(std::to_string(sizeof(factory::VehicleQueueComponent))) !=
                 std::string::npos);
    }

    // ...and it is the same condition that stops the save, not a separate
    // opinion about it. If these two ever disagree the early check is
    // worthless.
    bool threw = false;
    try
    {
        ser::BinaryWriter writer;
        save::saveWorld(world, schema, writer);
    }
    catch (const Exception&)
    {
        threw = true;
    }
    SW_CHECK(threw);

    // Registering it makes both agree again.
    schema.registerComponent<factory::VehicleQueueComponent>("factory.VehicleQueue", 1);
    SW_CHECK(save::unsaveableComponents(world, schema).empty());
    {
        ser::BinaryWriter writer;
        save::saveWorld(world, schema, writer);
        SW_CHECK(writer.size() > 0);
    }

    // An EMPTY archetype is not a problem: saveWorld skips it, so reporting
    // it would be a false alarm, and a false alarm teaches people to ignore
    // the real one. Destroying the only holder leaves exactly that shape.
    const ecs::Entity doomed = world.createEntity();
    world.addComponent(doomed, factory::AssemblyComponent{});
    SW_CHECK_EQ(save::unsaveableComponents(world, schema).size(), usize{1});
    world.destroyEntity(doomed);
    SW_CHECK(save::unsaveableComponents(world, schema).empty());
}
