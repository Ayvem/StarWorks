// ============================================================================
// NetProbe — what multiplayer costs, measured rather than guessed.
//
// Builds a world with the REAL component types the game replicates, stands a
// Host and a Client on real UDP sockets, and runs them against each other in
// one process. Everything it prints is a measurement:
//
//   * the size of the world transfer a joining player is sent,
//   * the size of a steady-state delta and therefore the bandwidth per
//     client,
//   * what the encoder costs the host per snapshot, at several world sizes,
//   * how long a join takes end to end.
//
// Two sockets on the loopback interface is not the internet. What it does
// measure honestly is the CPU cost and the BYTE COUNT, which are the two
// things a game can get wrong on its own; latency and loss are measured
// against the simulated wire in the test suite instead, where they can be
// specified exactly.
// ============================================================================

#include <Core/Log.hpp>
#include <ECS/World.hpp>
#include <Network/Session.hpp>
#include <Network/Transport.hpp>
#include <Physics/PhysicsComponents.hpp>
#include <Save/Snapshot.hpp>
#include <Scene/TransformComponents.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

using namespace sw;

namespace
{
    /// The set a session actually replicates: where things are, how they are
    /// moving, and what they are. Not the derived state — a client can
    /// recompute a conveyor's cargo positions from the link's throughput,
    /// and sending them would be paying twice for the same fact.
    save::Schema probeSchema()
    {
        save::Schema schema;
        schema.registerComponent<TransformComponent>("sw.Transform", 1);
        schema.registerComponent<PreviousTransformComponent>("sw.PreviousTransform", 1);
        schema.registerComponent<phys::DynamicBodyComponent>("phys.DynamicBody", 2);
        schema.registerComponent<phys::OnRailsComponent>("phys.OnRails", 2);
        return schema;
    }

    net::ReplicationSet probeSet()
    {
        net::ReplicationSet set;
        set.include("sw.Transform").include("phys.DynamicBody").include("phys.OnRails");
        return set;
    }

    struct Scene
    {
        std::vector<ecs::Entity> moving;   // integrated, changes every tick
        std::vector<ecs::Entity> onRails;  // analytic, almost never changes
        std::vector<ecs::Entity> still;    // buildings and scenery
    };

    /// A world shaped like a session: a handful of craft under thrust, some
    /// debris and stations on rails, and a base that does not move.
    Scene buildScene(ecs::World& world, u32 movingCount, u32 railsCount, u32 stillCount)
    {
        Scene scene;
        for (u32 i = 0; i < movingCount; ++i)
        {
            const ecs::Entity entity = world.createEntity();
            world.addComponent(entity, TransformComponent{});
            world.addComponent(entity, phys::DynamicBodyComponent{});
            scene.moving.push_back(entity);
        }
        for (u32 i = 0; i < railsCount; ++i)
        {
            const ecs::Entity entity = world.createEntity();
            world.addComponent(entity, TransformComponent{});
            world.addComponent(entity, phys::OnRailsComponent{});
            scene.onRails.push_back(entity);
        }
        for (u32 i = 0; i < stillCount; ++i)
        {
            const ecs::Entity entity = world.createEntity();
            world.addComponent(entity, TransformComponent{});
            scene.still.push_back(entity);
        }
        return scene;
    }

    /// Advances only what would really be moving.
    void stepScene(ecs::World& world, const Scene& scene, f64 t)
    {
        for (usize i = 0; i < scene.moving.size(); ++i)
        {
            TransformComponent& transform = world.getComponent<TransformComponent>(
                scene.moving[i]);
            const f64 phase = t + static_cast<f64>(i);
            transform.position =
                WorldVec3{6.4e6 * std::cos(phase * 1.0e-4), 1000.0 * std::sin(phase),
                          6.4e6 * std::sin(phase * 1.0e-4)};
            transform.rotation =
                Quat(std::cos(static_cast<f32>(phase) * 0.5f), 0.0f,
                     std::sin(static_cast<f32>(phase) * 0.5f), 0.0f);
            world.getComponent<phys::DynamicBodyComponent>(scene.moving[i]).velocity =
                WorldVec3{0.0, 1000.0 * std::cos(phase), 0.0};
        }
    }

    f64 nowSeconds()
    {
        using Clock = std::chrono::steady_clock;
        static const Clock::time_point origin = Clock::now();
        return std::chrono::duration<f64>(Clock::now() - origin).count();
    }

    // ------------------------------------------------------------------------
    // 1. Encoder cost and delta size, with no sockets in the way.
    // ------------------------------------------------------------------------
    void measureEncoder(const save::Schema& schema)
    {
        std::printf("\n1. ENCODER — capture + diff + write, per snapshot\n");
        std::printf("   %-10s %-9s %10s %10s %12s %12s\n", "entities", "moving", "full B",
                    "delta B", "capture us", "encode us");

        const struct
        {
            u32 moving;
            u32 rails;
            u32 still;
        } cases[] = {{5, 20, 75}, {20, 100, 380}, {60, 400, 1540}, {200, 1800, 8000}};

        for (const auto& shape : cases)
        {
            ecs::World world;
            const Scene scene = buildScene(world, shape.moving, shape.rails, shape.still);
            const u32 total = shape.moving + shape.rails + shape.still;

            const net::ReplicationTable table =
                net::ReplicationTable::build(schema, probeSet());
            net::ReplicationEncoder encoder(table);
            net::ReplicationDecoder decoder(table);
            ecs::World mirror;

            stepScene(world, scene, 0.0);
            ser::BinaryWriter full;
            const u32 firstId = encoder.encode(world, 0.0, full);
            {
                ser::BinaryReader reader(full.bytes());
                if (!decoder.apply(mirror, reader))
                {
                    std::printf("   full snapshot refused — probe is broken\n");
                    return;
                }
            }
            encoder.acknowledge(firstId);
            const usize fullBytes = encoder.lastStats().payloadBytes;

            // Steady state: many deltas, acknowledged each time, as a client
            // on a healthy link would.
            constexpr u32 kRounds = 200;
            f64 captureTotal = 0.0;
            f64 encodeTotal = 0.0;
            usize deltaTotal = 0;
            net::WorldState scratch;
            for (u32 round = 1; round <= kRounds; ++round)
            {
                stepScene(world, scene, round * 0.05);

                const auto captureStart = std::chrono::steady_clock::now();
                net::captureState(world, table, scratch);
                captureTotal += std::chrono::duration<f64, std::micro>(
                                    std::chrono::steady_clock::now() - captureStart)
                                    .count();

                ser::BinaryWriter delta;
                const auto encodeStart = std::chrono::steady_clock::now();
                const u32 id = encoder.encode(world, round * 0.05, delta);
                encodeTotal += std::chrono::duration<f64, std::micro>(
                                   std::chrono::steady_clock::now() - encodeStart)
                                   .count();
                deltaTotal += encoder.lastStats().payloadBytes;

                ser::BinaryReader reader(delta.bytes());
                if (!decoder.apply(mirror, reader))
                {
                    std::printf("   delta %u refused — probe is broken\n", id);
                    return;
                }
                encoder.acknowledge(id);
            }

            std::printf("   %-10u %-9u %10zu %10zu %12.1f %12.1f\n", total, shape.moving,
                        fullBytes, deltaTotal / kRounds, captureTotal / kRounds,
                        encodeTotal / kRounds);
        }
    }

    // ------------------------------------------------------------------------
    // 2. A real join over real sockets.
    // ------------------------------------------------------------------------
    void measureSession(const save::Schema& schema, const char* label, u32 moving,
                        u32 rails, u32 still)
    {
        std::printf("\n   -- %s: %u craft, %u entities --\n", label, moving,
                    moving + rails + still);

        auto hostSocket = std::make_unique<net::UdpSocket>(0);
        const u16 hostPort = hostSocket->localPort();
        net::Host::Config hostConfig;
        hostConfig.snapshotHz = 20.0;
        net::Host host(std::move(hostSocket), schema, probeSet(), hostConfig);

        net::Client client(std::make_unique<net::UdpSocket>(0), schema, "probe");

        ecs::World world;
        const Scene scene = buildScene(world, moving, rails, still);
        stepScene(world, scene, 0.0);
        ecs::World mirror;

        const f64 start = nowSeconds();
        client.connect(net::PeerAddress::localhost(hostPort), start);

        f64 joinedAt = -1.0;
        f64 firstMirrorAt = -1.0;
        f64 simulated = 0.0;
        const f64 until = start + 5.0;
        u32 commands = 0;
        f64 lastCommandAt = start;
        for (f64 t = start; t < until; t = nowSeconds())
        {
            simulated = t - start;
            stepScene(world, scene, simulated);
            host.update(t, world, simulated);
            client.update(t, mirror, simulated);

            if (joinedAt < 0.0 && client.state() == net::ClientState::Connected)
            {
                joinedAt = t;
            }
            if (firstMirrorAt < 0.0 && client.appliedSnapshotId() > 0)
            {
                firstMirrorAt = t;
            }
            // Pilot input at the physics rate — fifty a second, the same
            // beat the host integrates on. Sending faster would only measure
            // this loop.
            if (client.state() == net::ClientState::Connected && t - lastCommandAt >= 0.02)
            {
                lastCommandAt = t;
                ser::BinaryWriter writer;
                writer.write(commands++);
                writer.write(0.6f); // throttle
                client.sendCommand(writer.bytes());
            }
            // A game would be doing something else here; a busy loop would
            // measure the loop rather than the network.
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }

        const f64 elapsed = nowSeconds() - start;
        const std::vector<net::Host::ClientView> views = host.clients();

        std::printf("   state                %s\n",
                    std::string(net::clientStateName(client.state())).c_str());
        std::printf("   handshake            %.2f ms\n", (joinedAt - start) * 1000.0);
        std::printf("   world in the mirror  %.2f ms after connect\n",
                    (firstMirrorAt - start) * 1000.0);
        std::printf("   mirrored entities    %u of %u\n", mirror.aliveCount(),
                    world.aliveCount());
        std::printf("   snapshots applied    %u\n", client.appliedSnapshotId());
        std::printf("   snapshots re-based   %u (lost or overtaken)\n",
                    client.rebasedCount());
        std::printf("   commands delivered   %u\n", commands);

        if (!views.empty())
        {
            const net::Host::ClientView& view = views.front();
            const f64 down = static_cast<f64>(view.connection.bytesSent) / elapsed;
            const f64 up = static_cast<f64>(view.connection.bytesReceived) / elapsed;
            std::printf("   downstream           %.1f kB/s (%llu datagrams)\n", down / 1000.0,
                        static_cast<unsigned long long>(view.connection.datagramsSent));
            std::printf("   upstream             %.1f kB/s (%llu datagrams)\n", up / 1000.0,
                        static_cast<unsigned long long>(view.connection.datagramsReceived));
            std::printf("   resends              %llu\n",
                        static_cast<unsigned long long>(view.connection.resends));
            std::printf("   round trip           %.2f ms\n", view.roundTripSeconds * 1000.0);
            std::printf("   last delta           %zu B, %u changed of %u\n",
                        view.lastSnapshot.payloadBytes, view.lastSnapshot.changed,
                        view.lastSnapshot.recordCount);
        }

        // The point of the whole exercise: is the mirror actually right?
        u32 compared = 0;
        f64 worstError = 0.0;
        for (const ecs::Entity entity : scene.moving)
        {
            if (!mirror.isAlive(entity))
            {
                continue;
            }
            const WorldVec3 there = mirror.getComponent<TransformComponent>(entity).position;
            const WorldVec3 here = world.getComponent<TransformComponent>(entity).position;
            worstError = std::max(worstError, glm::length(here - there));
            ++compared;
        }
        std::printf("   compared             %u moving entities, worst gap %.1f m\n",
                    compared, worstError);
        std::printf("     (a gap is EXPECTED: the mirror holds the last snapshot, not\n"
                    "      the host's current instant — %.0f ms of motion at these speeds)\n",
                    1000.0 / 20.0);

        host.shutdown(nowSeconds());
    }
} // namespace

int main()
{
    Log::initialize(Log::Config{LogLevel::Warn, {}, true});

    const save::Schema schema = probeSchema();

    std::printf("NetProbe — StarWorks multiplayer, measured\n");
    std::printf("  datagram %zu B, header %zu B, reliable window %u\n", net::kMaxDatagramBytes,
                net::kHeaderBytes, net::kReliableWindow);

    measureEncoder(schema);

    std::printf("\n2. SESSION — two UDP sockets, loopback interface\n");
    measureSession(schema, "co-op", 5, 20, 75);
    measureSession(schema, "busy", 20, 100, 380);

    std::printf("\n");
    return 0;
}
