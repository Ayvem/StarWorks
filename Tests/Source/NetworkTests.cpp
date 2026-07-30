// ============================================================================
// NetworkTests.cpp — the multiplayer stack, from sequence arithmetic to a
// client that joins a running world across a link that loses one packet in
// five and delivers the rest out of order.
//
// Not one of these tests opens a socket. The transport is a simulated wire
// with a seeded generator and a clock that is passed in, so "80 ms of
// latency, 60 ms of jitter, 20 % loss, for four seconds" runs in
// microseconds and gives the same answer every time. A test that needs a
// real network to fail is a test that fails for reasons other than the code.
// ============================================================================

#include "TestFramework.hpp"

#include <ECS/World.hpp>
#include <Network/Protocol.hpp>
#include <Network/Replication.hpp>
#include <Network/Session.hpp>
#include <Network/Transport.hpp>
#include <Save/Snapshot.hpp>

#include <cmath>
#include <cstring>

using namespace sw;

namespace
{
    // Two components with entity handles inside them, because that is the
    // case the whole mirror design exists for.
    struct NetPosition
    {
        f64 x = 0.0;
        f64 y = 0.0;
        f64 z = 0.0;
    };

    struct NetLink
    {
        ecs::Entity target{};
        u32 tag = 0;
    };

    struct NetLocalOnly
    {
        u32 value = 0;
    };

    save::Schema makeNetSchema()
    {
        save::Schema schema;
        schema.registerComponent<NetPosition>("net.Position", 1);
        schema.registerComponent<NetLink>("net.Link", 1);
        schema.registerComponent<NetLocalOnly>("net.LocalOnly", 1);
        return schema;
    }

    net::ReplicationSet makeNetSet()
    {
        net::ReplicationSet set;
        set.include("net.Position").include("net.Link");
        return set;
    }

    /// Pumps both ends of a loopback wire for `seconds`, in 10 ms steps.
    template <typename HostFn, typename ClientFn>
    void run(net::LoopbackNetwork& wire, f64& now, f64 seconds, HostFn hostStep,
             ClientFn clientStep)
    {
        const f64 step = 0.01;
        const f64 until = now + seconds;
        while (now < until)
        {
            now += step;
            wire.setTime(now);
            hostStep(now);
            clientStep(now);
        }
    }
} // namespace

// ----------------------------------------------------------------------------
// Sequence arithmetic
// ----------------------------------------------------------------------------

SW_TEST(SequenceNumbersCompareCorrectlyAcrossTheWrap)
{
    SW_CHECK(net::sequenceNewer(1, 0));
    SW_CHECK(!net::sequenceNewer(0, 1));
    SW_CHECK(!net::sequenceNewer(5, 5));

    // The whole point: 0 follows 65535, and a plain `<` would call that a
    // jump of 65535 backwards.
    SW_CHECK(net::sequenceNewer(0, 65535));
    SW_CHECK(net::sequenceNewer(3, 65530));
    SW_CHECK(!net::sequenceNewer(65535, 0));

    SW_CHECK_EQ(net::sequenceDelta(0, 65535), 1);
    SW_CHECK_EQ(net::sequenceDelta(65535, 0), -1);
    SW_CHECK_EQ(net::sequenceDelta(100, 90), 10);
}

// ----------------------------------------------------------------------------
// Connection: reliability, ordering, fragmentation
// ----------------------------------------------------------------------------

namespace
{
    /// A pair of Connections wired to each other, with an explicit loss list
    /// so a test can say exactly which datagram dies.
    struct Pair
    {
        net::Connection a;
        net::Connection b;
        std::vector<std::vector<u8>> outbox;
        u32 sent = 0;
        std::vector<u32> dropIndices; // indices into the a->b stream

        void pumpAtoB(f64 now)
        {
            outbox.clear();
            a.collectOutgoing(now, outbox);
            for (const std::vector<u8>& datagram : outbox)
            {
                const bool drop = std::find(dropIndices.begin(), dropIndices.end(), sent) !=
                                  dropIndices.end();
                ++sent;
                if (!drop)
                {
                    b.receive(now, datagram);
                }
            }
        }

        void pumpBtoA(f64 now)
        {
            outbox.clear();
            b.collectOutgoing(now, outbox);
            for (const std::vector<u8>& datagram : outbox)
            {
                a.receive(now, datagram);
            }
        }
    };
} // namespace

SW_TEST(AReliableMessageSurvivesTheLossOfItsFirstAttempt)
{
    Pair pair;
    pair.dropIndices = {0};

    const std::vector<u8> payload{1, 2, 3, 4, 5};
    pair.a.queueReliable(net::MessageType::Command, payload);

    f64 now = 0.0;
    pair.pumpAtoB(now); // dropped
    SW_CHECK(pair.b.delivered().empty());

    // Nothing is resent before the timeout; that would be waste, not safety.
    now = 0.02;
    pair.pumpAtoB(now);
    SW_CHECK(pair.b.delivered().empty());

    now = 0.10;
    pair.pumpAtoB(now);
    SW_CHECK_EQ(pair.b.delivered().size(), 1u);
    SW_CHECK(pair.b.delivered()[0].payload == payload);
    SW_CHECK_EQ(pair.a.stats().resends, 1u);
}

SW_TEST(ReliableMessagesArriveInOrderEvenWhenTheWireDoesNot)
{
    Pair pair;
    pair.dropIndices = {0}; // the FIRST of three dies

    for (u8 i = 0; i < 3; ++i)
    {
        const std::vector<u8> payload{i};
        pair.a.queueReliable(net::MessageType::Command, payload);
    }

    f64 now = 0.0;
    pair.pumpAtoB(now);
    // Two arrived, but the one they follow did not: nothing may be handed up.
    SW_CHECK(pair.b.delivered().empty());

    now = 0.10;
    pair.pumpAtoB(now);
    SW_CHECK_EQ(pair.b.delivered().size(), 3u);
    SW_CHECK_EQ(pair.b.delivered()[0].payload[0], 0u);
    SW_CHECK_EQ(pair.b.delivered()[1].payload[0], 1u);
    SW_CHECK_EQ(pair.b.delivered()[2].payload[0], 2u);
}

SW_TEST(ALargeReliableMessageIsFragmentedAndReassembled)
{
    Pair pair;

    std::vector<u8> payload(40000);
    for (usize i = 0; i < payload.size(); ++i)
    {
        payload[i] = static_cast<u8>((i * 31u + 7u) & 0xFF);
    }
    pair.a.queueReliable(net::MessageType::ConnectAccept, payload);

    // 40 kB against a 1178-byte fragment is 34 datagrams, which is more than
    // the 32-datagram window: the test also proves the window opens as acks
    // come back rather than deadlocking.
    f64 now = 0.0;
    for (int round = 0; round < 40 && pair.b.delivered().empty(); ++round)
    {
        now += 0.01;
        pair.pumpAtoB(now);
        pair.pumpBtoA(now);
    }

    SW_CHECK_EQ(pair.b.delivered().size(), 1u);
    if (!pair.b.delivered().empty())
    {
        SW_CHECK(pair.b.delivered()[0].payload == payload);
        SW_CHECK(pair.b.delivered()[0].type == net::MessageType::ConnectAccept);
    }
}

SW_TEST(AnUnreliableMessageOlderThanOneAlreadyAppliedIsDiscarded)
{
    net::Connection sender;
    net::Connection receiver;

    std::vector<std::vector<u8>> datagrams;
    for (u8 i = 0; i < 3; ++i)
    {
        const std::vector<u8> payload{i};
        sender.queueUnreliable(net::MessageType::Snapshot, payload);
        sender.collectOutgoing(0.0, datagrams);
    }
    SW_CHECK_EQ(datagrams.size(), 3u);

    // Deliver them backwards, as a path with two routes would.
    receiver.receive(0.0, datagrams[2]);
    receiver.receive(0.0, datagrams[1]);
    receiver.receive(0.0, datagrams[0]);

    SW_CHECK_EQ(receiver.delivered().size(), 1u);
    SW_CHECK_EQ(receiver.delivered()[0].payload[0], 2u);
    SW_CHECK_EQ(receiver.stats().staleDropped, 2u);
}

SW_TEST(AStrangersDatagramIsRejectedWithoutBeingParsed)
{
    net::Connection connection;
    const std::vector<u8> noise{0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01, 0x02, 0x03,
                                0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B};
    connection.receive(0.0, noise);
    connection.receive(0.0, std::vector<u8>{1, 2, 3});
    SW_CHECK(connection.delivered().empty());
    SW_CHECK_EQ(connection.stats().malformedDropped, 2u);
}

// ----------------------------------------------------------------------------
// The ECS mirror primitive
// ----------------------------------------------------------------------------

SW_TEST(MirroringPlacesAnEntityAtAnExactIndexAndGeneration)
{
    ecs::World world;
    const ecs::Entity mirrored = world.mirrorEntity(ecs::Entity{7, 3});
    SW_CHECK_EQ(mirrored.index, 7u);
    SW_CHECK(world.isAlive(mirrored));
    SW_CHECK(!world.isAlive(ecs::Entity{7, 2}));

    // Idempotent: the same handle again changes nothing.
    world.addComponent(mirrored, NetPosition{1.0, 2.0, 3.0});
    world.mirrorEntity(ecs::Entity{7, 3});
    SW_CHECK(world.hasComponent<NetPosition>(mirrored));

    // A different generation at the same index means the host recycled it:
    // the old occupant and everything on it must go.
    const ecs::Entity recycled = world.mirrorEntity(ecs::Entity{7, 4});
    SW_CHECK(world.isAlive(recycled));
    SW_CHECK(!world.isAlive(mirrored));
    SW_CHECK(!world.hasComponent<NetPosition>(recycled));

    // And the slot must not still be handed out by createEntity.
    for (int i = 0; i < 12; ++i)
    {
        const ecs::Entity fresh = world.createEntity();
        SW_CHECK(fresh.index != 7u);
    }
}

SW_TEST(RawComponentAccessMatchesTheTypedAccess)
{
    ecs::World world;
    const ecs::Entity entity = world.createEntity();
    const ecs::ComponentTypeId type = ecs::componentTypeId<NetPosition>();

    SW_CHECK(!world.hasComponentRaw(entity, type));
    std::byte* storage = world.addComponentRaw(entity, type);
    SW_CHECK(storage != nullptr);
    SW_CHECK(world.hasComponent<NetPosition>(entity));

    const NetPosition value{4.0, 5.0, 6.0};
    std::memcpy(storage, &value, sizeof(value));
    SW_CHECK_EQ(world.getComponent<NetPosition>(entity).y, 5.0);

    world.removeComponentRaw(entity, type);
    SW_CHECK(!world.hasComponent<NetPosition>(entity));
    SW_CHECK(world.tryGetComponentRaw(entity, type) == nullptr);
}

// ----------------------------------------------------------------------------
// Replication
// ----------------------------------------------------------------------------

SW_TEST(AFullSnapshotRebuildsAWorldEntityForEntity)
{
    const save::Schema schema = makeNetSchema();
    const net::ReplicationTable table = net::ReplicationTable::build(schema, makeNetSet());

    ecs::World host;
    const ecs::Entity a = host.createEntity();
    const ecs::Entity b = host.createEntity();
    host.addComponent(a, NetPosition{1.0, 2.0, 3.0});
    host.addComponent(a, NetLink{b, 42});
    host.addComponent(b, NetPosition{-4.0, 0.0, 9.5});
    host.addComponent(b, NetLocalOnly{999}); // not in the set: must not travel

    net::ReplicationEncoder encoder(table);
    net::ReplicationDecoder decoder(table);

    ser::BinaryWriter writer;
    encoder.encode(host, 12.5, writer);
    SW_CHECK(encoder.lastStats().wasFull);

    ecs::World mirror;
    ser::BinaryReader reader(writer.bytes());
    SW_CHECK(decoder.apply(mirror, reader));
    SW_CHECK_EQ(decoder.simulatedSeconds(), 12.5);

    SW_CHECK(mirror.isAlive(a));
    SW_CHECK(mirror.isAlive(b));
    SW_CHECK_EQ(mirror.getComponent<NetPosition>(a).x, 1.0);
    SW_CHECK_EQ(mirror.getComponent<NetPosition>(b).z, 9.5);
    SW_CHECK(!mirror.hasComponent<NetLocalOnly>(b));

    // THE POINT OF INDEX-EXACT MIRRORING: the handle stored inside the
    // component still names the right entity on the other machine.
    SW_CHECK(mirror.getComponent<NetLink>(a).target == b);
    SW_CHECK(mirror.isAlive(mirror.getComponent<NetLink>(a).target));
    SW_CHECK_EQ(mirror.getComponent<NetLink>(a).tag, 42u);
}

SW_TEST(ADeltaCarriesOnlyWhatMoved)
{
    const save::Schema schema = makeNetSchema();
    const net::ReplicationTable table = net::ReplicationTable::build(schema, makeNetSet());

    ecs::World host;
    std::vector<ecs::Entity> entities;
    for (int i = 0; i < 200; ++i)
    {
        const ecs::Entity entity = host.createEntity();
        host.addComponent(entity, NetPosition{static_cast<f64>(i), 0.0, 0.0});
        entities.push_back(entity);
    }

    net::ReplicationEncoder encoder(table);
    net::ReplicationDecoder decoder(table);
    ecs::World mirror;

    ser::BinaryWriter full;
    const u32 firstId = encoder.encode(host, 0.0, full);
    {
        ser::BinaryReader reader(full.bytes());
        SW_CHECK(decoder.apply(mirror, reader));
    }
    encoder.acknowledge(firstId);
    const usize fullBytes = encoder.lastStats().payloadBytes;

    // Move three of two hundred.
    host.getComponent<NetPosition>(entities[5]).y = 1.0;
    host.getComponent<NetPosition>(entities[50]).y = 2.0;
    host.getComponent<NetPosition>(entities[199]).y = 3.0;

    ser::BinaryWriter delta;
    encoder.encode(host, 0.02, delta);
    SW_CHECK(!encoder.lastStats().wasFull);
    SW_CHECK_EQ(encoder.lastStats().changed, 3u);
    // Three of two hundred should cost far less than a twentieth of the whole.
    SW_CHECK(encoder.lastStats().payloadBytes * 20 < fullBytes);

    {
        ser::BinaryReader reader(delta.bytes());
        SW_CHECK(decoder.apply(mirror, reader));
    }
    SW_CHECK_EQ(mirror.getComponent<NetPosition>(entities[50]).y, 2.0);
    SW_CHECK_EQ(mirror.getComponent<NetPosition>(entities[199]).y, 3.0);
    SW_CHECK_EQ(mirror.getComponent<NetPosition>(entities[7]).y, 0.0);
}

SW_TEST(ADeltaOnAnUnheldBaselineIsRefusedRatherThanApplied)
{
    const save::Schema schema = makeNetSchema();
    const net::ReplicationTable table = net::ReplicationTable::build(schema, makeNetSet());

    ecs::World host;
    const ecs::Entity entity = host.createEntity();
    host.addComponent(entity, NetPosition{1.0, 1.0, 1.0});

    net::ReplicationEncoder encoder(table);
    net::ReplicationDecoder decoder(table);
    ecs::World mirror;

    ser::BinaryWriter first;
    const u32 firstId = encoder.encode(host, 0.0, first);
    ser::BinaryReader firstReader(first.bytes());
    SW_CHECK(decoder.apply(mirror, firstReader));
    encoder.acknowledge(firstId);

    // Snapshot 2 is built on 1 and then LOST. Snapshot 3 is built on 1 too
    // (nothing newer was acknowledged), so it still applies — a lost
    // snapshot must not need a repair path.
    host.getComponent<NetPosition>(entity).x = 2.0;
    ser::BinaryWriter lost;
    encoder.encode(host, 0.02, lost);

    host.getComponent<NetPosition>(entity).x = 3.0;
    ser::BinaryWriter next;
    encoder.encode(host, 0.04, next);
    ser::BinaryReader nextReader(next.bytes());
    SW_CHECK(decoder.apply(mirror, nextReader));
    SW_CHECK_EQ(mirror.getComponent<NetPosition>(entity).x, 3.0);

    // And the one that was overtaken is refused, leaving the world alone.
    ser::BinaryReader lostReader(lost.bytes());
    SW_CHECK(!decoder.apply(mirror, lostReader));
    SW_CHECK_EQ(mirror.getComponent<NetPosition>(entity).x, 3.0);
}

SW_TEST(DestroyedEntitiesAndRemovedComponentsDisappearFromTheMirror)
{
    const save::Schema schema = makeNetSchema();
    const net::ReplicationTable table = net::ReplicationTable::build(schema, makeNetSet());

    ecs::World host;
    const ecs::Entity keep = host.createEntity();
    const ecs::Entity die = host.createEntity();
    host.addComponent(keep, NetPosition{});
    host.addComponent(keep, NetLink{die, 1});
    host.addComponent(die, NetPosition{});

    net::ReplicationEncoder encoder(table);
    net::ReplicationDecoder decoder(table);
    ecs::World mirror;

    ser::BinaryWriter full;
    const u32 id = encoder.encode(host, 0.0, full);
    ser::BinaryReader fullReader(full.bytes());
    SW_CHECK(decoder.apply(mirror, fullReader));
    encoder.acknowledge(id);
    SW_CHECK(mirror.isAlive(die));

    host.destroyEntity(die);
    host.removeComponent<NetLink>(keep);

    ser::BinaryWriter delta;
    encoder.encode(host, 0.02, delta);
    SW_CHECK_EQ(encoder.lastStats().removed, 1u);
    SW_CHECK_EQ(encoder.lastStats().dropped, 1u);

    ser::BinaryReader deltaReader(delta.bytes());
    SW_CHECK(decoder.apply(mirror, deltaReader));
    SW_CHECK(!mirror.isAlive(die));
    SW_CHECK(mirror.isAlive(keep));
    SW_CHECK(!mirror.hasComponent<NetLink>(keep));
}

SW_TEST(ARecycledEntityIndexIsRebuiltRatherThanReinterpreted)
{
    const save::Schema schema = makeNetSchema();
    const net::ReplicationTable table = net::ReplicationTable::build(schema, makeNetSet());

    ecs::World host;
    const ecs::Entity first = host.createEntity();
    host.addComponent(first, NetPosition{1.0, 1.0, 1.0});
    host.addComponent(first, NetLink{first, 7});

    net::ReplicationEncoder encoder(table);
    net::ReplicationDecoder decoder(table);
    ecs::World mirror;

    ser::BinaryWriter full;
    const u32 id = encoder.encode(host, 0.0, full);
    ser::BinaryReader fullReader(full.bytes());
    SW_CHECK(decoder.apply(mirror, fullReader));
    encoder.acknowledge(id);

    // Same index, new generation, and this time WITHOUT the link component.
    host.destroyEntity(first);
    const ecs::Entity second = host.createEntity();
    SW_CHECK_EQ(second.index, first.index);
    SW_CHECK(second.generation != first.generation);
    host.addComponent(second, NetPosition{1.0, 1.0, 1.0}); // identical bytes!

    ser::BinaryWriter delta;
    encoder.encode(host, 0.02, delta);
    ser::BinaryReader deltaReader(delta.bytes());
    SW_CHECK(decoder.apply(mirror, deltaReader));

    SW_CHECK(mirror.isAlive(second));
    SW_CHECK(!mirror.isAlive(first));
    SW_CHECK(mirror.hasComponent<NetPosition>(second));
    // The bytes of the position were identical, so a plain memcmp diff would
    // have skipped it and left the mirror with an empty entity — and the
    // link belonged to somebody else entirely.
    SW_CHECK_EQ(mirror.getComponent<NetPosition>(second).x, 1.0);
    SW_CHECK(!mirror.hasComponent<NetLink>(second));
}

SW_TEST(AComponentThatDiffersBetweenBuildsIsRefusedAtTheHandshake)
{
    save::Schema hostSchema;
    hostSchema.registerComponent<NetPosition>("net.Position", 1);

    save::Schema clientSchema;
    clientSchema.registerComponent<NetPosition>("net.Position", 2); // v2 here

    net::ReplicationSet set;
    set.include("net.Position");

    ser::BinaryWriter writer;
    net::ReplicationTable::build(hostSchema, set).write(writer);

    bool threw = false;
    try
    {
        ser::BinaryReader reader(writer.bytes());
        (void)net::ReplicationTable::read(reader, clientSchema);
    }
    catch (const Exception&)
    {
        threw = true;
    }
    SW_CHECK(threw);
}

// ----------------------------------------------------------------------------
// The whole stack
// ----------------------------------------------------------------------------

SW_TEST(AClientJoinsAndMirrorsALivingWorld)
{
    const save::Schema schema = makeNetSchema();

    net::LoopbackNetwork wire;
    net::LoopbackNetwork::Conditions conditions;
    conditions.latencySeconds = 0.03;
    wire.setConditions(conditions);

    net::Host host(wire.endpoint(7777), schema, makeNetSet());
    net::Client client(wire.endpoint(40001), schema, "arthur");

    ecs::World hostWorld;
    ecs::World mirror;
    std::vector<ecs::Entity> entities;
    for (int i = 0; i < 30; ++i)
    {
        const ecs::Entity entity = hostWorld.createEntity();
        hostWorld.addComponent(entity, NetPosition{static_cast<f64>(i), 0.0, 0.0});
        entities.push_back(entity);
    }
    hostWorld.addComponent(entities[0], NetLink{entities[29], 5});

    f64 now = 0.0;
    wire.setTime(now);
    client.connect(net::PeerAddress::localhost(7777), now);

    f64 simulated = 0.0;
    run(wire, now, 1.0,
        [&](f64 t) {
            simulated = t;
            for (usize i = 0; i < entities.size(); ++i)
            {
                hostWorld.getComponent<NetPosition>(entities[i]).y = t * (i + 1);
            }
            host.update(t, hostWorld, simulated);
        },
        [&](f64 t) { client.update(t, mirror, t); });

    SW_CHECK(client.state() == net::ClientState::Connected);
    SW_CHECK_EQ(host.clientCount(), 1u);
    SW_CHECK(client.appliedSnapshotId() > 1u);

    for (usize i = 0; i < entities.size(); ++i)
    {
        SW_CHECK(mirror.isAlive(entities[i]));
    }
    SW_CHECK(mirror.getComponent<NetLink>(entities[0]).target == entities[29]);
    // The mirror is one snapshot behind at most, so compare against what the
    // client was told, not against the host's current instant.
    const f64 mirrored = mirror.getComponent<NetPosition>(entities[3]).y;
    SW_CHECK(std::abs(mirrored - client.hostSimulatedSeconds() * 4.0) < 1.0e-9);
}

SW_TEST(TheMirrorCatchesUpAcrossALinkThatLosesAFifthOfEverything)
{
    const save::Schema schema = makeNetSchema();

    net::LoopbackNetwork wire;
    net::LoopbackNetwork::Conditions conditions;
    conditions.latencySeconds = 0.08;
    conditions.jitterSeconds = 0.06; // > the send interval, so packets overtake
    conditions.lossRatio = 0.20f;
    conditions.seed = 12345;
    wire.setConditions(conditions);

    net::Host host(wire.endpoint(7777), schema, makeNetSet());
    net::Client client(wire.endpoint(40002), schema, "arthur");

    ecs::World hostWorld;
    ecs::World mirror;
    std::vector<ecs::Entity> entities;
    for (int i = 0; i < 40; ++i)
    {
        const ecs::Entity entity = hostWorld.createEntity();
        hostWorld.addComponent(entity, NetPosition{static_cast<f64>(i), 0.0, 0.0});
        entities.push_back(entity);
    }

    f64 now = 0.0;
    wire.setTime(now);
    client.connect(net::PeerAddress::localhost(7777), now);

    run(wire, now, 4.0,
        [&](f64 t) {
            for (usize i = 0; i < entities.size(); ++i)
            {
                hostWorld.getComponent<NetPosition>(entities[i]).y = t * (i + 1);
            }
            // Something appears halfway through and something else goes away.
            if (t > 2.0 && entities.size() == 40)
            {
                const ecs::Entity extra = hostWorld.createEntity();
                hostWorld.addComponent(extra, NetPosition{99.0, 0.0, 0.0});
                entities.push_back(extra);
            }
            host.update(t, hostWorld, t);
        },
        [&](f64 t) { client.update(t, mirror, t); });

    SW_CHECK(client.state() == net::ClientState::Connected);
    SW_CHECK_EQ(mirror.count<NetPosition>(), hostWorld.count<NetPosition>());
    SW_CHECK(wire.droppedCount() > 0);
    SW_CHECK(wire.reorderedCount() > 0);

    // Every mirrored value must be self-consistent with the instant the
    // client believes it is looking at. A single wrong byte anywhere in the
    // delta chain shows up here.
    const f64 instant = client.hostSimulatedSeconds();
    u32 checked = 0;
    for (usize i = 0; i < 40; ++i)
    {
        if (!mirror.isAlive(entities[i]))
        {
            continue;
        }
        const f64 y = mirror.getComponent<NetPosition>(entities[i]).y;
        SW_CHECK(std::abs(y - instant * static_cast<f64>(i + 1)) < 1.0e-9);
        ++checked;
    }
    SW_CHECK_EQ(checked, 40u);
    SW_CHECK(mirror.isAlive(entities.back()));
}

SW_TEST(AClientsCommandsReachTheHostInOrder)
{
    const save::Schema schema = makeNetSchema();

    net::LoopbackNetwork wire;
    net::LoopbackNetwork::Conditions conditions;
    conditions.latencySeconds = 0.05;
    conditions.jitterSeconds = 0.04;
    conditions.lossRatio = 0.25f;
    conditions.seed = 777;
    wire.setConditions(conditions);

    net::Host host(wire.endpoint(7777), schema, makeNetSet());
    net::Client client(wire.endpoint(40003), schema, "arthur");

    ecs::World hostWorld;
    ecs::World mirror;

    f64 now = 0.0;
    wire.setTime(now);
    client.connect(net::PeerAddress::localhost(7777), now);

    std::vector<u32> received;
    u32 nextCommand = 0;
    run(wire, now, 3.0,
        [&](f64 t) {
            host.update(t, hostWorld, t);
            for (const net::Host::CommandEvent& event : host.commands())
            {
                ser::BinaryReader reader(event.payload);
                received.push_back(reader.read<u32>());
                SW_CHECK_EQ(event.clientId, 1u);
            }
        },
        [&](f64 t) {
            client.update(t, mirror, t);
            if (client.state() == net::ClientState::Connected && nextCommand < 50)
            {
                ser::BinaryWriter writer;
                writer.write(nextCommand++);
                client.sendCommand(writer.bytes());
            }
        });

    SW_CHECK_EQ(received.size(), static_cast<usize>(nextCommand));
    for (usize i = 0; i < received.size(); ++i)
    {
        SW_CHECK_EQ(received[i], static_cast<u32>(i));
    }
    SW_CHECK(nextCommand > 40);
}

SW_TEST(AHostThatStopsTalkingTimesTheClientOut)
{
    const save::Schema schema = makeNetSchema();

    net::LoopbackNetwork wire;
    net::Host host(wire.endpoint(7777), schema, makeNetSet());

    net::Client::Config config;
    config.connection.timeoutSeconds = 0.5;
    net::Client client(wire.endpoint(40004), schema, "arthur", config);

    ecs::World hostWorld;
    ecs::World mirror;
    hostWorld.addComponent(hostWorld.createEntity(), NetPosition{});

    f64 now = 0.0;
    wire.setTime(now);
    client.connect(net::PeerAddress::localhost(7777), now);

    run(wire, now, 0.3, [&](f64 t) { host.update(t, hostWorld, t); },
        [&](f64 t) { client.update(t, mirror, t); });
    SW_CHECK(client.state() == net::ClientState::Connected);

    // The host goes away without saying goodbye.
    run(wire, now, 1.0, [](f64) {}, [&](f64 t) { client.update(t, mirror, t); });
    SW_CHECK(client.state() == net::ClientState::TimedOut);
}

// ----------------------------------------------------------------------------
// Timelines: every player owns their clock
// ----------------------------------------------------------------------------

SW_TEST(AnEventWaitsForTheInstantItIsStampedWith)
{
    net::Timeline timeline;

    // Arrive out of order, as a relayed broadcast will.
    timeline.push(net::TimelineEvent{300.0, 1, 7, {}});
    timeline.push(net::TimelineEvent{100.0, 1, 5, {}});
    timeline.push(net::TimelineEvent{200.0, 2, 6, {}});
    SW_CHECK_EQ(timeline.pendingCount(), 3u);
    SW_CHECK_EQ(timeline.nextStampSeconds(), 100.0);

    // Our clock has not reached any of them: nothing happens. This is the
    // whole point — a player three hours ahead cannot make his separation
    // occur in our present.
    SW_CHECK(timeline.advance(50.0).empty());
    SW_CHECK_EQ(timeline.pendingCount(), 3u);

    // Walk forward. Each event lands exactly when we reach its instant.
    SW_CHECK_EQ(timeline.advance(150.0).size(), 1u);
    SW_CHECK_EQ(timeline.pendingCount(), 2u);

    const std::span<const net::TimelineEvent> rest = timeline.advance(1000.0);
    SW_CHECK_EQ(rest.size(), 2u);
    SW_CHECK_EQ(rest[0].kind, 6u);   // stamp order, not arrival order
    SW_CHECK_EQ(rest[1].kind, 7u);
    SW_CHECK_EQ(timeline.pendingCount(), 0u);
    SW_CHECK_EQ(timeline.releasedCount(), 3u);
    SW_CHECK_EQ(timeline.lateCount(), 0u);
}

SW_TEST(AnEventStampedInOurPastLandsAtOnceAndIsCounted)
{
    net::Timeline timeline;
    (void)timeline.advance(500.0); // our clock is at 500

    timeline.push(net::TimelineEvent{200.0, 3, 1, {}});
    SW_CHECK_EQ(timeline.lateCount(), 1u);
    SW_CHECK_EQ(timeline.advance(500.0).size(), 1u);
}

SW_TEST(EveryPlayersOwnClockReachesTheOthersUntouched)
{
    const save::Schema schema = makeNetSchema();

    net::LoopbackNetwork wire;
    net::LoopbackNetwork::Conditions conditions;
    conditions.latencySeconds = 0.02;
    wire.setConditions(conditions);

    net::Host::Config hostConfig;
    hostConfig.hostName = "arthur";
    net::Host host(wire.endpoint(7777), schema, makeNetSet(), hostConfig);
    net::Client client(wire.endpoint(40010), schema, "copilot");

    ecs::World hostWorld;
    ecs::World mirror;
    hostWorld.addComponent(hostWorld.createEntity(), NetPosition{});

    f64 now = 0.0;
    wire.setTime(now);
    client.connect(net::PeerAddress::localhost(7777), now);

    // The client warps: its own clock runs a thousand times faster. The
    // host's does not budge, and that is the requirement.
    f64 hostClock = 1000.0;
    f64 clientClock = 1000.0;
    run(wire, now, 2.0,
        [&](f64 t) {
            hostClock = 1000.0 + (t - 0.0);
            host.update(t, hostWorld, hostClock);
        },
        [&](f64 t) {
            clientClock += 10.0; // 10 s of simulation per 10 ms step
            client.update(t, mirror, clientClock);
        });

    SW_CHECK(client.state() == net::ClientState::Connected);
    SW_CHECK_EQ(host.roster().size(), 2u);
    SW_CHECK_EQ(client.roster().size(), 2u);

    // The host knows the client is far ahead. The reported value lags by up
    // to one clock beat (5 Hz against a client running 1000x), which is why
    // this checks the GAP and not equality.
    const f64 known = host.roster()[1].simulatedSeconds;
    SW_CHECK(known > hostClock + 1000.0);
    SW_CHECK(known <= clientClock);
    // ...and its own clock was not dragged along by one second.
    SW_CHECK(std::abs(host.roster()[0].simulatedSeconds - hostClock) < 1.0e-9);
    SW_CHECK(hostClock < 1010.0);

    // The client sees the same two entries and the same gap.
    SW_CHECK_EQ(client.roster()[0].id, 0u);
    SW_CHECK(client.roster()[1].simulatedSeconds > client.roster()[0].simulatedSeconds);
}

SW_TEST(AnActionInTheFutureHappensAtItsOwnInstantForEveryoneElse)
{
    const save::Schema schema = makeNetSchema();

    net::LoopbackNetwork wire;
    net::LoopbackNetwork::Conditions conditions;
    conditions.latencySeconds = 0.02;
    conditions.lossRatio = 0.15f;
    conditions.seed = 4242;
    wire.setConditions(conditions);

    net::Host host(wire.endpoint(7777), schema, makeNetSet());
    net::Client client(wire.endpoint(40011), schema, "traveller");

    ecs::World hostWorld;
    ecs::World mirror;
    hostWorld.addComponent(hostWorld.createEntity(), NetPosition{});

    f64 now = 0.0;
    wire.setTime(now);
    client.connect(net::PeerAddress::localhost(7777), now);

    // The client is FAR ahead and announces something at its own instant.
    constexpr f64 kFutureInstant = 9000.0;
    bool announced = false;
    f64 hostClock = 100.0;
    u32 hostSaw = 0;
    f64 hostSawAt = 0.0;

    run(wire, now, 3.0,
        [&](f64 t) {
            // The host creeps forward in real time, then jumps past the
            // announced instant near the end of the run.
            hostClock = (t < 2.0) ? 100.0 + t * 10.0 : 20000.0;
            host.update(t, hostWorld, hostClock);
            for (const net::TimelineEvent& event : host.timeline().advance(hostClock))
            {
                ++hostSaw;
                hostSawAt = hostClock;
                SW_CHECK_EQ(event.kind, 77u);
                SW_CHECK_EQ(event.originClientId, 1u);
            }
        },
        [&](f64 t) {
            client.update(t, mirror, kFutureInstant + t);
            if (!announced && client.state() == net::ClientState::Connected)
            {
                announced = true;
                const std::vector<u8> payload{1, 2, 3};
                client.sendEvent(kFutureInstant, 77u, payload);
            }
        });

    SW_CHECK(announced);
    // Exactly once, and NOT while the host's clock was still short of the
    // instant it was stamped with.
    SW_CHECK_EQ(hostSaw, 1u);
    SW_CHECK(hostSawAt >= kFutureInstant);
}

// ----------------------------------------------------------------------------
// The address a peer should aim at
// ----------------------------------------------------------------------------

SW_TEST(TheRoutedLocalAddressIsUsableAndCarriesTheRequestedPort)
{
    // This is the one place the stack asks the operating system a question
    // instead of being told the answer, so what is checked is the contract,
    // not a particular address: whatever comes back must be a real IPv4
    // endpoint on the requested port, and it must survive a round trip
    // through the same text form the join field uses. A machine with no
    // route out legitimately answers 127.0.0.1 — that is a fallback, not a
    // failure, and the test accepts it.
    const net::PeerAddress routed = net::localAddress(7777);
    SW_CHECK_EQ(routed.port, u16{7777});
    SW_CHECK(routed.ipv4 != 0);

    net::PeerAddress reparsed{};
    SW_CHECK(net::PeerAddress::parse(routed.toString(), reparsed));
    SW_CHECK(reparsed == routed);

    // Port zero means "any port" to a socket, but as an address to aim at it
    // is meaningless; the caller always passes the port it bound.
    SW_CHECK_EQ(net::localAddress(1).port, u16{1});
}

// ----------------------------------------------------------------------------
// Saying which kind of silence it is
// ----------------------------------------------------------------------------

SW_TEST(AHostCountsWhatReachedItAndNamesAVersionMismatch)
{
    // The failure this guards against: a client whose build speaks a
    // different protocol version has its datagram rejected by readHeader and
    // dropped without a word, which from the client is INDISTINGUISHABLE
    // from a firewall eating the packet. The two have nothing in common as
    // problems, so the host has to be able to tell them apart.
    const save::Schema schema = makeNetSchema();
    net::LoopbackNetwork wire;
    net::Host host(wire.endpoint(7777), schema, makeNetSet());

    std::unique_ptr<net::Transport> stranger = wire.endpoint(40002);
    const net::PeerAddress hostAddress{0x7F000001u, 7777};

    ecs::World hostWorld;
    f64 now = 0.0;

    auto pump = [&]() {
        for (int i = 0; i < 8; ++i)
        {
            now += 0.01;
            wire.setTime(now);
            host.update(now, hostWorld, now);
        }
    };

    // Nothing has been sent: the host must say so rather than shrug.
    pump();
    SW_CHECK_EQ(host.reception().arrived, u64{0});

    // A datagram from a build one version behind. Only the first six bytes
    // decide its fate, and those are exactly what the diagnosis reads.
    {
        ser::BinaryWriter writer;
        writer.write(net::kProtocolMagic);
        writer.write(static_cast<u16>(net::kProtocolVersion - 1));
        writer.write(static_cast<u8>(net::MessageType::ConnectRequest));
        writer.write(static_cast<u8>(0));
        writer.write(static_cast<u16>(0));
        writer.write(static_cast<u16>(0));
        writer.write(static_cast<u32>(0));
        SW_CHECK(stranger->send(hostAddress, writer.bytes()));
    }
    pump();

    SW_CHECK_EQ(host.reception().arrived, u64{1});
    SW_CHECK_EQ(host.reception().fromStrangers, u64{1});
    SW_CHECK_EQ(host.reception().refused, u64{1});
    SW_CHECK_EQ(host.reception().wrongVersion, u64{1});
    SW_CHECK_EQ(host.reception().lastForeignVersion,
                static_cast<u16>(net::kProtocolVersion - 1));
    SW_CHECK_EQ(host.reception().notOurs, u64{0});
    // Rejected, so no peer was allocated for it — a stranger cannot cost the
    // host memory by being wrong.
    SW_CHECK_EQ(host.clientCount(), u32{0});

    // Something that is not ours at all: same silence, different cause.
    {
        const std::vector<u8> noise{0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x00, 0x02, 0x00};
        SW_CHECK(stranger->send(hostAddress, noise));
    }
    pump();

    SW_CHECK_EQ(host.reception().arrived, u64{2});
    SW_CHECK_EQ(host.reception().notOurs, u64{1});
    SW_CHECK_EQ(host.reception().wrongVersion, u64{1});
}
