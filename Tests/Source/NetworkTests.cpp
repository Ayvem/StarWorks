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

// ----------------------------------------------------------------------------
// A HOSTILE PEER
//
// Everything below feeds the stack bytes that no correct sender would ever
// produce. The bar is not "it copes"; it is that the process is still running
// afterwards and that nothing it allocated was sized by the attacker. The
// LoopbackNetwork makes this exact: an endpoint labelled with the host's port
// sends from the host's address, which is all the client checks, so a test
// can be the man in the middle without a socket, a thread or a race.
// ----------------------------------------------------------------------------

namespace
{
    /// One datagram, built by hand. The point is to write the fields a
    /// correct sender never would.
    std::vector<u8> datagram(net::MessageType type, u8 flags, u16 sequence,
                             std::span<const u8> payload, u16 messageId = 0,
                             u16 fragmentIndex = 0, u16 fragmentCount = 0)
    {
        net::PacketHeader header{};
        header.type = type;
        header.flags = flags;
        header.sequence = sequence;
        header.messageId = messageId;
        header.fragmentIndex = fragmentIndex;
        header.fragmentCount = fragmentCount;
        ser::BinaryWriter writer;
        net::writeHeader(writer, header);
        writer.writeBytes(payload.data(), payload.size());
        return writer.bytes();
    }

    /// A client that has answered a challenge from a host that does not
    /// exist, so the test can be the host and say anything it likes. Returns
    /// with the client in Connecting and its reliable channel at sequence
    /// zero, which is where a ConnectAccept is expected.
    struct FakeHost
    {
        net::LoopbackNetwork wire;
        save::Schema schema = makeNetSchema();
        std::unique_ptr<net::Transport> voice = wire.endpoint(7777);
        net::Client client{wire.endpoint(41000), schema, "victim"};
        ecs::World mirror;
        f64 now = 0.0;

        FakeHost()
        {
            wire.setTime(now);
            client.connect(net::PeerAddress::localhost(7777), now);
            step();

            // Challenge it, the way a real host does, and let it answer.
            ser::BinaryWriter cookie;
            cookie.write(static_cast<u64>(0xA5A5'0000'1234'5678ull));
            send(datagram(net::MessageType::ConnectChallenge, 0, 0, cookie.bytes()));
            step();
        }

        void send(const std::vector<u8>& bytes)
        {
            (void)voice->send(net::PeerAddress::localhost(41000), bytes);
        }

        void step()
        {
            now += 0.01;
            wire.setTime(now);
            client.update(now, mirror, now);
        }
    };
} // namespace

SW_TEST(ATruncatedDatagramFromTheHostIsDroppedRatherThanFatal)
{
    // Session.cpp used to parse a ConnectAccept with no try/catch at all, so
    // a ConnectAccept that stops after its version field threw straight out
    // of Client::update and ended the process. From the client this is
    // indistinguishable from a corrupted packet on the path, which is a thing
    // that happens.
    FakeHost fake;
    SW_CHECK(fake.client.state() == net::ClientState::Connecting);

    ser::BinaryWriter truncated;
    truncated.write(net::kSessionVersion); // ...and then nothing at all
    fake.send(datagram(net::MessageType::ConnectAccept,
                       net::PacketHeader::kFlagReliable, 0, truncated.bytes()));
    fake.step();

    SW_CHECK_EQ(fake.client.refusedMessages(), 1u);
    SW_CHECK(fake.client.state() == net::ClientState::Connecting);

    // Still alive, and still able to be told the truth afterwards.
    fake.step();
    SW_CHECK(fake.client.state() == net::ClientState::Connecting);
}

SW_TEST(AConnectAcceptClaimingAHugeComponentTableIsRefusedBeforeAnyAllocation)
{
    // 0xFFFF, NOT 0xFFFFFFFF. Wire ids are sixteen bits, and a table claiming
    // more than 0xFFFF entries has been refused by that rule since long
    // before any of this — so a four-billion count never reaches the bound
    // this test is for, and a test written with one proves nothing about it.
    // 65,535 is a legal count, and every entry costs a std::string: 786,420
    // bytes of names, sizes and versions that a 1,200-byte datagram does not
    // have.
    const save::Schema schema = makeNetSchema();

    ser::BinaryWriter table;
    table.write(static_cast<u32>(0x53575254u)); // 'SWRT', the table magic
    table.write(static_cast<u32>(0xFFFFu));
    // Two entries that are entirely believable, so that without the bound the
    // loop is properly under way — building strings, resolving them against
    // the schema — before the message runs out. A count with nothing at all
    // behind it throws on the first read whether the bound exists or not.
    table.writeString("net.Position");
    table.write(static_cast<u32>(sizeof(NetPosition)));
    table.write(static_cast<u32>(1));
    table.writeString("net.Link");
    table.write(static_cast<u32>(sizeof(NetLink)));
    table.write(static_cast<u32>(1));

    std::string refusal;
    try
    {
        ser::BinaryReader reader(table.bytes());
        (void)net::ReplicationTable::read(reader, schema);
    }
    catch (const std::exception& error)
    {
        // std::exception, not sw::Exception: an unbounded count fails at the
        // allocator, and a test that only caught ours would die with it
        // instead of reporting.
        refusal = error.what();
    }
    // Refused by the count-against-bytes bound and by nothing else: the
    // reader eventually running dry says something else entirely, and that is
    // exactly the failure this test has to be able to tell apart.
    SW_CHECK(refusal.find("entries of at least 12 bytes each") != std::string::npos);

    // And end to end: the client drops the message rather than dying at the
    // allocator, which is where it used to die.
    FakeHost fake;
    ser::BinaryWriter payload;
    payload.write(net::kSessionVersion);
    payload.write(static_cast<u32>(1));   // client id
    payload.write(static_cast<f64>(0.0)); // host clock
    payload.writeBytes(table.bytes().data(), table.size());
    fake.send(datagram(net::MessageType::ConnectAccept,
                       net::PacketHeader::kFlagReliable, 0, payload.bytes()));
    fake.step();

    SW_CHECK_EQ(fake.client.refusedMessages(), 1u);
    SW_CHECK(fake.client.state() == net::ClientState::Connecting);
}

SW_TEST(ARosterClaimingMorePlayersThanTheDatagramHoldsIsRefused)
{
    // sizeof(PlayerView) is 56 in this build, so reserve() on the largest
    // count this field can hold asked for 240 GB from a 20-byte datagram.
    FakeHost fake;

    ser::BinaryWriter payload;
    payload.write(static_cast<u32>(0xFFFFFFFFu));
    fake.send(datagram(net::MessageType::Roster, 0, 1000, payload.bytes()));
    fake.step();

    SW_CHECK(fake.client.roster().empty());
    SW_CHECK(fake.client.state() == net::ClientState::Connecting);

    // ...and the count that MATTERS, which is not the biggest one.
    // 240,518,168,520 bytes fails at the allocator and is survivable by
    // accident; 20,000,000 players is 1,120,000,000 bytes, measured, and
    // reserve succeeds. Nothing throws, so nothing can be caught: the only
    // thing that stops it is comparing the count against the twenty bytes
    // the datagram actually holds. Reaching
    // reserve() and unwinding afterwards leaves this counter at one instead
    // of two, because the unwinding path is the parser's own catch and counts
    // nothing.
    ser::BinaryWriter satisfiable;
    satisfiable.write(static_cast<u32>(20'000'000u));
    fake.send(datagram(net::MessageType::Roster, 0, 1001, satisfiable.bytes()));
    fake.step();
    SW_CHECK(fake.client.roster().empty());
    SW_CHECK_EQ(fake.client.refusedMessages(), 2u);

    // The honest version of the same message still works, which is the half
    // of the fix that is easy to break: bound the count, do not refuse it.
    ser::BinaryWriter good;
    good.write(static_cast<u32>(1));
    good.write(static_cast<u32>(0));
    good.writeString("host");
    good.write(static_cast<f64>(12.5));
    fake.send(datagram(net::MessageType::Roster, 0, 1002, good.bytes()));
    fake.step();
    SW_CHECK_EQ(fake.client.roster().size(), 1u);
    SW_CHECK_EQ(fake.client.roster()[0].simulatedSeconds, 12.5);
}

namespace
{
    /// Everything about a mirror that replication can change: which slots are
    /// alive, what generation each carries, and the exact component bytes of
    /// every replicated entity. Two equal fingerprints mean the world is
    /// EXACTLY as it was — which is the only honest way to state the property
    /// a refused snapshot has to leave behind.
    std::vector<u8> fingerprint(const ecs::World& world, const net::ReplicationTable& table)
    {
        net::WorldState state;
        net::captureState(world, table, state);

        ser::BinaryWriter writer;
        writer.write(world.recordCount());
        for (u32 index = 0; index < world.recordCount(); ++index)
        {
            writer.write(static_cast<u8>(world.isSlotAlive(index) ? 1 : 0));
            writer.write(world.slotGeneration(index));
        }
        // captureState sorts both lists, so this is canonical: it does not
        // change when archetypes happen to be visited in another order.
        writer.write(static_cast<u32>(state.entities.size()));
        for (const net::WorldState::EntityRef& entity : state.entities)
        {
            writer.write(entity.index);
            writer.write(entity.generation);
        }
        writer.write(static_cast<u32>(state.records.size()));
        for (const net::WorldState::Record& record : state.records)
        {
            writer.write(record.entityIndex);
            writer.write(record.componentId);
            writer.write(record.size);
            writer.writeBytes(state.blob.data() + record.offset, record.size);
        }
        return writer.bytes();
    }
} // namespace

SW_TEST(ASnapshotClaimingAHugeCountIsRefusedBeforeItReachesAnAllocator)
{
    const save::Schema schema = makeNetSchema();
    const net::ReplicationTable table = net::ReplicationTable::build(schema, makeNetSet());

    // Every count in a snapshot is chosen by whoever sent it, and each drives
    // a loop that allocates. A count is only believable if the bytes it
    // describes are still in the message.
    //
    // EVERY CASE BELOW CARRIES REAL TRAILING DATA, and it has to. Four of
    // these five used to have nothing at all after the count, so the first
    // read threw on iteration zero and the assertions held identically with
    // the bound and without it — a green test measuring nothing. Here the
    // bytes after each count describe entities the mirror really has, so
    // without the bound the loop gets far enough to destroy them.
    const auto refuses = [&](auto fill) {
        ecs::World mirror;
        for (u32 i = 0; i < 4; ++i)
        {
            const ecs::Entity entity = mirror.createEntity();
            mirror.addComponent(entity, NetPosition{static_cast<f64>(i), 0.0, 0.0});
        }
        const std::vector<u8> before = fingerprint(mirror, table);

        ser::BinaryWriter writer;
        writer.write(static_cast<u32>(0x53575250u)); // 'SWRP'
        writer.write(static_cast<u32>(1));           // snapshot id
        writer.write(static_cast<u32>(0));           // baseline: what we hold
        writer.write(static_cast<f64>(0.0));
        fill(writer);

        net::ReplicationDecoder decoder(table);
        ser::BinaryReader reader(writer.bytes());
        std::string refusal;
        try
        {
            (void)decoder.apply(mirror, reader);
        }
        catch (const std::exception& error)
        {
            // std::exception, not sw::Exception: an unbounded count reaches
            // the allocator, and std::bad_alloc is not one of ours.
            refusal = error.what();
        }
        // The count-against-bytes bound is what refused it. The reader
        // running dry three entries later would also throw, and that is the
        // outcome this test exists to distinguish itself from.
        SW_CHECK(refusal.find("entries of at least") != std::string::npos);
        SW_CHECK(fingerprint(mirror, table) == before);
        SW_CHECK_EQ(decoder.appliedSnapshotId(), 0u);
    };

    refuses([](ser::BinaryWriter& w) {
        w.write(static_cast<u32>(0xFFFFFFFFu));  // removals...
        w.write(static_cast<u32>(0));            // ...three of which are real,
        w.write(static_cast<u32>(1));            // and every one of them names
        w.write(static_cast<u32>(2));            // an entity the mirror holds.
    });
    refuses([](ser::BinaryWriter& w) {
        w.write(static_cast<u32>(0));            // removals
        w.write(static_cast<u32>(0xFFFFFFFFu));  // spawns...
        w.write(static_cast<u32>(5));            // ...two of which are real
        w.write(static_cast<u32>(1));
        w.write(static_cast<u32>(6));
        w.write(static_cast<u32>(1));
    });
    refuses([](ser::BinaryWriter& w) {
        w.write(static_cast<u32>(0));
        w.write(static_cast<u32>(0));
        w.write(static_cast<u32>(0xFFFFFFFFu));  // component drops...
        w.write(static_cast<u32>(0));            // ...two of which are real,
        w.write(static_cast<u16>(0));            // taking net.Position off
        w.write(static_cast<u32>(1));            // entities 0 and 1.
        w.write(static_cast<u16>(0));
    });
    refuses([](ser::BinaryWriter& w) {
        w.write(static_cast<u32>(0));
        w.write(static_cast<u32>(0));
        w.write(static_cast<u32>(0));
        w.write(static_cast<u32>(0xFFFFFFFFu));  // change groups...
        w.write(static_cast<u32>(0));            // ...one of which is real:
        w.write(static_cast<u16>(1));            // entity 0, one component,
        w.write(static_cast<u16>(0));            // net.Position, overwritten.
        w.write(static_cast<f64>(-1.0));
        w.write(static_cast<f64>(-1.0));
        w.write(static_cast<f64>(-1.0));
    });
    refuses([](ser::BinaryWriter& w) {
        w.write(static_cast<u32>(0));
        w.write(static_cast<u32>(0));
        w.write(static_cast<u32>(0));
        w.write(static_cast<u32>(1));            // one group...
        w.write(static_cast<u32>(0));            // ...for entity 0...
        w.write(static_cast<u16>(0xFFFFu));      // ...with 65535 components,
        w.write(static_cast<u16>(0));            // one of which is really here
        w.write(static_cast<f64>(-1.0));
        w.write(static_cast<f64>(-1.0));
        w.write(static_cast<f64>(-1.0));
    });
}

SW_TEST(ASnapshotRefusedPartWayThroughLeavesTheMirrorByteForByteAsItWas)
{
    // THE ONE THAT SILENTLY BREAKS EVERYTHING AFTERWARDS. Every count below
    // is honest and every bound passes; the message simply stops before the
    // component bytes its last group promised. A decoder that applies as it
    // parses has by then destroyed an entity, spawned another, taken a
    // component off a third and added a component it never wrote — and
    // because the caller catches the throw and drops the datagram, nothing
    // crashes and nothing complains. The mirror is just wrong, and stays
    // wrong, with every later delta diffed against a baseline it does not
    // hold.
    const save::Schema schema = makeNetSchema();
    const net::ReplicationTable table = net::ReplicationTable::build(schema, makeNetSet());

    ecs::World mirror;
    const ecs::Entity kept = mirror.createEntity();   // index 0
    mirror.addComponent(kept, NetPosition{1.0, 2.0, 3.0});
    const ecs::Entity doomed = mirror.createEntity(); // index 1
    mirror.addComponent(doomed, NetPosition{4.0, 5.0, 6.0});
    mirror.addComponent(doomed, NetLink{kept, 9});

    const std::vector<u8> before = fingerprint(mirror, table);

    ser::BinaryWriter writer;
    writer.write(static_cast<u32>(0x53575250u)); // 'SWRP'
    writer.write(static_cast<u32>(1));           // snapshot id
    writer.write(static_cast<u32>(0));           // baseline: what we hold
    writer.write(static_cast<f64>(17.5));
    writer.write(static_cast<u32>(1));           // one removal...
    writer.write(static_cast<u32>(1));           // ...entity 1
    writer.write(static_cast<u32>(1));           // one spawn...
    writer.write(static_cast<u32>(5));           // ...entity 5,
    writer.write(static_cast<u32>(1));           // ...generation 1
    writer.write(static_cast<u32>(1));           // one drop...
    writer.write(static_cast<u32>(0));           // ...entity 0 loses
    writer.write(static_cast<u16>(0));           // ...net.Position
    writer.write(static_cast<u32>(1));           // one change group...
    writer.write(static_cast<u32>(5));           // ...for entity 5,
    writer.write(static_cast<u16>(1));           // ...one component,
    writer.write(static_cast<u16>(0));           // ...net.Position — and then
                                                 // the message ends.

    net::ReplicationDecoder decoder(table);
    ser::BinaryReader reader(writer.bytes());
    bool threw = false;
    try
    {
        (void)decoder.apply(mirror, reader);
    }
    catch (const Exception&)
    {
        threw = true;
    }
    SW_CHECK(threw);

    SW_CHECK(fingerprint(mirror, table) == before);
    SW_CHECK(mirror.isAlive(doomed));
    SW_CHECK(!mirror.isAlive(ecs::Entity{5, 1}));
    SW_CHECK(mirror.hasComponent<NetPosition>(kept));
    SW_CHECK_EQ(mirror.getComponent<NetPosition>(kept).x, 1.0);
    // And the decoder still names the state the mirror really holds, so the
    // host's next delta is built on something that exists.
    SW_CHECK_EQ(decoder.appliedSnapshotId(), 0u);
    SW_CHECK_EQ(decoder.simulatedSeconds(), 0.0);

    // The other half of the fix, and the easy one to break: the same snapshot
    // with its last twenty-four bytes present is applied in full. Staging is
    // supposed to delay the work, not refuse it.
    ser::BinaryWriter complete;
    complete.writeBytes(writer.bytes().data(), writer.size());
    complete.write(static_cast<f64>(7.0));
    complete.write(static_cast<f64>(8.0));
    complete.write(static_cast<f64>(9.0));

    ser::BinaryReader good(complete.bytes());
    SW_CHECK(decoder.apply(mirror, good));
    SW_CHECK(!mirror.isAlive(doomed));
    SW_CHECK(mirror.isAlive(ecs::Entity{5, 1}));
    SW_CHECK(!mirror.hasComponent<NetPosition>(kept));
    SW_CHECK_EQ(mirror.getComponent<NetPosition>(ecs::Entity{5, 1}).y, 8.0);
    SW_CHECK_EQ(decoder.appliedSnapshotId(), 1u);
    SW_CHECK_EQ(decoder.simulatedSeconds(), 17.5);
}

SW_TEST(AMirrorRefusesAnEntityIndexItWouldHaveToGrowGigabytesFor)
{
    // Eight bytes of spawn record naming index 4,294,967,295 grew the slot
    // table to hold it: 24 bytes per slot, so a hundred gigabytes, so
    // std::bad_alloc, so a dead process.
    ecs::World mirror;
    bool threw = false;
    try
    {
        mirror.mirrorEntity(ecs::Entity{0xFFFFFFFFu, 1});
    }
    catch (const Exception&)
    {
        threw = true;
    }
    SW_CHECK(threw);
    SW_CHECK_EQ(mirror.recordCount(), 0u);

    // The bound is a ceiling, not a ban: an ordinary index still works.
    SW_CHECK(mirror.isAlive(mirror.mirrorEntity(ecs::Entity{4096, 1})));
}

SW_TEST(AFragmentNamingAnotherMessageIsRefusedRatherThanSpliced)
{
    // Protocol.cpp stored the header's messageId and never once read it, so
    // the tail of one fragmented message concatenated onto the head of
    // another and was handed up as a single believable payload.
    net::Connection receiver;
    const u8 flags = net::PacketHeader::kFlagReliable | net::PacketHeader::kFlagFragment;

    const std::vector<u8> first(net::kMaxFragmentPayload, 0xAA);
    const std::vector<u8> second(16, 0xBB);
    receiver.receive(0.0, datagram(net::MessageType::Snapshot, flags, 0, first, 7, 0, 2));
    SW_CHECK(receiver.delivered().empty());

    // Index 1 of two, as expected — but of message 8, not message 7.
    receiver.receive(0.0, datagram(net::MessageType::Snapshot, flags, 1, second, 8, 1, 2));
    SW_CHECK(receiver.delivered().empty());
    SW_CHECK(receiver.stats().fragmentsRejected > 0u);

    // And the half-message it was refused from is gone rather than waiting to
    // be finished by whatever arrives next: message 7's real tail, arriving
    // now, must not complete anything either.
    receiver.receive(0.0, datagram(net::MessageType::Snapshot, flags, 2, second, 7, 1, 2));
    SW_CHECK(receiver.delivered().empty());
}

SW_TEST(FragmentsOfAMessageThatNeverCompletesDoNotPinMemoryForever)
{
    // The old code cleared the partial buffer only on fragment index 0, so a
    // peer sending index 1 of three forever grew it by kMaxFragmentPayload
    // per datagram and never delivered anything. Nothing gave up, ever.
    net::Connection receiver;
    const u8 flags = net::PacketHeader::kFlagReliable | net::PacketHeader::kFlagFragment;
    const std::vector<u8> slice(net::kMaxFragmentPayload, 0xCD);

    constexpr u16 kFloodCount = 400;
    for (u16 i = 0; i < kFloodCount; ++i)
    {
        receiver.receive(0.0, datagram(net::MessageType::Snapshot, flags, i, slice, 3, 1, 3));
    }
    SW_CHECK(receiver.delivered().empty());
    // Every single one was refused: none of them was ever appended to
    // anything, so 400 datagrams bought the attacker no bytes at all.
    SW_CHECK_EQ(receiver.stats().fragmentsRejected, u64{kFloodCount});

    // A header announcing more fragments than the reassembly ceiling allows
    // is refused before the first one is stored.
    net::Connection other;
    other.receive(0.0, datagram(net::MessageType::Snapshot, flags, 0, slice, 1, 0, 0xFFFFu));
    SW_CHECK(other.delivered().empty());
    SW_CHECK_EQ(other.stats().malformedDropped, u64{1});

    // ...and the sender refuses to build one, so nothing legitimate can ever
    // hit the receiver's ceiling.
    net::Connection sender;
    const std::vector<u8> enormous(net::kMaxReassemblyBytes + 1, 0);
    bool threw = false;
    try
    {
        sender.queueReliable(net::MessageType::Snapshot, enormous);
    }
    catch (const Exception&)
    {
        threw = true;
    }
    SW_CHECK(threw);
}

SW_TEST(AConnectAttemptThatIgnoresTheChallengeCostsTheHostNothing)
{
    // THE AMPLIFICATION. One spoofed 24-byte ConnectRequest used to make the
    // host allocate a Peer and an encoder and then fragment the entire world
    // at an address that never asked for it, resending for ten seconds.
    // Measured against this build before the challenge: 5,926,514 bytes back
    // for 39 bytes in, a factor of 151,962.
    const save::Schema schema = makeNetSchema();
    net::LoopbackNetwork wire;
    net::Host host(wire.endpoint(7777), schema, makeNetSet());
    std::unique_ptr<net::Transport> spoofed = wire.endpoint(40501);
    const net::PeerAddress hostAddress = net::PeerAddress::localhost(7777);

    ecs::World hostWorld;
    for (int i = 0; i < 200; ++i)
    {
        const ecs::Entity entity = hostWorld.createEntity();
        hostWorld.addComponent(entity, NetPosition{static_cast<f64>(i), 0.0, 0.0});
    }

    ser::BinaryWriter request;
    request.write(net::kSessionVersion);
    request.write(static_cast<u64>(0)); // no cookie: nothing has been proven
    request.write(static_cast<u64>(0xC0FFEE)); // session nonce
    request.writeString("spoofed");
    const std::vector<u8> bytes =
        datagram(net::MessageType::ConnectRequest, net::PacketHeader::kFlagReliable, 0,
                 request.bytes());
    SW_CHECK(spoofed->send(hostAddress, bytes));

    usize received = 0;
    u64 datagrams = 0;
    u64 cookie = 0;
    std::vector<u8> buffer(net::kMaxDatagramBytes);
    f64 now = 0.0;
    for (int step = 0; step < 300; ++step) // three seconds of not answering
    {
        now += 0.01;
        wire.setTime(now);
        host.update(now, hostWorld, now);
        for (;;)
        {
            net::PeerAddress from{};
            const usize size = spoofed->receive(from, buffer);
            if (size == 0)
            {
                break;
            }
            received += size;
            ++datagrams;
            ser::BinaryReader reader(std::span<const u8>(buffer.data(), size));
            net::PacketHeader header{};
            SW_CHECK(net::readHeader(reader, header));
            SW_CHECK(header.type == net::MessageType::ConnectChallenge);
            cookie = reader.read<u64>();
        }
    }

    // One answer, and it is SMALLER than the question. There is nothing here
    // to amplify.
    SW_CHECK_EQ(datagrams, u64{1});
    SW_CHECK_EQ(received, usize{24});
    SW_CHECK(received < bytes.size());
    SW_CHECK_EQ(host.reception().challenged, u64{1});

    // And nothing was allocated for it: not a Peer, not an encoder, nothing
    // that survives the datagram. That is the whole point of a cookie.
    SW_CHECK_EQ(host.clients().size(), usize{0});
    SW_CHECK_EQ(host.clientCount(), u32{0});

    // Echoing the cookie back proves the address receives what is sent to it,
    // and only then does the world start moving.
    ser::BinaryWriter answer;
    answer.write(net::kSessionVersion);
    answer.write(cookie);
    answer.write(static_cast<u64>(0xC0FFEE)); // the same attempt as above
    answer.writeString("spoofed");
    SW_CHECK(spoofed->send(hostAddress,
                           datagram(net::MessageType::ConnectRequest,
                                    net::PacketHeader::kFlagReliable, 0, answer.bytes())));
    for (int step = 0; step < 20; ++step)
    {
        now += 0.01;
        wire.setTime(now);
        host.update(now, hostWorld, now);
    }
    SW_CHECK_EQ(host.clientCount(), u32{1});

    // A cookie for somebody else's address is worth nothing, which is what
    // stops a single captured cookie from reopening the hole for everyone.
    std::unique_ptr<net::Transport> elsewhere = wire.endpoint(40502);
    SW_CHECK(elsewhere->send(hostAddress,
                             datagram(net::MessageType::ConnectRequest,
                                      net::PacketHeader::kFlagReliable, 0,
                                      answer.bytes())));
    for (int step = 0; step < 20; ++step)
    {
        now += 0.01;
        wire.setTime(now);
        host.update(now, hostWorld, now);
    }
    SW_CHECK_EQ(host.clientCount(), u32{1});
    SW_CHECK_EQ(host.reception().challenged, u64{2});
}

SW_TEST(AClientCanJoinAgainWhileTheHostStillRemembersTheLastTime)
{
    // A Peer outlives its client by a timeout, so a player who quits and
    // comes straight back finds the host still holding the old conversation.
    // The client restarts its reliable channel at sequence zero; the host's
    // channel was already past zero, so it counted the request as a duplicate
    // and threw it away, and the client sat in Connecting until it timed out
    // fifteen seconds later. Nothing in either machine ever said why.
    const save::Schema schema = makeNetSchema();
    net::LoopbackNetwork wire;
    net::Host host(wire.endpoint(7777), schema, makeNetSet());
    net::Client client(wire.endpoint(40001), schema, "arthur");

    ecs::World hostWorld;
    ecs::World mirror;
    std::vector<ecs::Entity> entities;
    for (int i = 0; i < 6; ++i)
    {
        const ecs::Entity entity = hostWorld.createEntity();
        hostWorld.addComponent(entity, NetPosition{static_cast<f64>(i), 0.0, 0.0});
        entities.push_back(entity);
    }

    f64 now = 0.0;
    wire.setTime(now);
    const auto pump = [&](f64 seconds) {
        run(wire, now, seconds, [&](f64 t) { host.update(t, hostWorld, t); },
            [&](f64 t) { client.update(t, mirror, t); });
    };

    client.connect(net::PeerAddress::localhost(7777), now);
    pump(1.0);
    SW_CHECK(client.state() == net::ClientState::Connected);
    SW_CHECK_EQ(host.clientCount(), u32{1});
    const u32 firstId = client.clientId();

    // Leave, and give the host nowhere near long enough to forget: the Peer
    // is only swept on the timeout, which is ten seconds away.
    client.disconnect(now);
    pump(0.3);
    SW_CHECK(client.state() == net::ClientState::Disconnected);
    SW_CHECK_EQ(host.clientCount(), u32{0});
    SW_CHECK_EQ(host.clients().size(), usize{1}); // ...but it is still there

    // Straight back in. One round trip for the challenge, one for the
    // request; a second is generous.
    client.connect(net::PeerAddress::localhost(7777), now);
    pump(1.0);
    SW_CHECK(client.state() == net::ClientState::Connected);
    SW_CHECK_EQ(host.clientCount(), u32{1});
    SW_CHECK_EQ(host.clients().size(), usize{1}); // the stale one is gone
    // A new session is a new client, not a resurrection of the old one.
    SW_CHECK(client.clientId() != firstId);

    // And the world really arrives, which is the part that would still be
    // missing if the handshake completed on a channel nobody agreed about.
    pump(0.5);
    for (const ecs::Entity entity : entities)
    {
        SW_CHECK(mirror.isAlive(entity));
    }
    SW_CHECK_EQ(mirror.getComponent<NetPosition>(entities[4]).x, 4.0);
    SW_CHECK(client.appliedSnapshotId() > 0u);
}

SW_TEST(AnEventClaimingAHugePayloadDoesNotTakeTheHostDown)
{
    // The host side of the same class of bug: `event.payload.resize(size)`
    // with a size a client chose. The client below is real up to the moment
    // it stops being polite — the injected datagram carries the reliable
    // sequence the host is next expecting from that address, which after a
    // plain join is one.
    const save::Schema schema = makeNetSchema();
    net::LoopbackNetwork wire;
    net::Host host(wire.endpoint(7777), schema, makeNetSet());
    net::Client client(wire.endpoint(40601), schema, "arthur");
    std::unique_ptr<net::Transport> impostor = wire.endpoint(40601);

    ecs::World hostWorld;
    ecs::World mirror;
    hostWorld.addComponent(hostWorld.createEntity(), NetPosition{});

    f64 now = 0.0;
    wire.setTime(now);
    client.connect(net::PeerAddress::localhost(7777), now);
    run(wire, now, 0.5, [&](f64 t) { host.update(t, hostWorld, t); },
        [&](f64 t) { client.update(t, mirror, t); });
    SW_CHECK(client.state() == net::ClientState::Connected);

    ser::BinaryWriter payload;
    payload.write(static_cast<f64>(0.0));        // stamp
    payload.write(static_cast<u32>(0));          // claimed origin
    payload.write(static_cast<u32>(1));          // kind
    payload.write(static_cast<u32>(0xFFFFFFFFu)); // ...and 4 GB of payload
    SW_CHECK(impostor->send(net::PeerAddress::localhost(7777),
                            datagram(net::MessageType::Event,
                                     net::PacketHeader::kFlagReliable, 1,
                                     payload.bytes())));

    run(wire, now, 0.3, [&](f64 t) { host.update(t, hostWorld, t); },
        [&](f64 t) { client.update(t, mirror, t); });

    // Still serving, and the lie was not believed.
    SW_CHECK_EQ(host.clientCount(), u32{1});
    SW_CHECK_EQ(host.timeline().pendingCount(), 0u);
    // Refused by the arithmetic, not survived after the fact. Without the
    // check the resize succeeds — four gigabytes of resident memory, measured
    // — and the truncation two lines later reports the failure through the
    // parser's own catch, which counts nothing. So zero here would mean the
    // host really did allocate it.
    SW_CHECK_EQ(host.refusedMessages(), u64{1});
    SW_CHECK(client.state() == net::ClientState::Connected);
}

// ----------------------------------------------------------------------------
// The guards nothing was proving
// ----------------------------------------------------------------------------

SW_TEST(ASnapshotWritingToASlotTheMirrorDoesNotHoldIsRefused)
{
    // THE GUARD THAT WAS LOAD-BEARING AND UNTESTED. `aliveAfterwards` is what
    // stops a change group from naming an entity the mirror has never been
    // told about; without it the staged write reaches addComponentRaw on a
    // dead slot. That is not a caught exception, it is memory the mirror does
    // not own — an adversarial re-read mutated the predicate to `return true`
    // and the ENTIRE 220-test suite stayed green, then the mutated build
    // segfaulted on a 44-byte snapshot.
    //
    // SW_ASSERT does not cover this: it compiles out of RelWithDebInfo and
    // Release, which are the builds anyone actually plays. An assertion is
    // not a guard.
    const save::Schema schema = makeNetSchema();
    const net::ReplicationTable table = net::ReplicationTable::build(schema, makeNetSet());

    // Three separate ways for a slot to be absent, because the predicate has
    // three branches and only one of them is the obvious one.
    struct Case
    {
        const char* what;
        u32 removals;      // entity 1 removed, or not
        u32 target;        // which index the change group names
    };
    const Case cases[] = {
        {"never spawned at all", 0, 7},
        {"removed by this very snapshot", 1, 1},
        {"beyond anything the mirror ever held", 0, 4000},
    };

    for (const Case& test : cases)
    {
        ecs::World mirror;
        const ecs::Entity kept = mirror.createEntity();   // index 0
        mirror.addComponent(kept, NetPosition{1.0, 2.0, 3.0});
        const ecs::Entity doomed = mirror.createEntity(); // index 1
        mirror.addComponent(doomed, NetPosition{4.0, 5.0, 6.0});

        ser::BinaryWriter writer;
        writer.write(static_cast<u32>(0x53575250u));
        writer.write(static_cast<u32>(1));
        writer.write(static_cast<u32>(0));
        writer.write(static_cast<f64>(3.0));
        writer.write(test.removals);
        if (test.removals == 1) { writer.write(static_cast<u32>(1)); }
        writer.write(static_cast<u32>(0));   // no spawns — this is the point
        writer.write(static_cast<u32>(0));   // no drops
        writer.write(static_cast<u32>(1));   // one change group...
        writer.write(test.target);           // ...naming a slot we do not hold
        writer.write(static_cast<u16>(1));
        writer.write(static_cast<u16>(0));   // net.Position...
        // ...and the bytes ARE present. Without the predicate the decoder has
        // everything it needs to go through with the write, which is exactly
        // why a bound on lengths alone does not save it.
        writer.write(static_cast<f64>(11.0));
        writer.write(static_cast<f64>(12.0));
        writer.write(static_cast<f64>(13.0));

        const std::vector<u8> before = fingerprint(mirror, table);
        net::ReplicationDecoder decoder(table);
        ser::BinaryReader reader(writer.bytes());
        bool threw = false;
        std::string message;
        try
        {
            (void)decoder.apply(mirror, reader);
        }
        catch (const Exception& e)
        {
            threw = true;
            message = e.what();
        }
        SW_CHECK(threw);
        // Named, so a failure says which rule refused and about which slot.
        SW_CHECK(message.find("which the mirror does not have") != std::string::npos);
        SW_CHECK(fingerprint(mirror, table) == before);
        SW_CHECK_EQ(decoder.appliedSnapshotId(), 0u);
        // The removal in case 2 must NOT have been applied either: a refused
        // snapshot is refused whole.
        SW_CHECK(mirror.isAlive(doomed));
    }

    // ...and the same shape with a spawn present IS accepted, so the test is
    // about the predicate and not about change groups in general.
    {
        ecs::World mirror;
        ser::BinaryWriter writer;
        writer.write(static_cast<u32>(0x53575250u));
        writer.write(static_cast<u32>(1));
        writer.write(static_cast<u32>(0));
        writer.write(static_cast<f64>(3.0));
        writer.write(static_cast<u32>(0));   // no removals
        writer.write(static_cast<u32>(1));   // one spawn...
        writer.write(static_cast<u32>(7));   // ...entity 7
        writer.write(static_cast<u32>(1));
        writer.write(static_cast<u32>(0));   // no drops
        writer.write(static_cast<u32>(1));   // ...and now the write lands
        writer.write(static_cast<u32>(7));
        writer.write(static_cast<u16>(1));
        writer.write(static_cast<u16>(0));
        writer.write(static_cast<f64>(11.0));
        writer.write(static_cast<f64>(12.0));
        writer.write(static_cast<f64>(13.0));

        net::ReplicationDecoder decoder(table);
        ser::BinaryReader reader(writer.bytes());
        SW_CHECK(decoder.apply(mirror, reader));
        SW_CHECK(mirror.isAlive(ecs::Entity{7, 1}));
        SW_CHECK_EQ(mirror.getComponent<NetPosition>(ecs::Entity{7, 1}).x, 11.0);
    }
}

SW_TEST(ASpoofedRejoinDoesNotDisturbAPlayingClient)
{
    // The stale-peer path exists so a client that crashed can come back on
    // the same port. That same path is a weapon if it acts on an UNPROVEN
    // address, and the cookie check is what closes it.
    //
    // WHAT THE ATTACK ACTUALLY DOES, measured rather than assumed. It does
    // not disconnect the victim: with the check removed the host tears the
    // peer down and rebuilds it, and the victim's own reliable channel
    // re-establishes, so it is still Connected 34 s later. What it DOES do is
    // make the host announce a departure for a client that never left —
    // `departed()` names the id, and gameplay code holding that id drops
    // whatever it was tracking. That announcement is the discriminator here,
    // and it is why this test watches every update instead of only the last:
    // `departed()` is cleared at the top of each one, so a spurious departure
    // is invisible to anyone who looks afterwards.
    //
    // The attacker's difficulty is worth stating: the challenge goes to the
    // ADDRESS IN THE PACKET, which for a spoofed source is the victim — who
    // is Connected and ignores it — so the forger never learns the cookie.
    const save::Schema schema = makeNetSchema();

    net::LoopbackNetwork wire;
    net::Host host(wire.endpoint(7777), schema, makeNetSet());
    net::Client client(wire.endpoint(40001), schema, "arthur");

    ecs::World hostWorld;
    ecs::World mirror;
    const ecs::Entity entity = hostWorld.createEntity();
    hostWorld.addComponent(entity, NetPosition{1.0, 2.0, 3.0});

    f64 now = 0.0;
    client.connect(net::PeerAddress{0x7F000001u, 7777}, now);
    run(wire, now, 2.0, [&](f64 t) { host.update(t, hostWorld, t); },
        [&](f64 t) { client.update(t, mirror, t); });
    SW_CHECK_EQ(client.state(), net::ClientState::Connected);
    SW_CHECK_EQ(host.clientCount(), 1u);
    const u32 establishedId = client.clientId();
    const u64 challengedBefore = host.reception().challenged;

    // A whole, well-formed ConnectRequest carrying a nonce the host has never
    // seen and a cookie the attacker had to guess, arriving FROM THE VICTIM'S
    // OWN ADDRESS. On this wire a port is a label and nothing is bound for
    // real, so a second endpoint on the client's port IS a spoofed source —
    // the whole threat, without needing a second machine.
    std::unique_ptr<net::Transport> spoofer = wire.endpoint(40001);
    ser::BinaryWriter payload;
    payload.write(net::kSessionVersion);
    payload.write(static_cast<u64>(0));          // a wrong cookie
    payload.write(static_cast<u64>(0xDEADBEEF)); // a nonce the host never saw
    payload.writeString("attacker");
    std::vector<u8> datagram;
    net::buildBareDatagram(net::MessageType::ConnectRequest, payload.bytes(), datagram);
    SW_CHECK(spoofer->send(net::PeerAddress{0x7F000001u, 7777}, datagram));

    bool anyDeparture = false;
    for (int step = 0; step < 200; ++step)
    {
        now += 0.01;
        wire.setTime(now);
        host.update(now, hostWorld, now);
        if (!host.departed().empty())
        {
            anyDeparture = true;
        }
        client.update(now, mirror, now);
    }

    // THE ASSERTION THAT DISCRIMINATES: nobody left.
    SW_CHECK(!anyDeparture);
    // ...because the host answered the unproven address with a challenge
    // instead of acting on it. Exactly one: the forgery was one datagram.
    SW_CHECK_EQ(host.reception().challenged - challengedBefore, u64{1});
    // And the session is untouched.
    SW_CHECK_EQ(client.state(), net::ClientState::Connected);
    SW_CHECK_EQ(client.clientId(), establishedId);
    SW_CHECK_EQ(host.clientCount(), 1u);
    SW_CHECK_EQ(host.clients().size(), usize{1});
}

SW_TEST(RejoiningForgetsTheWorldTheLastSessionLeftBehind)
{
    // A reconnect rebuilds the decoder, the roster and the connection — but
    // the mirror is the caller's, so it used to survive. The re-accepted peer
    // gets a fresh encoder and therefore sends a FULL snapshot: all spawns,
    // NO removals. A removal is the only way an entity ever leaves a mirror,
    // so anything the host destroyed while the client was away stayed alive
    // in it for ever, and every later delta was diffed against a baseline the
    // mirror did not hold.
    const save::Schema schema = makeNetSchema();

    net::LoopbackNetwork wire;
    net::Host host(wire.endpoint(7777), schema, makeNetSet());
    net::Client client(wire.endpoint(40001), schema, "arthur");

    ecs::World hostWorld;
    ecs::World mirror;
    const ecs::Entity survivor = hostWorld.createEntity();
    hostWorld.addComponent(survivor, NetPosition{1.0, 2.0, 3.0});
    const ecs::Entity condemned = hostWorld.createEntity();
    hostWorld.addComponent(condemned, NetPosition{7.0, 8.0, 9.0});

    f64 now = 0.0;
    client.connect(net::PeerAddress{0x7F000001u, 7777}, now);
    run(wire, now, 2.0, [&](f64 t) { host.update(t, hostWorld, t); },
        [&](f64 t) { client.update(t, mirror, t); });
    SW_CHECK_EQ(client.state(), net::ClientState::Connected);
    SW_CHECK(mirror.isAlive(condemned));

    // Away — and while away, the host destroys one of them. The index is NOT
    // reused, which is the case that bites: a recycled index would be
    // overwritten by the full snapshot and hide the bug.
    client.disconnect(now);
    run(wire, now, 0.5, [&](f64 t) { host.update(t, hostWorld, t); }, [](f64) {});
    hostWorld.destroyEntity(condemned);

    client.connect(net::PeerAddress{0x7F000001u, 7777}, now);
    run(wire, now, 3.0, [&](f64 t) { host.update(t, hostWorld, t); },
        [&](f64 t) { client.update(t, mirror, t); });

    SW_CHECK_EQ(client.state(), net::ClientState::Connected);
    // The survivor came back...
    SW_CHECK(mirror.isAlive(survivor));
    SW_CHECK_EQ(mirror.getComponent<NetPosition>(survivor).x, 1.0);
    // ...and the ghost did not.
    SW_CHECK(!mirror.isAlive(condemned));
    // Exactly the host's population, nothing left over from before.
    SW_CHECK_EQ(mirror.aliveCount(), hostWorld.aliveCount());
}
