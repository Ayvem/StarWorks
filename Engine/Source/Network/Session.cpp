#include "Network/Session.hpp"

#include "Core/Log.hpp"
#include "ECS/World.hpp"

#include <algorithm>
#include <cstring>

namespace sw::net
{
    std::string_view clientStateName(ClientState state)
    {
        switch (state)
        {
            case ClientState::Connecting: return "Connecting";
            case ClientState::Connected: return "Connected";
            case ClientState::Rejected: return "Rejected";
            case ClientState::TimedOut: return "TimedOut";
            case ClientState::Disconnected:
            default: return "Disconnected";
        }
    }

    // ------------------------------------------------------------------------
    // Host
    // ------------------------------------------------------------------------

    Host::Host(std::unique_ptr<Transport> transport, const save::Schema& schema,
               ReplicationSet set, Config config)
        : m_transport(std::move(transport))
        , m_schema(schema)
        , m_table(ReplicationTable::build(schema, set))
        , m_config(config)
    {
        m_receiveBuffer.resize(kMaxDatagramBytes);
        SW_LOG_INFO("Network", "Host listening on port {} — {} replicated components",
                    m_transport->localPort(), m_table.size());
    }

    Host::Host(std::unique_ptr<Transport> transport, const save::Schema& schema,
               ReplicationSet set)
        : Host(std::move(transport), schema, std::move(set), Config{})
    {
    }

    Host::Peer* Host::findPeer(const PeerAddress& address)
    {
        for (const auto& peer : m_peers)
        {
            if (peer->address == address)
            {
                return peer.get();
            }
        }
        return nullptr;
    }

    void Host::update(f64 nowSeconds, const ecs::World& world, f64 simulatedSeconds)
    {
        m_commands.clear();
        m_departed.clear();
        m_localClock = simulatedSeconds;

        // ---- read ----------------------------------------------------------
        PeerAddress from{};
        for (;;)
        {
            const usize size = m_transport->receive(from, m_receiveBuffer);
            if (size == 0)
            {
                break;
            }

            ++m_reception.arrived;

            Peer* peer = findPeer(from);
            if (peer == nullptr)
            {
                ++m_reception.fromStrangers;

                // An unknown address only earns a Connection if it is trying
                // to connect and there is room. Anything else is dropped
                // without allocating, so a flood of noise on the port costs
                // one address comparison per datagram and nothing more.
                //
                // Dropping it SILENTLY, though, is what left "no reply"
                // ambiguous for the machine at the other end. Before
                // dropping, look at the two fields that explain it — read by
                // hand rather than through readHeader, because the whole
                // point is to see the values readHeader rejected.
                ser::BinaryReader probe(std::span<const u8>(m_receiveBuffer.data(), size));
                PacketHeader header{};
                if (!readHeader(probe, header) ||
                    header.type != MessageType::ConnectRequest ||
                    m_peers.size() >= m_config.maxClients)
                {
                    ++m_reception.refused;
                    if (size >= 6)
                    {
                        u32 magic = 0;
                        u16 version = 0;
                        std::memcpy(&magic, m_receiveBuffer.data(), sizeof(magic));
                        std::memcpy(&version, m_receiveBuffer.data() + 4, sizeof(version));
                        if (magic != kProtocolMagic)
                        {
                            ++m_reception.notOurs;
                        }
                        else if (version != kProtocolVersion)
                        {
                            ++m_reception.wrongVersion;
                            if (m_reception.lastForeignVersion != version)
                            {
                                m_reception.lastForeignVersion = version;
                                SW_LOG_ERROR("Network",
                                             "{} speaks protocol version {}, this build speaks "
                                             "{} — the two machines are running different "
                                             "builds of the game",
                                             from.toString(), version, kProtocolVersion);
                            }
                        }
                    }
                    continue;
                }
                SW_LOG_INFO("Network", "Connection attempt from {}", from.toString());
                auto created = std::make_unique<Peer>(m_config.connection);
                created->address = from;
                created->connection.markAlive(nowSeconds);
                peer = created.get();
                m_peers.push_back(std::move(created));
            }

            peer->connection.receive(nowSeconds,
                                     std::span<const u8>(m_receiveBuffer.data(), size));
        }

        // ---- act -----------------------------------------------------------
        for (const auto& peer : m_peers)
        {
            handleMessages(nowSeconds, *peer, world, simulatedSeconds);
        }

        // ---- snapshot beat ---------------------------------------------------
        const f64 interval = (m_config.snapshotHz > 0.0) ? 1.0 / m_config.snapshotHz : 0.0;
        for (const auto& peer : m_peers)
        {
            if (peer->accepted && nowSeconds - peer->lastSnapshotAt >= interval)
            {
                pushSnapshot(nowSeconds, *peer, world, simulatedSeconds);
            }
        }

        // ---- the roster of everyone's clocks -----------------------------
        m_roster.clear();
        m_roster.push_back(PlayerView{0, m_config.hostName, m_localClock, 0.0});
        for (const auto& peer : m_peers)
        {
            if (peer->accepted)
            {
                m_roster.push_back(PlayerView{peer->id, peer->name,
                                              peer->clockKnown ? peer->clockSeconds
                                                               : m_localClock,
                                              peer->connection.roundTripSeconds()});
            }
        }

        const f64 rosterInterval = (m_config.rosterHz > 0.0) ? 1.0 / m_config.rosterHz : 0.0;
        if (m_lastRosterAt < 0.0 || nowSeconds - m_lastRosterAt >= rosterInterval)
        {
            m_lastRosterAt = nowSeconds;
            ser::BinaryWriter writer;
            writer.write(static_cast<u32>(m_roster.size()));
            for (const PlayerView& player : m_roster)
            {
                writer.write(player.id);
                writer.writeString(player.name);
                writer.write(player.simulatedSeconds);
            }
            if (writer.size() <= kMaxUnreliablePayload)
            {
                for (const auto& peer : m_peers)
                {
                    if (peer->accepted)
                    {
                        peer->connection.queueUnreliable(MessageType::Roster,
                                                         writer.bytes());
                    }
                }
            }
        }

        // ---- write, then bury the dead -----------------------------------
        flush(nowSeconds);

        for (auto it = m_peers.begin(); it != m_peers.end();)
        {
            if ((*it)->connection.hasTimedOut(nowSeconds))
            {
                SW_LOG_INFO("Network", "Client {} ('{}') timed out", (*it)->id, (*it)->name);
                if ((*it)->accepted)
                {
                    m_departed.push_back((*it)->id);
                }
                it = m_peers.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    void Host::handleMessages(f64 nowSeconds, Peer& peer, const ecs::World& world,
                              f64 simulatedSeconds)
    {
        bool leaving = false;
        for (const Message& message : peer.connection.delivered())
        {
            switch (message.type)
            {
                case MessageType::ConnectRequest:
                {
                    if (peer.accepted)
                    {
                        break; // a duplicate request; the accept is on its way
                    }
                    ser::BinaryReader reader(message.payload);
                    u32 version = 0;
                    std::string name;
                    try
                    {
                        version = reader.read<u32>();
                        name = reader.readString();
                    }
                    catch (const Exception&)
                    {
                        version = 0;
                    }

                    if (version != kSessionVersion)
                    {
                        ser::BinaryWriter writer;
                        writer.writeString(std::format(
                            "Session version {} does not match the host's {}", version,
                            kSessionVersion));
                        peer.connection.queueReliable(MessageType::ConnectReject,
                                                      writer.bytes());
                        break;
                    }
                    peer.name = name.empty() ? std::string("pilot") : name;
                    acceptPeer(nowSeconds, peer, world, simulatedSeconds);
                    break;
                }
                case MessageType::SnapshotAck:
                {
                    if (peer.encoder == nullptr)
                    {
                        break;
                    }
                    ser::BinaryReader reader(message.payload);
                    try
                    {
                        peer.encoder->acknowledge(reader.read<u32>());
                    }
                    catch (const Exception&)
                    {
                    }
                    break;
                }
                case MessageType::Command:
                {
                    if (peer.accepted)
                    {
                        m_commands.push_back(CommandEvent{peer.id, message.payload});
                    }
                    break;
                }
                case MessageType::Clock:
                {
                    ser::BinaryReader reader(message.payload);
                    try
                    {
                        peer.clockSeconds = reader.read<f64>();
                        peer.clockKnown = true;
                    }
                    catch (const Exception&)
                    {
                    }
                    break;
                }
                case MessageType::Event:
                {
                    if (!peer.accepted)
                    {
                        break;
                    }
                    TimelineEvent event{};
                    try
                    {
                        ser::BinaryReader reader(message.payload);
                        event.stampSeconds = reader.read<f64>();
                        (void)reader.read<u32>(); // origin as claimed; we know better
                        event.kind = reader.read<u32>();
                        const u32 size = reader.read<u32>();
                        event.payload.resize(size);
                        if (size > 0)
                        {
                            reader.readBytes(event.payload.data(), size);
                        }
                    }
                    catch (const Exception&)
                    {
                        break;
                    }
                    // The origin is whoever the datagram actually came from,
                    // not whoever the payload says: a client cannot speak in
                    // another player's name.
                    event.originClientId = peer.id;

                    // Relay to every OTHER player, then to ourselves. Each
                    // of them holds it until their own clock reaches its
                    // instant; that is the entire model.
                    ser::BinaryWriter writer;
                    writer.write(event.stampSeconds);
                    writer.write(event.originClientId);
                    writer.write(event.kind);
                    writer.write(static_cast<u32>(event.payload.size()));
                    writer.writeBytes(event.payload.data(), event.payload.size());
                    for (const auto& other : m_peers)
                    {
                        if (other->accepted && other->id != peer.id)
                        {
                            other->connection.queueReliable(MessageType::Event,
                                                            writer.bytes());
                        }
                    }
                    m_timeline.push(std::move(event));
                    break;
                }
                case MessageType::Disconnect:
                    leaving = true;
                    break;
                default:
                    break;
            }
        }
        peer.connection.clearDelivered();

        if (leaving)
        {
            SW_LOG_INFO("Network", "Client {} ('{}') left", peer.id, peer.name);
            if (peer.accepted)
            {
                m_departed.push_back(peer.id);
            }
            peer.accepted = false;
            // Removal happens on the timeout sweep so the goodbye's own
            // acknowledgement still has somewhere to land.
        }
    }

    void Host::acceptPeer(f64 nowSeconds, Peer& peer, const ecs::World& world,
                          f64 simulatedSeconds)
    {
        peer.id = m_nextClientId++;
        peer.accepted = true;
        peer.encoder = std::make_unique<ReplicationEncoder>(m_table);

        ser::BinaryWriter writer;
        writer.write(kSessionVersion);
        writer.write(peer.id);
        writer.write(simulatedSeconds);
        m_table.write(writer);
        peer.connection.queueReliable(MessageType::ConnectAccept, writer.bytes());

        SW_LOG_INFO("Network", "Client {} ('{}') accepted from {}", peer.id, peer.name,
                    peer.address.toString());

        // The first snapshot goes out immediately rather than on the next
        // beat: a client that has been accepted but has no world yet has
        // nothing to draw, and the beat is fifty milliseconds of that.
        pushSnapshot(nowSeconds, peer, world, simulatedSeconds);
    }

    void Host::pushSnapshot(f64 nowSeconds, Peer& peer, const ecs::World& world,
                            f64 simulatedSeconds)
    {
        peer.lastSnapshotAt = nowSeconds;
        ser::BinaryWriter writer;
        peer.encoder->encode(world, simulatedSeconds, writer);
        peer.lastSnapshot = peer.encoder->lastStats();

        // A delta that fits one datagram goes UNRELIABLE, because a lost one
        // is superseded by the next. One that does not fit — the full state
        // at join, or the aftermath of something enormous — goes reliable,
        // where it is fragmented and resent until it lands. The decoder's
        // baseline check makes the two safe to mix: a snapshot that arrives
        // after a newer one is refused rather than applied backwards.
        if (writer.size() <= kMaxUnreliablePayload)
        {
            peer.connection.queueUnreliable(MessageType::Snapshot, writer.bytes());
        }
        else
        {
            peer.connection.queueReliable(MessageType::Snapshot, writer.bytes());
        }
    }

    void Host::broadcastEvent(f64 stampSeconds, u32 kind, std::span<const u8> payload)
    {
        ser::BinaryWriter writer;
        writer.write(stampSeconds);
        writer.write(static_cast<u32>(0)); // origin: the host
        writer.write(kind);
        writer.write(static_cast<u32>(payload.size()));
        writer.writeBytes(payload.data(), payload.size());
        for (const auto& peer : m_peers)
        {
            if (peer->accepted)
            {
                peer->connection.queueReliable(MessageType::Event, writer.bytes());
            }
        }
    }

    void Host::flush(f64 nowSeconds)
    {
        for (const auto& peer : m_peers)
        {
            m_outgoing.clear();
            peer->connection.collectOutgoing(nowSeconds, m_outgoing);
            for (const std::vector<u8>& datagram : m_outgoing)
            {
                m_transport->send(peer->address, datagram);
            }
        }
    }

    std::vector<Host::ClientView> Host::clients() const
    {
        std::vector<ClientView> views;
        views.reserve(m_peers.size());
        for (const auto& peer : m_peers)
        {
            ClientView view{};
            view.id = peer->id;
            view.name = peer->name;
            view.address = peer->address;
            view.accepted = peer->accepted;
            view.roundTripSeconds = peer->connection.roundTripSeconds();
            view.lastSnapshotId =
                (peer->encoder != nullptr) ? peer->encoder->lastSnapshotId() : 0;
            view.acknowledgedId =
                (peer->encoder != nullptr) ? peer->encoder->acknowledgedId() : 0;
            view.lastSnapshot = peer->lastSnapshot;
            view.connection = peer->connection.stats();
            views.push_back(std::move(view));
        }
        return views;
    }

    u32 Host::clientCount() const
    {
        u32 count = 0;
        for (const auto& peer : m_peers)
        {
            if (peer->accepted)
            {
                ++count;
            }
        }
        return count;
    }

    void Host::shutdown(f64 nowSeconds)
    {
        for (const auto& peer : m_peers)
        {
            peer->connection.queueReliable(MessageType::Disconnect, {});
        }
        flush(nowSeconds);
    }

    // ------------------------------------------------------------------------
    // Client
    // ------------------------------------------------------------------------

    Client::Client(std::unique_ptr<Transport> transport, const save::Schema& schema,
                   std::string playerName, Config config)
        : m_transport(std::move(transport))
        , m_schema(schema)
        , m_name(std::move(playerName))
        , m_config(config)
        , m_connection(config.connection)
    {
        m_receiveBuffer.resize(kMaxDatagramBytes);
    }

    Client::Client(std::unique_ptr<Transport> transport, const save::Schema& schema,
                   std::string playerName)
        : Client(std::move(transport), schema, std::move(playerName), Config{})
    {
    }

    void Client::connect(const PeerAddress& host, f64 nowSeconds)
    {
        m_host = host;
        m_state = ClientState::Connecting;
        m_rejectReason.clear();
        m_clientId = 0;
        m_rebased = 0;
        m_decoder.reset();
        m_timeline.clear();
        m_roster.clear();
        m_lastClockAt = -1.0;
        m_connection = Connection(m_config.connection);
        m_connection.markAlive(nowSeconds);

        ser::BinaryWriter writer;
        writer.write(kSessionVersion);
        writer.writeString(m_name);
        m_connection.queueReliable(MessageType::ConnectRequest, writer.bytes());
        m_lastRequestAt = nowSeconds;
        flush(nowSeconds);
    }

    void Client::disconnect(f64 nowSeconds)
    {
        if (m_state == ClientState::Connected || m_state == ClientState::Connecting)
        {
            m_connection.queueReliable(MessageType::Disconnect, {});
            flush(nowSeconds);
        }
        m_state = ClientState::Disconnected;
    }

    void Client::sendCommand(std::span<const u8> payload)
    {
        if (m_state == ClientState::Connected)
        {
            m_connection.queueReliable(MessageType::Command, payload);
        }
    }

    f64 Client::hostSimulatedSeconds() const
    {
        return (m_decoder != nullptr) ? m_decoder->simulatedSeconds() : 0.0;
    }

    u32 Client::appliedSnapshotId() const
    {
        return (m_decoder != nullptr) ? m_decoder->appliedSnapshotId() : 0;
    }

    void Client::sendEvent(f64 stampSeconds, u32 kind, std::span<const u8> payload)
    {
        if (m_state != ClientState::Connected)
        {
            return;
        }
        ser::BinaryWriter writer;
        writer.write(stampSeconds);
        writer.write(m_clientId);
        writer.write(kind);
        writer.write(static_cast<u32>(payload.size()));
        writer.writeBytes(payload.data(), payload.size());
        m_connection.queueReliable(MessageType::Event, writer.bytes());
    }

    void Client::sendClock(f64 nowSeconds)
    {
        ser::BinaryWriter writer;
        writer.write(m_localClock);
        m_connection.queueUnreliable(MessageType::Clock, writer.bytes());
        m_lastClockAt = nowSeconds;
    }

    void Client::update(f64 nowSeconds, ecs::World& mirror, f64 simulatedSeconds)
    {
        m_localClock = simulatedSeconds;
        if (m_state != ClientState::Connecting && m_state != ClientState::Connected)
        {
            return;
        }

        PeerAddress from{};
        for (;;)
        {
            const usize size = m_transport->receive(from, m_receiveBuffer);
            if (size == 0)
            {
                break;
            }
            if (!(from == m_host))
            {
                continue; // not the host we asked; ignore it entirely
            }
            m_connection.receive(nowSeconds,
                                 std::span<const u8>(m_receiveBuffer.data(), size));
        }

        bool applied = false;
        for (const Message& message : m_connection.delivered())
        {
            switch (message.type)
            {
                case MessageType::ConnectAccept:
                {
                    ser::BinaryReader reader(message.payload);
                    const u32 version = reader.read<u32>();
                    if (version != kSessionVersion)
                    {
                        m_state = ClientState::Rejected;
                        m_rejectReason =
                            std::format("Host speaks session version {}", version);
                        break;
                    }
                    m_clientId = reader.read<u32>();
                    (void)reader.read<f64>(); // host clock at accept
                    m_decoder = std::make_unique<ReplicationDecoder>(
                        ReplicationTable::read(reader, m_schema));
                    m_state = ClientState::Connected;
                    SW_LOG_INFO("Network", "Connected to {} as client {}",
                                m_host.toString(), m_clientId);
                    break;
                }
                case MessageType::ConnectReject:
                {
                    ser::BinaryReader reader(message.payload);
                    m_rejectReason = reader.readString();
                    m_state = ClientState::Rejected;
                    break;
                }
                case MessageType::Snapshot:
                {
                    if (m_decoder == nullptr)
                    {
                        break;
                    }
                    ser::BinaryReader reader(message.payload);
                    if (m_decoder->apply(mirror, reader))
                    {
                        applied = true;
                    }
                    else
                    {
                        ++m_rebased;
                    }
                    break;
                }
                case MessageType::Roster:
                {
                    ser::BinaryReader reader(message.payload);
                    try
                    {
                        const u32 count = reader.read<u32>();
                        std::vector<PlayerView> roster;
                        roster.reserve(count);
                        for (u32 i = 0; i < count; ++i)
                        {
                            PlayerView player{};
                            player.id = reader.read<u32>();
                            player.name = reader.readString();
                            player.simulatedSeconds = reader.read<f64>();
                            roster.push_back(std::move(player));
                        }
                        m_roster = std::move(roster);
                    }
                    catch (const Exception&)
                    {
                    }
                    break;
                }
                case MessageType::Event:
                {
                    TimelineEvent event{};
                    try
                    {
                        ser::BinaryReader reader(message.payload);
                        event.stampSeconds = reader.read<f64>();
                        event.originClientId = reader.read<u32>();
                        event.kind = reader.read<u32>();
                        const u32 size = reader.read<u32>();
                        event.payload.resize(size);
                        if (size > 0)
                        {
                            reader.readBytes(event.payload.data(), size);
                        }
                    }
                    catch (const Exception&)
                    {
                        break;
                    }
                    m_timeline.push(std::move(event));
                    break;
                }
                case MessageType::Disconnect:
                    m_state = ClientState::Disconnected;
                    break;
                default:
                    break;
            }
        }
        m_connection.clearDelivered();

        if (m_state == ClientState::Connecting && m_lastRequestAt >= 0.0 &&
            nowSeconds - m_lastRequestAt >= m_config.connectRetrySeconds)
        {
            // The reliable channel is already resending the request; this
            // only refreshes the "still trying" timestamp so the retry log
            // and the timeout below agree about how long it has been.
            m_lastRequestAt = nowSeconds;
        }

        if (m_state == ClientState::Connected)
        {
            const f64 interval =
                (m_config.acknowledgeHz > 0.0) ? 1.0 / m_config.acknowledgeHz : 0.0;
            if (applied || m_lastAckAt < 0.0 || nowSeconds - m_lastAckAt >= interval)
            {
                sendAcknowledgement(nowSeconds);
            }

            const f64 clockInterval =
                (m_config.clockHz > 0.0) ? 1.0 / m_config.clockHz : 0.0;
            if (m_lastClockAt < 0.0 || nowSeconds - m_lastClockAt >= clockInterval)
            {
                sendClock(nowSeconds);
            }
        }

        flush(nowSeconds);

        if (m_connection.hasTimedOut(nowSeconds))
        {
            m_state = ClientState::TimedOut;
        }
    }

    void Client::sendAcknowledgement(f64 nowSeconds)
    {
        ser::BinaryWriter writer;
        writer.write(appliedSnapshotId());
        m_connection.queueUnreliable(MessageType::SnapshotAck, writer.bytes());
        m_lastAckAt = nowSeconds;
    }

    void Client::flush(f64 nowSeconds)
    {
        m_outgoing.clear();
        m_connection.collectOutgoing(nowSeconds, m_outgoing);
        for (const std::vector<u8>& datagram : m_outgoing)
        {
            m_transport->send(m_host, datagram);
        }
    }
} // namespace sw::net
