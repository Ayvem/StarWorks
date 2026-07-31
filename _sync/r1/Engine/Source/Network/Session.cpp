#include "Network/Session.hpp"

#include "Core/Log.hpp"
#include "ECS/World.hpp"

#include <algorithm>
#include <cstring>
#include <random>

namespace sw::net
{
    namespace
    {
        /// splitmix64. Not a cryptographic MAC, and it does not need to be:
        /// the attacker sees no cookie at all (the challenge goes to the
        /// address being tested, which for a spoofed source is not the
        /// attacker), so what is required is unpredictability from the
        /// address alone, not resistance to known-plaintext analysis.
        [[nodiscard]] u64 mix(u64 value)
        {
            value += 0x9E3779B97F4A7C15ull;
            value = (value ^ (value >> 30)) * 0xBF58476D1CE4E5B9ull;
            value = (value ^ (value >> 27)) * 0x94D049BB133111EBull;
            return value ^ (value >> 31);
        }

        /// Names one attempt to join. Never zero: zero is what a Peer
        /// carries before it has seen a request at all, and a nonce that
        /// collided with that would make a genuine reconnect look like a
        /// retransmission of a request nobody sent.
        [[nodiscard]] u64 newSessionNonce()
        {
            std::random_device entropy;
            const u64 value = (static_cast<u64>(entropy()) << 32) ^ entropy();
            return (value == 0) ? 1ull : value;
        }

        /// The fixed head of a ConnectRequest, read straight out of a
        /// datagram with nothing allocated and nothing remembered.
        struct ConnectRequestHead
        {
            u32 sessionVersion = 0;
            u64 cookie = 0;
            u64 nonce = 0;
        };

        /// False for anything that is not a whole, well-formed
        /// ConnectRequest — including a fragment of one, which cannot be
        /// read without reassembly state and so is never a reconnect signal.
        [[nodiscard]] bool readConnectRequestHead(std::span<const u8> bytes,
                                                  ConnectRequestHead& out)
        {
            ser::BinaryReader reader(bytes);
            PacketHeader header{};
            if (!readHeader(reader, header) ||
                header.type != MessageType::ConnectRequest ||
                (header.flags & PacketHeader::kFlagFragment) != 0)
            {
                return false;
            }
            try
            {
                out.sessionVersion = reader.read<u32>();
                out.cookie = reader.read<u64>();
                out.nonce = reader.read<u64>();
            }
            catch (const Exception&)
            {
                return false;
            }
            return true;
        }
    } // namespace

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

        // The secret behind every cookie. std::random_device rather than the
        // clock: a secret an attacker can guess by knowing roughly when the
        // host started is not a secret, and the whole challenge rests on
        // nobody being able to compute a cookie for an address they cannot
        // receive on.
        std::random_device entropy;
        m_cookieSecret = (static_cast<u64>(entropy()) << 32) ^ entropy();
        m_cookieSecret = mix(m_cookieSecret ^ 0x5157'4F52'4B53'0001ull);

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

    u64 Host::cookieFor(const PeerAddress& address) const
    {
        // Both halves of the address go in. Binding only the IP would let a
        // machine that legitimately connected once mint cookies for every
        // port on itself, which is harmless here but costs nothing to close;
        // binding neither would make one captured cookie work for the whole
        // internet, which is the attack.
        const u64 packed = (static_cast<u64>(address.ipv4) << 16) | address.port;
        const u64 cookie = mix(m_cookieSecret ^ mix(packed));
        // Zero is the client's "I have not been challenged yet" value, so it
        // must never be a real cookie or an unchallenged request would look
        // like a correct answer.
        return (cookie == 0) ? 1u : cookie;
    }

    void Host::sendChallenge(const PeerAddress& to)
    {
        ser::BinaryWriter payload;
        payload.write(cookieFor(to));
        std::vector<u8> datagram;
        buildBareDatagram(MessageType::ConnectChallenge, payload.bytes(), datagram);
        m_transport->send(to, datagram);
        ++m_reception.challenged;
    }

    Host::Peer& Host::replacePeer(f64 nowSeconds, Peer& stale, u64 nonce)
    {
        SW_LOG_INFO("Network",
                    "{} is joining again; dropping the peer left over from client {} ('{}')",
                    stale.address.toString(), stale.id, stale.name);
        const PeerAddress address = stale.address;
        if (stale.accepted)
        {
            // The old client id is gone whether or not it ever said goodbye,
            // and gameplay code that is holding on to it has to be told —
            // the same way it is told about a timeout.
            m_departed.push_back(stale.id);
        }
        const auto it = std::find_if(m_peers.begin(), m_peers.end(),
                                     [&](const std::unique_ptr<Peer>& p) {
                                         return p.get() == &stale;
                                     });
        m_peers.erase(it); // `stale` is dangling from here on

        auto created = std::make_unique<Peer>(m_config.connection);
        created->address = address;
        created->sessionNonce = nonce;
        created->connection.markAlive(nowSeconds);
        Peer& fresh = *created;
        m_peers.push_back(std::move(created));
        return fresh;
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
                // to connect, there is room, AND it has already proved it can
                // receive what it asks for. Anything else is dropped without
                // allocating, so a flood of noise on the port costs one
                // address comparison per datagram and nothing more.
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
                    (header.flags & PacketHeader::kFlagFragment) != 0 ||
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

                // THE ONE THING THAT HAPPENS BEFORE ANY ALLOCATION. Read the
                // cookie straight out of the request's payload — no
                // Connection, no Peer, no reassembly buffer, nothing that
                // outlives this iteration. An address that cannot echo the
                // cookie back gets a 24-byte challenge and nothing else, so
                // the most a spoofed source can extract is one datagram the
                // same size as the one it sent.
                u64 offered = 0;
                u64 nonce = 0;
                bool readable = false;
                try
                {
                    (void)probe.read<u32>(); // session version, checked once accepted
                    offered = probe.read<u64>();
                    nonce = probe.read<u64>();
                    readable = true;
                }
                catch (const Exception&)
                {
                    readable = false;
                }
                if (!readable || offered != cookieFor(from))
                {
                    sendChallenge(from);
                    continue;
                }

                SW_LOG_INFO("Network", "Connection attempt from {}", from.toString());
                auto created = std::make_unique<Peer>(m_config.connection);
                created->address = from;
                created->sessionNonce = nonce;
                created->connection.markAlive(nowSeconds);
                peer = created.get();
                m_peers.push_back(std::move(created));
            }
            else
            {
                // A CONNECTREQUEST FROM AN ADDRESS WE ALREADY HAVE A PEER
                // FOR. Handled here, before the datagram reaches the reliable
                // channel, because the channel is exactly what cannot handle
                // it: the request is sequence zero of a conversation this
                // Peer's channel is long past, so it is discarded as a
                // duplicate and the client waits out its timeout in
                // Connecting. That is what made every reconnect fail, and it
                // needs the host to forget rather than the client to
                // remember — a client that has been away has no way to know
                // where the old conversation had got to.
                ConnectRequestHead head{};
                if (readConnectRequestHead(
                        std::span<const u8>(m_receiveBuffer.data(), size), head) &&
                    head.nonce != peer->sessionNonce)
                {
                    // Proof first. Without the cookie this would let anyone
                    // who can guess a playing client's address and port throw
                    // that client's connection away by spoofing one datagram;
                    // with it, the challenge goes to the real address, which
                    // ignores it while it is Connected, and the spoofer never
                    // sees the cookie it would need.
                    if (head.cookie != cookieFor(from))
                    {
                        sendChallenge(from);
                        continue;
                    }
                    peer = &replacePeer(nowSeconds, *peer, head.nonce);
                }
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
            // ONE MESSAGE MUST NOT BE ABLE TO END THE HOST. Every parser
            // below reads lengths a client chose, and a length that is a lie
            // throws — sw::Exception from our own bounds checks,
            // std::bad_alloc from the allocator when a bound is missed. Both
            // are caught here, at the boundary between "bytes somebody sent
            // us" and "facts we act on", so the cost of a hostile datagram is
            // exactly that datagram. Catching per message rather than per
            // update also means the other clients' traffic in the same batch
            // is still processed.
            try
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
                        (void)reader.read<u64>(); // cookie, already validated
                        (void)reader.read<u64>(); // nonce, already matched in update()
                        name = reader.readString();
                    }
                    catch (const Exception&)
                    {
                        version = 0;
                    }
                    if (name.size() > kMaxPlayerNameBytes)
                    {
                        name.resize(kMaxPlayerNameBytes);
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
                        // REJECTED HERE, NOT CAUGHT LATER. The size is four
                        // bytes a client chose and it reached resize()
                        // untouched, so a 36-byte Event claiming 0xFFFFFFFF
                        // bytes of payload made the host allocate AND
                        // zero-fill four gigabytes before the next read
                        // discovered the datagram was empty. MEASURED: that
                        // resize grows resident memory by 4,295,036,928 bytes
                        // for one 36-byte datagram, and it does not throw, so
                        // it cannot be caught — it is simply the host gone,
                        // at a cost to the attacker of one packet.
                        //
                        // Counting the refusal rather than letting the
                        // truncation below report it keeps the two distinct:
                        // a short datagram is damage, a length longer than
                        // the datagram is a lie.
                        if (size > reader.remaining())
                        {
                            ++m_refusedMessages;
                            SW_LOG_ERROR("Network",
                                         "Client {} ('{}') sent an Event claiming {} bytes "
                                         "of payload with {} left in the message",
                                         peer.id, peer.name, size, reader.remaining());
                            break;
                        }
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
            catch (const std::exception& error)
            {
                ++m_refusedMessages;
                SW_LOG_ERROR("Network", "Dropped a {} from client {} ('{}'): {}",
                             messageTypeName(message.type), peer.id, peer.name,
                             error.what());
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
        m_cookie = 0;
        // A NEW ATTEMPT, AND THE HOST HAS TO BE ABLE TO SEE THAT IT IS ONE.
        // Rolled here rather than in sendConnectRequest because answering a
        // challenge is still THIS attempt: a fresh nonce per datagram would
        // make the host tear its own Peer down and rebuild it on every
        // retransmission.
        m_sessionNonce = newSessionNonce();
        m_decoder.reset();
        m_timeline.clear();
        m_roster.clear();
        m_lastClockAt = -1.0;
        // AND THE MIRROR HAS TO BE FORGOTTEN TOO — but it cannot be forgotten
        // here, because this class does not own it: the world arrives as an
        // argument to update(). So the intent is recorded and acted on at the
        // first update that follows.
        //
        // THE GHOST THIS PREVENTS. Everything else about a reconnect is
        // rebuilt from nothing — decoder, roster, timeline, connection — and
        // the re-accepted peer gets a fresh encoder, so its first snapshot is
        // a FULL one: baseline 0, all spawns, no removals. A removal is the
        // only way an entity ever leaves a mirror, so anything the host
        // destroyed while this client was away, and did not recycle the index
        // of, would simply stay alive in the mirror for ever — and every
        // later delta would then be diffed against a baseline the mirror does
        // not actually hold.
        m_mirrorStale = true;
        sendConnectRequest(nowSeconds);
    }

    void Client::sendConnectRequest(f64 nowSeconds)
    {
        // The connection is rebuilt, not added to. The first request went out
        // with no cookie and the host will never accept it; leaving it at the
        // head of the reliable channel would make the host reassemble a
        // conversation that starts with a message it must refuse. Starting
        // the channel again at sequence zero means the first thing the host
        // ever delivers up is the request it can actually act on.
        //
        // Which is only safe because the request names the attempt it belongs
        // to. Restarting the channel is unilateral: a host that still holds a
        // Peer for this address is past sequence zero and threw the request
        // away as a duplicate, so every reconnect timed out. The nonce is
        // what lets that host recognise a new session and start a channel of
        // its own at zero to match.
        m_connection = Connection(m_config.connection);
        m_connection.markAlive(nowSeconds);

        ser::BinaryWriter writer;
        writer.write(kSessionVersion);
        writer.write(m_cookie);
        writer.write(m_sessionNonce);
        writer.writeString(std::string_view(m_name).substr(
            0, std::min(m_name.size(), kMaxPlayerNameBytes)));
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

        // The world connect() could not reach. Done before a single datagram
        // is read, so no snapshot can ever be applied on top of the previous
        // session's leftovers. clearForRestore rather than a fresh World: the
        // caller owns this object and may hold references to it.
        if (m_mirrorStale)
        {
            m_mirrorStale = false;
            mirror.clearForRestore();
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
        bool rechallenged = false;
        for (const Message& message : m_connection.delivered())
        {
            // A CORRUPTED DATAGRAM FROM THE HOST'S ADDRESS USED TO KILL THE
            // CLIENT. Everything below parses lengths that came off the wire
            // — the ConnectAccept's component table, a snapshot's entity and
            // component counts — and the parsers throw by design, because a
            // bounds check that returns a value nobody looks at is not a
            // bounds check. None of that was caught here, so one flipped bit
            // on the path (or one hostile packet from anyone who can guess
            // the host's address and port) terminated the process.
            //
            // std::exception, not sw::Exception: the counts drive resize and
            // reserve, and the failure a hostile length produces at the
            // allocator is std::bad_alloc, which is not one of ours. Missing
            // it is exactly how the old catch blocks failed.
            try
            {
            switch (message.type)
            {
                case MessageType::ConnectChallenge:
                {
                    // The host will not spend a byte on us until we prove we
                    // can receive what it sends. Echo the cookie back and let
                    // the handshake start again from sequence zero.
                    if (m_state != ClientState::Connecting)
                    {
                        break;
                    }
                    ser::BinaryReader reader(message.payload);
                    const u64 cookie = reader.read<u64>();
                    if (cookie == 0 || cookie == m_cookie || rechallenged)
                    {
                        // Same cookie again: our answer is already in flight
                        // and being resent by the reliable channel. Rebuilding
                        // the connection for it would throw that answer away
                        // and restart the handshake forever.
                        break;
                    }
                    m_cookie = cookie;
                    rechallenged = true;
                    break;
                }
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
                        // REJECTED HERE, NOT CAUGHT LATER. The reserve() is on
                        // the next line and there was nothing at all between
                        // it and the wire. A roster entry cannot be shorter
                        // than sixteen bytes — a four-byte id, a four-byte
                        // name length, an eight-byte clock — so a count above
                        // remaining/16 is describing bytes that are not in
                        // the datagram.
                        //
                        // MEASURED, sizeof(PlayerView) being 56 in this
                        // build: the largest value this field can hold asks
                        // for 240,518,168,520 bytes, which at least fails
                        // loudly — std::bad_alloc. But a twenty-byte datagram
                        // claiming 20,000,000 players asks for 1,120,000,000
                        // bytes and SUCCEEDS, and that is the dangerous one,
                        // because nothing throws and so nothing can be
                        // caught.
                        //
                        // BE PRECISE ABOUT WHAT IT COSTS, because the obvious
                        // claim is wrong: the resident set moves by 4,096
                        // bytes, not by a gigabyte. reserve() only maps
                        // address space (VmSize climbs by 1,120,002,048) and
                        // never touches a page, and the vector is local to
                        // this parse, so eight such datagrams in a row leave
                        // resident memory exactly where they found it —
                        // measured at 0 bytes. What the guard is really
                        // buying is the case one step along: a datagram that
                        // carries enough bytes to keep the loop below running
                        // touches those pages for real, and filling the same
                        // vector costs 1,120,002,048 bytes resident. An
                        // unbounded reserve is also 1.12 GB of address space
                        // per datagram at packet rate, which a 32-bit build
                        // does not have to give.
                        if (count > reader.remaining() / 16)
                        {
                            ++m_refusedMessages;
                            SW_LOG_ERROR("Network",
                                         "Roster from {} claims {} players in {} bytes",
                                         m_host.toString(), count, reader.remaining());
                            break;
                        }
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
                        // Same lie, same refusal, from the other direction: a
                        // host can be hostile to a client just as easily.
                        if (size > reader.remaining())
                        {
                            ++m_refusedMessages;
                            SW_LOG_ERROR("Network",
                                         "Event from {} claims {} bytes of payload with {} "
                                         "left in the message",
                                         m_host.toString(), size, reader.remaining());
                            break;
                        }
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
            catch (const std::exception& error)
            {
                ++m_refusedMessages;
                SW_LOG_ERROR("Network", "Dropped a {} from {}: {}",
                             messageTypeName(message.type), m_host.toString(),
                             error.what());
            }
        }
        m_connection.clearDelivered();

        if (rechallenged)
        {
            // Done AFTER the loop, never inside it: answering the challenge
            // replaces m_connection, and the range being iterated belongs to
            // the connection being replaced.
            sendConnectRequest(nowSeconds);
            return;
        }

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
