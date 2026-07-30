#pragma once

// ============================================================================
// Network/Session.hpp
// The two halves of a game: a Host that owns the truth and Clients that
// mirror it.
//
// WHY AUTHORITATIVE HOST AND NOT LOCKSTEP. Deterministic lockstep — every
// machine runs the same simulation from the same inputs — is tempting for a
// game this size because it sends almost nothing. It does not survive
// contact with this engine, for three separate reasons, any one of which
// would be fatal:
//
//   * Bit-exact floating point across machines is not something this code
//     can promise. The physics lane sums forces over parts in archetype
//     order, the aerodynamic tables are interpolated, and the compiler is
//     free to contract a multiply-add. Lockstep tolerates none of that: one
//     bit of divergence compounds into two different worlds within minutes,
//     with no way to notice until the difference is enormous.
//   * Nobody could join a game in progress. Lockstep starts everyone from
//     the same frame zero; there is no state to hand a latecomer.
//   * Time warp would have to be unanimous. Deciding as a group to run at
//     10000x is not a mechanic anyone wants.
//
// So the host simulates and the clients watch. The host's world IS the game;
// a client's world is a mirror maintained by Replication, and its own
// simulation lanes never touch replicated entities.
//
// WHAT A CLIENT SENDS BACK IS INTENT, NOT STATE. A Command says "I am
// holding pitch-up and the throttle at 60 %", never "my ship is here". The
// host decides what that means. That is the difference between a game where
// physics is negotiated and one where it is not, and it is also the whole
// of the anti-cheat story: a client that lies about its inputs can fly
// badly, and that is all.
// ============================================================================

#include "Core/Types.hpp"
#include "Network/Protocol.hpp"
#include "Network/Replication.hpp"
#include "Network/Timeline.hpp"
#include "Network/Transport.hpp"
#include "Save/Snapshot.hpp"

#include <memory>
#include <string>
#include <vector>

namespace sw::ecs
{
    class World;
} // namespace sw::ecs

namespace sw::net
{
    /// Version of the session handshake itself, independent of the packet
    /// protocol version: the framing can stay put while what a
    /// ConnectRequest carries changes.
    constexpr u32 kSessionVersion = 1;

    enum class ClientState : u8
    {
        Disconnected = 0,
        Connecting,   // request sent, no answer yet
        Connected,    // accepted; the mirror is being kept up to date
        Rejected,     // the host said no, and said why
        TimedOut,     // silence for longer than the connection tolerates
    };

    [[nodiscard]] std::string_view clientStateName(ClientState state);

    /// One player, as everybody else sees them.
    ///
    /// `simulatedSeconds` is THEIR clock, not yours. In this game a warp is
    /// personal — one player running at ten million times real time does not
    /// drag anyone else forward — so two players can legitimately be hours
    /// apart, and the difference is a first-class thing the interface shows
    /// rather than a fault to be corrected.
    struct PlayerView
    {
        u32 id = 0;              // 0 = the host
        std::string name;
        f64 simulatedSeconds = 0.0;
        f64 roundTripSeconds = 0.0;
    };

    // ------------------------------------------------------------------------
    // Host
    // ------------------------------------------------------------------------

    class Host
    {
    public:
        struct Config
        {
            /// How often a client is sent a state delta. Twenty is not
            /// stinginess: the physics lane runs at fifty, but a client
            /// interpolates between the states it has (the renderer already
            /// does exactly this between simulation ticks), so sending every
            /// tick would more than double the bandwidth to buy smoothness
            /// that is already there.
            f64 snapshotHz = 20.0;
            /// How often the roster of everyone's clocks goes out. Slow on
            /// purpose: a player's clock is read by a human off a panel, and
            /// four times a second is already faster than anyone can read.
            f64 rosterHz = 4.0;
            u32 maxClients = 8;
            /// What the host calls itself in the roster.
            std::string hostName = "host";
            Connection::Config connection{};
        };

        /// The schema is COPIED: it defines the wire contract for the whole
        /// session and must not be able to change under it.
        Host(std::unique_ptr<Transport> transport, const save::Schema& schema,
             ReplicationSet set, Config config);
        Host(std::unique_ptr<Transport> transport, const save::Schema& schema,
             ReplicationSet set);

        /// One pump: read the socket, act on what arrived, encode a snapshot
        /// if one is due, write the socket. `simulatedSeconds` is the host's
        /// OWN clock — it is stamped into snapshots, published in the roster
        /// and used to release stamped events, and it is nobody else's.
        void update(f64 nowSeconds, const ecs::World& world, f64 simulatedSeconds);

        /// Announces something that happened AT AN INSTANT. Every other
        /// player holds it until their own clock reaches that instant; see
        /// Network/Timeline.hpp for why that is the whole model.
        void broadcastEvent(f64 stampSeconds, u32 kind, std::span<const u8> payload);

        /// Events from clients, released as the host's own clock passes
        /// them. Drain it every frame after update().
        [[nodiscard]] Timeline& timeline() { return m_timeline; }

        /// Every player including the host, host first. Rebuilt each update.
        [[nodiscard]] const std::vector<PlayerView>& roster() const { return m_roster; }

        struct CommandEvent
        {
            u32 clientId = 0;
            std::vector<u8> payload;
        };
        /// Commands that arrived during the last update, in arrival order.
        [[nodiscard]] const std::vector<CommandEvent>& commands() const { return m_commands; }

        struct ClientView
        {
            u32 id = 0;
            std::string name;
            PeerAddress address{};
            bool accepted = false;
            f64 roundTripSeconds = 0.0;
            u32 lastSnapshotId = 0;
            u32 acknowledgedId = 0;
            SnapshotStats lastSnapshot{};
            ConnectionStats connection{};
        };
        [[nodiscard]] std::vector<ClientView> clients() const;
        [[nodiscard]] u32 clientCount() const;

        /// Ids of clients that dropped during the last update.
        [[nodiscard]] const std::vector<u32>& departed() const { return m_departed; }

        /// Sends a Disconnect to everyone. Courtesy, not protocol: without
        /// it a quitting host leaves every client waiting out a timeout.
        void shutdown(f64 nowSeconds);

        [[nodiscard]] u16 port() const { return m_transport->localPort(); }
        [[nodiscard]] const ReplicationTable& table() const { return m_table; }

        /// WHAT REACHED THE SOCKET, before any of it was believed.
        ///
        /// This exists because "the client says it got no reply" has three
        /// causes that are indistinguishable FROM THE CLIENT — something
        /// eating the packet on the way, a wrong address, and a host that
        /// received the packet and threw it away — and the host is the only
        /// machine that can tell them apart. `arrived` still zero while
        /// someone is trying means nothing reached this process at all.
        /// `arrived` climbing together with `refused` means the packets got
        /// here and were not understood, which no firewall rule will fix.
        struct Reception
        {
            /// Datagrams handed over by the socket, from anyone, ever.
            u64 arrived = 0;
            /// Of those, from an address that was not already a peer.
            u64 fromStrangers = 0;
            /// Strangers turned away: unreadable, not a connection attempt,
            /// or the session was full.
            u64 refused = 0;
            /// Refused specifically because the header named a protocol
            /// version this build does not speak — almost always two
            /// machines running two different builds.
            u64 wrongVersion = 0;
            /// The last such version seen, for the message that says so.
            u16 lastForeignVersion = 0;
            /// Refused because it was not ours at all: wrong magic. Noise on
            /// the port, or some other program entirely.
            u64 notOurs = 0;
        };
        [[nodiscard]] const Reception& reception() const { return m_reception; }

    private:
        struct Peer
        {
            u32 id = 0;
            std::string name;
            PeerAddress address{};
            bool accepted = false;
            Connection connection;
            std::unique_ptr<ReplicationEncoder> encoder;
            SnapshotStats lastSnapshot{};
            /// THIS PLAYER'S OWN CLOCK, as last reported. Not the host's.
            f64 clockSeconds = 0.0;
            bool clockKnown = false;
            /// PER PEER, not global: a client accepted mid-beat is sent its
            /// world immediately, and a global timer would then fire again
            /// milliseconds later and send a second full snapshot — tens of
            /// kilobytes the client refuses, because it is built on the same
            /// empty baseline as the one it just applied.
            f64 lastSnapshotAt = -1.0;

            explicit Peer(const Connection::Config& config) : connection(config) {}
        };

        Peer* findPeer(const PeerAddress& address);
        void handleMessages(f64 nowSeconds, Peer& peer, const ecs::World& world,
                            f64 simulatedSeconds);
        void acceptPeer(f64 nowSeconds, Peer& peer, const ecs::World& world,
                        f64 simulatedSeconds);
        void pushSnapshot(f64 nowSeconds, Peer& peer, const ecs::World& world,
                          f64 simulatedSeconds);
        void flush(f64 nowSeconds);

        std::unique_ptr<Transport> m_transport;
        save::Schema m_schema;
        ReplicationTable m_table;
        Config m_config{};
        std::vector<std::unique_ptr<Peer>> m_peers;
        std::vector<CommandEvent> m_commands;
        std::vector<u32> m_departed;
        u32 m_nextClientId = 1;
        Timeline m_timeline;
        std::vector<PlayerView> m_roster;
        f64 m_localClock = 0.0;
        f64 m_lastRosterAt = -1.0;
        std::vector<std::vector<u8>> m_outgoing;
        std::vector<u8> m_receiveBuffer;
        Reception m_reception{};
    };

    // ------------------------------------------------------------------------
    // Client
    // ------------------------------------------------------------------------

    class Client
    {
    public:
        struct Config
        {
            /// A bare acknowledgement is also sent on this beat even when no
            /// snapshot arrived, so a host whose snapshots are all being lost
            /// still learns which baseline the client actually holds.
            f64 acknowledgeHz = 20.0;
            /// Give up and re-send the ConnectRequest after this long.
            f64 connectRetrySeconds = 0.5;
            /// How often this player's own clock is reported to the host.
            f64 clockHz = 5.0;
            Connection::Config connection{};
        };

        Client(std::unique_ptr<Transport> transport, const save::Schema& schema,
               std::string playerName, Config config);
        Client(std::unique_ptr<Transport> transport, const save::Schema& schema,
               std::string playerName);

        void connect(const PeerAddress& host, f64 nowSeconds);
        void disconnect(f64 nowSeconds);

        /// One pump. `mirror` is the world this client keeps in step with the
        /// host; it should contain nothing the client creates itself.
        /// `simulatedSeconds` is THIS player's own clock — reported to the
        /// session and used to release stamped events.
        void update(f64 nowSeconds, ecs::World& mirror, f64 simulatedSeconds);

        /// Announces something that happened at an instant, to everyone.
        void sendEvent(f64 stampSeconds, u32 kind, std::span<const u8> payload);

        /// Events from other players, released as this player's own clock
        /// passes their instant. Drain it every frame after update().
        [[nodiscard]] Timeline& timeline() { return m_timeline; }

        /// Every player in the session with their own clock, host first.
        [[nodiscard]] const std::vector<PlayerView>& roster() const { return m_roster; }

        /// Queued reliably and delivered in order — an input that arrives
        /// out of order is worse than one that arrives late.
        void sendCommand(std::span<const u8> payload);

        [[nodiscard]] ClientState state() const { return m_state; }
        [[nodiscard]] const std::string& rejectReason() const { return m_rejectReason; }
        [[nodiscard]] u32 clientId() const { return m_clientId; }
        /// Host simulation time carried by the newest applied snapshot.
        [[nodiscard]] f64 hostSimulatedSeconds() const;
        [[nodiscard]] u32 appliedSnapshotId() const;
        /// Snapshots refused for naming a baseline this client does not
        /// hold — the honest count of how much the link is losing.
        [[nodiscard]] u32 rebasedCount() const { return m_rebased; }
        [[nodiscard]] f64 roundTripSeconds() const { return m_connection.roundTripSeconds(); }
        [[nodiscard]] const ConnectionStats& stats() const { return m_connection.stats(); }

    private:
        void flush(f64 nowSeconds);
        void sendAcknowledgement(f64 nowSeconds);
        void sendClock(f64 nowSeconds);

        std::unique_ptr<Transport> m_transport;
        save::Schema m_schema;
        std::string m_name;
        Config m_config{};
        Connection m_connection;
        std::unique_ptr<ReplicationDecoder> m_decoder;
        PeerAddress m_host{};
        ClientState m_state = ClientState::Disconnected;
        std::string m_rejectReason;
        u32 m_clientId = 0;
        u32 m_rebased = 0;
        f64 m_lastRequestAt = -1.0;
        f64 m_lastAckAt = -1.0;
        f64 m_lastClockAt = -1.0;
        f64 m_localClock = 0.0;
        Timeline m_timeline;
        std::vector<PlayerView> m_roster;
        std::vector<std::vector<u8>> m_outgoing;
        std::vector<u8> m_receiveBuffer;
    };
} // namespace sw::net
