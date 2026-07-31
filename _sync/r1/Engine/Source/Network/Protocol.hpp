#pragma once

// ============================================================================
// Network/Protocol.hpp
// The wire: what a StarWorks datagram looks like, and the one connection
// object that turns a stream of unordered, lossy datagrams into two usable
// delivery services.
//
// WHY TWO SERVICES AND NOT ONE. The traffic splits cleanly in half and the
// halves want opposite things:
//
//   * The per-tick state delta is worthless the moment a newer one exists.
//     Resending a lost one would deliver stale truth LATE, behind the fresh
//     truth that overtook it. So it is UNRELIABLE and SEQUENCED: one
//     datagram, sent once, and an older arrival than the newest already
//     seen is dropped on the floor rather than applied backwards.
//   * Everything else — the handshake, the world transfer at join, a pilot's
//     input, a disconnect — is a fact that has to land exactly once and in
//     order. That is RELIABLE ORDERED: sequenced, acknowledged, resent until
//     confirmed, and held back on the receiving side until its predecessors
//     have arrived.
//
// Nothing here knows what a vessel is. This file moves bytes.
//
// WHY UDP AND NOT TCP. TCP gives the reliable half for free and then forces
// it on the unreliable half too: one lost state delta stalls every delta
// behind it until the retransmission lands (head-of-line blocking), which
// is precisely the freeze-then-teleport that makes a game feel broken. The
// two services above cannot both be built on one ordered stream, so the
// ordered stream is the thing that has to go.
// ============================================================================

#include "Core/Types.hpp"
#include "Serialization/Binary.hpp"

#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace sw::net
{
    /// 'SWNT'. First four bytes of every datagram: anything else is not ours
    /// and is discarded without further parsing. A public UDP port receives
    /// scanner noise constantly and none of it should reach the decoder.
    constexpr u32 kProtocolMagic = 0x53574E54u;
    /// Bumped to 3 for the connect challenge: the handshake gained a message
    /// type and a cookie field, so a version-2 client and a version-3 host
    /// would talk past each other forever — the client resending a request
    /// with no cookie in it, the host challenging a client that does not know
    /// what a challenge is. Refusing the datagram outright at readHeader
    /// turns that silent deadlock into the "different builds" message the
    /// host already knows how to print.
    constexpr u16 kProtocolVersion = 3;

    /// Total size of one datagram, header included. 1200 bytes clears every
    /// path MTU that matters (1500-byte Ethernet less IPv6, UDP and any
    /// tunnel) so IP never fragments for us — IP fragmentation loses the
    /// WHOLE datagram when any one of its pieces is lost, which turns a 1 %
    /// packet loss into a much larger message loss.
    constexpr usize kMaxDatagramBytes = 1200;

    /// Fixed part of every header, in bytes.
    constexpr usize kHeaderBytes = 16;
    /// Extra fields carried only by a fragment of a larger reliable message.
    constexpr usize kFragmentHeaderBytes = 6;

    /// Largest payload that fits in one unfragmented datagram.
    constexpr usize kMaxUnreliablePayload = kMaxDatagramBytes - kHeaderBytes;
    /// Largest slice of a fragmented reliable message.
    constexpr usize kMaxFragmentPayload =
        kMaxDatagramBytes - kHeaderBytes - kFragmentHeaderBytes;

    /// Ceiling on ONE reassembled reliable message, and therefore on the
    /// memory a peer can pin by sending fragments of a message it never
    /// finishes. Without it the receiver's partial buffer grows by up to
    /// kMaxFragmentPayload for every datagram a hostile peer cares to send,
    /// forever, because nothing in the old code ever gave up on a
    /// reassembly.
    ///
    /// EIGHT MEBIBYTES IS NOT ARBITRARY. The largest thing this protocol
    /// legitimately fragments is the full world snapshot sent at join.
    /// MEASURED against this build: a full snapshot of 20,000 entities each
    /// carrying a 24-byte position and a 12-byte link encodes to 1,080,036
    /// bytes, i.e. 54.00 bytes per entity, so 8 MiB covers 155,339
    /// replicated entities — far more than this game puts in one world, and
    /// still small enough that a peer holding one hostage costs less than a
    /// single texture.
    constexpr usize kMaxReassemblyBytes = 8u * 1024u * 1024u;

    /// The same ceiling expressed as fragments, which is what the wire
    /// actually carries. The SENDER refuses to build a message needing more
    /// than this for exactly the reason the receiver refuses to accept one:
    /// a message the far side will throw away is a message that must never
    /// be put on the wire, and finding that out at queue time names the
    /// caller instead of blaming the network.
    constexpr u16 kMaxFragments =
        static_cast<u16>(kMaxReassemblyBytes / kMaxFragmentPayload);

    /// How many reliable datagrams may be in flight unacknowledged.
    ///
    /// THIRTY-TWO IS NOT A TUNING KNOB, it is forced. An acknowledgement is
    /// one sequence number plus a 32-bit field covering the 32 sequences
    /// before it, so the newest and oldest unacked datagram can never be
    /// more than 32 apart or the oldest falls out of every future ack and is
    /// resent forever. Widening the window means widening the ack field.
    constexpr u32 kReliableWindow = 32;

    enum class MessageType : u8
    {
        Invalid = 0,
        /// client -> host, reliable. Protocol version and player name.
        ConnectRequest,
        /// host -> client, reliable. Client id, the component table this
        /// session replicates, and where the host's clock is.
        ConnectAccept,
        /// host -> client, reliable. A human-readable reason.
        ConnectReject,
        /// host -> client, UNRELIABLE and STATELESS. Carries the cookie the
        /// source address must echo before the host will spend a byte of
        /// memory or a kilobyte of upstream on it. Sent by hand, without a
        /// Connection behind it, because allocating one is precisely what
        /// this message exists to defer.
        ConnectChallenge,
        /// either direction, reliable. Also sent as a courtesy on quit so
        /// the peer does not have to wait out a timeout.
        Disconnect,
        /// host -> client, UNRELIABLE. One replication delta.
        Snapshot,
        /// client -> host, UNRELIABLE. The newest snapshot the client has
        /// applied in full; the host diffs against that from then on.
        SnapshotAck,
        /// client -> host, reliable. Pilot input and other intents.
        Command,
        /// client -> host, UNRELIABLE. Where this player's own simulation
        /// clock has got to. Every player owns their clock in this game —
        /// one warping does not drag the others — so the session has to be
        /// told rather than able to assume.
        Clock,
        /// host -> client, UNRELIABLE. Every player's name and clock, so
        /// each client can show how far ahead or behind the others are.
        Roster,
        /// either direction, reliable. A STAMPED action: what happened, and
        /// the simulation instant it happened at. A player whose clock has
        /// not reached that instant holds it until it does.
        Event,
        /// either direction, unreliable. Keeps the round-trip estimate fresh
        /// and the connection provably alive while nothing else is moving.
        Ping,
        Pong,
        Count
    };

    [[nodiscard]] std::string_view messageTypeName(MessageType type);

    /// Refuses a count read off the wire that the bytes still in hand could
    /// not possibly describe. Throws sw::Exception, which every parser in
    /// this stack already catches.
    ///
    /// THE FAILURE THIS PREVENTS. Every count in every message is chosen by
    /// whoever sent the datagram, and every one of them drove a loop that
    /// allocates — a snapshot's spawn count grows the mirror's slot table, a
    /// table's component count builds a std::string per entry. Unbounded,
    /// those loops run until the reader happens to hit the end of the
    /// message, which means the world has already been half-modified by a
    /// snapshot that was never valid, and it means the work is done before
    /// the lie is discovered rather than instead of it.
    ///
    /// The bound is arithmetic, not a guess: N elements need at least
    /// N * minElementBytes bytes still unread, so any larger count is a lie
    /// and is refused before the first iteration.
    ///
    /// Where a count feeds a single `resize` or `reserve` directly — the
    /// Roster and the Event payload in Session.cpp — the check is written
    /// out at the call site instead, non-throwing and counted, because there
    /// the interesting failure is not an exception at all: a count the
    /// allocator can SATISFY (measured: 20,000,000 roster entries is
    /// 1,120,000,000 bytes, and reserve succeeds) throws nothing and so can
    /// never be caught.
    void requireWireCount(u64 count, usize minElementBytes, usize remaining,
                          std::string_view what);

    /// Builds one complete datagram with NO Connection behind it.
    ///
    /// Exists for the single case that must not allocate per-address state:
    /// answering an address that has not proven it can receive what it asks
    /// for. Sequence zero, no acknowledgement, no retransmission — there is
    /// nothing on the host holding this conversation, and that is the point.
    void buildBareDatagram(MessageType type, std::span<const u8> payload,
                           std::vector<u8>& out);

    /// Wrap-safe comparison of 16-bit sequence numbers. `a` is newer than
    /// `b` when the shortest way round the circle from `b` to `a` goes
    /// forwards. Everything downstream depends on never comparing these with
    /// `<`: sequence 0 follows 65535, and a naive compare reads that as a
    /// 65535-step jump backwards.
    [[nodiscard]] inline bool sequenceNewer(u16 a, u16 b)
    {
        return a != b && static_cast<u16>(a - b) < 0x8000u;
    }

    /// Signed distance from `b` to `a`, wrap-safe. Positive means `a` is
    /// ahead.
    [[nodiscard]] inline i32 sequenceDelta(u16 a, u16 b)
    {
        return static_cast<i16>(static_cast<u16>(a - b));
    }

    struct PacketHeader
    {
        static constexpr u8 kFlagFragment = 0x01;
        /// The ack fields carry a real acknowledgement. Without this bit
        /// they are zero padding — and zero is a LEGAL sequence number, so a
        /// peer that has received nothing yet would otherwise appear to be
        /// acknowledging datagram 0 on its very first packet.
        static constexpr u8 kFlagAck = 0x02;
        /// This datagram belongs to the reliable ordered channel.
        ///
        /// WHICH CHANNEL A DATAGRAM RODE IS A PROPERTY OF THE DATAGRAM, not
        /// of its message type, and the receiver must be told rather than
        /// left to infer it. A Snapshot is normally unreliable, but one too
        /// big for a single datagram is promoted to the reliable channel so
        /// it can be fragmented — and a receiver that decided by type alone
        /// would hand each fragment upward as if it were a whole snapshot.
        static constexpr u8 kFlagReliable = 0x04;

        u32 magic = kProtocolMagic;
        u16 version = kProtocolVersion;
        MessageType type = MessageType::Invalid;
        u8 flags = 0;
        /// Per-channel. Reliable and unreliable number independently.
        u16 sequence = 0;
        /// Newest RELIABLE sequence the sender has received from the peer.
        u16 ack = 0;
        /// Bit i set = the sender also received reliable sequence
        /// (ack - 1 - i). One datagram therefore acknowledges up to 33.
        u32 ackBits = 0;

        // Present only when flags has kFragment.
        u16 messageId = 0;
        u16 fragmentIndex = 0;
        u16 fragmentCount = 0;
    };

    void writeHeader(ser::BinaryWriter& writer, const PacketHeader& header);
    /// Returns false — without throwing — on anything that is not a
    /// well-formed StarWorks datagram. A malformed packet from a stranger
    /// is normal traffic on a public port, not an exceptional condition.
    [[nodiscard]] bool readHeader(ser::BinaryReader& reader, PacketHeader& header);

    /// One complete message handed up by a Connection.
    struct Message
    {
        MessageType type = MessageType::Invalid;
        std::vector<u8> payload;
    };

    /// Everything worth putting on a diagnostics panel, and nothing that
    /// costs anything to keep.
    struct ConnectionStats
    {
        u64 datagramsSent = 0;
        u64 datagramsReceived = 0;
        u64 bytesSent = 0;
        u64 bytesReceived = 0;
        /// Reliable datagrams put back on the wire because no ack came.
        u64 resends = 0;
        /// Reliable datagrams that arrived twice (their ack was lost).
        u64 duplicatesReceived = 0;
        /// Unreliable datagrams discarded for being older than one already
        /// applied. The honest measure of reordering on the path.
        u64 staleDropped = 0;
        /// Datagrams rejected before decoding: bad magic, bad version, runt,
        /// or longer than a datagram of this protocol can be.
        u64 malformedDropped = 0;
        /// Reassemblies thrown away part-built because the fragment that
        /// should have continued them named another message, arrived out of
        /// order, or would have pushed the buffer past kMaxReassemblyBytes.
        /// Non-zero against a peer that is not on a broken path means that
        /// peer is lying to you.
        u64 fragmentsRejected = 0;
        /// Reliable messages queued but not yet sent because the window is
        /// full. Sustained non-zero means the link cannot keep up.
        u32 reliableBacklog = 0;
    };

    /// ONE PEER, seen from one side. Owns both delivery services, all
    /// sequencing, the ack bookkeeping and the round-trip estimate.
    ///
    /// It has no socket and no clock: `collectOutgoing` hands back the bytes
    /// to send and `receive` takes the bytes that arrived, both with the
    /// time passed in. That is what lets the entire protocol be tested
    /// against a simulated link with exact, repeatable loss and latency,
    /// with no socket, no port and no timing race anywhere in the test.
    class Connection
    {
    public:
        struct Config
        {
            /// No ack after this long and the datagram goes out again.
            /// Floor, not the whole story: the real timeout is
            /// max(resendDelay, 1.5 x measured round trip).
            f64 resendDelaySeconds = 0.05;
            /// Owing an acknowledgement for longer than this sends a bare
            /// Ping purely to carry it. Without this the reliable channel
            /// stalls whenever one side has nothing to say.
            f64 ackDelaySeconds = 0.02;
            /// Silence longer than this counts as gone.
            f64 timeoutSeconds = 10.0;
        };

        // Two constructors rather than one with a default argument: a
        // default argument of `= {}` would need Config's own default member
        // initializers before Connection is a complete type, which the
        // language does not allow.
        Connection() = default;
        explicit Connection(const Config& config) : m_config(config) {}

        /// Exactly once, in order, resent until confirmed. Any size: it is
        /// cut into fragments and reassembled on the far side.
        void queueReliable(MessageType type, std::span<const u8> payload);

        /// Sent once, never resent, dropped on arrival if a newer one of the
        /// same type already landed. Payloads over kMaxUnreliablePayload
        /// throw — that is a caller bug, not a network condition.
        void queueUnreliable(MessageType type, std::span<const u8> payload);

        /// Appends every datagram that should go on the wire now: fresh
        /// reliable ones the window has room for, overdue resends, queued
        /// unreliable ones, and a bare Ping if an acknowledgement is owed.
        void collectOutgoing(f64 nowSeconds, std::vector<std::vector<u8>>& out);

        /// Feeds one arrived datagram. Never throws on bad input.
        void receive(f64 nowSeconds, std::span<const u8> bytes);

        /// Messages completed since the last `clearDelivered`. Reliable ones
        /// appear in send order; unreliable ones in arrival order.
        [[nodiscard]] const std::vector<Message>& delivered() const { return m_delivered; }
        void clearDelivered() { m_delivered.clear(); }

        /// Smoothed round trip, seconds. Zero until the first ack lands.
        [[nodiscard]] f64 roundTripSeconds() const { return m_roundTrip; }
        [[nodiscard]] f64 lastReceiveSeconds() const { return m_lastReceive; }
        [[nodiscard]] bool hasTimedOut(f64 nowSeconds) const
        {
            return m_lastReceive > 0.0 && nowSeconds - m_lastReceive > m_config.timeoutSeconds;
        }
        [[nodiscard]] const ConnectionStats& stats() const { return m_stats; }

        /// Starts the clock without any traffic having arrived yet, so a
        /// brand-new connection is not instantly "timed out".
        void markAlive(f64 nowSeconds) { m_lastReceive = nowSeconds; }

    private:
        struct PendingOut
        {
            std::vector<u8> bytes;   // header included, ready for the wire
            u16 sequence = 0;
            f64 sentAt = -1.0;       // < 0 = never sent
            bool acked = false;
        };

        struct PendingIn
        {
            u16 sequence = 0;
            bool used = false;
            bool fragment = false;
            MessageType type = MessageType::Invalid;
            u16 messageId = 0;
            u16 fragmentIndex = 0;
            u16 fragmentCount = 0;
            std::vector<u8> payload;
        };

        void buildDatagram(std::vector<u8>& bytes, const PacketHeader& header,
                           std::span<const u8> payload) const;
        void recordRemoteSequence(u16 sequence);
        void applyAck(f64 nowSeconds, u16 ack, u32 ackBits);
        void ackOne(f64 nowSeconds, u16 sequence);
        void drainInOrder();
        /// Appends one fragment to the reassembly in progress, or refuses it
        /// and throws the reassembly away. False means nothing was appended.
        bool acceptFragment(const PendingIn& slot);
        void abandonReassembly();

        Config m_config{};
        // --- outgoing -------------------------------------------------------
        /// Every reliable datagram whose turn has come and whose ack has
        /// not, in send order. Entries at the front are dropped as they are
        /// confirmed; entries with sentAt < 0 are still waiting for room in
        /// the window.
        std::vector<PendingOut> m_inFlight;
        std::vector<std::vector<u8>> m_unreliableOut;    // ready-made datagrams
        u16 m_nextReliableSequence = 0;
        u16 m_nextUnreliableSequence = 0;
        u16 m_nextMessageId = 0;
        f64 m_lastSend = -1.0;

        // --- incoming -------------------------------------------------------
        u16 m_remoteAck = 0;              // newest reliable sequence seen
        u32 m_remoteAckBits = 0;
        bool m_haveRemoteAck = false;
        bool m_ackOwed = false;
        u16 m_nextDelivery = 0;           // next reliable sequence to hand up
        /// Out-of-order holding pen, indexed by sequence modulo its size.
        /// The window is 32, so nothing can arrive more than 32 ahead of
        /// what is owed; 256 slots is that with room to spare.
        static constexpr usize kHoldingPen = 256;
        std::vector<PendingIn> m_received{kHoldingPen};
        /// A fragmented message is carried by CONSECUTIVE sequences, so
        /// walking the reliable channel in order accumulates its pieces in
        /// order. That keeps the reassembly table down to ONE entry — there
        /// is structurally never a second message in flight on this channel
        /// — but it is not the same thing as trusting what arrives.
        ///
        /// An honest sender emits index 0..count-1 of one message id back to
        /// back. A hostile one emits index 1 of count 3 forever: the old code
        /// only cleared the buffer on index 0, so that grew m_partial without
        /// limit and never completed anything. It also never compared the
        /// message id the header carries, so the tail of one message
        /// concatenated onto the head of another and was handed up as a
        /// single believable payload. The four fields below are what make
        /// both of those impossible: a fragment is appended only when it
        /// continues the reassembly already running, in the right message, at
        /// the next index, within kMaxReassemblyBytes.
        std::vector<u8> m_partial;
        bool m_partialActive = false;
        u16 m_partialMessageId = 0;
        u16 m_partialCount = 0;
        u16 m_partialNextIndex = 0;
        /// Newest unreliable sequence applied, per message type — that is
        /// what makes the channel SEQUENCED rather than merely unreliable.
        u16 m_newestUnreliable[static_cast<usize>(MessageType::Count)]{};
        bool m_sawUnreliable[static_cast<usize>(MessageType::Count)]{};

        std::vector<Message> m_delivered;
        f64 m_roundTrip = 0.0;
        f64 m_lastReceive = -1.0;
        ConnectionStats m_stats{};
    };
} // namespace sw::net
