#pragma once

// ============================================================================
// Network/Transport.hpp
// The only part of the network stack that touches an operating system, kept
// behind an interface two lines wide so that everything above it can be
// tested without one.
//
// A Transport moves datagrams between addresses. It does not retry, order,
// or interpret them — that is Protocol's job, and the separation is what
// makes the protocol testable: LoopbackNetwork below is a real transport
// implementation whose "wire" is a priority queue and whose loss, latency
// and reordering come from a seeded generator. A test that says "drop every
// third packet and deliver the rest 80 ms late, out of order" is exact and
// repeats byte for byte, which is not true of any test that involves a
// second machine, or even a second socket.
// ============================================================================

#include "Core/Types.hpp"

#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace sw::net
{
    /// An IPv4 endpoint. Host byte order throughout — the conversion to
    /// network order happens once, at the socket boundary, where it belongs.
    struct PeerAddress
    {
        u32 ipv4 = 0;
        u16 port = 0;

        [[nodiscard]] constexpr bool operator==(const PeerAddress&) const = default;
        [[nodiscard]] constexpr bool isNull() const { return ipv4 == 0 && port == 0; }

        [[nodiscard]] std::string toString() const;
        /// "1.2.3.4:5678", or "5678" for the local host. Returns false on
        /// anything it does not understand.
        [[nodiscard]] static bool parse(std::string_view text, PeerAddress& out);
        [[nodiscard]] static PeerAddress localhost(u16 port);
    };

    /// The address another machine on the same network should aim at.
    ///
    /// Found by ASKING THE ROUTING TABLE, not by listing adapters: a
    /// developer machine with Docker, WSL, a VPN and a Bluetooth PAN has
    /// half a dozen addresses and exactly one of them is the answer. A UDP
    /// socket is connected to a far-away address and immediately asked which
    /// source it would use — connect() on a datagram socket only fixes a
    /// destination, so nothing is transmitted and nothing is contacted.
    /// Falls back to loopback on a machine with no route out.
    [[nodiscard]] PeerAddress localAddress(u16 port);

    class Transport
    {
    public:
        virtual ~Transport() = default;

        /// Sends one datagram. False means the operating system refused it
        /// outright (unreachable, buffer full); it does NOT mean delivery
        /// failed, because nothing at this layer can know that.
        virtual bool send(const PeerAddress& to, std::span<const u8> bytes) = 0;

        /// Pops one waiting datagram into `buffer`, sets `from`, and returns
        /// its size. Zero means nothing was waiting — never blocks.
        [[nodiscard]] virtual usize receive(PeerAddress& from, std::span<u8> buffer) = 0;

        [[nodiscard]] virtual u16 localPort() const = 0;
    };

    /// A real non-blocking UDP socket. Constructing it binds; a port of 0
    /// asks the operating system for a free one, which `localPort()` then
    /// reports (that is how a client gets an address without a config file).
    class UdpSocket final : public Transport
    {
    public:
        explicit UdpSocket(u16 port = 0);
        ~UdpSocket() override;

        UdpSocket(const UdpSocket&) = delete;
        UdpSocket& operator=(const UdpSocket&) = delete;

        bool send(const PeerAddress& to, std::span<const u8> bytes) override;
        [[nodiscard]] usize receive(PeerAddress& from, std::span<u8> buffer) override;
        [[nodiscard]] u16 localPort() const override { return m_port; }

    private:
        i64 m_handle = -1; // SOCKET on Windows, int fd elsewhere
        u16 m_port = 0;
    };

    /// An in-process wire shared by any number of endpoints.
    ///
    /// Time is PASSED IN rather than read from a clock: a test that wants to
    /// see what happens across three seconds of a bad link should not take
    /// three seconds to run, and it should give the same answer on a loaded
    /// machine as on an idle one.
    class LoopbackTransport;

    class LoopbackNetwork
    {
    public:
        struct Conditions
        {
            /// One-way delay applied to every datagram.
            f64 latencySeconds = 0.0;
            /// Uniform noise added to the delay, in [0, jitter). This is
            /// what produces REORDERING: with jitter above the send
            /// interval, datagrams overtake each other exactly as they do
            /// across a real path with multiple routes.
            f64 jitterSeconds = 0.0;
            /// Fraction of datagrams dropped, [0, 1].
            f32 lossRatio = 0.0f;
            /// Seeds the generator. Same seed, same losses, same order.
            u64 seed = 0x9E3779B97F4A7C15ull;
        };

        void setConditions(const Conditions& conditions);
        [[nodiscard]] const Conditions& conditions() const { return m_conditions; }

        /// Advances the wire's clock. Datagrams become receivable once their
        /// arrival time has passed.
        void setTime(f64 seconds) { m_now = seconds; }
        [[nodiscard]] f64 time() const { return m_now; }

        /// A transport bound to `port` on this wire. Ports are arbitrary
        /// labels here; nothing is bound for real.
        [[nodiscard]] std::unique_ptr<Transport> endpoint(u16 port);

        [[nodiscard]] u64 sentCount() const { return m_sent; }
        [[nodiscard]] u64 droppedCount() const { return m_dropped; }
        /// Datagrams that arrived after one sent later than them.
        [[nodiscard]] u64 reorderedCount() const { return m_reordered; }

    private:
        friend class LoopbackTransport;

        struct InFlight
        {
            PeerAddress from{};
            PeerAddress to{};
            std::vector<u8> bytes;
            f64 arriveAt = 0.0;
            u64 order = 0; // send order, for the reordering counter
        };

        bool submit(const PeerAddress& from, const PeerAddress& to,
                    std::span<const u8> bytes);
        usize collect(u16 port, PeerAddress& from, std::span<u8> buffer);
        [[nodiscard]] f64 nextRandom();

        Conditions m_conditions{};
        f64 m_now = 0.0;
        u64 m_state = 0x9E3779B97F4A7C15ull;
        u64 m_nextOrder = 0;
        u64 m_deliveredOrder = 0;
        u64 m_sent = 0;
        u64 m_dropped = 0;
        u64 m_reordered = 0;
        std::vector<InFlight> m_wire;
    };
} // namespace sw::net
