#include "Network/Transport.hpp"

#include "Core/Error.hpp"
#include "Core/Log.hpp"

#include <algorithm>
#include <charconv>
#include <cstring>
#include <format>

#if defined(_WIN32)
#    include <winsock2.h>
#    include <ws2tcpip.h>
using SocketHandle = SOCKET;
#    define SW_INVALID_SOCKET INVALID_SOCKET
#else
#    include <arpa/inet.h>
#    include <cerrno>
#    include <fcntl.h>
#    include <netinet/in.h>
#    include <sys/socket.h>
#    include <unistd.h>
using SocketHandle = int;
#    define SW_INVALID_SOCKET (-1)
#endif

namespace sw::net
{
    namespace
    {
#if defined(_WIN32)
        /// Winsock needs a process-wide start-up call and a matching
        /// shut-down. A function-local static gives both, exactly once, on
        /// first use — no init order to get wrong and nothing for the game
        /// to remember to call.
        struct WinsockGuard
        {
            WinsockGuard()
            {
                WSADATA data{};
                if (WSAStartup(MAKEWORD(2, 2), &data) != 0)
                {
                    SW_THROW("WSAStartup failed ({})", WSAGetLastError());
                }
            }
            ~WinsockGuard() { WSACleanup(); }
        };

        void ensureWinsock()
        {
            static WinsockGuard guard;
            (void)guard;
        }

        int lastSocketError() { return WSAGetLastError(); }
        bool wouldBlock(int error) { return error == WSAEWOULDBLOCK; }
        void closeSocket(SocketHandle handle) { closesocket(handle); }
#else
        void ensureWinsock() {}
        int lastSocketError() { return errno; }
        bool wouldBlock(int error) { return error == EAGAIN || error == EWOULDBLOCK; }
        void closeSocket(SocketHandle handle) { ::close(handle); }
#endif
    } // namespace

    std::string PeerAddress::toString() const
    {
        return std::format("{}.{}.{}.{}:{}", (ipv4 >> 24) & 0xFF, (ipv4 >> 16) & 0xFF,
                           (ipv4 >> 8) & 0xFF, ipv4 & 0xFF, port);
    }

    PeerAddress PeerAddress::localhost(u16 port)
    {
        return PeerAddress{0x7F000001u, port};
    }

    bool PeerAddress::parse(std::string_view text, PeerAddress& out)
    {
        const auto colon = text.rfind(':');
        std::string_view hostPart;
        std::string_view portPart;
        if (colon == std::string_view::npos)
        {
            hostPart = "127.0.0.1";
            portPart = text;
        }
        else
        {
            hostPart = text.substr(0, colon);
            portPart = text.substr(colon + 1);
        }
        if (hostPart.empty())
        {
            hostPart = "127.0.0.1";
        }

        u32 address = 0;
        usize cursor = 0;
        for (int part = 0; part < 4; ++part)
        {
            if (cursor > hostPart.size())
            {
                return false;
            }
            usize end = hostPart.find('.', cursor);
            if (part == 3)
            {
                end = hostPart.size();
            }
            else if (end == std::string_view::npos)
            {
                return false;
            }
            u32 value = 0;
            const char* first = hostPart.data() + cursor;
            const char* last = hostPart.data() + end;
            if (first == last)
            {
                return false;
            }
            const auto result = std::from_chars(first, last, value);
            if (result.ec != std::errc{} || result.ptr != last || value > 255)
            {
                return false;
            }
            address = (address << 8) | value;
            cursor = end + 1;
        }

        u32 port = 0;
        const auto result =
            std::from_chars(portPart.data(), portPart.data() + portPart.size(), port);
        if (result.ec != std::errc{} ||
            result.ptr != portPart.data() + portPart.size() || port > 0xFFFFu)
        {
            return false;
        }

        out.ipv4 = address;
        out.port = static_cast<u16>(port);
        return true;
    }

    PeerAddress localAddress(u16 port)
    {
        ensureWinsock();
        PeerAddress result = PeerAddress::localhost(port);

        const SocketHandle handle = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (handle == SW_INVALID_SOCKET)
        {
            return result;
        }

        // A public address that is never actually contacted. All this does
        // is make the kernel resolve the route, which is the only way to
        // learn which of the machine's addresses faces the network.
        sockaddr_in probe{};
        probe.sin_family = AF_INET;
        probe.sin_addr.s_addr = htonl(0x08080808u); // 8.8.8.8
        probe.sin_port = htons(53);
        if (::connect(handle, reinterpret_cast<const sockaddr*>(&probe), sizeof(probe)) == 0)
        {
            sockaddr_in local{};
#if defined(_WIN32)
            int length = static_cast<int>(sizeof(local));
#else
            socklen_t length = sizeof(local);
#endif
            if (::getsockname(handle, reinterpret_cast<sockaddr*>(&local), &length) == 0 &&
                local.sin_addr.s_addr != 0)
            {
                result.ipv4 = ntohl(local.sin_addr.s_addr);
                result.port = port;
            }
        }
        closeSocket(handle);
        return result;
    }

    // ------------------------------------------------------------------------
    // UdpSocket
    // ------------------------------------------------------------------------

    UdpSocket::UdpSocket(u16 port)
    {
        ensureWinsock();

        const SocketHandle handle = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (handle == SW_INVALID_SOCKET)
        {
            SW_THROW("Could not create a UDP socket ({})", lastSocketError());
        }

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_ANY);
        address.sin_port = htons(port);
        if (::bind(handle, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0)
        {
            const int error = lastSocketError();
            closeSocket(handle);
            SW_THROW("Could not bind UDP port {} ({})", port, error);
        }

        // Non-blocking, always. A game loop that can block inside receive()
        // is a game loop whose frame rate is set by the network.
#if defined(_WIN32)
        u_long nonBlocking = 1;
        if (ioctlsocket(handle, FIONBIO, &nonBlocking) != 0)
#else
        const int flags = ::fcntl(handle, F_GETFL, 0);
        if (flags < 0 || ::fcntl(handle, F_SETFL, flags | O_NONBLOCK) != 0)
#endif
        {
            const int error = lastSocketError();
            closeSocket(handle);
            SW_THROW("Could not make the UDP socket non-blocking ({})", error);
        }

        sockaddr_in bound{};
#if defined(_WIN32)
        int boundSize = static_cast<int>(sizeof(bound));
#else
        socklen_t boundSize = sizeof(bound);
#endif
        if (::getsockname(handle, reinterpret_cast<sockaddr*>(&bound), &boundSize) == 0)
        {
            m_port = ntohs(bound.sin_port);
        }
        else
        {
            m_port = port;
        }

        m_handle = static_cast<i64>(handle);
    }

    UdpSocket::~UdpSocket()
    {
        if (m_handle >= 0)
        {
            closeSocket(static_cast<SocketHandle>(m_handle));
        }
    }

    bool UdpSocket::send(const PeerAddress& to, std::span<const u8> bytes)
    {
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(to.ipv4);
        address.sin_port = htons(to.port);

        const auto sent = ::sendto(static_cast<SocketHandle>(m_handle),
                                   reinterpret_cast<const char*>(bytes.data()),
#if defined(_WIN32)
                                   static_cast<int>(bytes.size()),
#else
                                   bytes.size(),
#endif
                                   0, reinterpret_cast<const sockaddr*>(&address),
                                   sizeof(address));
        return sent >= 0 && static_cast<usize>(sent) == bytes.size();
    }

    usize UdpSocket::receive(PeerAddress& from, std::span<u8> buffer)
    {
        sockaddr_in address{};
#if defined(_WIN32)
        int addressSize = static_cast<int>(sizeof(address));
#else
        socklen_t addressSize = sizeof(address);
#endif
        const auto received = ::recvfrom(static_cast<SocketHandle>(m_handle),
                                         reinterpret_cast<char*>(buffer.data()),
#if defined(_WIN32)
                                         static_cast<int>(buffer.size()),
#else
                                         buffer.size(),
#endif
                                         0, reinterpret_cast<sockaddr*>(&address),
                                         &addressSize);
        if (received <= 0)
        {
            const int error = lastSocketError();
            if (received < 0 && !wouldBlock(error))
            {
                // On Windows an ICMP "port unreachable" surfaces here as an
                // error on a CONNECTIONLESS socket, which is noise, not a
                // fault: log it once at debug level and carry on.
                SW_LOG_DEBUG("Network", "recvfrom returned {}", error);
            }
            return 0;
        }

        from.ipv4 = ntohl(address.sin_addr.s_addr);
        from.port = ntohs(address.sin_port);
        return static_cast<usize>(received);
    }

    // ------------------------------------------------------------------------
    // LoopbackNetwork
    // ------------------------------------------------------------------------

    class LoopbackTransport final : public Transport
    {
    public:
        LoopbackTransport(LoopbackNetwork& network, u16 port)
            : m_network(network)
            , m_address(PeerAddress::localhost(port))
        {
        }

        bool send(const PeerAddress& to, std::span<const u8> bytes) override
        {
            return m_network.submit(m_address, to, bytes);
        }

        usize receive(PeerAddress& from, std::span<u8> buffer) override
        {
            return m_network.collect(m_address.port, from, buffer);
        }

        [[nodiscard]] u16 localPort() const override { return m_address.port; }

    private:
        LoopbackNetwork& m_network;
        PeerAddress m_address{};
    };

    void LoopbackNetwork::setConditions(const Conditions& conditions)
    {
        m_conditions = conditions;
        m_state = conditions.seed != 0 ? conditions.seed : 0x9E3779B97F4A7C15ull;
    }

    std::unique_ptr<Transport> LoopbackNetwork::endpoint(u16 port)
    {
        return std::make_unique<LoopbackTransport>(*this, port);
    }

    f64 LoopbackNetwork::nextRandom()
    {
        // splitmix64: small, fast, and identical on every platform, which is
        // the entire point of using our own rather than <random> (whose
        // distributions are explicitly allowed to differ between libraries).
        m_state += 0x9E3779B97F4A7C15ull;
        u64 z = m_state;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        z ^= z >> 31;
        return static_cast<f64>(z >> 11) / static_cast<f64>(1ull << 53);
    }

    bool LoopbackNetwork::submit(const PeerAddress& from, const PeerAddress& to,
                                 std::span<const u8> bytes)
    {
        ++m_sent;
        if (m_conditions.lossRatio > 0.0f &&
            nextRandom() < static_cast<f64>(m_conditions.lossRatio))
        {
            ++m_dropped;
            return true; // the sender cannot tell, and must not be able to
        }

        InFlight entry{};
        entry.from = from;
        entry.to = to;
        entry.bytes.assign(bytes.begin(), bytes.end());
        entry.arriveAt = m_now + m_conditions.latencySeconds;
        if (m_conditions.jitterSeconds > 0.0)
        {
            entry.arriveAt += nextRandom() * m_conditions.jitterSeconds;
        }
        entry.order = m_nextOrder++;
        m_wire.push_back(std::move(entry));
        return true;
    }

    usize LoopbackNetwork::collect(u16 port, PeerAddress& from, std::span<u8> buffer)
    {
        // Earliest arrival first; ties broken by send order so a zero-latency
        // wire is exactly first-in first-out.
        usize best = m_wire.size();
        for (usize i = 0; i < m_wire.size(); ++i)
        {
            const InFlight& entry = m_wire[i];
            if (entry.to.port != port || entry.arriveAt > m_now)
            {
                continue;
            }
            if (best == m_wire.size() || entry.arriveAt < m_wire[best].arriveAt ||
                (entry.arriveAt == m_wire[best].arriveAt && entry.order < m_wire[best].order))
            {
                best = i;
            }
        }
        if (best == m_wire.size())
        {
            return 0;
        }

        InFlight entry = std::move(m_wire[best]);
        m_wire.erase(m_wire.begin() + static_cast<std::ptrdiff_t>(best));

        if (entry.order < m_deliveredOrder)
        {
            ++m_reordered;
        }
        else
        {
            m_deliveredOrder = entry.order;
        }

        const usize size = std::min(entry.bytes.size(), buffer.size());
        std::memcpy(buffer.data(), entry.bytes.data(), size);
        from = entry.from;
        return size;
    }
} // namespace sw::net
