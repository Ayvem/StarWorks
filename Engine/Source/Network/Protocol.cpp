#include "Network/Protocol.hpp"

#include "Core/Error.hpp"

#include <algorithm>
#include <cstring>

namespace sw::net
{
    namespace
    {
        // Byte offsets inside the fixed header. The acknowledgement fields
        // are PATCHED into an already-built datagram at send time: a
        // reliable datagram may sit in the window for several round trips,
        // and it must carry the freshest ack we have when it finally goes
        // out, not the one that was current when it was queued.
        constexpr usize kOffsetFlags = 7;
        constexpr usize kOffsetAck = 10;
        constexpr usize kOffsetAckBits = 12;

        void patchAck(std::vector<u8>& bytes, bool haveAck, u16 ack, u32 ackBits)
        {
            if (bytes.size() < kHeaderBytes)
            {
                return;
            }
            u8 flags = bytes[kOffsetFlags];
            flags = haveAck ? static_cast<u8>(flags | PacketHeader::kFlagAck)
                            : static_cast<u8>(flags & ~PacketHeader::kFlagAck);
            bytes[kOffsetFlags] = flags;
            std::memcpy(bytes.data() + kOffsetAck, &ack, sizeof(ack));
            std::memcpy(bytes.data() + kOffsetAckBits, &ackBits, sizeof(ackBits));
        }
    } // namespace

    std::string_view messageTypeName(MessageType type)
    {
        switch (type)
        {
            case MessageType::ConnectRequest: return "ConnectRequest";
            case MessageType::ConnectAccept: return "ConnectAccept";
            case MessageType::ConnectReject: return "ConnectReject";
            case MessageType::ConnectChallenge: return "ConnectChallenge";
            case MessageType::Disconnect: return "Disconnect";
            case MessageType::Snapshot: return "Snapshot";
            case MessageType::SnapshotAck: return "SnapshotAck";
            case MessageType::Command: return "Command";
            case MessageType::Clock: return "Clock";
            case MessageType::Roster: return "Roster";
            case MessageType::Event: return "Event";
            case MessageType::Ping: return "Ping";
            case MessageType::Pong: return "Pong";
            case MessageType::Invalid:
            case MessageType::Count:
            default: return "Invalid";
        }
    }

    void requireWireCount(u64 count, usize minElementBytes, usize remaining,
                          std::string_view what)
    {
        // Divide rather than multiply. A bounds check that overflows is a
        // bounds check that passes, and count comes off the wire: on a
        // 32-bit usize `count * minElementBytes` wraps for exactly the
        // enormous values this function exists to refuse.
        const usize element = (minElementBytes == 0) ? 1 : minElementBytes;
        if (count > static_cast<u64>(remaining / element))
        {
            SW_THROW("{} claims {} entries of at least {} bytes each, but only {} bytes "
                     "remain in the message",
                     what, count, element, remaining);
        }
    }

    void buildBareDatagram(MessageType type, std::span<const u8> payload,
                           std::vector<u8>& out)
    {
        PacketHeader header{};
        header.type = type;
        header.sequence = 0;
        ser::BinaryWriter writer;
        writeHeader(writer, header);
        writer.writeBytes(payload.data(), payload.size());
        out = writer.bytes();
    }

    void writeHeader(ser::BinaryWriter& writer, const PacketHeader& header)
    {
        writer.write(header.magic);
        writer.write(header.version);
        writer.write(static_cast<u8>(header.type));
        writer.write(header.flags);
        writer.write(header.sequence);
        writer.write(header.ack);
        writer.write(header.ackBits);
        if ((header.flags & PacketHeader::kFlagFragment) != 0)
        {
            writer.write(header.messageId);
            writer.write(header.fragmentIndex);
            writer.write(header.fragmentCount);
        }
    }

    bool readHeader(ser::BinaryReader& reader, PacketHeader& header)
    {
        if (reader.remaining() < kHeaderBytes)
        {
            return false;
        }
        header.magic = reader.read<u32>();
        if (header.magic != kProtocolMagic)
        {
            return false;
        }
        header.version = reader.read<u16>();
        if (header.version != kProtocolVersion)
        {
            return false;
        }
        const u8 rawType = reader.read<u8>();
        if (rawType == 0 || rawType >= static_cast<u8>(MessageType::Count))
        {
            return false;
        }
        header.type = static_cast<MessageType>(rawType);
        header.flags = reader.read<u8>();
        header.sequence = reader.read<u16>();
        header.ack = reader.read<u16>();
        header.ackBits = reader.read<u32>();

        header.messageId = 0;
        header.fragmentIndex = 0;
        header.fragmentCount = 0;
        if ((header.flags & PacketHeader::kFlagFragment) != 0)
        {
            if (reader.remaining() < kFragmentHeaderBytes)
            {
                return false;
            }
            header.messageId = reader.read<u16>();
            header.fragmentIndex = reader.read<u16>();
            header.fragmentCount = reader.read<u16>();
            if (header.fragmentCount == 0 || header.fragmentIndex >= header.fragmentCount)
            {
                return false;
            }
        }
        return true;
    }

    // ------------------------------------------------------------------------
    // Connection
    // ------------------------------------------------------------------------

    void Connection::buildDatagram(std::vector<u8>& bytes, const PacketHeader& header,
                                   std::span<const u8> payload) const
    {
        ser::BinaryWriter writer;
        writeHeader(writer, header);
        writer.writeBytes(payload.data(), payload.size());
        bytes = writer.bytes();
    }

    void Connection::queueReliable(MessageType type, std::span<const u8> payload)
    {
        if (payload.size() <= kMaxUnreliablePayload)
        {
            PacketHeader header{};
            header.type = type;
            header.flags = PacketHeader::kFlagReliable;
            header.sequence = m_nextReliableSequence++;
            PendingOut entry{};
            entry.sequence = header.sequence;
            buildDatagram(entry.bytes, header, payload);
            m_inFlight.push_back(std::move(entry));
            return;
        }

        // Fragmented. The pieces take CONSECUTIVE reliable sequences, which
        // is what lets the receiver reassemble by simply walking the channel
        // in order.
        const usize total = payload.size();
        const usize count = (total + kMaxFragmentPayload - 1) / kMaxFragmentPayload;
        if (count > kMaxFragments)
        {
            // The receiver abandons a reassembly past kMaxReassemblyBytes, so
            // putting this on the wire would burn the whole reliable window
            // on something guaranteed to be discarded — and, worse, would
            // stall every reliable message queued behind it forever.
            SW_THROW("Reliable message of {} bytes needs {} fragments (limit {}, which is "
                     "the {}-byte reassembly ceiling the receiver enforces)",
                     total, count, kMaxFragments, kMaxReassemblyBytes);
        }
        const u16 messageId = m_nextMessageId++;
        for (usize i = 0; i < count; ++i)
        {
            const usize offset = i * kMaxFragmentPayload;
            const usize size = std::min(kMaxFragmentPayload, total - offset);

            PacketHeader header{};
            header.type = type;
            header.flags = PacketHeader::kFlagFragment | PacketHeader::kFlagReliable;
            header.sequence = m_nextReliableSequence++;
            header.messageId = messageId;
            header.fragmentIndex = static_cast<u16>(i);
            header.fragmentCount = static_cast<u16>(count);

            PendingOut entry{};
            entry.sequence = header.sequence;
            buildDatagram(entry.bytes, header, payload.subspan(offset, size));
            m_inFlight.push_back(std::move(entry));
        }
    }

    void Connection::queueUnreliable(MessageType type, std::span<const u8> payload)
    {
        if (payload.size() > kMaxUnreliablePayload)
        {
            SW_THROW("Unreliable {} message is {} bytes; one datagram holds {}",
                     messageTypeName(type), payload.size(), kMaxUnreliablePayload);
        }
        PacketHeader header{};
        header.type = type;
        header.sequence = m_nextUnreliableSequence++;
        std::vector<u8> bytes;
        buildDatagram(bytes, header, payload);
        m_unreliableOut.push_back(std::move(bytes));
    }

    void Connection::collectOutgoing(f64 nowSeconds, std::vector<std::vector<u8>>& out)
    {
        const usize before = out.size();

        // Retire the confirmed prefix. Entries are only ever appended in
        // sequence order, so the acked run at the front is contiguous.
        usize retire = 0;
        while (retire < m_inFlight.size() && m_inFlight[retire].acked)
        {
            ++retire;
        }
        if (retire > 0)
        {
            m_inFlight.erase(m_inFlight.begin(),
                             m_inFlight.begin() + static_cast<std::ptrdiff_t>(retire));
        }

        // 1.5 x the measured round trip, floored by the configured delay: a
        // resend fired before an ack could possibly have come back is pure
        // waste, and on a long link it is waste that repeats.
        const f64 resendAfter = std::max(m_config.resendDelaySeconds, 1.5 * m_roundTrip);

        u32 outstanding = 0;
        for (const PendingOut& entry : m_inFlight)
        {
            if (!entry.acked && entry.sentAt >= 0.0)
            {
                ++outstanding;
            }
        }

        u32 unsent = 0;
        for (PendingOut& entry : m_inFlight)
        {
            if (entry.acked)
            {
                continue;
            }
            if (entry.sentAt < 0.0)
            {
                if (outstanding >= kReliableWindow)
                {
                    ++unsent;
                    continue;
                }
                entry.sentAt = nowSeconds;
                ++outstanding;
                patchAck(entry.bytes, m_haveRemoteAck, m_remoteAck, m_remoteAckBits);
                out.push_back(entry.bytes);
                continue;
            }
            if (nowSeconds - entry.sentAt >= resendAfter)
            {
                entry.sentAt = nowSeconds;
                ++m_stats.resends;
                patchAck(entry.bytes, m_haveRemoteAck, m_remoteAck, m_remoteAckBits);
                out.push_back(entry.bytes);
            }
        }
        m_stats.reliableBacklog = unsent;

        for (std::vector<u8>& bytes : m_unreliableOut)
        {
            patchAck(bytes, m_haveRemoteAck, m_remoteAck, m_remoteAckBits);
            out.push_back(std::move(bytes));
        }
        m_unreliableOut.clear();

        // Nothing to say, but an acknowledgement is owed and the peer is
        // waiting on it before it will send anything more. A bare Ping is
        // the cheapest envelope for it.
        if (out.size() == before && m_ackOwed &&
            (m_lastSend < 0.0 || nowSeconds - m_lastSend >= m_config.ackDelaySeconds))
        {
            PacketHeader header{};
            header.type = MessageType::Ping;
            header.sequence = m_nextUnreliableSequence++;
            std::vector<u8> bytes;
            buildDatagram(bytes, header, {});
            patchAck(bytes, m_haveRemoteAck, m_remoteAck, m_remoteAckBits);
            out.push_back(std::move(bytes));
        }

        if (out.size() > before)
        {
            m_ackOwed = false;
            m_lastSend = nowSeconds;
            for (usize i = before; i < out.size(); ++i)
            {
                ++m_stats.datagramsSent;
                m_stats.bytesSent += out[i].size();
            }
        }
    }

    void Connection::recordRemoteSequence(u16 sequence)
    {
        if (!m_haveRemoteAck)
        {
            m_remoteAck = sequence;
            m_remoteAckBits = 0;
            m_haveRemoteAck = true;
            return;
        }
        if (sequenceNewer(sequence, m_remoteAck))
        {
            const i32 shift = sequenceDelta(sequence, m_remoteAck);
            if (shift >= 32)
            {
                m_remoteAckBits = 0;
            }
            else
            {
                m_remoteAckBits = (m_remoteAckBits << shift) | (1u << (shift - 1));
            }
            m_remoteAck = sequence;
            return;
        }
        const i32 back = sequenceDelta(m_remoteAck, sequence);
        if (back >= 1 && back <= 32)
        {
            m_remoteAckBits |= (1u << (back - 1));
        }
    }

    void Connection::ackOne(f64 nowSeconds, u16 sequence)
    {
        for (PendingOut& entry : m_inFlight)
        {
            if (entry.sequence != sequence || entry.acked)
            {
                continue;
            }
            entry.acked = true;
            if (entry.sentAt >= 0.0)
            {
                const f64 sample = nowSeconds - entry.sentAt;
                m_roundTrip = (m_roundTrip <= 0.0) ? sample : m_roundTrip * 0.9 + sample * 0.1;
            }
            return;
        }
    }

    void Connection::applyAck(f64 nowSeconds, u16 ack, u32 ackBits)
    {
        ackOne(nowSeconds, ack);
        for (u32 bit = 0; bit < 32; ++bit)
        {
            if ((ackBits & (1u << bit)) != 0)
            {
                ackOne(nowSeconds, static_cast<u16>(ack - 1 - bit));
            }
        }
    }

    void Connection::abandonReassembly()
    {
        ++m_stats.fragmentsRejected;
        // shrink_to_fit, not clear(): clear() leaves the capacity behind, and
        // the capacity is the memory a peer was trying to make us hold. A
        // reassembly abandoned at eight megabytes must give the eight
        // megabytes back, or refusing it bought nothing.
        m_partial.clear();
        m_partial.shrink_to_fit();
        m_partialActive = false;
        m_partialMessageId = 0;
        m_partialCount = 0;
        m_partialNextIndex = 0;
    }

    bool Connection::acceptFragment(const PendingIn& slot)
    {
        if (slot.fragmentIndex == 0)
        {
            // A new message starts here. Whatever was half-assembled before
            // it is dead — an honest sender would have finished it first.
            if (m_partialActive)
            {
                abandonReassembly();
            }
            m_partialActive = true;
            m_partialMessageId = slot.messageId;
            m_partialCount = slot.fragmentCount;
            m_partialNextIndex = 0;
        }
        else if (!m_partialActive || slot.messageId != m_partialMessageId ||
                 slot.fragmentCount != m_partialCount ||
                 slot.fragmentIndex != m_partialNextIndex)
        {
            // THE THREE LIES THIS REFUSES, all of which the old code
            // believed: a continuation of a message that never began, a
            // continuation whose message id belongs to a DIFFERENT message
            // (whose bytes would have been spliced onto ours and handed up
            // as one plausible payload), and an index that skips or repeats
            // (which would have silently reordered the payload).
            abandonReassembly();
            return false;
        }

        if (m_partial.size() + slot.payload.size() > kMaxReassemblyBytes)
        {
            // The bound that stops a peer pinning memory with a message it
            // never intends to finish. queueReliable refuses to build one
            // this large, so nothing legitimate can reach here.
            abandonReassembly();
            return false;
        }

        m_partial.insert(m_partial.end(), slot.payload.begin(), slot.payload.end());
        ++m_partialNextIndex;
        return true;
    }

    void Connection::drainInOrder()
    {
        for (;;)
        {
            PendingIn& slot = m_received[m_nextDelivery % kHoldingPen];
            if (!slot.used || slot.sequence != m_nextDelivery)
            {
                return;
            }

            if (!slot.fragment)
            {
                // An honest sender never interleaves: the fragments of one
                // message take consecutive sequences with nothing between
                // them. A whole message appearing mid-reassembly therefore
                // says the pieces held so far will never be completed, so
                // the buffer goes now rather than sitting there until
                // something else happens to clear it.
                if (m_partialActive)
                {
                    abandonReassembly();
                }
                m_delivered.push_back(Message{slot.type, std::move(slot.payload)});
            }
            else if (acceptFragment(slot))
            {
                if (m_partialNextIndex == m_partialCount)
                {
                    m_delivered.push_back(Message{slot.type, std::move(m_partial)});
                    m_partial.clear();
                    m_partialActive = false;
                }
            }

            slot.used = false;
            slot.payload.clear();
            ++m_nextDelivery;
        }
    }

    void Connection::receive(f64 nowSeconds, std::span<const u8> bytes)
    {
        if (bytes.size() > kMaxDatagramBytes)
        {
            // Nothing this protocol sends is bigger, so nothing this size is
            // ours. The check is what makes the holding pen's worst case
            // arithmetic rather than a hope: 256 slots of at most
            // kMaxDatagramBytes is 300 kB and cannot become anything else.
            ++m_stats.malformedDropped;
            return;
        }

        ser::BinaryReader reader(bytes);
        PacketHeader header{};
        if (!readHeader(reader, header))
        {
            ++m_stats.malformedDropped;
            return;
        }

        if ((header.flags & PacketHeader::kFlagFragment) != 0 &&
            header.fragmentCount > kMaxFragments)
        {
            // Rejecting here rather than when the buffer finally overflows
            // means a peer announcing a 77 MB message (the 65535 fragments
            // the header field allows) never gets to start one.
            ++m_stats.malformedDropped;
            return;
        }

        ++m_stats.datagramsReceived;
        m_stats.bytesReceived += bytes.size();
        m_lastReceive = nowSeconds;

        if ((header.flags & PacketHeader::kFlagAck) != 0)
        {
            applyAck(nowSeconds, header.ack, header.ackBits);
        }

        std::vector<u8> payload(reader.remaining());
        if (!payload.empty())
        {
            reader.readBytes(payload.data(), payload.size());
        }

        if ((header.flags & PacketHeader::kFlagReliable) == 0)
        {
            // Sequenced: an older arrival than the newest already applied is
            // stale truth and is thrown away rather than applied backwards.
            const auto slot = static_cast<usize>(header.type);
            if (m_sawUnreliable[slot] && !sequenceNewer(header.sequence, m_newestUnreliable[slot]))
            {
                ++m_stats.staleDropped;
                return;
            }
            m_sawUnreliable[slot] = true;
            m_newestUnreliable[slot] = header.sequence;

            // A Ping exists only to carry an acknowledgement and to prove the
            // link is alive; there is nothing above this layer to hand it to.
            if (header.type != MessageType::Ping)
            {
                m_delivered.push_back(Message{header.type, std::move(payload)});
            }
            return;
        }

        // Reliable. Acknowledge it whether or not it is new: a duplicate
        // means our previous ack was lost, and staying silent would leave
        // the sender resending forever.
        recordRemoteSequence(header.sequence);
        m_ackOwed = true;

        if (sequenceDelta(header.sequence, m_nextDelivery) < 0)
        {
            ++m_stats.duplicatesReceived; // already handed up
            return;
        }
        if (sequenceDelta(header.sequence, m_nextDelivery) >= static_cast<i32>(kHoldingPen))
        {
            // Impossible while the sender honours the window; refusing it is
            // still better than overwriting a slot that is owed.
            ++m_stats.malformedDropped;
            return;
        }

        PendingIn& slot = m_received[header.sequence % kHoldingPen];
        if (slot.used && slot.sequence == header.sequence)
        {
            ++m_stats.duplicatesReceived;
            return;
        }
        slot.used = true;
        slot.sequence = header.sequence;
        slot.type = header.type;
        slot.fragment = (header.flags & PacketHeader::kFlagFragment) != 0;
        slot.messageId = header.messageId;
        slot.fragmentIndex = header.fragmentIndex;
        slot.fragmentCount = header.fragmentCount;
        slot.payload = std::move(payload);

        drainInOrder();
    }
} // namespace sw::net
