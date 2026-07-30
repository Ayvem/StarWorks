#pragma once

// ============================================================================
// Network/Replication.hpp
// What actually crosses the wire: the difference between the world the host
// has now and the world the client last CONFIRMED it had.
//
// THE ONE DECISION EVERYTHING ELSE FOLLOWS FROM: the client's world carries
// the SAME ENTITY INDICES as the host's. It is a mirror, not a translation.
//
// The alternative — give each replicated entity a network id and map it to
// a locally allocated entity — breaks on the first component that holds an
// entity handle, and this game is full of them: a conveyor names its body
// and its link, a cable names its two poles, a cloud deck names its planet,
// a part names its vessel. Nothing declares which of a component's fields
// are handles, so nothing could rewrite them. The save file reached the same
// conclusion years earlier and restores indices exactly for exactly this
// reason; replication simply does it incrementally.
//
// THE SECOND DECISION: a delta is computed against the last snapshot the
// client ACKNOWLEDGED, never against the last one sent. On a lossy link
// those differ, and diffing against something the client never received
// produces a world that is wrong and stays wrong. Diffing against the last
// confirmed state means a lost snapshot costs bandwidth (the next one
// carries more) and nothing else. There is no repair path because there is
// nothing to repair.
//
// Change detection is a memcmp. Components are trivially copyable by ECS
// rule, so "did this change" is a byte comparison and needs no per-component
// code, no dirty flags, and no chance of a system forgetting to raise one.
// ============================================================================

#include "Core/Types.hpp"
#include "ECS/Component.hpp"
#include "ECS/Entity.hpp"
#include "Save/Snapshot.hpp"
#include "Serialization/Binary.hpp"

#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace sw::ecs
{
    class World;
} // namespace sw::ecs

namespace sw::net
{
    /// Which components a session replicates, by the same stable names the
    /// save file uses. Deliberately a WHITELIST: a component nobody named is
    /// a component that stays home, so adding one to the game cannot
    /// silently add it to everyone's bandwidth bill.
    class ReplicationSet
    {
    public:
        ReplicationSet& include(std::string_view name);
        [[nodiscard]] bool includes(std::string_view name) const;
        [[nodiscard]] const std::vector<std::string>& names() const { return m_names; }

    private:
        std::vector<std::string> m_names;
    };

    /// The component table both ends agree on, sent once at connect.
    ///
    /// Wire ids are indices into THIS table, never runtime component type
    /// ids. Runtime ids come from the order types were first touched, which
    /// differs between a host and a client that took different code paths on
    /// the way to the same world — a difference that would be invisible
    /// until it silently reinterpreted one component's bytes as another's.
    class ReplicationTable
    {
    public:
        struct Entry
        {
            std::string name;
            ecs::ComponentTypeId typeId = 0;
            u32 size = 0;
            u32 version = 0;
        };

        /// Every schema entry the set names, in schema order. Throws if the
        /// set names something the schema does not have — a typo there would
        /// otherwise cost a component that silently never replicates.
        [[nodiscard]] static ReplicationTable build(const save::Schema& schema,
                                                    const ReplicationSet& set);

        void write(ser::BinaryWriter& writer) const;
        /// Resolves the received names against the LOCAL schema. Throws on a
        /// size or version mismatch: two builds that disagree about a
        /// component's layout must not play together, and finding out at the
        /// handshake is infinitely better than finding out in flight.
        [[nodiscard]] static ReplicationTable read(ser::BinaryReader& reader,
                                                   const save::Schema& schema);

        [[nodiscard]] const std::vector<Entry>& entries() const { return m_entries; }
        [[nodiscard]] usize size() const { return m_entries.size(); }
        /// -1 when the type is not replicated.
        [[nodiscard]] i32 localIdOf(ecs::ComponentTypeId typeId) const;

    private:
        std::vector<Entry> m_entries;
        /// runtime type id -> wire id, or -1. Sized to kMaxComponentTypes so
        /// the lookup on the capture hot path is an array index.
        i32 m_byTypeId[ecs::kMaxComponentTypes]{};
    };

    /// A flattened copy of everything replicated, at one instant.
    ///
    /// Sorted by (entity index, wire component id) so that diffing two of
    /// them is a single merge walk — no hashing, no allocation, and a
    /// deterministic order that makes a snapshot's bytes reproducible.
    struct WorldState
    {
        struct EntityRef
        {
            u32 index = 0;
            u32 generation = 0;
        };
        struct Record
        {
            u32 entityIndex = 0;
            u16 componentId = 0;
            u32 offset = 0; // into blob
            u32 size = 0;
        };

        std::vector<EntityRef> entities; // sorted by index
        std::vector<Record> records;     // sorted by (entityIndex, componentId)
        std::vector<u8> blob;

        void clear();
        [[nodiscard]] usize byteSize() const { return blob.size(); }
    };

    /// Walks the world once and fills `out`. Only entities carrying at least
    /// one replicated component appear.
    void captureState(const ecs::World& world, const ReplicationTable& table,
                      WorldState& out);

    struct SnapshotStats
    {
        u32 entityCount = 0;
        u32 recordCount = 0;
        u32 spawned = 0;
        u32 removed = 0;
        u32 changed = 0;
        u32 dropped = 0;
        usize payloadBytes = 0;
        bool wasFull = false;
    };

    /// HOST SIDE, one per client — because "the last state this client
    /// confirmed" is per client and nothing else about the encoder is.
    class ReplicationEncoder
    {
    public:
        explicit ReplicationEncoder(ReplicationTable table) : m_table(std::move(table)) {}

        /// Captures the world and writes the delta against the last
        /// acknowledged baseline. Returns the new snapshot's id, which the
        /// caller sends and the client eventually acknowledges.
        u32 encode(const ecs::World& world, f64 simulatedSeconds, ser::BinaryWriter& out);

        /// The client says it has applied this snapshot in full. Older
        /// history is dropped; an id we no longer hold is ignored, and the
        /// next encode falls back to a full snapshot.
        void acknowledge(u32 snapshotId);

        [[nodiscard]] u32 lastSnapshotId() const { return m_lastId; }
        [[nodiscard]] u32 acknowledgedId() const { return m_acknowledged; }
        [[nodiscard]] const SnapshotStats& lastStats() const { return m_stats; }
        [[nodiscard]] const ReplicationTable& table() const { return m_table; }

        /// How many past states are retained while waiting for acks. Beyond
        /// this the oldest is dropped and the client is served a full
        /// snapshot instead — bandwidth, not breakage.
        static constexpr usize kMaxHistory = 16;

    private:
        struct HistoryEntry
        {
            u32 id = 0;
            WorldState state;
        };

        ReplicationTable m_table;
        u32 m_lastId = 0;
        u32 m_acknowledged = 0;
        std::vector<HistoryEntry> m_history; // oldest first
        WorldState m_current;
        SnapshotStats m_stats{};
    };

    /// CLIENT SIDE. Applies snapshots to the mirror world.
    class ReplicationDecoder
    {
    public:
        explicit ReplicationDecoder(ReplicationTable table) : m_table(std::move(table)) {}

        /// Applies one snapshot. Returns false, having touched nothing, when
        /// its baseline is not the snapshot we hold — which happens whenever
        /// a snapshot is lost, and is not an error: our acknowledgement still
        /// names the state we do have, and the host re-bases onto it.
        bool apply(ecs::World& world, ser::BinaryReader& reader);

        [[nodiscard]] u32 appliedSnapshotId() const { return m_applied; }
        [[nodiscard]] f64 simulatedSeconds() const { return m_simulatedSeconds; }
        [[nodiscard]] const ReplicationTable& table() const { return m_table; }

    private:
        ReplicationTable m_table;
        u32 m_applied = 0;
        f64 m_simulatedSeconds = 0.0;
    };
} // namespace sw::net
