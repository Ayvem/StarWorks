#include "Network/Replication.hpp"

#include "Core/Error.hpp"
#include "ECS/Archetype.hpp"
#include "ECS/World.hpp"
#include "Network/Protocol.hpp"

#include <algorithm>
#include <cstring>

namespace sw::net
{
    namespace
    {
        constexpr u32 kSnapshotMagic = 0x53575250u; // 'SWRP'
        constexpr u32 kTableMagic = 0x53575254u;    // 'SWRT'
    } // namespace

    ReplicationSet& ReplicationSet::include(std::string_view name)
    {
        if (!includes(name))
        {
            m_names.emplace_back(name);
        }
        return *this;
    }

    bool ReplicationSet::includes(std::string_view name) const
    {
        return std::find(m_names.begin(), m_names.end(), name) != m_names.end();
    }

    // ------------------------------------------------------------------------
    // ReplicationTable
    // ------------------------------------------------------------------------

    ReplicationTable ReplicationTable::build(const save::Schema& schema,
                                             const ReplicationSet& set)
    {
        ReplicationTable table;
        std::fill(std::begin(table.m_byTypeId), std::end(table.m_byTypeId), -1);

        for (const std::string& name : set.names())
        {
            if (schema.findByName(name) == nullptr)
            {
                SW_THROW("Replication set names '{}', which the save schema does not "
                         "declare — a component nobody can serialize cannot be replicated",
                         name);
            }
        }

        // Schema order, not set order: the table is part of the wire contract
        // and a stable order makes two hosts built from the same schema
        // produce byte-identical tables.
        for (const save::Schema::Entry& entry : schema.entries())
        {
            if (!set.includes(entry.name))
            {
                continue;
            }
            if (table.m_entries.size() > 0xFFFFu)
            {
                SW_THROW("Replication table overflow (wire ids are 16 bits)");
            }
            table.m_byTypeId[entry.typeId] = static_cast<i32>(table.m_entries.size());
            table.m_entries.push_back(Entry{entry.name, entry.typeId, entry.size,
                                            entry.version});
        }
        return table;
    }

    i32 ReplicationTable::localIdOf(ecs::ComponentTypeId typeId) const
    {
        return typeId < ecs::kMaxComponentTypes ? m_byTypeId[typeId] : -1;
    }

    void ReplicationTable::write(ser::BinaryWriter& writer) const
    {
        writer.write(kTableMagic);
        writer.write(static_cast<u32>(m_entries.size()));
        for (const Entry& entry : m_entries)
        {
            writer.writeString(entry.name);
            writer.write(entry.size);
            writer.write(entry.version);
        }
    }

    ReplicationTable ReplicationTable::read(ser::BinaryReader& reader,
                                            const save::Schema& schema)
    {
        if (reader.read<u32>() != kTableMagic)
        {
            SW_THROW("Malformed replication table");
        }
        ReplicationTable table;
        std::fill(std::begin(table.m_byTypeId), std::end(table.m_byTypeId), -1);

        const u32 count = reader.read<u32>();
        if (count > 0xFFFFu)
        {
            SW_THROW("Replication table claims {} components", count);
        }
        // 0xFFFF entries still reserve nothing here, but each one costs a
        // std::string and two u32s to build — so the count is also checked
        // against what is actually left to read. An entry cannot occupy
        // fewer than twelve bytes (a four-byte name length, then size and
        // version), so a table claiming more than remaining/12 entries is
        // describing bytes that are not there.
        requireWireCount(count, 12, reader.remaining(), "Replication table");
        for (u32 i = 0; i < count; ++i)
        {
            const std::string name = reader.readString();
            const u32 size = reader.read<u32>();
            const u32 version = reader.read<u32>();

            const save::Schema::Entry* entry = schema.findByName(name);
            if (entry == nullptr)
            {
                SW_THROW("Host replicates component '{}', which this build does not know",
                         name);
            }
            if (entry->version != version || entry->size != size)
            {
                SW_THROW("Component '{}' differs between host and client (host v{} {}B, "
                         "local v{} {}B) — the builds are not compatible",
                         name, version, size, entry->version, entry->size);
            }
            table.m_byTypeId[entry->typeId] = static_cast<i32>(table.m_entries.size());
            table.m_entries.push_back(Entry{name, entry->typeId, size, version});
        }
        return table;
    }

    // ------------------------------------------------------------------------
    // capture
    // ------------------------------------------------------------------------

    void WorldState::clear()
    {
        entities.clear();
        records.clear();
        blob.clear();
    }

    void captureState(const ecs::World& world, const ReplicationTable& table,
                      WorldState& out)
    {
        out.clear();

        struct Column
        {
            u16 wireId = 0;
            u32 size = 0;
            const std::byte* data = nullptr;
        };
        std::vector<Column> columns;

        for (const ecs::Archetype* archetype : world.archetypeList())
        {
            const u32 entityCount = archetype->entityCount();
            if (entityCount == 0)
            {
                continue;
            }

            columns.clear();
            for (u32 c = 0; c < archetype->columnCount(); ++c)
            {
                const ecs::ComponentInfo& info = archetype->columnInfoAt(c);
                const i32 wireId = table.localIdOf(info.id);
                if (wireId < 0)
                {
                    continue;
                }
                columns.push_back(
                    Column{static_cast<u16>(wireId), info.size, archetype->columnDataAt(c)});
            }
            if (columns.empty())
            {
                continue; // nothing here replicates; the whole archetype is local
            }

            for (u32 row = 0; row < entityCount; ++row)
            {
                const ecs::Entity entity = archetype->entityAt(row);
                out.entities.push_back(WorldState::EntityRef{entity.index, entity.generation});
                for (const Column& column : columns)
                {
                    const auto* source = reinterpret_cast<const u8*>(
                        column.data + static_cast<usize>(row) * column.size);
                    WorldState::Record record{};
                    record.entityIndex = entity.index;
                    record.componentId = column.wireId;
                    record.offset = static_cast<u32>(out.blob.size());
                    record.size = column.size;
                    out.blob.insert(out.blob.end(), source, source + column.size);
                    out.records.push_back(record);
                }
            }
        }

        std::sort(out.entities.begin(), out.entities.end(),
                  [](const WorldState::EntityRef& a, const WorldState::EntityRef& b) {
                      return a.index < b.index;
                  });
        std::sort(out.records.begin(), out.records.end(),
                  [](const WorldState::Record& a, const WorldState::Record& b) {
                      if (a.entityIndex != b.entityIndex)
                      {
                          return a.entityIndex < b.entityIndex;
                      }
                      return a.componentId < b.componentId;
                  });
    }

    // ------------------------------------------------------------------------
    // ReplicationEncoder
    // ------------------------------------------------------------------------

    void ReplicationEncoder::acknowledge(u32 snapshotId)
    {
        if (snapshotId == 0 || snapshotId <= m_acknowledged)
        {
            return;
        }
        const auto it = std::find_if(m_history.begin(), m_history.end(),
                                     [&](const HistoryEntry& e) { return e.id == snapshotId; });
        if (it == m_history.end())
        {
            return; // too old to be a baseline; the next snapshot will be full
        }
        m_acknowledged = snapshotId;
        m_history.erase(m_history.begin(), it); // keep the baseline itself
    }

    u32 ReplicationEncoder::encode(const ecs::World& world, f64 simulatedSeconds,
                                   ser::BinaryWriter& out)
    {
        captureState(world, m_table, m_current);

        const WorldState* baseline = nullptr;
        if (m_acknowledged != 0)
        {
            const auto it =
                std::find_if(m_history.begin(), m_history.end(),
                             [&](const HistoryEntry& e) { return e.id == m_acknowledged; });
            if (it != m_history.end())
            {
                baseline = &it->state;
            }
        }

        const u32 snapshotId = ++m_lastId;
        const u32 baselineId = (baseline != nullptr) ? m_acknowledged : 0u;

        // ---- entities: what appeared, what vanished, what was recycled ----
        std::vector<u32> removed;
        std::vector<WorldState::EntityRef> spawned;
        std::vector<u32> forced; // must resend every component, sorted
        {
            usize a = 0;
            usize b = 0;
            const std::vector<WorldState::EntityRef> empty;
            const std::vector<WorldState::EntityRef>& before =
                (baseline != nullptr) ? baseline->entities : empty;
            const std::vector<WorldState::EntityRef>& now = m_current.entities;
            while (a < before.size() || b < now.size())
            {
                if (b >= now.size() || (a < before.size() && before[a].index < now[b].index))
                {
                    removed.push_back(before[a].index);
                    ++a;
                }
                else if (a >= before.size() || now[b].index < before[a].index)
                {
                    spawned.push_back(now[b]);
                    forced.push_back(now[b].index);
                    ++b;
                }
                else
                {
                    if (before[a].generation != now[b].generation)
                    {
                        // The host recycled this index. The mirror must throw
                        // the old occupant away wholesale, which means every
                        // component of the new one is "changed" even where
                        // the bytes happen to match.
                        spawned.push_back(now[b]);
                        forced.push_back(now[b].index);
                    }
                    ++a;
                    ++b;
                }
            }
        }

        // ---- components: changed, and gone ---------------------------------
        struct Change
        {
            u32 entityIndex = 0;
            u16 componentId = 0;
            u32 offset = 0;
            u32 size = 0;
        };
        std::vector<Change> changes;
        std::vector<Change> dropped;
        {
            const std::vector<WorldState::Record> empty;
            const std::vector<WorldState::Record>& before =
                (baseline != nullptr) ? baseline->records : empty;
            const std::vector<WorldState::Record>& now = m_current.records;
            const std::vector<u8>& beforeBlob =
                (baseline != nullptr) ? baseline->blob : m_current.blob;

            const auto key = [](const WorldState::Record& r) {
                return (static_cast<u64>(r.entityIndex) << 16) | r.componentId;
            };

            usize a = 0;
            usize b = 0;
            while (a < before.size() || b < now.size())
            {
                if (b >= now.size() || (a < before.size() && key(before[a]) < key(now[b])))
                {
                    // Present then, absent now. If the whole entity went, the
                    // removal already says so; otherwise the component alone
                    // was taken off and the mirror must take it off too.
                    if (!std::binary_search(removed.begin(), removed.end(),
                                            before[a].entityIndex))
                    {
                        dropped.push_back(Change{before[a].entityIndex,
                                                 before[a].componentId, 0, 0});
                    }
                    ++a;
                }
                else if (a >= before.size() || key(now[b]) < key(before[a]))
                {
                    changes.push_back(Change{now[b].entityIndex, now[b].componentId,
                                             now[b].offset, now[b].size});
                    ++b;
                }
                else
                {
                    const bool mustResend =
                        std::binary_search(forced.begin(), forced.end(), now[b].entityIndex);
                    const bool differs =
                        before[a].size != now[b].size ||
                        std::memcmp(beforeBlob.data() + before[a].offset,
                                    m_current.blob.data() + now[b].offset, now[b].size) != 0;
                    if (mustResend || differs)
                    {
                        changes.push_back(Change{now[b].entityIndex, now[b].componentId,
                                                 now[b].offset, now[b].size});
                    }
                    ++a;
                    ++b;
                }
            }
        }

        // ---- wire ----------------------------------------------------------
        const usize startBytes = out.size();
        out.write(kSnapshotMagic);
        out.write(snapshotId);
        out.write(baselineId);
        out.write(simulatedSeconds);

        out.write(static_cast<u32>(removed.size()));
        for (const u32 index : removed)
        {
            out.write(index);
        }

        out.write(static_cast<u32>(spawned.size()));
        for (const WorldState::EntityRef& entity : spawned)
        {
            out.write(entity.index);
            out.write(entity.generation);
        }

        out.write(static_cast<u32>(dropped.size()));
        for (const Change& change : dropped)
        {
            out.write(change.entityIndex);
            out.write(change.componentId);
        }

        // Changed components are GROUPED BY ENTITY: the entity index is four
        // bytes and most entities that changed at all changed several
        // components, so repeating it per component is pure overhead on the
        // one message that goes out fifty times a second.
        usize entityGroups = 0;
        for (usize i = 0; i < changes.size();)
        {
            usize j = i;
            while (j < changes.size() && changes[j].entityIndex == changes[i].entityIndex)
            {
                ++j;
            }
            ++entityGroups;
            i = j;
        }
        out.write(static_cast<u32>(entityGroups));
        for (usize i = 0; i < changes.size();)
        {
            usize j = i;
            while (j < changes.size() && changes[j].entityIndex == changes[i].entityIndex)
            {
                ++j;
            }
            out.write(changes[i].entityIndex);
            out.write(static_cast<u16>(j - i));
            for (usize k = i; k < j; ++k)
            {
                out.write(changes[k].componentId);
                out.writeBytes(m_current.blob.data() + changes[k].offset, changes[k].size);
            }
            i = j;
        }

        m_stats = SnapshotStats{};
        m_stats.entityCount = static_cast<u32>(m_current.entities.size());
        m_stats.recordCount = static_cast<u32>(m_current.records.size());
        m_stats.spawned = static_cast<u32>(spawned.size());
        m_stats.removed = static_cast<u32>(removed.size());
        m_stats.changed = static_cast<u32>(changes.size());
        m_stats.dropped = static_cast<u32>(dropped.size());
        m_stats.payloadBytes = out.size() - startBytes;
        m_stats.wasFull = (baselineId == 0);

        m_history.push_back(HistoryEntry{snapshotId, std::move(m_current)});
        m_current.clear();
        if (m_history.size() > kMaxHistory)
        {
            m_history.erase(m_history.begin());
            // The dropped state may have been the acknowledged baseline; if
            // so the next encode finds no baseline and sends a full snapshot,
            // which is exactly the right recovery.
        }
        return snapshotId;
    }

    // ------------------------------------------------------------------------
    // ReplicationDecoder
    // ------------------------------------------------------------------------

    bool ReplicationDecoder::apply(ecs::World& world, ser::BinaryReader& reader)
    {
        if (reader.read<u32>() != kSnapshotMagic)
        {
            SW_THROW("Malformed snapshot");
        }
        const u32 snapshotId = reader.read<u32>();
        const u32 baselineId = reader.read<u32>();
        const f64 simulatedSeconds = reader.read<f64>();

        if (baselineId != m_applied)
        {
            // Built on a state we do not have. Nothing has been touched, and
            // our acknowledgement still names what we do have.
            return false;
        }

        const auto handleOf = [&world](u32 index) {
            return ecs::Entity{index, world.slotGeneration(index)};
        };
        const auto slotAlive = [&world](u32 index) {
            return index < world.recordCount() && world.isSlotAlive(index);
        };

        // A SNAPSHOT IS ALL OR NOTHING, and the only way to get that is to
        // read the whole thing before believing any of it.
        //
        // A count or an entity index that is a lie is not discovered until
        // the reader runs off the end, which is arbitrarily far into the
        // message — and everything before that point had already been done
        // to the mirror. MEASURED: a 52-byte snapshot spawning entity 5 and
        // then naming a component it does not carry the bytes for passed
        // every bound, spawned the entity, added a never-written component
        // and only then threw — aliveCount 1, the component present. The
        // caller drops the datagram and carries on, so the mirror keeps that
        // entity FOREVER: the host's next delta is diffed against a baseline
        // the client does not actually hold, and nothing ever notices,
        // because divergence has no symptom until something looks wrong on
        // screen.
        //
        // So the snapshot is parsed into the staged lists below, which own
        // nothing of the world, and the world is touched only once the last
        // byte has been read and checked. Staging costs one copy of the
        // component bytes the message already carries — bounded by the
        // message, not by anything the sender claims — which is cheaper than
        // the alternative of copying the mirror to roll back to.
        //
        // EVERY COUNT IS CHOSEN BY WHOEVER SENT THE SNAPSHOT, and each drives
        // a loop that allocates. A count is only believable if the bytes it
        // describes are still in the message, so each is measured against
        // what remains before the loop is entered: four bytes per removal,
        // eight per spawn (index and generation), six per drop (index and a
        // two-byte component id), six per changed-entity group (index and a
        // two-byte component count). That check is also what makes the
        // reserve() that follows each of them safe — the count has by then
        // been shown to fit in the datagram.
        struct StagedDrop
        {
            u32 index = 0;
            ecs::ComponentTypeId type = 0;
        };
        struct StagedWrite
        {
            u32 index = 0;
            ecs::ComponentTypeId type = 0;
            u32 offset = 0; // into `staged`
            u32 size = 0;
        };

        const u32 removeCount = reader.read<u32>();
        requireWireCount(removeCount, 4, reader.remaining(), "Snapshot removals");
        std::vector<u32> removed;
        removed.reserve(removeCount);
        for (u32 i = 0; i < removeCount; ++i)
        {
            removed.push_back(reader.read<u32>());
        }

        const u32 spawnCount = reader.read<u32>();
        requireWireCount(spawnCount, 8, reader.remaining(), "Snapshot spawns");
        std::vector<ecs::Entity> spawned;
        spawned.reserve(spawnCount);
        for (u32 i = 0; i < spawnCount; ++i)
        {
            const u32 index = reader.read<u32>();
            const u32 generation = reader.read<u32>();
            if (index > ecs::World::kMaxMirrorIndex)
            {
                // mirrorEntity refuses the same value; refusing it HERE is
                // what keeps the refusal out of the apply phase, which must
                // not be able to fail halfway.
                SW_THROW("Snapshot spawns index {}, past the {} a mirror will grow to",
                         index, ecs::World::kMaxMirrorIndex);
            }
            spawned.push_back(ecs::Entity{index, generation});
        }

        // Which slots the apply phase will leave alive, worked out here so a
        // change group naming an entity nobody spawned is refused before the
        // first removal happens. Removals are applied before spawns, so a
        // spawn of an index this same snapshot removed wins.
        std::vector<u32> removedIndices = removed;
        std::sort(removedIndices.begin(), removedIndices.end());
        std::vector<u32> spawnedIndices;
        spawnedIndices.reserve(spawned.size());
        for (const ecs::Entity entity : spawned)
        {
            spawnedIndices.push_back(entity.index);
        }
        std::sort(spawnedIndices.begin(), spawnedIndices.end());
        const auto aliveAfterwards = [&](u32 index) {
            if (std::binary_search(spawnedIndices.begin(), spawnedIndices.end(), index))
            {
                return true;
            }
            if (std::binary_search(removedIndices.begin(), removedIndices.end(), index))
            {
                return false;
            }
            return slotAlive(index);
        };

        const u32 dropCount = reader.read<u32>();
        requireWireCount(dropCount, 6, reader.remaining(), "Snapshot component drops");
        std::vector<StagedDrop> drops;
        drops.reserve(dropCount);
        for (u32 i = 0; i < dropCount; ++i)
        {
            const u32 index = reader.read<u32>();
            const u16 componentId = reader.read<u16>();
            if (componentId >= m_table.size())
            {
                SW_THROW("Snapshot names component id {} (table holds {})", componentId,
                         m_table.size());
            }
            drops.push_back(StagedDrop{index, m_table.entries()[componentId].typeId});
        }

        std::vector<StagedWrite> writes;
        std::vector<u8> staged;
        const u32 groupCount = reader.read<u32>();
        requireWireCount(groupCount, 6, reader.remaining(), "Snapshot change groups");
        for (u32 g = 0; g < groupCount; ++g)
        {
            const u32 index = reader.read<u32>();
            const u16 componentCount = reader.read<u16>();
            requireWireCount(componentCount, 2, reader.remaining(),
                             "Snapshot components in one group");
            for (u16 c = 0; c < componentCount; ++c)
            {
                const u16 componentId = reader.read<u16>();
                if (componentId >= m_table.size())
                {
                    SW_THROW("Snapshot names component id {} (table holds {})", componentId,
                             m_table.size());
                }
                const ReplicationTable::Entry& entry = m_table.entries()[componentId];
                if (!aliveAfterwards(index))
                {
                    // The host says this entity exists; we were never told it
                    // spawned. That is a protocol violation, not packet loss:
                    // the delta chain is verified by baseline id.
                    SW_THROW("Snapshot {} writes component '{}' to entity {}, which the "
                             "mirror does not have",
                             snapshotId, entry.name, index);
                }
                const u32 offset = static_cast<u32>(staged.size());
                staged.resize(offset + entry.size);
                // Throws if the bytes are not there, which is the case that
                // used to leave a half-applied entity behind.
                reader.readBytes(staged.data() + offset, entry.size);
                writes.push_back(StagedWrite{index, entry.typeId, offset, entry.size});
            }
        }

        // ---- from here on nothing can be refused --------------------------
        // Every index, component id and byte count above has been checked
        // against both the message and the mirror, so this half only copies.
        for (const u32 index : removed)
        {
            if (slotAlive(index))
            {
                world.destroyEntity(handleOf(index));
            }
        }
        for (const ecs::Entity entity : spawned)
        {
            world.mirrorEntity(entity);
        }
        for (const StagedDrop& drop : drops)
        {
            if (slotAlive(drop.index))
            {
                world.removeComponentRaw(handleOf(drop.index), drop.type);
            }
        }
        for (const StagedWrite& write : writes)
        {
            std::byte* destination = world.addComponentRaw(handleOf(write.index), write.type);
            std::memcpy(destination, staged.data() + write.offset, write.size);
        }

        m_applied = snapshotId;
        m_simulatedSeconds = simulatedSeconds;
        return true;
    }
} // namespace sw::net
