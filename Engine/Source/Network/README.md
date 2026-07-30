# Network module

> Status: **active**.

Authoritative-host multiplayer. The host simulates and owns the truth; a
client keeps a **mirror** of the host's world and sends back **intent**, never
state.

Four files, each with one job and no knowledge of the ones above it:

| File | What it is |
|---|---|
| `Transport` | the only code that touches an operating system. A non-blocking UDP socket, plus `LoopbackNetwork` — a simulated wire with seeded loss, latency and jitter and a clock that is passed in, so the whole stack above it is tested exactly and repeatably without a socket. |
| `Protocol` | the datagram, and the `Connection` that turns a lossy unordered stream into two delivery services: **unreliable sequenced** (state deltas) and **reliable ordered** (everything else). Sequencing, acknowledgement bitfields, resends, fragmentation. Knows nothing about a vessel. |
| `Replication` | the difference between the world the host has now and the world the client last **confirmed**. Diffs ECS component columns by `memcmp` against the last acknowledged baseline, keyed by the same stable component names the save file uses. |
| `Session` | `Host` and `Client`: handshake, world transfer at join, the snapshot beat, commands upstream, timeouts. |

## The three decisions everything else follows from

**The mirror carries the host's entity indices exactly.** Not a translation
table. Entity handles live *inside* components — a conveyor names its body, a
cable names its poles — and nothing declares which fields are handles, so
nothing could rewrite them. The save file restores indices exactly for the
same reason; this does it incrementally, via `World::mirrorEntity`.

**A delta is diffed against the last snapshot the client ACKNOWLEDGED**, never
against the last one sent. A lost snapshot then costs bandwidth and nothing
else. There is no repair path because there is nothing to repair.

**State is unreliable, intent is reliable.** A lost state delta is worthless —
a newer one exists — so resending it would deliver stale truth behind fresh
truth. A pilot's input is a fact that must land exactly once and in order. One
socket, two services; that split is why this is UDP and not TCP.

## Using it

```cpp
net::ReplicationSet set;
set.include("sw.Transform").include("phys.DynamicBody");

net::Host host(std::make_unique<net::UdpSocket>(7777), saveSchema, set);
// every frame:
host.update(now, world, simulation.simulatedSeconds());
for (const auto& command : host.commands()) { /* apply this client's intent */ }
```

```cpp
net::Client client(std::make_unique<net::UdpSocket>(0), saveSchema, "arthur");
client.connect(net::PeerAddress::localhost(7777), now);
// every frame:
client.update(now, mirrorWorld);
client.sendCommand(encodedInput);
```

The replication set is a **whitelist**: a component nobody names stays home,
so adding one to the game cannot silently add it to everyone's bandwidth bill.
Every name in it must be registered in the save schema — the wire contract and
the file format are the same contract, and a component that cannot be
serialized cannot be replicated.

## Measuring it

`Tools/NetProbe` stands a host and a client on real UDP sockets and prints
bytes, bandwidth and encoder cost at several world sizes. Every network figure
in `docs/Performance.md` comes from there.

Rules: no God objects, minimal dependencies on other modules, every public
type documented.
