// ============================================================================
// GameNetwork.cpp — Multiplayer session: host/join, the F3 panel, per-player clocks.
// Split out of StarWorksGame.cpp; same class, one theme per translation unit.
// ============================================================================

#include "StarWorksGame.hpp"

#include "GameInternal.hpp"
#include "Systems.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <format>
#include <limits>

namespace game
{

    std::vector<sw::net::PlayerView> StarWorksGame::netRoster() const
    {
        if (m_netHost != nullptr)
        {
            return m_netHost->roster();
        }
        if (m_netClient != nullptr)
        {
            return m_netClient->roster();
        }
        return {};
    }

    void StarWorksGame::netHost()
    {
        netLeave();
        try
        {
            sw::net::ReplicationSet set;
            set.include("sw.Transform")
                .include("phys.DynamicBody")
                .include("phys.OnRails")
                .include("parts.Part")
                .include("parts.Vessel")
                .include("game.Ship");

            sw::net::PeerAddress bind{};
            const bool parsed = sw::net::PeerAddress::parse(m_netAddress, bind);
            const sw::u16 port = parsed && bind.port != 0 ? bind.port : sw::u16{7777};

            sw::net::Host::Config config;
            config.hostName = "host";
            m_netHost = std::make_unique<sw::net::Host>(
                std::make_unique<sw::net::UdpSocket>(port), m_saveSchema, set, config);
            // Show the address the OTHER machine has to type, not the port
            // on its own. Guessing your own LAN address out of ipconfig is
            // the first thing that goes wrong, and the socket already knows.
            const sw::net::PeerAddress lan = sw::net::localAddress(m_netHost->port());
            m_netStatus = std::format("HOSTING ON {}", lan.toString());
            SW_LOG_INFO("Game", "Multiplayer: hosting on UDP port {} — others join with {}",
                        m_netHost->port(), lan.toString());

            // THE FIREWALL, HERE AND NOWHERE ELSE. Hosting is the only thing
            // in the game that needs an unsolicited inbound datagram, and
            // pressing HOST is the only moment at which asking for it is
            // both expected and explicable. The call is a no-op — no prompt,
            // no process — when the rule already exists, so a player who
            // accepted once never sees it again.
            const sw::platform::FirewallRequest firewall = sw::platform::allowInboundUdp();
            m_netFirewall = firewall;
            m_netPublicNetwork = sw::platform::onPublicNetwork();
            if (m_netPublicNetwork)
            {
                SW_LOG_WARN("Game",
                            "Multiplayer: this machine is on a PUBLIC network profile. The "
                            "inbound rule covers Private and Domain only, so it is inert: "
                            "Windows will drop the connection attempt however many rules "
                            "exist. Set-NetConnectionProfile -NetworkCategory Private.");
            }
            switch (firewall)
            {
                case sw::platform::FirewallRequest::Added:
                    SW_LOG_INFO("Game", "Multiplayer: firewall rule added for this executable");
                    break;
                case sw::platform::FirewallRequest::Declined:
                    // Hosting continues. Refusing administrator rights is a
                    // legitimate answer, the session is perfectly playable
                    // from this machine, and someone on the same network may
                    // still get through if a rule exists by another route.
                    // The verdict goes on its OWN line in the panel, not on
                    // the end of the status: measured, "HOSTING ON
                    // 192.168.1.61:7777 - FIREWALL REFUSED" is 0.725 NDC
                    // wide against 0.494 of usable width.
                    SW_LOG_WARN("Game",
                                "Multiplayer: administrator rights refused, so no inbound rule "
                                "was added. Players on other machines will most likely see "
                                "NO REPLY. Run firewall.ps1 as administrator to add it later.");
                    break;
                case sw::platform::FirewallRequest::Failed:
                    SW_LOG_ERROR("Game",
                                 "Multiplayer: the inbound rule could not be added. "
                                 "Run firewall.ps1 as administrator.");
                    break;
                case sw::platform::FirewallRequest::AlreadyAllowed:
                case sw::platform::FirewallRequest::Unsupported:
                    break;
            }
        }
        catch (const sw::Exception& e)
        {
            m_netHost.reset();
            m_netStatus = "HOST FAILED";
            SW_LOG_ERROR("Game", "Could not host: {}", e.what());
        }
    }

    void StarWorksGame::netJoin()
    {
        netLeave();
        sw::net::PeerAddress address{};
        if (!sw::net::PeerAddress::parse(m_netAddress, address) || address.port == 0)
        {
            m_netStatus = "BAD ADDRESS";
            return;
        }
        try
        {
            m_netMirror.clearForRestore();
            m_netClient = std::make_unique<sw::net::Client>(
                std::make_unique<sw::net::UdpSocket>(0), m_saveSchema, "pilot");
            m_netClient->connect(address, clock().totalSeconds());
            m_netStatus = std::format("JOINING {}", address.toString());
            SW_LOG_INFO("Game", "Multiplayer: {}", m_netStatus);
        }
        catch (const sw::Exception& e)
        {
            m_netClient.reset();
            m_netStatus = "JOIN FAILED";
            SW_LOG_ERROR("Game", "Could not join: {}", e.what());
        }
    }

    void StarWorksGame::netLeave()
    {
        const sw::f64 now = clock().totalSeconds();
        if (m_netClient != nullptr)
        {
            m_netClient->disconnect(now);
            m_netClient.reset();
        }
        if (m_netHost != nullptr)
        {
            m_netHost->shutdown(now);
            m_netHost.reset();
        }
        m_syncWarpTo = 0.0;
        m_syncWarpPlayer = 0;
        m_netStatus.clear();
        m_netTimeoutLogged = false;
        m_netFirewall = sw::platform::FirewallRequest::Unsupported;
        m_netPublicNetwork = false;
    }

    void StarWorksGame::netSyncTo(sw::u32 playerId, sw::f64 targetSeconds)
    {
        // The catch-up warp is still a warp: it puts the world on rails and
        // stops integrating, so the same rule applies. What it does bypass is
        // the ALTITUDE ladder — a player parked in a 200 km orbit would
        // otherwise be capped at x100 and never close a three-hour gap.
        if (!warpAllowed())
        {
            m_netStatus = std::format("CANNOT SYNC - {}", warpBlockReason());
            return;
        }
        if (targetSeconds <= m_physicsLane->presentSeconds())
        {
            m_netStatus = "ALREADY THERE";
            return;
        }
        m_syncWarpTo = targetSeconds;
        m_syncWarpPlayer = playerId;
        m_warpToSeconds = 0.0; // one destination at a time
        m_simulation.setPaused(false);
        SW_LOG_INFO("Game", "Sync warp to player {} at t={:.0f}", playerId, targetSeconds);
    }

    void StarWorksGame::updateTextField()
    {
        // THE SAVE NAME, when the menu's field has focus. Same shape as the
        // address box below and deliberately not merged with it: they accept
        // different alphabets, and one field that behaves two ways depending
        // on which screen is up is harder to read than two fields.
        if (m_saveNameFocused)
        {
            for (const sw::u32 codepoint : input().charsTyped())
            {
                if (m_saveName.size() >= 24)
                {
                    break;
                }
                // The HUD font has no lowercase, so a lowercase letter is
                // folded rather than dropped: typing "moon base" and seeing
                // "MOON BASE" is right, seeing "" is not.
                const sw::u32 upper =
                    (codepoint >= 'a' && codepoint <= 'z') ? codepoint - 32 : codepoint;
                if ((upper >= 'A' && upper <= 'Z') || (upper >= '0' && upper <= '9') ||
                    upper == ' ' || upper == '-')
                {
                    m_saveName.push_back(static_cast<char>(upper));
                }
            }
            if (input().wasKeyPressed(sw::KeyCode::Backspace) && !m_saveName.empty())
            {
                m_saveName.pop_back();
            }
            if (input().wasKeyPressed(sw::KeyCode::Enter) ||
                input().wasKeyPressed(sw::KeyCode::Escape))
            {
                m_saveNameFocused = false;
            }
            return;
        }
        if (!m_netAddressFocused)
        {
            return;
        }
        // Only what an address is made of. The HUD font has no lowercase and
        // no underscore, so a hostname could not be displayed even if it
        // could be typed; digits, dots and a colon are the whole alphabet.
        for (const sw::u32 codepoint : input().charsTyped())
        {
            if (m_netAddress.size() >= 21)
            {
                break;
            }
            if ((codepoint >= '0' && codepoint <= '9') || codepoint == '.' ||
                codepoint == ':')
            {
                m_netAddress.push_back(static_cast<char>(codepoint));
            }
        }
        if (input().wasKeyPressed(sw::KeyCode::Backspace) && !m_netAddress.empty())
        {
            m_netAddress.pop_back();
        }
        if (input().wasKeyPressed(sw::KeyCode::Enter) ||
            input().wasKeyPressed(sw::KeyCode::Escape))
        {
            m_netAddressFocused = false;
        }
    }

    void StarWorksGame::updateNetwork(sw::f32 deltaSeconds)
    {
        (void)deltaSeconds;
        if (!netActive())
        {
            return;
        }

        const sw::f64 wall = clock().totalSeconds();
        const sw::f64 simulated = m_physicsLane->presentSeconds();

        if (m_netHost != nullptr)
        {
            m_netHost->update(wall, m_world, simulated);
        }
        if (m_netClient != nullptr)
        {
            m_netClient->update(wall, m_netMirror, simulated);
            if (m_netClient->state() == sw::net::ClientState::Connected)
            {
                m_netStatus = std::format("JOINED AS {}", m_netClient->clientId());
            }
            else if (m_netClient->state() == sw::net::ClientState::Rejected)
            {
                m_netStatus = "REJECTED";
            }
            else if (m_netClient->state() == sw::net::ClientState::TimedOut)
            {
                // A timeout has two completely different causes and the cure
                // for one is useless against the other, so say WHICH. If not
                // one datagram ever came back, nothing on the far side ever
                // answered: the packets die before the host's process sees
                // them — a firewall, a wrong address, or nobody hosting. If
                // some arrived and then stopped, the link was alive and died.
                const bool everHeard = m_netClient->stats().datagramsReceived > 0;
                m_netStatus = everHeard ? "TIMED OUT - LINK LOST" : "NO REPLY - CHECK FIREWALL";
                if (!m_netTimeoutLogged)
                {
                    m_netTimeoutLogged = true;
                    if (everHeard)
                    {
                        SW_LOG_WARN("Game",
                                    "Multiplayer: the host stopped answering after {} datagrams",
                                    m_netClient->stats().datagramsReceived);
                    }
                    else
                    {
                        SW_LOG_WARN(
                            "Game",
                            "Multiplayer: sent {} datagrams to {} and got NOTHING back. Nothing "
                            "at that address is answering: the host is not listening, the "
                            "address is wrong, or inbound UDP is blocked. On the HOSTING "
                            "machine run firewall.ps1 as administrator, and check that its "
                            "network profile is Private and not Public.",
                            m_netClient->stats().datagramsSent, m_netAddress);
                    }
                }
            }
        }

        // A beacon: where this player's craft is, stamped with the instant it
        // was there. Peers behind us hold it until their own clock reaches
        // that instant — which is the whole rule, exercised on real traffic
        // rather than only in a test.
        if (m_netLastBeaconAt < 0.0 || wall - m_netLastBeaconAt >= 0.5)
        {
            m_netLastBeaconAt = wall;
            const sw::ecs::Entity flown = controlledEntity();
            if (!flown.isNull() && m_world.isAlive(flown))
            {
                const sw::WorldVec3 position =
                    m_world.getComponent<TransformComponent>(flown).position;
                sw::ser::BinaryWriter writer;
                writer.write(position.x);
                writer.write(position.y);
                writer.write(position.z);
                if (m_netHost != nullptr)
                {
                    m_netHost->broadcastEvent(simulated, kNetEventBeacon, writer.bytes());
                }
                else if (m_netClient != nullptr)
                {
                    m_netClient->sendEvent(simulated, kNetEventBeacon, writer.bytes());
                }
            }
        }

        // Release whatever the timeline says is due AT OUR CLOCK.
        sw::net::Timeline* timeline = (m_netHost != nullptr) ? &m_netHost->timeline()
                                                             : &m_netClient->timeline();
        m_netEventsApplied += timeline->advance(simulated).size();
    }

    void StarWorksGame::collectNetPanel()
    {
        constexpr sw::f32 kRight = 0.97f;
        constexpr sw::f32 kLeft = 0.44f;
        constexpr sw::f32 kTop = -0.95f;
        constexpr sw::f32 kPad = 0.018f;
        constexpr sw::f32 kHeaderH = 0.086f;
        constexpr sw::f32 kRowH = 0.062f;
        constexpr sw::f32 kGap = 0.008f;

        sw::f32 cursorX = -2.0f;
        sw::f32 cursorY = -2.0f;
        const bool haveCursor = hudCursor(cursorX, cursorY);
        auto hovering = [&](sw::f32 x0, sw::f32 x1, sw::f32 y) {
            return haveCursor && cursorX >= x0 && cursorX <= x1 && cursorY >= y &&
                   cursorY <= y + kRowH;
        };

        const std::vector<sw::net::PlayerView> roster = netRoster();
        const sw::u32 selfId =
            (m_netClient != nullptr) ? m_netClient->clientId() : sw::u32{0};
        const sw::f64 selfClock = m_physicsLane->presentSeconds();

        // Height: address, buttons, status, the PILOTS label, one row per
        // player, and the footer verdict — five fixed rows plus the roster.
        // Counting four put the footer outside the panel, which a mock of the
        // layout showed before the code ever ran.
        // Plus one row while hosting, for what has actually reached the
        // socket. That row is the only place either machine can see the
        // difference between "the packets never arrived" and "the packets
        // arrived and were thrown away".
        const sw::f32 bodyRows = 5.0f + (m_netHost != nullptr ? 1.0f : 0.0f) +
                                 static_cast<sw::f32>(roster.size());
        const sw::f32 bottom =
            kTop + kHeaderH + bodyRows * (kRowH + kGap) + kPad * 2.0f;

        hudPanel(kLeft, kTop, kRight, bottom, hud::kPanel);
        hudQuad(kLeft, kTop, kRight, kTop + kHeaderH - 0.006f, hud::kHeader);
        hudText("MULTIPLAYER", kLeft + 0.022f, kTop + 0.026f, 0.048f, hud::kTitle);
        hudText("F3", kRight - 0.075f, kTop + 0.030f, 0.036f, hud::kTextDim);

        sw::f32 y = kTop + kHeaderH + kPad;

        // ---- the address, typed -------------------------------------------
        {
            const bool hot = hovering(kLeft + kPad, kRight - kPad, y);
            const sw::Vec4 fill = m_netAddressFocused ? hud::kRowOn
                                  : hot                ? hud::kRowHover
                                                       : hud::kRow;
            hudQuad(kLeft + kPad, y, kRight - kPad, y + kRowH, fill);
            // A caret rather than a cursor: the field only ever appends or
            // deletes at the end, so there is nothing to move.
            const std::string shown =
                m_netAddressFocused ? m_netAddress + "-" : m_netAddress;
            hudText(shown, kLeft + kPad + 0.014f, y + 0.014f, 0.034f, hud::kText);
            m_hudButtons.push_back({kLeft + kPad, y, kRight - kPad, y + kRowH, 1003u});
            y += kRowH + kGap;
        }

        // ---- host / join / leave -------------------------------------------
        {
            const sw::f32 mid = (kLeft + kRight) * 0.5f;
            if (!netActive())
            {
                const bool hotHost = hovering(kLeft + kPad, mid - 0.004f, y);
                hudQuad(kLeft + kPad, y, mid - 0.004f, y + kRowH,
                        hotHost ? hud::kRowHover : hud::kRow);
                hudText("HOST", kLeft + kPad + 0.030f, y + 0.014f, 0.034f, hud::kText);
                m_hudButtons.push_back({kLeft + kPad, y, mid - 0.004f, y + kRowH, 1000u});

                const bool hotJoin = hovering(mid + 0.004f, kRight - kPad, y);
                hudQuad(mid + 0.004f, y, kRight - kPad, y + kRowH,
                        hotJoin ? hud::kRowHover : hud::kRow);
                hudText("JOIN", mid + 0.034f, y + 0.014f, 0.034f, hud::kText);
                m_hudButtons.push_back({mid + 0.004f, y, kRight - kPad, y + kRowH, 1001u});
            }
            else
            {
                const bool hot = hovering(kLeft + kPad, kRight - kPad, y);
                hudQuad(kLeft + kPad, y, kRight - kPad, y + kRowH,
                        hot ? hud::kRowHover : hud::kRowStop);
                hudText("LEAVE SESSION", kLeft + kPad + 0.030f, y + 0.014f, 0.034f,
                        hud::kText);
                m_hudButtons.push_back({kLeft + kPad, y, kRight - kPad, y + kRowH, 1002u});
            }
            y += kRowH + kGap;
        }

        // ---- status ---------------------------------------------------------
        hudText(m_netStatus.empty() ? "OFFLINE" : hud::caps(m_netStatus),
                kLeft + kPad, y + 0.012f, 0.032f, hud::kTextDim);
        // The firewall verdict rides in the same row's spare height rather
        // than on the end of the status line: the panel is 0.494 NDC wide
        // and the concatenated string measured 0.725. Only shown when there
        // is something to act on — a rule that already exists is not news.
        if (m_netHost != nullptr)
        {
            if (m_netFirewall == sw::platform::FirewallRequest::Declined)
            {
                hudText("FIREWALL: RIGHTS REFUSED", kLeft + kPad, y + 0.044f, 0.024f,
                        hud::kBad);
            }
            else if (m_netFirewall == sw::platform::FirewallRequest::Failed)
            {
                hudText("FIREWALL: RULE FAILED", kLeft + kPad, y + 0.044f, 0.024f, hud::kBad);
            }
            else if (m_netPublicNetwork)
            {
                // The rule covers Private and Domain. On a Public network it
                // exists, lists as present in every tool that shows rules,
                // and blocks the packet anyway — the single most confusing
                // state this whole feature can be in.
                hudText("NETWORK IS PUBLIC - RULE INACTIVE", kLeft + kPad, y + 0.044f, 0.024f,
                        hud::kBad);
            }
            else if (m_netFirewall == sw::platform::FirewallRequest::Added)
            {
                hudText("FIREWALL: RULE ADDED", kLeft + kPad, y + 0.044f, 0.024f, hud::kOk);
            }
        }
        y += kRowH + kGap;

        // ---- what has actually reached the socket ---------------------------
        // The host is the only machine that can tell the three causes of a
        // client-side timeout apart, and until now it never said.
        if (m_netHost != nullptr)
        {
            const sw::net::Host::Reception& rx = m_netHost->reception();
            // Clamped for display. At 20 snapshots a second these counters
            // reach seven digits in an hour, and the row is 34 characters
            // wide — the exact total stopped being the point after the
            // first one arrived anyway.
            auto shortCount = [](sw::u64 value) {
                return (value > 99999u) ? std::string("99999+") : std::format("{}", value);
            };
            hudText(std::format("RX {}  REFUSED {}", shortCount(rx.arrived),
                                shortCount(rx.refused)),
                    kLeft + kPad, y + 0.006f, 0.030f,
                    rx.arrived == 0 ? hud::kTextDim : hud::kOk);

            std::string verdict;
            sw::Vec4 colour = hud::kTextDim;
            if (rx.wrongVersion > 0)
            {
                verdict = std::format("PEER SPEAKS V{} - REBUILD IT", rx.lastForeignVersion);
                colour = hud::kBad;
            }
            else if (rx.arrived == 0)
            {
                verdict = "NOTHING HAS REACHED THIS PC";
                colour = hud::kWarn;
            }
            else if (rx.notOurs > 0 && rx.notOurs == rx.fromStrangers)
            {
                verdict = "TRAFFIC ARRIVES BUT IS NOT OURS";
                colour = hud::kWarn;
            }
            else
            {
                verdict = "PACKETS ARE GETTING THROUGH";
                colour = hud::kOk;
            }
            hudText(verdict, kLeft + kPad, y + 0.040f, 0.024f, colour);
            y += kRowH + kGap;
        }

        // ---- the players, and how far apart their clocks are ---------------
        hudText(std::format("PILOTS {}", roster.size()), kLeft + kPad, y + 0.012f, 0.032f,
                hud::kTextDim);
        y += kRowH + kGap;

        for (sw::usize i = 0; i < roster.size(); ++i)
        {
            const sw::net::PlayerView& player = roster[i];
            const bool self = (player.id == selfId);
            const sw::f64 offset = player.simulatedSeconds - selfClock;
            // Only a real gap is worth a button. Half a second is network
            // jitter, not a temporality.
            const bool ahead = !self && offset > 1.0;

            const sw::f32 syncX0 = kRight - kPad - 0.15f;
            const sw::f32 rowRight = ahead ? syncX0 - 0.008f : kRight - kPad;
            const bool hot = hovering(kLeft + kPad, rowRight, y);
            hudQuad(kLeft + kPad, y, rowRight, y + kRowH,
                    self ? hud::kRowOn : (hot ? hud::kRowHover
                                              : ((i % 2 == 0) ? hud::kRow : hud::kRowAlt)));
            hudText(hud::caps(player.name.empty() ? std::string("PILOT") : player.name),
                    kLeft + kPad + 0.016f, y + 0.008f, 0.032f, hud::kText);

            const std::string when =
                self ? std::string("NOW") : hud::signedDuration(offset);
            hudText(when, kLeft + kPad + 0.016f, y + 0.036f, 0.024f,
                    (std::abs(offset) < 1.0 || self) ? hud::kOk
                    : (offset > 0.0)                 ? hud::kWarn
                                                     : hud::kTextDim);

            if (ahead)
            {
                const bool hotSync = hovering(syncX0, kRight - kPad, y);
                const bool can = warpAllowed();
                hudQuad(syncX0, y, kRight - kPad, y + kRowH,
                        !can       ? sw::Vec4{0.12f, 0.14f, 0.18f, 0.90f}
                        : hotSync  ? hud::kRowOnHover
                                   : hud::kRowOn);
                hudText("SYNC", syncX0 + 0.028f, y + 0.020f, 0.030f,
                        can ? hud::kText : hud::kTextDim);
                if (can)
                {
                    m_hudButtons.push_back(
                        {syncX0, y, kRight - kPad, y + kRowH,
                         1100u + static_cast<sw::u32>(i)});
                }
            }
            y += kRowH + kGap;
        }

        // ---- footer: ONLY ABOUT SYNCING -------------------------------------
        //
        // This line is in the multiplayer panel, so it answers a multiplayer
        // question: can you catch the player who is ahead of you? It used to
        // report the warp gate unconditionally, which meant a red WARP LOCKED
        // sat under the pilot list at every moment of ordinary play — during
        // an ascent, during a reentry, and for the second and a half of a
        // jump — with no one ahead to sync to and nothing being refused. A
        // permanent alarm about a thing nobody asked for reads as the game
        // taking something away.
        //
        // So: a running sync counts down, a reachable one says whether it
        // can start, and when nobody is ahead of you the row is simply not
        // there.
        bool someoneAhead = false;
        for (const sw::net::PlayerView& player : roster)
        {
            if (player.id != selfId && player.simulatedSeconds - selfClock > 1.0)
            {
                someoneAhead = true;
            }
        }

        if (m_syncWarpTo > 0.0)
        {
            const sw::f64 remaining = m_syncWarpTo - selfClock;
            hudText(std::format("SYNCING  T-{}", hud::signedDuration(remaining)),
                    kLeft + kPad, y + 0.010f, 0.032f, hud::kOk);
        }
        else if (someoneAhead)
        {
            if (warpAllowed())
            {
                hudText("SYNC READY", kLeft + kPad, y + 0.010f, 0.030f, hud::kOk);
            }
            else
            {
                hudText(std::format("CANNOT SYNC - {}", warpBlockReason()), kLeft + kPad,
                        y + 0.010f, 0.030f, hud::kBad);
            }
        }
    }
} // namespace game
