#pragma once

// ============================================================================
// Platform/Firewall.hpp
// Asking Windows to let other machines reach this one.
//
// WHY THIS IS HERE AT ALL. Hosting is the only thing this game does that
// needs an unsolicited inbound datagram, and Windows Defender drops those by
// default. The failure is completely silent on the hosting side — the packet
// never reaches our process — so the player sees a timeout on the OTHER
// machine and has nothing at all to look at on this one.
//
// TWO RULES THIS FILE OBEYS.
//
// 1. THE GAME IS NEVER ELEVATED. A game running as administrator is a game
//    whose every save, log and crash dump is written by an administrator,
//    and whose window refuses drag-and-drop from Explorer. What gets
//    elevated is a one-shot `netsh` that adds a rule and exits. The player
//    sees one UAC prompt, answers it, and the game they are playing is still
//    the ordinary unprivileged process it was a second earlier.
//
// 2. ASK ONLY WHEN THERE IS SOMETHING TO ASK FOR. Reading the firewall does
//    NOT need elevation, so the rule is checked first, in-process, through
//    the firewall's COM interface. A prompt on every press of HOST would
//    train the player to click through it, which is how a UAC prompt stops
//    being a security feature.
//
// The rule is matched by EXECUTABLE PATH rather than by port: it stays valid
// when the port changes, it does not open a port for anything else on the
// machine, and it recognises a rule Windows itself created through its own
// "allow this app?" prompt as satisfying the requirement.
// ============================================================================

#include "Core/Types.hpp"

#include <string>

namespace sw::platform
{
    enum class FirewallState : u8
    {
        /// Not Windows, or the firewall could not be queried. Nothing to do
        /// and nothing to promise — Linux hosts are the user's own business.
        Unknown,
        /// An enabled inbound Allow rule already covers this executable.
        Allowed,
        /// The firewall is queryable and there is no such rule.
        Blocked,
    };

    enum class FirewallRequest : u8
    {
        /// A rule was already there; no prompt was shown.
        AlreadyAllowed,
        /// The user accepted the prompt and the rule now exists.
        Added,
        /// The user dismissed the UAC prompt. Not an error — a decision.
        Declined,
        /// The elevated helper ran and failed, or could not be started.
        Failed,
        /// Not Windows. Nothing was attempted.
        Unsupported,
    };

    /// Is inbound UDP already allowed for the running executable? Cheap, done
    /// in-process, needs no privileges of any kind.
    [[nodiscard]] FirewallState inboundUdpState();

    /// True when one of the network profiles CURRENTLY IN FORCE is Public.
    ///
    /// This matters more than it looks. The rule added below covers Private
    /// and Domain, because a game is not a reason to accept inbound traffic
    /// in a cafe — so on a Public network the rule exists, reads as present
    /// in every tool that lists it, and blocks the packet anyway. Windows
    /// picks Public in silence for any network whose "make this PC
    /// discoverable?" prompt was never answered, which includes most
    /// networks nobody thought about.
    [[nodiscard]] bool onPublicNetwork();

    /// Adds the inbound rule for the running executable, elevating a helper
    /// with ONE UAC prompt if — and only if — the rule is missing.
    ///
    /// Blocks while the prompt is on screen: it is modal to the user anyway,
    /// and the alternative is a game that keeps rendering behind a dialog it
    /// is waiting on. Call it from a deliberate action, never from a frame
    /// that happens every tick.
    FirewallRequest allowInboundUdp();

    /// What happened, in one line, for the status bar and the log.
    [[nodiscard]] std::string describe(FirewallRequest result);
} // namespace sw::platform
