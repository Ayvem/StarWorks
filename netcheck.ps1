# ============================================================================
# netcheck.ps1 — can these two machines reach each other AT ALL?
#
# WHY THIS IS SEPARATE FROM THE GAME. When a join times out there are three
# suspects — the firewall, the network, and the game — and the game is the
# worst possible instrument for telling them apart, because it is one of the
# suspects. This sends and receives plain UDP with nothing of ours in it. If
# this works and the game does not, the game is at fault. If this fails too,
# the game never had a chance and nothing you change in it will help.
#
# READ THIS BEFORE TRUSTING PING. `ping` failing between two Windows machines
# proves NOTHING. Windows Defender blocks inbound ICMP echo by default — the
# rule "File and Printer Sharing (Echo Request - ICMPv4-In)" ships disabled.
# Two perfectly connected PCs on the same switch will refuse to ping each
# other out of the box. That is why this script speaks UDP, which is what the
# game actually uses, and reports ICMP separately as a nice-to-have.
#
# USE IT IN PAIRS. On the machine that will HOST:
#
#     .\netcheck.ps1 -Listen
#
# then, on the other machine, with the first one's address:
#
#     .\netcheck.ps1 -Send 192.168.1.61
#
# The listener prints every datagram it receives and answers each one, so the
# sender learns whether the path works in BOTH directions — which matters,
# because a firewall is a one-way device and the return trip is the half that
# usually works.
#
#     .\netcheck.ps1 -Local       # neither machine: just describe this one
#     .\netcheck.ps1 -AllowPing   # add the inbound ICMP rule (administrator)
# ============================================================================

[CmdletBinding()]
param(
    [switch]$Listen,
    [string]$Send,
    [int]$Port = 7777,
    [int]$Seconds = 30,
    [switch]$Local,
    [switch]$AllowPing
)

$ErrorActionPreference = 'Stop'

function Show-Local {
    Write-Host ''
    Write-Host 'THIS MACHINE' -ForegroundColor Cyan

    # Every IPv4 address with its prefix length. The prefix is the point:
    # two machines only talk directly when they are on the SAME subnet, and
    # a mesh extender or a second router hands out addresses on its own.
    Get-NetIPAddress -AddressFamily IPv4 |
        Where-Object { $_.IPAddress -ne '127.0.0.1' } |
        Sort-Object InterfaceAlias |
        ForEach-Object {
            Write-Host ("  {0,-28} {1}/{2}" -f $_.InterfaceAlias, $_.IPAddress, $_.PrefixLength)
        }

    $gateway = Get-NetRoute -DestinationPrefix '0.0.0.0/0' -ErrorAction SilentlyContinue |
               Sort-Object RouteMetric |
               Select-Object -First 1
    if ($gateway) {
        Write-Host ("  Gateway                      {0}" -f $gateway.NextHop)
    }

    foreach ($net in Get-NetConnectionProfile) {
        $colour = if ($net.NetworkCategory -eq 'Public') { 'Yellow' } else { 'Gray' }
        Write-Host ("  Profile: {0,-19} {1}" -f $net.Name, $net.NetworkCategory) -ForegroundColor $colour
        if ($net.NetworkCategory -eq 'Public') {
            Write-Host '    ^ Public blocks inbound whatever rules exist. As administrator:' -ForegroundColor Yellow
            Write-Host ("      Set-NetConnectionProfile -InterfaceIndex {0} -NetworkCategory Private" -f $net.InterfaceIndex) -ForegroundColor Yellow
        }
    }
    Write-Host ''
}

# ---- -AllowPing -------------------------------------------------------------
if ($AllowPing) {
    $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsPrincipal($identity)
    if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
        throw 'Needs an administrator PowerShell.'
    }
    $name = 'StarWorks diagnostics (ICMPv4 echo in)'
    if (-not (Get-NetFirewallRule -DisplayName $name -ErrorAction SilentlyContinue)) {
        New-NetFirewallRule -DisplayName $name `
                            -Description 'Lets this machine answer ping, for diagnosing a LAN game.' `
                            -Direction Inbound -Protocol ICMPv4 -IcmpType 8 `
                            -Action Allow -Profile Private,Domain | Out-Null
        Write-Host "Added: $name" -ForegroundColor Green
    } else {
        Write-Host 'Already there.' -ForegroundColor DarkGray
    }
    Write-Host 'This is for DIAGNOSIS only — the game does not need ICMP.' -ForegroundColor DarkGray
    return
}

if ($Local -or (-not $Listen -and -not $Send)) {
    Show-Local
    if (-not $Listen -and -not $Send) {
        Write-Host 'Now run  .\netcheck.ps1 -Listen  here, and  .\netcheck.ps1 -Send <this address>  there.' -ForegroundColor Cyan
    }
    return
}

# ---- listener ---------------------------------------------------------------
if ($Listen) {
    Show-Local
    Write-Host ("LISTENING on UDP {0} for {1}s. Ctrl+C to stop." -f $Port, $Seconds) -ForegroundColor Cyan
    Write-Host 'Every datagram that arrives is printed and answered.'
    Write-Host ''

    $socket = New-Object System.Net.Sockets.UdpClient($Port)
    # A receive timeout rather than a blocking wait: a script you cannot
    # interrupt is a script people stop running.
    $socket.Client.ReceiveTimeout = 1000
    $deadline = (Get-Date).AddSeconds($Seconds)
    $count = 0
    try {
        while ((Get-Date) -lt $deadline) {
            $remote = New-Object System.Net.IPEndPoint([System.Net.IPAddress]::Any, 0)
            try {
                $bytes = $socket.Receive([ref]$remote)
            } catch [System.Net.Sockets.SocketException] {
                continue   # the timeout above; keep waiting
            }
            $count++
            $text = [Text.Encoding]::UTF8.GetString($bytes)
            Write-Host ("  <- {0}  {1} bytes  '{2}'" -f $remote, $bytes.Length, $text) -ForegroundColor Green
            $reply = [Text.Encoding]::UTF8.GetBytes("PONG $text")
            [void]$socket.Send($reply, $reply.Length, $remote)
        }
    } finally {
        $socket.Close()
    }

    Write-Host ''
    if ($count -eq 0) {
        Write-Host 'NOTHING ARRIVED.' -ForegroundColor Red
        Write-Host '  Either the packets are being dropped before this process (firewall,'
        Write-Host '  Public profile), or the two machines cannot reach each other at all.'
        Write-Host '  Compare the addresses and prefixes printed above with the other'
        Write-Host "  machine's: if the subnets differ, no rule on either PC will help."
    } else {
        Write-Host ("{0} datagram(s) arrived and were answered." -f $count) -ForegroundColor Green
        Write-Host '  Inbound UDP works on this machine. If the game still times out, the'
        Write-Host '  fault is in the game or in its rule, not in the network.'
    }
    return
}

# ---- sender -----------------------------------------------------------------
Show-Local
$target = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Parse($Send), $Port)
Write-Host ("SENDING to {0}" -f $target) -ForegroundColor Cyan

# ICMP first, reported honestly: informative when it works, meaningless when
# it does not, because Windows blocks inbound echo by default.
$pinged = Test-Connection -ComputerName $Send -Count 2 -Quiet -ErrorAction SilentlyContinue
if ($pinged) {
    Write-Host '  ping: replies' -ForegroundColor Green
} else {
    Write-Host '  ping: no reply — EXPECTED on default Windows, proves nothing either way' -ForegroundColor DarkGray
}

# ARP is the real layer-2 evidence: an entry means the two machines are on
# the same link and have exchanged frames, whatever the firewall does with
# them afterwards.
$arp = Get-NetNeighbor -IPAddress $Send -ErrorAction SilentlyContinue |
       Where-Object { $_.State -ne 'Unreachable' }
if ($arp) {
    Write-Host ("  arp:  {0} is on this link ({1})" -f $Send, $arp[0].LinkLayerAddress) -ForegroundColor Green
} else {
    Write-Host '  arp:  no entry — the machines may not share a subnet, or the router' -ForegroundColor Yellow
    Write-Host '        is isolating clients from each other (guest Wi-Fi, AP isolation)' -ForegroundColor Yellow
}

$socket = New-Object System.Net.Sockets.UdpClient
$socket.Client.ReceiveTimeout = 2000
$replies = 0
try {
    for ($i = 1; $i -le 5; $i++) {
        $payload = [Text.Encoding]::UTF8.GetBytes("STARWORKS-CHECK-$i")
        [void]$socket.Send($payload, $payload.Length, $target)
        Write-Host ("  -> sent probe {0}" -f $i)
        $remote = New-Object System.Net.IPEndPoint([System.Net.IPAddress]::Any, 0)
        try {
            $bytes = $socket.Receive([ref]$remote)
            $replies++
            Write-Host ("  <- {0}: '{1}'" -f $remote, [Text.Encoding]::UTF8.GetString($bytes)) -ForegroundColor Green
        } catch [System.Net.Sockets.SocketException] {
            Write-Host '  .. no answer within 2s' -ForegroundColor DarkGray
        }
    }
} finally {
    $socket.Close()
}

Write-Host ''
if ($replies -gt 0) {
    Write-Host ("UDP works in BOTH directions ({0}/5 answered)." -f $replies) -ForegroundColor Green
    Write-Host '  The network is fine and inbound UDP reaches the other machine. If the'
    Write-Host '  game still times out, compare the two builds: a protocol version'
    Write-Host '  mismatch is silently dropped and looks exactly like this.'
} else {
    Write-Host 'NO ANSWER.' -ForegroundColor Red
    Write-Host '  Check, in this order:'
    Write-Host '   1. Is the listener actually running on the other machine right now?'
    Write-Host '   2. Do both machines print the SAME subnet above (address/prefix)?'
    Write-Host '      Different subnets = the router will not bridge them, and no'
    Write-Host '      firewall rule on either side changes that.'
    Write-Host '   3. Is either machine on a PUBLIC network profile?'
    Write-Host '   4. Is one of them on guest Wi-Fi, or a router with "client isolation" /'
    Write-Host '      "AP isolation" on? That blocks PC-to-PC traffic by design.'
    Write-Host '   5. Third-party security suites (Avast, Bitdefender, Norton, ESET) have'
    Write-Host '      their own firewall that ignores the Windows rules entirely.'
}
