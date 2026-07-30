# ============================================================================
# firewall.ps1 — let other machines on your network reach the host.
#
# YOU NORMALLY DO NOT NEED THIS. The game asks for administrator rights by
# itself the first time you press HOST, and adds exactly the rule below. This
# script exists for the cases where that is not what you want: adding the
# rule ahead of time on a machine you are setting up, adding it for a build
# that is not the one you are about to run, checking what is actually there,
# or taking it back out.
#
# WHY A RULE IS NEEDED AT ALL. A LAN game needs no router configuration:
# two machines on 192.168.1.x talk directly and the router only forwards the
# frames. What blocks it is the HOST machine's own firewall.
#
# Windows Defender Firewall drops every UNSOLICITED inbound datagram. The
# client side is fine — it sends first, so the reply comes back through the
# flow its outbound packet opened. The host never sends first: it waits to be
# spoken to, and that first ConnectRequest is exactly what gets dropped. The
# client retries until it gives up, which is the "connection timed out" you
# see. Nothing is logged on the host, because the host's process never saw a
# byte.
#
# THE RULE FOLLOWS THE EXECUTABLE, NOT THE PORT. It stays valid if you host
# on a different port, it does not open a port for anything else on the
# machine, and it is the same rule the game creates — so if you add it here,
# the game will not ask.
#
# Run this ON THE MACHINE THAT HOSTS, in an ADMINISTRATOR PowerShell:
#
#   .\firewall.ps1                        # allow inbound UDP for the game
#   .\firewall.ps1 -Exe <path>\StarWorks.exe # for a specific build
#   .\firewall.ps1 -Remove                # take the rule back out
#   .\firewall.ps1 -Check                 # change nothing, just report
# ============================================================================

[CmdletBinding()]
param(
    [string]$Exe,
    [int]$Port = 7777,
    [switch]$Remove,
    [switch]$Check
)

$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
$ruleName = 'StarWorks (inbound UDP)'

function Test-Administrator {
    $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsPrincipal($identity)
    return $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

# ---- which executable -------------------------------------------------------
# In the order you are most likely to mean it: the packaged folder, then the
# packaging build, then the development build.
if (-not $Exe) {
    $candidates = @(
        'dist\StarWorks\StarWorks.exe'
        'build\package\bin\RelWithDebInfo\StarWorks.exe'
        'build\windows\bin\Debug\StarWorks.exe'
    ) | ForEach-Object { Join-Path $root $_ }
    $Exe = $candidates | Where-Object { Test-Path $_ } | Select-Object -First 1
}
if (-not $Exe) {
    throw "No StarWorks.exe found. Build it first, or pass -Exe <path>."
}
# The firewall stores an absolute path and matches on it literally, so a
# relative path here would produce a rule that silently covers nothing.
$Exe = (Resolve-Path -LiteralPath $Exe).Path

# ---- report -----------------------------------------------------------------
# Printed in every mode: the three facts that decide whether a LAN game works.
Write-Host ''
Write-Host 'THIS MACHINE' -ForegroundColor Cyan
Write-Host ("  Executable         {0}" -f $Exe)

# The address that faces the network, asked of the routing table rather than
# guessed out of the adapter list — a machine with Docker, WSL, a VPN and a
# Bluetooth PAN has half a dozen addresses and only one of them is right.
# Find-NetRoute emits more than one object per route and only some of them
# carry IPAddress, so filter rather than taking the first blindly.
$routed = $null
try {
    $routed = Find-NetRoute -RemoteIPAddress 8.8.8.8 -ErrorAction Stop |
              Where-Object { $_.IPAddress } |
              Select-Object -First 1 -ExpandProperty IPAddress
} catch { }
if ($routed) {
    Write-Host ("  Others join with   {0}:{1}" -f $routed, $Port) -ForegroundColor Green
} else {
    Write-Host '  No route to the network — is this machine actually connected?' -ForegroundColor Yellow
}

# A Public profile blocks inbound whatever the rule says, and Windows picks
# Public in silence for any network you never answered the "make this PC
# discoverable?" prompt for.
#
# NOT $profile as the loop variable: that is an automatic variable in
# PowerShell (the path to the profile script), and shadowing it is how a
# script starts behaving differently depending on what ran before it.
foreach ($net in Get-NetConnectionProfile) {
    $colour = if ($net.NetworkCategory -eq 'Public') { 'Yellow' } else { 'Gray' }
    Write-Host ("  Network profile    {0} is {1}" -f $net.Name, $net.NetworkCategory) -ForegroundColor $colour
    if ($net.NetworkCategory -eq 'Public') {
        Write-Host '    ^ Public blocks inbound whatever the rule says. As administrator:' -ForegroundColor Yellow
        Write-Host ("      Set-NetConnectionProfile -InterfaceIndex {0} -NetworkCategory Private" -f $net.InterfaceIndex) -ForegroundColor Yellow
    }
}

# Look for ANY enabled inbound Allow rule covering this executable, not just
# ours by name — a rule Windows created through its own "allow this app?"
# prompt does the job just as well, and reporting it as missing would send
# you looking for a problem you do not have.
function Get-CoveringRules {
    param([string]$Path)
    Get-NetFirewallApplicationFilter -ErrorAction SilentlyContinue |
        Where-Object { $_.Program -eq $Path } |
        ForEach-Object { $_ | Get-NetFirewallRule -ErrorAction SilentlyContinue } |
        Where-Object { $_.Enabled -eq 'True' -and $_.Direction -eq 'Inbound' -and $_.Action -eq 'Allow' }
}

$covering = @(Get-CoveringRules -Path $Exe)
if ($covering.Count -gt 0) {
    Write-Host ("  Inbound rule       present ({0})" -f ($covering[0].DisplayName)) -ForegroundColor Green
} else {
    Write-Host '  Inbound rule       ABSENT' -ForegroundColor Yellow
}
Write-Host ''

if ($Check) {
    Write-Host 'Nothing changed (-Check).' -ForegroundColor DarkGray
    return
}

if (-not (Test-Administrator)) {
    throw 'Adding or removing a firewall rule needs an ADMINISTRATOR PowerShell. Right-click PowerShell -> Run as administrator, then run this again. (Or just press HOST in the game and accept the prompt.)'
}

# ---- remove -----------------------------------------------------------------
if ($Remove) {
    $ours = Get-NetFirewallRule -DisplayName $ruleName -ErrorAction SilentlyContinue
    if ($ours) {
        Remove-NetFirewallRule -DisplayName $ruleName
        Write-Host "Removed: $ruleName" -ForegroundColor Green
    } else {
        Write-Host "No rule named '$ruleName' to remove." -ForegroundColor DarkGray
    }
    # Anything else covering the executable is not ours to delete, but you
    # should know it is still there or you will wonder why nothing changed.
    $left = @(Get-CoveringRules -Path $Exe)
    if ($left.Count -gt 0) {
        Write-Host ("Still allowed by another rule: {0}" -f ($left.DisplayName -join ', ')) -ForegroundColor Yellow
    }
    return
}

# ---- add --------------------------------------------------------------------
if ($covering.Count -gt 0) {
    Write-Host 'Already allowed; nothing to do.' -ForegroundColor DarkGray
} else {
    # Private and Domain only. Public is the profile Windows puts you on in a
    # cafe, and a game is not a reason to accept inbound traffic there.
    New-NetFirewallRule -DisplayName $ruleName `
                        -Description 'Lets other machines on your local network join a StarWorks session hosted here.' `
                        -Direction Inbound `
                        -Protocol UDP `
                        -Program $Exe `
                        -Action Allow `
                        -Profile Private,Domain | Out-Null
    Write-Host "Added: $ruleName (Private and Domain only)" -ForegroundColor Green
}

Write-Host ''
Write-Host 'Now, in the game on THIS machine: F3, then HOST.' -ForegroundColor Cyan
Write-Host 'The panel prints the address the other machine should type into its own F3 field.'
