# ============================================================================
# package.ps1 — build a folder you can copy to another machine and run.
#
# WHY THIS EXISTS. `build\windows\bin\Debug\StarWorks.exe` runs on the machine
# that built it and on no other. A DEBUG build links the DEBUG C++ runtime —
# vcruntime140d.dll, msvcp140d.dll, ucrtbased.dll — and Microsoft does not
# redistribute those: they ship with Visual Studio and nothing else. Windows
# refuses to start the process before any of our code runs, which is why the
# failure has no log, no message and no clue in it.
#
# What this produces instead: dist\StarWorks\ — a RelWithDebInfo build with
# the runtime linked STATICALLY, plus the shaders and the assets. Copy the
# folder anywhere. The target machine needs a Vulkan 1.3 graphics driver and
# nothing else at all — no Visual Studio, no redistributable, no Vulkan SDK.
#
#   .\package.ps1                 # build and stage into dist\StarWorks
#   .\package.ps1 -Clean          # purge the packaging build tree first
#   .\package.ps1 -Zip            # ...and produce dist\StarWorks.zip
# ============================================================================

[CmdletBinding()]
param(
    [switch]$Clean,
    [switch]$Zip
)

$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
$buildDir = Join-Path $root 'build\package'
$distDir = Join-Path $root 'dist\StarWorks'

if ($Clean -and (Test-Path $buildDir)) {
    Write-Host 'Purging the packaging build tree...' -ForegroundColor DarkGray
    Remove-Item -Recurse -Force $buildDir
}

# ---- 1. configure ----------------------------------------------------------
# A SEPARATE build tree from build\windows on purpose: the static runtime is
# a different ABI, and sharing a cache with the development build would mean
# a full rebuild every time you switched between them.
Write-Host 'Configuring (static runtime, no tests)...' -ForegroundColor Cyan
cmake -S $root -B $buildDir -G 'Visual Studio 17 2022' -A x64 `
      -DSW_STATIC_RUNTIME=ON -DSW_BUILD_TESTS=OFF
if ($LASTEXITCODE -ne 0) { throw 'CMake configure failed.' }

# ---- 2. build --------------------------------------------------------------
Write-Host 'Building RelWithDebInfo...' -ForegroundColor Cyan
cmake --build $buildDir --config RelWithDebInfo --target StarWorks
if ($LASTEXITCODE -ne 0) { throw 'Build failed.' }

# ---- 3. stage --------------------------------------------------------------
$binDir = Join-Path $buildDir 'bin\RelWithDebInfo'
$exe = Join-Path $binDir 'StarWorks.exe'
if (-not (Test-Path $exe)) { throw "Built, but $exe is missing." }

if (Test-Path $distDir) { Remove-Item -Recurse -Force $distDir }
New-Item -ItemType Directory -Force -Path $distDir | Out-Null

Copy-Item $exe $distDir
# The shaders and the assets are ALREADY next to the executable — the build
# mirrors them there so the game resolves "Shaders/Mesh.vert.spv" relative to
# its own path and never to the working directory. Copying from there rather
# than from the source tree is deliberate: it packages exactly what was
# built, including the compiled .spv, and never a shader edited since.
foreach ($folder in @('Shaders', 'Assets')) {
    $source = Join-Path $binDir $folder
    if (-not (Test-Path $source)) { throw "Missing $folder next to the executable." }
    Copy-Item -Recurse $source (Join-Path $distDir $folder)
}

@"
StarWorks
=========

Run StarWorks.exe. Keep it in this folder: the game finds Shaders\ and
Assets\ next to its own executable, so a lone .exe will not start.

REQUIREMENTS ON THIS MACHINE
  * Windows 10 or 11, 64-bit.
  * A graphics driver with Vulkan 1.3. Any current NVIDIA, AMD or Intel
    driver has it; a fresh Windows install with the Microsoft Basic Display
    Adapter does not. If the game exits complaining that no Vulkan device
    was found, install the manufacturer's driver.
  * Nothing else. The C++ runtime is linked into the executable, so there is
    no Visual C++ redistributable to install and no Vulkan SDK to fetch.

IF IT DOES NOT START
  Run it from a terminal so you can read what it says:

      .\StarWorks.exe --log-file starworks.log

  A window that never appears, with no output at all, means Windows refused
  to load the executable itself — that is a missing DLL, and on THIS build
  the only one that can be missing is vulkan-1.dll, which comes with the
  graphics driver.

USEFUL FLAGS
  --log-file <path>    mirror the log to a file
  --quality low|medium|high
  --cpu                software rendering (very slow, but it runs anywhere)
"@ | Set-Content -Path (Join-Path $distDir 'RUNNING.txt') -Encoding UTF8

$size = (Get-ChildItem -Recurse $distDir | Measure-Object -Property Length -Sum).Sum
Write-Host ''
Write-Host "Packaged: $distDir" -ForegroundColor Green
Write-Host ("  {0:N1} MB, {1} files" -f ($size / 1MB), (Get-ChildItem -Recurse -File $distDir).Count)
Write-Host '  Copy the whole folder. The target needs a Vulkan 1.3 driver and nothing else.'

if ($Zip) {
    $zipPath = Join-Path $root 'dist\StarWorks.zip'
    if (Test-Path $zipPath) { Remove-Item -Force $zipPath }
    Compress-Archive -Path $distDir -DestinationPath $zipPath
    Write-Host "  Archive: $zipPath" -ForegroundColor Green
}
