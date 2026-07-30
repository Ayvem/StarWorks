# ============================================================================
# launch.ps1 — configurer, compiler, lancer. La boucle de dev en une commande.
#
# AUCUN CHEMIN ABSOLU ICI. Ce script commençait par un `cd` vers une lettre
# de lecteur précise : déplacer le projet d'un disque à l'autre le cassait,
# en silence, dès la première ligne. Tout ci-dessous part de $PSScriptRoot —
# le dossier où CE fichier se trouve — donc le projet marche depuis F:\,
# depuis G:\, depuis une clé USB ou depuis un chemin avec un espace dedans,
# sans une seule modification.
#
#   .\launch.ps1                 # configure si besoin, compile en Debug, lance
#   .\launch.ps1 -Clean          # purge l'arbre de build d'abord
#   .\launch.ps1 -Release        # RelWithDebInfo au lieu de Debug
#   .\launch.ps1 -Test           # lance la suite de tests au lieu du jeu
#   .\launch.ps1 -NoRun          # compile seulement
#   .\launch.ps1 -GameArgs '--log-file starworks.log'
# ============================================================================

[CmdletBinding()]
param(
    [switch]$Clean,
    [switch]$Release,
    [switch]$Test,
    [switch]$NoRun,
    # PAS $Args : c'est une variable automatique de PowerShell (les arguments
    # non liés de la fonction courante), et la nommer ainsi dans un param()
    # est une erreur de syntaxe, pas un avertissement.
    [string]$GameArgs = ''
)

$ErrorActionPreference = 'Stop'

# LA RACINE, DEMANDÉE PLUTÔT QUE SUPPOSÉE. $PSScriptRoot est le dossier de ce
# fichier, quel que soit le disque du jour.
$root = $PSScriptRoot
$buildDir = Join-Path $root 'build\windows'
$config = if ($Release) { 'RelWithDebInfo' } else { 'Debug' }

Write-Host ("StarWorks dans {0}" -f $root) -ForegroundColor DarkGray

if ($Clean -and (Test-Path $buildDir)) {
    Write-Host "Purge de l'arbre de build..." -ForegroundColor DarkGray
    Remove-Item -Recurse -Force $buildDir
}

# ---- configurer -------------------------------------------------------------
# Seulement s'il n'y a pas encore de cache, ou après -Clean. Reconfigurer à
# chaque lancement ajouterait une demi-minute à une boucle qui doit durer
# quelques secondes.
if (-not (Test-Path (Join-Path $buildDir 'CMakeCache.txt'))) {
    Write-Host 'Configuration...' -ForegroundColor Cyan
    # -S et -B plutôt que --preset : le résultat est le même, mais être
    # explicite garantit que ce script ne dépend pas en douce du répertoire
    # courant de celui qui l'a lancé.
    cmake -S $root -B $buildDir -G 'Visual Studio 17 2022' -A x64
    if ($LASTEXITCODE -ne 0) { throw 'La configuration CMake a échoué.' }
}

# ---- compiler ----------------------------------------------------------------
Write-Host ("Compilation {0}..." -f $config) -ForegroundColor Cyan
cmake --build $buildDir --config $config
if ($LASTEXITCODE -ne 0) { throw 'La compilation a échoué.' }

# ---- tests -------------------------------------------------------------------
if ($Test) {
    ctest --test-dir $buildDir -C $config --output-on-failure
    exit $LASTEXITCODE
}

if ($NoRun) { return }

# ---- lancer ------------------------------------------------------------------
$exe = Join-Path $buildDir "bin\$config\StarWorks.exe"
if (-not (Test-Path $exe)) { throw "Compilé, mais $exe est introuvable." }

# Lancé depuis son propre dossier, pour qu'un --log-file relatif atterrisse à
# côté de l'exécutable et pas là où le shell se trouvait.
Write-Host ("Lancement de {0}" -f $exe) -ForegroundColor Cyan
$workingDirectory = Split-Path $exe
if ($GameArgs) {
    # Découpé sur les espaces : -ArgumentList avec UNE chaîne se comporte
    # différemment selon la version de PowerShell (un seul argument, ou la
    # ligne de commande entière). Un tableau est sans ambiguïté partout.
    Start-Process -FilePath $exe -WorkingDirectory $workingDirectory `
                  -ArgumentList ($GameArgs -split '\s+') -Wait
} else {
    Start-Process -FilePath $exe -WorkingDirectory $workingDirectory -Wait
}
