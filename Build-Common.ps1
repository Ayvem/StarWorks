# ============================================================================
# Build-Common.ps1 — helpers partagés par launch.ps1 et package.ps1.
#
# Dot-source depuis un script frère :
#     . (Join-Path $PSScriptRoot 'Build-Common.ps1')
#
# Rien ici ne suppose où le projet vit : tous les chemins arrivent en
# paramètre, depuis le $PSScriptRoot de l'appelant.
#
# Commentaires en `#` ligne à ligne, et non en blocs `<# #>`, comme dans les
# quatre autres scripts — c'est aussi ce que le garde-fou de portabilité sait
# reconnaître comme du commentaire.
# ============================================================================

# Lit une entrée du CMakeCache.txt. Renvoie $null si le cache ou l'entrée
# n'existe pas.
#
# Le format est `NOM:TYPE=valeur`, une par ligne. On découpe sur le PREMIER
# `=` seulement : une valeur peut en contenir — un chemin Windows non, mais
# une liste de flags de compilation, oui.
function Get-CMakeCacheEntry {
    param(
        [Parameter(Mandatory)][string]$CachePath,
        [Parameter(Mandatory)][string]$Name
    )
    if (-not (Test-Path -LiteralPath $CachePath)) { return $null }
    foreach ($line in Get-Content -LiteralPath $CachePath) {
        if ($line -match "^\s*$([regex]::Escape($Name)):[^=]*=(.*)$") {
            return $Matches[1]
        }
    }
    return $null
}

# Deux chemins désignent-ils le même endroit ? CMake écrit des barres
# obliques, PowerShell des antislashs, et Windows se moque de la casse.
# Comparer les chaînes brutes dirait « différent » pour deux écritures du
# même dossier.
function Test-SamePath {
    param([string]$A, [string]$B)
    if (-not $A -or -not $B) { return $false }
    $normalise = {
        param($p)
        $p.Replace('\', '/').TrimEnd('/').ToLowerInvariant()
    }
    return (& $normalise $A) -eq (& $normalise $B)
}

# UN ARBRE DE BUILD N'EST PAS DÉPLAÇABLE.
#
# CMakeCache.txt enregistre en dur, en absolu, le dossier source ET son
# propre dossier. Déplacer le projet d'un disque à l'autre — ou simplement le
# copier ailleurs — laisse un cache qui pointe sur l'ancien endroit, et CMake
# refuse alors de configurer : « The current CMakeCache.txt directory ... is
# different than the directory ... where CMakeCache.txt was created ».
#
# Ce n'est pas rattrapable en éditant le cache : les chemins absolus sont
# aussi dans les fichiers générés du projet, dans les règles de dépendances
# et dans les .vcxproj. La seule réponse correcte est de jeter l'arbre et de
# reconfigurer — ce que fait cette fonction, en expliquant pourquoi, plutôt
# que de laisser une erreur brute à quelqu'un qui n'a rien fait de mal.
#
# Ne touche à rien quand le cache est cohérent : c'est le cas normal, et
# purger à tout hasard coûterait une recompilation complète à chaque appel.
function Reset-StaleBuildTree {
    param(
        [Parameter(Mandatory)][string]$BuildDir,
        [Parameter(Mandatory)][string]$SourceDir
    )

    $cache = Join-Path $BuildDir 'CMakeCache.txt'
    if (-not (Test-Path -LiteralPath $cache)) { return $false }

    # CMAKE_HOME_DIRECTORY = le dossier source d'origine.
    # CMAKE_CACHEFILE_DIR  = le dossier de build d'origine.
    # Les deux se décalent quand on déplace le projet, et l'un ou l'autre
    # suffit à condamner l'arbre.
    $recordedSource = Get-CMakeCacheEntry -CachePath $cache -Name 'CMAKE_HOME_DIRECTORY'
    $recordedBuild = Get-CMakeCacheEntry -CachePath $cache -Name 'CMAKE_CACHEFILE_DIR'

    $sourceMoved = $recordedSource -and -not (Test-SamePath $recordedSource $SourceDir)
    $buildMoved = $recordedBuild -and -not (Test-SamePath $recordedBuild $BuildDir)
    if (-not $sourceMoved -and -not $buildMoved) { return $false }

    Write-Host ''
    Write-Host 'Arbre de build périmé — le projet a été déplacé.' -ForegroundColor Yellow
    if ($sourceMoved) {
        Write-Host ("  cache configuré pour  {0}" -f $recordedSource) -ForegroundColor DarkGray
        Write-Host ("  source actuelle       {0}" -f $SourceDir) -ForegroundColor DarkGray
    }
    if ($buildMoved) {
        Write-Host ("  cache écrit dans      {0}" -f $recordedBuild) -ForegroundColor DarkGray
        Write-Host ("  build actuel          {0}" -f $BuildDir) -ForegroundColor DarkGray
    }
    Write-Host '  Purge et reconfiguration (un arbre CMake est plein de chemins absolus).' -ForegroundColor Yellow
    Write-Host ''

    Remove-Item -Recurse -Force -LiteralPath $BuildDir
    return $true
}
