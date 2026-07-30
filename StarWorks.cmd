@echo off
rem ==========================================================================
rem StarWorks.cmd — double-clique ce fichier pour jouer.
rem
rem POURQUOI CE FICHIER REMPLACE UN RACCOURCI. Un raccourci Windows (.lnk)
rem enregistre un chemin ABSOLU vers sa cible : celui qui traînait ici
rem pointait encore sur F:\StarWorks\build\windows\bin\Debug\StarWorks.exe et
rem ne marchait donc plus du tout depuis le déplacement sur G:. On ne peut pas
rem rendre un .lnk relatif, mais un .cmd sait où il est : %~dp0 est le dossier
rem de CE fichier, avec son antislash final. C'est l'équivalent batch de
rem $PSScriptRoot, et il suit le projet partout.
rem
rem Cherche l'exécutable dans l'ordre où tu veux probablement le lancer :
rem le dossier packagé d'abord, puis les builds de développement.
rem ==========================================================================

setlocal
set "ROOT=%~dp0"
set "EXE="

for %%C in (
    "dist\StarWorks\StarWorks.exe"
    "build\package\bin\RelWithDebInfo\StarWorks.exe"
    "build\windows\bin\RelWithDebInfo\StarWorks.exe"
    "build\windows\bin\Debug\StarWorks.exe"
) do (
    rem `if not defined` est evalue a l'execution, donc le premier trouve
    rem gagne et les suivants ne l'ecrasent pas.
    if not defined EXE if exist "%ROOT%%%~C" set "EXE=%ROOT%%%~C"
)

if not defined EXE (
    echo.
    echo StarWorks.exe est introuvable. Compile-le d'abord :
    echo.
    echo     powershell -ExecutionPolicy Bypass -File "%ROOT%launch.ps1"
    echo.
    pause
    exit /b 1
)

rem Lance depuis SON dossier : le jeu cherche Shaders\ et Assets\ a cote de
rem son propre executable, et un --log-file relatif doit atterrir la aussi.
for %%F in ("%EXE%") do set "EXEDIR=%%~dpF"

echo Lancement de %EXE%
pushd "%EXEDIR%"
"%EXE%" %*
set "CODE=%ERRORLEVEL%"
popd

rem Une fenetre qui se ferme instantanement sur une erreur ne dit rien a
rem personne : on garde la console ouverte quand ca s'est mal termine.
if not "%CODE%"=="0" (
    echo.
    echo StarWorks s'est termine avec le code %CODE%.
    pause
)
exit /b %CODE%
