cd F:\StarWorks
Remove-Item -Recurse -Force build\windows   # purge le cache de la config précédente 
cmake --preset windows-msvc

cmake --build --preset windows-debug

.\build\windows\bin\Debug\StarWorks.exe # lancement 