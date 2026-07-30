# StarWorks — installation, de zéro à une partie en réseau

Toutes les commandes, dans l'ordre. Chaque bloc se copie-colle tel quel dans **PowerShell**.

Il y a deux rôles de machine et ils ne demandent pas du tout la même chose :

| | Ce qu'il faut installer |
|---|---|
| **Machine de développement** — celle qui compile | Visual Studio 2022 (C++), CMake, Git, Vulkan SDK |
| **Machine invitée** — celle qui reçoit juste le jeu | **rien**, à part un pilote graphique Vulkan 1.3 |

Si tu veux seulement faire jouer quelqu'un, saute directement à [§3](#3-fabriquer-le-dossier-à-copier) puis [§4](#4-sur-lautre-machine).

---

## 1. Machine de développement — les outils

À faire **une seule fois**. PowerShell **administrateur**.

```powershell
winget install --id Git.Git -e
winget install --id Kitware.CMake -e
winget install --id KhronosGroup.VulkanSDK -e
winget install --id Microsoft.VisualStudio.2022.Community -e --override "--quiet --add Microsoft.VisualStudio.Workload.NativeDesktop --includeRecommended"
```

Le SDK Vulkan n'est pas optionnel **sur cette machine** : il fournit les en-têtes, le loader, et surtout `glslangValidator`, qui transforme les shaders `.glsl` en SPIR-V pendant le build. Sans lui, CMake s'arrête net avec « no GLSL compiler found ». Il n'est en revanche **pas** nécessaire sur la machine qui joue.

Ferme et rouvre PowerShell ensuite — les installeurs modifient le `PATH` et la session en cours ne le voit pas. Vérification :

```powershell
git --version
cmake --version          # doit être >= 3.24
glslangValidator --version
```

---

## 2. Compiler et lancer

```powershell
# Place-toi dans le dossier du projet, où qu'il soit — G:\StarWorks,
# D:\Jeux\StarWorks, une clé USB : rien dans le projet ne suppose l'endroit.
cd <le dossier StarWorks>
cmake --preset windows-msvc
cmake --build --preset windows-debug
.\build\windows\bin\Debug\StarWorks.exe
```

Le premier `cmake --preset` télécharge GLFW, GLM, VulkanMemoryAllocator et cgltf depuis GitHub — il lui faut **internet**, et il prend quelques minutes. Les suivants sont instantanés.

Ou double-clique **`StarWorks.cmd`** à la racine : il cherche l'exécutable tout seul (dossier packagé d'abord, puis les builds de dev) et le lance depuis son propre dossier. Il remplace le raccourci `.lnk` qui traînait ici — un `.lnk` enregistre un chemin absolu, celui-là pointait encore sur `F:\` et ne marchait plus depuis le déplacement.

Ou, tout en un — configure si nécessaire, compile, lance :

```powershell
.\launch.ps1
.\launch.ps1 -Clean      # purge l'arbre de build d'abord
.\launch.ps1 -Release    # RelWithDebInfo au lieu de Debug
.\launch.ps1 -Test       # la suite de tests au lieu du jeu
.\launch.ps1 -GameArgs '--log-file starworks.log'
```

`launch.ps1` ne reconfigure que s'il n'y a pas encore de cache : reconfigurer à chaque lancement ajouterait une demi-minute à une boucle qui doit durer quelques secondes.

**Après avoir changé de branche ou récupéré de nouveaux fichiers**, si la configuration part en erreur bizarre :

```powershell
Remove-Item -Recurse -Force build\windows
cmake --preset windows-msvc
cmake --build --preset windows-debug
```

### Les tests

Ni fenêtre, ni carte graphique, ni socket. Une seconde et demie.

```powershell
ctest --test-dir build/windows -C Debug --output-on-failure
```

### Options utiles au lancement

```powershell
.\build\windows\bin\Debug\StarWorks.exe --log-file starworks.log
.\build\windows\bin\Debug\StarWorks.exe --quality low
.\build\windows\bin\Debug\StarWorks.exe --frames 300      # quitte après 300 images
.\build\windows\bin\Debug\StarWorks.exe --cpu             # rendu logiciel, très lent
```

---

## 3. Fabriquer le dossier à copier

**Ne copie jamais `build\windows\bin\Debug\`.** Un build Debug se lie au runtime C++ *debug* — `vcruntime140d.dll`, `msvcp140d.dll`, `ucrtbased.dll` — que Microsoft ne redistribue pas : ces DLL n'existent que là où Visual Studio est installé. Windows refuse de démarrer le processus avant qu'une seule ligne de code ne s'exécute, donc l'échec n'a ni log, ni message, ni indice.

```powershell
cd <le dossier StarWorks>
.\package.ps1              # -> dist\StarWorks\
.\package.ps1 -Zip         # ...et dist\StarWorks.zip
.\package.ps1 -Clean       # repart d'un arbre de build vierge
```

Ça produit `dist\StarWorks\` : l'exécutable en RelWithDebInfo avec le **runtime lié statiquement**, plus `Shaders\`, `Assets\` et un `RUNNING.txt`. Compte quelques minutes : c'est un arbre de build séparé de `build\windows` (le runtime statique est un ABI différent, les partager forcerait une recompilation complète à chaque va-et-vient).

---

## 4. Sur l'autre machine

```powershell
# Copie dist\StarWorks\ (le DOSSIER ENTIER) où tu veux, puis :
cd <le dossier copié>
.\StarWorks.exe
```

**Le dossier, pas le fichier.** Le jeu cherche `Shaders\Mesh.vert.spv` et `Assets\Parts\*.swpart` à côté de son propre exécutable, jamais dans le répertoire courant. Un `.exe` copié tout seul ne démarrera pas.

Il n'y a **rien à installer** : pas de Visual Studio, pas de redistribuable Visual C++, pas de SDK Vulkan. Il faut seulement un pilote graphique avec Vulkan 1.3 — n'importe quel pilote NVIDIA, AMD ou Intel à jour l'a ; une installation Windows fraîche sur « Carte graphique de base Microsoft » ne l'a pas.

Si rien ne s'affiche :

```powershell
.\StarWorks.exe --log-file starworks.log
notepad starworks.log
```

Une fenêtre qui n'apparaît jamais **avec zéro sortie** signifie que Windows a refusé de charger l'exécutable : c'est une DLL manquante, et sur ce build la seule qui puisse manquer est `vulkan-1.dll`, qui vient avec le pilote graphique.

---

## 5. Jouer en réseau local

**Le routeur n'est pas concerné et n'a rien à configurer.** Deux machines en `192.168.1.x` se parlent directement ; rien ne sort sur internet, donc il n'y a aucun port à ouvrir dessus. Ce qui bloque, c'est le pare-feu de la machine qui héberge.

### Sur la machine qui héberge

1. Lance le jeu, `F3`, puis `HOST`.
2. Windows demande les droits administrateur → **accepte**. C'est un `netsh` isolé qui ajoute la règle et se termine ; le jeu, lui, n'est jamais élevé. Une seule fois : ensuite la règle existe et on ne te redemande rien.
3. Le panneau affiche l'adresse à donner à l'autre joueur, par exemple `HOSTING ON 192.168.1.61:7777`. Lis-la là — pas dans `ipconfig`, qui sur une machine avec Docker, WSL ou un VPN en propose une demi-douzaine dont une seule est la bonne.

Pour ajouter la règle à l'avance, sans passer par le jeu (PowerShell **administrateur**) :

```powershell
cd <le dossier StarWorks>
.\firewall.ps1                          # ajoute la règle
.\firewall.ps1 -Exe <chemin>\StarWorks.exe   # pour un build précis
.\firewall.ps1 -Check                   # ne change rien, dit juste ce qui cloche
.\firewall.ps1 -Remove                  # la retire
```

### Sur la machine invitée

`F3`, clique le champ, tape l'adresse (`192.168.1.61:7777`), puis `JOIN`.

---

## 6. Si la connexion échoue

Fais-les **dans cet ordre**. Chacune élimine une cause que les suivantes ne peuvent plus expliquer.

### 6.1 — Le même binaire des deux côtés

C'est la cause la plus fréquente et la plus invisible. Deux builds différents ne parlent pas la même version de protocole ; l'hôte rejette le paquet en silence, et côté client ça ressemble **exactement** à un port bloqué. Recopie `dist\StarWorks\` sur l'autre machine après chaque `package.ps1`.

Le jeu le dit maintenant : côté hôte, le panneau `F3` affiche `PEER SPEAKS Vn - REBUILD IT`.

### 6.2 — Regarde le compteur de l'hôte

Pendant que l'autre essaie de rejoindre, regarde la ligne `RX n  REFUSED n` du panneau `F3` de l'hôte. Elle coupe le problème en deux :

| Verdict affiché | Ce que ça veut dire |
|---|---|
| `NOTHING HAS REACHED THIS PC` | Aucun paquet n'arrive. Pare-feu, profil réseau Public, mauvaise adresse, ou les machines ne se voient pas. → 6.3 |
| `PEER SPEAKS Vn - REBUILD IT` | Les paquets arrivent très bien. Builds différents. → 6.1 |
| `NETWORK IS PUBLIC - RULE INACTIVE` | La règle existe mais Windows l'ignore. → 6.4 |
| `PACKETS ARE GETTING THROUGH` | Réseau et pare-feu OK. |

### 6.3 — Teste le réseau sans le jeu

**Le ping ne prouve rien ici.** Windows livre la règle ICMP echo entrante *désactivée* : deux PC parfaitement connectés sur le même switch refusent de se pinger par défaut.

`netcheck.ps1` envoie de l'UDP nu, sans rien de nous dedans. Les deux moitiés en même temps :

```powershell
# sur la machine qui hébergerait
.\netcheck.ps1 -Listen

# sur l'autre, avec l'adresse de la première
.\netcheck.ps1 -Send 192.168.1.61
```

L'écouteur répond à chaque datagramme, donc l'émetteur apprend si le chemin marche **dans les deux sens** — un pare-feu est à sens unique et le trajet retour est justement la moitié qui marche d'habitude.

- **Si `netcheck` passe et que le jeu échoue** → le problème est dans le jeu ou dans sa règle.
- **Si `netcheck` échoue aussi** → le jeu n'a jamais eu sa chance. Compare les adresses **avec leur préfixe** que le script affiche : deux sous-réseaux différents ne se joindront jamais, quelle que soit la règle. Pas d'entrée ARP = le routeur isole les clients entre eux (Wi-Fi invité, « AP isolation »).

Autres commandes :

```powershell
.\netcheck.ps1 -Local        # décrit juste cette machine
.\netcheck.ps1 -AllowPing    # autorise le ping entrant (admin, diagnostic seulement)
```

### 6.4 — Le profil réseau

Windows bloque tout le trafic entrant sur un réseau classé **Public**, quelles que soient les règles — et il choisit Public en silence pour tout réseau dont tu n'as jamais répondu à l'invite « rendre ce PC détectable ? ». La règle du jeu couvre Private et Domain uniquement, parce qu'un jeu n'est pas une raison d'accepter du trafic entrant dans un café.

```powershell
Get-NetConnectionProfile                                              # regarde NetworkCategory
Set-NetConnectionProfile -InterfaceIndex 12 -NetworkCategory Private  # admin ; mets ton vrai index
```

### 6.5 — Un antivirus tiers

Avast, Bitdefender, Norton, ESET et compagnie ont leur propre pare-feu, qui ignore complètement les règles Windows. L'autorisation doit être donnée dans leur interface à eux.

---

## Aide-mémoire

```powershell
# --- développement -----------------------------------------------------------
cmake --preset windows-msvc                             # configurer
cmake --build --preset windows-debug                    # compiler
.\build\windows\bin\Debug\StarWorks.exe                 # lancer
ctest --test-dir build/windows -C Debug --output-on-failure
.\StarWorks.cmd                                         # juste lancer (double-cliquable)
.\launch.ps1                                            # configurer + compiler + lancer
.\launch.ps1 -Test                                      # ...ou lancer les tests

# --- distribution ------------------------------------------------------------
.\package.ps1                                           # -> dist\StarWorks\
.\package.ps1 -Zip

# --- réseau ------------------------------------------------------------------
.\firewall.ps1 -Check                                   # état du pare-feu + adresse
.\firewall.ps1                                          # ajouter la règle (admin)
.\netcheck.ps1 -Listen                                  # test UDP, côté hôte
.\netcheck.ps1 -Send 192.168.1.61                       # test UDP, côté invité
```

Les touches du jeu sont dans `README.md`, section *Controls*.
