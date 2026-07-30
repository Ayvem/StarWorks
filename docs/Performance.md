# StarWorks — rapport de performance

Toutes les valeurs ci-dessous sont **mesurées**, pas estimées, sur le build Linux
release (`build/linux-release`, GCC 13, `-O2`), machine de développement, un seul
thread sauf mention contraire. Les sondes sont des programmes jetables liés contre
`libStarWorksEngine.a` : elles appellent les vrais systèmes, sur un vrai catalogue de
pièces et les vraies tables `.aero.json`, pas des reconstitutions.

Les chiffres marqués **(calculé)** sont dérivés de constantes du code, pas chronométrés.

---

## 1. La boucle physique — 50 Hz, budget 20 ms par tick

C'est le seul endroit où un coût se paie **cinquante fois par seconde**. Mesuré avec
Terra (gravité + atmosphère + terrain) plus deux autres corps, une fusée en vol à
9 km et 320 m/s, en atmosphère.

| Système | 7 pièces | 15 pièces | 31 pièces |
|---|---:|---:|---:|
| `VesselAssemblySystem` (masse, centre de masse, inertie) | 0,72 µs | 1,42 µs | 2,79 µs |
| `GravityIntegrationSystem` | 0,06 µs | 0,06 µs | 0,06 µs |
| `VesselAerodynamicsSystem` | **5,53 µs** | **21,44 µs** | **67,46 µs** |
| `SurfaceInteractionSystem` (en vol) | 2,42 µs | 2,46 µs | 2,33 µs |
| `SurfaceInteractionSystem` (posé, avec basculement + friction angulaire) | 2,33 µs | 2,36 µs | 2,30 µs |
| **Total de la voie physique** | **8,74 µs** | **25,39 µs** | **72,88 µs** |
| **En % d'un tick de 20 ms** | 0,044 % | 0,127 % | 0,364 % |

Autrement dit : à 31 pièces, l'ensemble aérodynamisme + gravité + sol + assemblage
consomme **0,36 %** du budget d'un tick. Le reste du tick appartient à tout ce qui
existait déjà.

### Le contact au sol est gratuit

Le basculement sous gravité (polygone de sustentation, huit coins, recherche du
porte-à-faux) et la friction angulaire **ne coûtent rien de mesurable** : 2,33 µs posé
contre 2,42 µs en vol — la différence est dans le bruit, parce qu'en vol le système
fait le calcul de traînée isotrope qu'il saute quand la pièce a une table.

### L'aérodynamisme est quadratique — et c'est la seule chose à surveiller

Le coût suit le carré du nombre de pièces, parce que l'occlusion teste chaque pièce
contre les boîtes de toutes les autres, neuf rayons à chaque fois. Extrapolation
depuis les trois points mesurés :

| Pièces | Coût mesuré / extrapolé | % d'un tick |
|---|---:|---:|
| 7 | 5,5 µs | 0,03 % |
| 15 | 21,4 µs | 0,11 % |
| 31 | 67,5 µs | 0,34 % |
| 60 | ~250 µs (extrapolé) | ~1,3 % |
| 120 | ~1,0 ms (extrapolé) | ~5 % |
| 250 | ~4,3 ms (extrapolé) | ~22 % |

C'est confortable jusqu'à des vaisseaux de plus de cent pièces. Au-delà, la sortie
évidente est de ne recalculer l'exposition que lorsque la direction du flux a bougé de
plus d'un degré ou deux — elle change lentement — ce qui ramènerait le terme dominant
à une fraction des ticks. Non fait, parce que rien ne le demande encore.

### L'optimisation trouvée en écrivant ce rapport

La première mesure donnait **283 µs** à 31 pièces. Deux changements l'ont ramenée à
**67 µs**, soit **4,2×** :

1. **Le bon rejet précoce.** Rejeter les boîtes loin de la *ligne* du rayon ne vaut
   presque rien ici : sur une fusée toutes les boîtes serrent l'axe le long duquel
   l'air circule, donc presque rien n'est hors-ligne — cela n'a rapporté que 11 %.
   Le rejet qui paie est **le long du flux** : une boîte que le rayon n'atteint
   qu'*après* avoir touché la pièce concernée ne peut pas l'ombrer, et nez en avant
   c'est la moitié du véhicule.
2. **Les tampons de travail sur le système, plus sur la pile.** Deux allocations tas
   par vaisseau par tick, cinquante fois par seconde, supprimées.

Gain aux trois tailles : 7 pièces 13,3 → 5,5 µs, 15 pièces 64,5 → 21,4 µs,
31 pièces 282,9 → 67,5 µs.

---

## 2. Le patch de terrain — thread de pool, au plus une fois par seconde

Le seul poste vraiment coûteux du projet, et il ne touche jamais le thread principal :
le job est soumis au pool, écrit un maillage en attente, et le thread principal
l'envoie au GPU une frame plus tard. Mesuré à l'échelle d'atterrissage (patch de
1 500 m, grille 192, mailles de 15,6 m) :

| Passe | Coût | Ce que c'est |
|---|---:|---|
| Heightfield | **39–47 ms** | 37 249 évaluations d'un champ à 16 octaves |
| Ombrage du relief | **17–21 ms** | ombre portée (20 pas) + occlusion du ciel (6 azimuts × 6 pas), sur la grille déjà en main — **zéro échantillon de heightfield en plus** |
| *(les plantes ont leur propre job — voir 2 bis)* | | |
| **Total** | **~57–68 ms** | sur un thread de pool, cadence ≥ 1 s, **tous les 450 m** |

### 2 bis. Le champ d'herbe — son propre job, sa propre cadence

Séparé du patch parce qu'il doit suivre le joueur à **40 m** quand le sol se contente
de 450 m, et que faire suivre le sol à 40 m multipliait par onze un envoi GPU de
5,8 Mo sur le thread principal.

| Passe | Coût | Ce que c'est |
|---|---:|---|
| Semis | **~2,5 ms** | 94 388 cellules parcourues, ~5 000 touffes retenues sur 260 m, lues dans la grille de sol déjà en cache (aucun échantillon de heightfield) |
| Copie de la grille | ~0,2 ms | 1,8 Mo copiés dans le job, pour qu'une reconstruction du terrain ne puisse pas tirer le sol de dessous |
| Envoi GPU | **6 × 603 Ko**, un par frame | contre **1 × 5,8 Mo** avant |

`uploadToBuffer` soumet sa copie puis **attend une fence** : l'attente draine ce que la
file graphique tient déjà, donc chaque envoi coûte jusqu'à une frame de travail GPU
quelle que soit sa taille. Six petits sur six frames transforment un pic visible en six
frames que personne ne remarque — et le champ à l'écran n'est jamais incomplet, l'ancien
jeu de chunks continuant d'être dessiné jusqu'à ce que le dernier nouveau ait atterri.

Le heightfield domine à 70 %. L'ombrage ajoute ~35 % au coût d'un patch, ce qui est le
prix conscient d'un relief lisible ; il tombe à zéro si on le supprime, et rien
d'autre n'en dépend.

**Ce que la résolution a coûté.** Passer de 96 à 192 mailles (le correctif du sol qui
traversait) multiplie par 4 le nombre de sommets : 9 400 → 37 249, donc ~12 ms →
~45 ms de heightfield. Les patchs lointains (≥ 25 km d'étendue) gardent 96 mailles et
leur ancien coût, parce que personne à cette distance ne touche le sol.

---

## 3. Géométrie et appels de dessin (GPU)

**(calculé)** à partir des constantes du code, à l'échelle d'atterrissage :

| Élément | Triangles | Appels |
|---|---:|---:|
| Sol du patch (192²×2) | 73 728 | |
| Jupe de bord (4 × 192 quads, deux enroulements) | 3 072 | |
| **Le patch entier (sol + jupe)** | **76 800** | **1** |
| Herbe (~5 000 touffes × 3 brins × 4 tris) | ~60 000 | 6 |
| Globe LOD 0 (150 anneaux × 225 segments) | 67 500 | 1 |

Le patch tient en **un seul appel de dessin** : c'est le point de l'avoir cuit dans le
maillage plutôt que d'instancier des plantes. Mémoire : 107 521 sommets × 48 octets =
**5,2 Mo**, doublé par le double-buffering = **10,3 Mo** de VRAM.

Le globe reste dessiné sous le patch, mais le tri des batches d'avant en arrière plus
le test de profondeur précoce déclaré dans `Mesh.frag` rejettent ses fragments cachés
**avant** qu'ils n'évaluent une seule octave de bruit — au sol, c'est la quasi-totalité
de l'écran.

---

## 4. Hors du jeu — AeroForge

Ne tourne jamais pendant une partie. **7,3 s** pour résoudre les neuf pièces de
vaisseau du catalogue (342 directions de vent chacune, rasterisation 192², champ de
triangles réel). À relancer seulement après avoir modifié une géométrie dans Part
Studio.

Les tables sur disque : **9 fichiers, 216 Ko au total, 21 Ko en moyenne**. Chargées une
fois au démarrage ; la recherche par pièce est une `lower_bound` sur un vecteur trié,
donc aucune allocation sur un tick.

---

## 4 bis. Le réseau — hôte autoritaire, un client

Mesuré par `Tools/NetProbe` : deux **vraies** sockets UDP sur l'interface de
bouclage, les **vrais** types de composants (`sw.Transform`,
`phys.DynamicBody`, `phys.OnRails`), le vrai encodeur. La boucle n'est pas
l'internet : ce qu'elle mesure honnêtement, c'est le **coût CPU** et le
**nombre d'octets**, les deux seules choses qu'un jeu peut se tromper tout
seul. La perte et la latence sont mesurées ailleurs, contre le fil simulé de
la suite de tests, où elles se spécifient exactement.

### Ce que coûte un instantané

| Entités | Dont mobiles | Instantané complet | Delta | Capture | Encodage |
|---|---:|---:|---:|---:|---:|
| 100 | 5 | 9,2 Ko | **606 o** | 2,4 µs | **5,0 µs** |
| 500 | 20 | 45,4 Ko | 2 316 o | 17,5 µs | 25,8 µs |
| 2 000 | 60 | 180,3 Ko | 6 876 o | 78,1 µs | 111,7 µs |
| 10 000 | 200 | 871,2 Ko | 22,8 Ko | 743,9 µs | 900,6 µs |

Le delta ne suit pas la taille du monde, il suit **ce qui a bougé** : à 500
entités, 40 enregistrements changent sur 620, et le delta fait 5 % de
l'instantané complet. Le coût, lui, suit la taille du monde, parce que la
capture parcourt tout : c'est un `memcmp` par composant, sans code par
composant et sans drapeau qu'un système puisse oublier de lever.

À vingt instantanés par seconde, un monde de 500 entités coûte à l'hôte
**0,05 % d'un cœur par client**. À 2 000 entités, 0,22 %.

### Ce que coûte une session

| | Coopératif (5 vaisseaux, 100 entités) | Chargé (20 vaisseaux, 500 entités) |
|---|---:|---:|
| Descendant | **14,7 Ko/s** | 56,6 Ko/s |
| Montant (entrées à 50 Hz) | **1,6 Ko/s** | 1,6 Ko/s |
| Delta | 606 o — **tient dans un datagramme** | 2 316 o — trois datagrammes |
| Monde dans le miroir | 0,0 ms après connexion | **0,9 ms** (45 Ko transférés en fiable) |
| Renvois | 0 | 0 |
| Instantanés rebasés | 0 | 0 |

Le montant ne dépend pas du monde : un client envoie une **intention**
(« cabré, poussée à 60 % »), jamais un état, donc son débit est fixé par la
cadence physique et rien d'autre.

**Le seuil qui compte est 1 184 octets**, la charge utile d'un datagramme. En
dessous, le delta part sur le canal **non fiable** — il n'est jamais renvoyé,
et un instantané perdu est simplement remplacé par le suivant. Au-dessus, il
est promu sur le canal fiable pour être fragmenté, ce qui est correct mais
réintroduit l'attente de retransmission qu'on cherchait à éviter pour l'état.
Une session coopérative reste sous le seuil ; une session chargée le dépasse.

C'est **le plafond connu de ce jalon**, et le remède est nommé : la gestion
d'intérêt (n'envoyer à un client que ce qu'il peut percevoir). Un joueur sur
Terra n'a pas besoin de la base sur Luna à vingt hertz, et le design du jeu le
dit déjà. Non fait ici — ce jalon est le fil, prouvé.

### Le warp de recalage

Mesuré sur le vrai servo (le plus grand cran dont la vitesse reste sous la
moitié du temps restant, rechoisi à chaque frame) :

| Écart | Temps réel | Cran max | Dépassement |
|---|---:|---:|---:|
| 1 minute | 13,0 s | x10 | 0,02 s |
| 1 heure | 38,5 s | x1 000 | 0,02 s |
| **3 heures** | **45,7 s** | x1 000 | 0,02 s |
| 1 jour | 61,6 s | x10 000 | 0,00 s |
| 30 jours | 91,0 s | x1 000 000 | 0,02 s |
| 1 an | 109,9 s | x10 000 000 | 0,00 s |

À 30 images par seconde les mêmes écarts prennent 45,5 s et 109,2 s : le servo
lit le **temps restant**, pas un nombre d'appuis, donc il est indépendant de la
cadence d'affichage. Le dépassement reste sous 0,03 s parce que le dernier cran
est toujours x1.

---

## 5. Prédiction de trajectoire

Mesurée plus tôt dans cette session, code inchangé depuis : **0,9 à 1,9 ms par appel**,
appelée depuis `refreshPrediction()` et non à chaque frame. La résolution du balayage
d'approche a été réduite de 4 096 à 256 échantillons après avoir mesuré qu'ils donnent
la même réponse au mètre près — **175 µs contre 6,4 ms**, seize fois moins de travail
pour le même résultat.

---

## 6. Suite de tests

**189 tests, 0 échec, ~1 s** en tout — sans fenêtre, sans périphérique Vulkan et
**sans une seule socket**. Les tests les plus lourds sont les aérodynamiques (le
solveur tourne pour de vrai sur des formes de manuel) et le contrat de résolution du
terrain, qui échantillonne le champ analytique des dizaines de milliers de fois.

Les vingt-trois tests réseau et de temporalité tournent contre un **fil simulé** : générateur graine,
horloge passée en paramètre. « 80 ms de latence, 60 ms de gigue, 20 % de perte,
pendant quatre secondes » s'exécute en microsecondes et donne la même réponse à
chaque fois — ce qui n'est vrai d'aucun test impliquant une deuxième machine, ni
même une deuxième socket. Le test qui compte : après ces quatre secondes, chaque
valeur du miroir s'accorde encore à 1e-9 avec l'instant que le client croit
regarder.

---

## 7. Où sont les plafonds

Par ordre de proximité :

1. **Le patch de terrain, ~60 ms par reconstruction.** Sur un thread de pool et au plus
   une fois par seconde, donc invisible — sauf si un jour plusieurs patchs sont
   reconstruits en même temps. Le levier évident est le heightfield (70 % du coût) :
   mémoriser les hauteurs des sommets partagés entre deux reconstructions successives
   éviterait de recalculer la zone qui n'a pas bougé.
2. **L'aérodynamisme au-delà de ~120 pièces.** Quadratique. Le remède est connu et non
   appliqué : ne recalculer l'exposition que quand l'attitude a changé.
3. **Le nombre de triangles du patch.** ~147 000 en un appel, dont la moitié en plantes.
   Le rayon des plantes (260 m) et leur espacement (1,5 m) sont deux constantes
   ajustables si une carte graphique modeste s'en plaint.
4. **La taille d'un delta face au datagramme.** Un monde de 500 entités produit
   2 316 octets là où un datagramme en porte 1 184 : l'état passe alors par le
   canal fiable. Le remède est la gestion d'intérêt, pas un réglage.
5. **La cadence de reconstruction du patch près du sol**, désormais tous les 40 m de
   déplacement pour que le champ d'herbe suive le joueur : 60 ms toutes les dix
   secondes en marchant, plafonné à un par seconde. Si cela devient gênant, la sortie
   est de donner à l'herbe son propre maillage et son propre job lisant la grille de
   hauteurs déjà en cache — recentrer le champ coûterait alors ~3 ms au lieu de 60.

Rien de tout cela n'est un problème aujourd'hui. Le budget d'un tick physique est
consommé à moins de 0,4 % par tout ce qui a été ajouté au cours de ce jalon.
