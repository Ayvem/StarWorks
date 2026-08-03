# StarWorks — propositions pour la suite

Écrit après F32, à partir de l'état réel du dépôt : 62 000 lignes, 261 tests verts
dans les deux configurations, `PARITY OK`, les deux vérificateurs d'échelle à zéro.
Chaque point ci-dessous nomme ce qui est mesuré, ce qui est supposé, et ce que ça
coûterait. Rien ici n'est une idée en l'air : tout part d'un fichier, d'un chiffre
ou d'une chose qui a cassé pendant les dernières milestones.

L'ordre est celui du **risque décroissant**, pas celui de l'envie. Les trois
premiers points sont des dettes que les milestones récentes ont créées et que
personne n'a encore heurtées ; le quatrième est le trou de gameplay que ces mêmes
milestones ont ouvert ; le reste est du confort, dans l'ordre du rapport
valeur/travail.

---

## 1. Le multijoueur ignore le repère flottant et l'horloge scindée

**C'est le point le plus risqué du dépôt aujourd'hui, et il est silencieux.**

F28 a donné à chaque système stellaire son propre repère : `m_originSystem` décide
de quoi les positions sont relatives, et `rebaseOrigin` translate le monde entier
quand le joueur passe le milieu entre deux étoiles. F31 a scindé l'horloge. La
couche de réplication ne connaît **ni l'un ni l'autre** — `grep originSystem` dans
`GameNetwork.cpp` et dans `Engine/Source/Network/` ne rend rien.

Trois conséquences, par ordre de gravité :

- Deux joueurs dans des systèmes différents s'échangent des positions exprimées
  dans des repères différents. Personne ne le détecte : les nombres sont
  plausibles, ils désignent juste un autre endroit de la galaxie.
- Un client qui traverse une frontière en cours de session translate son monde de
  quatre années-lumière pendant que l'hôte continue d'envoyer des deltas dans
  l'ancien repère.
- L'encodeur garde un historique de seize snapshots pour ses deltas et **cet
  historique n'est pas invalidé au changement de repère** (noté à l'époque, jamais
  corrigé). Un delta calculé contre une base d'avant le décalage produit un monde
  faux qui reste faux, ce qui est précisément le mode de panne que le design du
  réseau interdisait explicitement.

Le correctif est petit et bien délimité : l'identifiant du système d'origine dans
la poignée de main et dans l'en-tête de chaque snapshot, refus d'un snapshot dont
le repère ne correspond pas, purge de l'historique de base au décalage. Le harnais
de test existe déjà — le fil simulé avec générateur graine et horloge injectée —
donc le test « l'origine change au milieu du flux et le miroir reste juste » est
écrivable sans ouvrir une socket.

**Coût estimé : une journée.** Il faut le faire avant d'ajouter quoi que ce soit
au multijoueur, parce que tout ce qu'on ajouterait reposerait dessus.

## 2. La sauvegarde reperd la précision que F31 vient de gagner

`Snapshot.cpp:274` écrit `simulation.simulatedSeconds()` — un seul f64. Au retour,
`setSimulatedSeconds` refait le découpage, donc la partie entière est récupérée
exacte et **la vibration ne revient pas**. Ce qui se perd est la fraction sous
61 µs : après une traversée interstellaire, un aller-retour sauvegarde/chargement
déplace le vaisseau d'un mètre et demi.

Ce n'est pas visible en jeu, mais ça casse une propriété que le dépôt revendique
et teste : le déterminisme aller-retour. Écrire les deux champs et monter la
version du format. **Une heure.**

## 3. Les colliders des pièces animées restent à la pose de repos

Reconnu au moment où les animations ont été livrées et toujours vrai : `hullFor`
construit les hitboxes depuis la pose de repos, donc on traverse un panneau
solaire déployé à pied. Le clic fonctionne uniquement parce qu'il teste une sphère
englobante qui, elle, couvre les deux poses.

Deux options, et la deuxième est la bonne : soit interpoler les hitboxes comme les
formes, soit décider qu'une pièce animée déclare des hitboxes **par pose** dans le
`.swpart`. La deuxième garde le collider authored plutôt que dérivé, ce qui est la
règle du reste du format. **Une demi-journée**, plus l'édition des deux pièces qui
en ont besoin.

---

## 4. Vingt-quatre systèmes que personne ne peut atteindre en jouant

C'est la question de design, et je pense que c'est la prochaine milestone.

Le catalogue contient 24 systèmes, 36 étoiles et 25 exoplanètes confirmées, toutes
atterrissables, chargées paresseusement à l'entrée. Le rendu est correct, les
couleurs sont justes, le repère flottant tient, l'horloge tient. Et **il n'existe
aucun moyen d'y aller en jouant**.

La preuve est dans mes propres outils : pour tester cette moitié du jeu j'ai dû
écrire `SW_JUMP`, `SW_ESCAPE` et `SW_CLOCK`. Une traversée coûte 3,3e11 secondes de
temps simulé, soit dix mille ans, soit — au cran de warp le plus haut — une demi-
minute de temps réel passée à ne rien faire du tout, sans événement, sans décision,
sans rien à regarder. Ce n'est pas un voyage, c'est un écran de chargement de trente
secondes déguisé.

Trois directions possibles, à trancher :

**(a) Un vrai vaisseau interstellaire.** L'Endurance pousse 88 kN pour 500 tonnes.
Un moteur de croisière — fusion, voile, peu importe la fiction — qui monte à
quelques pour cent de c ramène la traversée à des années plutôt qu'à des
millénaires, et rend le warp x1B utile plutôt qu'obligatoire. Ça demande de
décider ce que « ravitailler pour quatre années-lumière » veut dire, ce qui est
exactement le genre de contrainte dont un jeu industriel a besoin : la traversée
devient un objectif de production.

**(b) Une structure de mission.** Le voyage reste long, mais il se passe des
choses : consommables, pannes, hypersommeil (la baie cryo existe déjà comme
pièce), fenêtres de tir. C'est le plus de travail et le plus de jeu.

**(c) Assumer que c'est un bac à sable.** Un « saut » diégétique déverrouillé par
la construction d'une infrastructure, et le voyage n'est pas simulé du tout.
Honnête, beaucoup moins cher, et ça vaut mieux qu'un warp de trente secondes qui
prétend être un voyage.

Mon avis : **(a) d'abord**, parce que la moitié des pièces nécessaires existe et
que ça transforme une fonctionnalité inerte en objectif, et **(b) ensuite** si le
jeu le mérite. À ne pas laisser en l'état : c'est aujourd'hui la plus grosse
quantité de travail livré et non jouable du projet.

---

## 5. Ce que le joueur demandera ensuite, par rapport valeur/travail

**Le delta-v par étage.** C'est le nombre le plus consulté de tout KSP et tout
existe déjà : `VesselAssemblySystem` connaît la masse sèche, les ergols et
l'impulsion spécifique de chaque pièce, et les découpleurs définissent déjà les
étages. C'est une somme et un logarithme dans le HUD. **Une demi-journée pour la
fonctionnalité la plus rentable de la liste.**

**Le train d'atterrissage animé.** Le système d'animation a un mois et exactement
deux utilisateurs. Un train qui sort et rentre est la troisième utilisation
évidente, elle valide le format sur un cas à charnière réelle, et elle a besoin du
point 3 (les colliders par pose) — ce qui est une bonne raison de faire le point 3.

**Les ports d'amarrage qui s'ouvrent.** Même argument, et l'amarrage existe déjà
comme mécanique.

**L'éditeur d'animation de Part Studio, terminé.** Les tâches #119 à #123 sont
encore ouvertes ; l'essentiel a été livré avec F29 mais la liste n'a jamais été
reprise. Une passe de vérification vaut mieux que de laisser cinq tâches mentir.

**La chaîne de production (F4).** La moitié usine du jeu est garée depuis F5 :
interface d'exploitation, fabrication de pièces, transport réel sur les
convoyeurs. C'est le cœur annoncé du jeu — « industrial space simulation » — et
c'est la partie qui a le moins bougé depuis quinze milestones. À rouvrir dès que
le point 4 est tranché, parce que les deux se nourrissent : un vaisseau
interstellaire est une cible de production.

---

## 6. Performance : deux seuils mesurés, aucun atteint

Rien ne presse, et c'est documenté plutôt que supposé.

L'aérodynamisme est **quadratique** en nombre de pièces : 5,5 µs à 7 pièces,
67 µs à 31, extrapolé à 4,3 ms à 250 — 22 % d'un tick. La sortie est connue et
écrite dans `Performance.md` : ne recalculer l'exposition que quand la direction du
flux a bougé d'un degré ou deux. À faire le jour où un vaisseau dépasse la centaine
de pièces, pas avant.

Les constructions de sphères LOD sont **sérialisées sur un cœur** et représentent
l'essentiel de la barre de chargement. Le pool de threads existe. C'est le gain le
plus visible par unité de travail côté moteur.

Et #113 est toujours ouverte : un matériau par fragment pour les anneaux, plus
l'ombre de la planète en travers de l'anneau et la diffusion vers l'avant qui
éclaire un anneau à contre-jour.

---

## 7. Une règle de méthode, tirée des échecs de cette session

Trois bugs d'affilée se sont cachés derrière la même chose : **un hook de debug qui
fabrique son propre entrant**.

`SW_PARTMENU` appelait `togglePartAnimation` directement. Quand le routage s'est
révélé cassé, je l'ai changé pour fabriquer un `HudButton`. Les deux versions
enjambaient l'étape réellement cassée — d'abord le routage, puis le fait que la
ligne avait déjà été effacée de la table. Trois captures d'affilée ont « prouvé »
une fonctionnalité inutilisable. Même schéma avec `SW_CLOCK` : posé dans la boucle
de frame, il mesurait un bond balistique et pas une perte de précision.

La règle, à appliquer à tout hook futur : **un hook presse la vraie chose et hurle
quand elle n'est pas là.** `SW_PARTMENU` cherche maintenant la ligne dans la table
que lit le gestionnaire de clic et écrit `BUTTON 900 IS NOT IN THE TABLE` quand
elle manque — parce qu'« absent » et « inerte » se ressemblent de l'extérieur et
n'ont rien en commun.

Corollaire pratique : les lignes Windows du README sont **dérivées des fichiers de
build, pas exécutées**. Une seule session Windows suffirait à les vérifier, et le
point le plus susceptible de coincer est le compilateur des scripts Python — il
leur faut `g++` ou `clang++`, pas `cl.exe`.

---

## Ce que je ferais dans cet ordre

1. Le repère et l'horloge sur le fil réseau (point 1) — une journée, et ça
   débloque tout le reste du multijoueur.
2. La sauvegarde exacte et les colliders par pose (points 2 et 3) — une journée à
   deux, et le second débloque le train d'atterrissage.
3. Le delta-v par étage (point 5) — une demi-journée, le meilleur rapport du lot.
4. Trancher la question interstellaire (point 4). C'est une décision de design
   avant d'être du code, et elle décide de la forme des six mois suivants.
5. Rouvrir l'usine (F4), en la branchant sur ce qui aura été décidé au point 4.
