# Plan — Réalisme planétaire (cible : rendu « image de référence »)

> Objectif : porter le rendu des planètes/lunes au niveau de l'image de référence
> (relief montagneux éclairé par pixel, biomes crédibles, nuages volumétriques
> projetant leurs ombres, limbe atmosphérique physique, océan spéculaire) —
> **sans jamais casser l'invariant fondateur : ce que le rendu montre est
> exactement ce que la physique collisionne** (une seule fonction analytique
> de terrain, échantillonnée par le CPU et par le GPU).
>
> Découpage en 6 milestones (M25→M30), chacun compilable, testable et
> livrable seul. Ordre choisi pour que chaque phase s'appuie sur la
> précédente sans jamais être bloquante pour le gameplay.

---

## État actuel (acquis M21–M24) et écart avec la cible

**Acquis :**
- Surface procédurale PAR FRAGMENT en orbite basse (port GLSL bit-exact du
  bruit CPU) : littoraux nets, mais albédo quasi plat (1 octave de détail).
- Relief par NORMALES DE SOMMETS uniquement (150×225) : les chaînes de
  montagnes sont des taches, pas des reliefs.
- Nuages : coquille à alpha PAR SOMMET (blobs flous), pas d'ombres portées.
- Atmosphère : hack fresnel (joli limbe, mais pas de gradient physique, pas
  de rougissement au terminateur, brouillard exponentiel simpliste au sol).
- Océan : couleur fixe + Blinn-Phong (pas de vagues, pas de bathymétrie).
- Grade cinématique Hable : à conserver tel quel (la cible a le même ton).

**Ce que montre l'image cible et qui nous manque, par ordre d'impact visuel :**
1. Relief par PIXEL avec ombrage directionnel (chaînes, canyons, piémonts).
2. Ombres des nuages sur le sol + nuages à bords définis, multi-couches.
3. Biomes : aride/ocre dominant, verts côtiers, roche sur pentes, neige.
4. Limbe atmosphérique en dégradé physique (épais côté jour, rougi au
   terminateur) + perspective aérienne teintée sur le terrain lointain.
5. Océan profond avec glint solaire étendu et micro-vagues.
6. Silhouette du relief au limbe (montagnes qui crochent l'horizon) —
   seule vraie modification GÉOMÉTRIQUE, reléguée en phase bonus.

---

## M25 — Heightfield v2 : le terrain qui mérite d'être éclairé

Le relief actuel (fbm 5 octaves lisse) ne PEUT pas ressembler à l'image
cible, quel que soit l'éclairage : il n'a ni crêtes, ni vallées, ni érosion.
Tout le reste du plan éclaire ce terrain — il faut donc le refaire d'abord.

**Contenu :**
- `Planet/Terrain.hpp` v2 : une fonction composée, toujours 100 % analytique
  et déterministe par graine :
  - *masque continental* : le fbm 5 octaves ACTUEL, inchangé — les
    littoraux, la carte du monde et les anciens sites (pad de lancement)
    sont préservés ;
  - *domain warping* (1 passe, 2 octaves) pour casser l'isotropie du value
    noise (les formes « patates » actuelles) ;
  - *ridged multifractal* (|1−2n| inversé, 4–5 octaves) modulé par le masque
    continental → chaînes de montagnes aux crêtes marquées ;
  - *billow* basse fréquence pour les piémonts/collines ;
  - *courbe d'érosion* : compression des vallées (pow) + terrasses douces ;
  - profondeur océanique RÉELLE (le clamp à 0 devient une vraie bathymétrie
    négative, nécessaire à M27) — la collision, elle, continue de clamper à 0
    (l'eau reste une surface solide au niveau de la mer).
- Port GLSL rigoureusement identique (même arithmétique entière) — revue
  ligne à ligne + capture comparative littoral CPU-patch vs GPU-globe.
- Consommateurs mis à jour gratuitement (même fonction) : collision,
  patch de terrain proche, normales de sommets des globes, placement VAB.
- Paramètres par corps dans `TerrainComponent` (poids ridged/billow/warp)
  → Luna cratérisée (bassins billow inversés), Mars canyons (warp fort).

**Tests/acceptation :** littoraux inchangés à ±1 km (comparaison d'un
échantillon de 10k directions avec la v1 pour le masque) ; tests existants
verts ; le pad reste posé au même endroit ; patch terrain et globe alignés.

**Risques :** coût CPU du patch (129² × ~12 octaves — mesuré, budget < 15 ms
en rebuild ponctuel, inchangé en fréquence) ; toute divergence CPU/GPU se
voit immédiatement au littoral (c'est notre test visuel).

---

## M26 — Éclairage du terrain par fragment (le cœur de la cible)

**Contenu :**
- **Normales par pixel** : gradient du heightfield v2 par différences finies
  (2 échantillons tangents + le centre), fusionné avec la normale de sphère ;
  l'exagération actuelle devient un paramètre par corps.
- **Anti-scintillement (LOD d'octaves)** : le nombre d'octaves échantillonnées
  décroît avec l'empreinte écran du fragment (dérivées `fwidth` sur la
  direction corps) — de 12 octaves au ras du sol à 4 au loin. Indispensable :
  sans ça, l'image fourmille dès qu'on bouge.
- **Auto-ombrage du relief (option qualité HIGH)** : marche courte vers le
  soleil dans le heightfield (6–8 pas, pas géométrique) → les vallées passent
  à l'ombre derrière les crêtes, exactement l'effet « image de référence ».
  En MEDIUM : approximation par le seul gradient (déjà très efficace).
- **Biomes** : albédo = f(altitude, pente, latitude, humidité) :
  - pente forte → roche nue ; altitude haute × latitude → neige à limite
    modulée par le bruit ; *humidité* = fbm dédié basse fréquence → verts
    côtiers vs ocres continentaux (la dominante aride de l'image cible) ;
  - désaturation légère vers les hautes latitudes (accord avec le grade).
- **Réglage qualité GLOBAL** : un uniform `quality` (LOW/MED/HIGH) dans le
  UBO — LOW pour llvmpipe/CI, HIGH par défaut sur GPU réel.

**Acceptation :** capture à 400 km : chaînes de montagnes lisibles avec
flancs jour/ombre ; zéro shimmer en warp ; 60 FPS GPU (budget fragment
~40 échantillons de noise en HIGH, mesuré).

---

## M27 — Océan vivant

**Contenu :**
- Bathymétrie réelle (M25) → dégradé profond/turquoise physique, plateaux
  continentaux visibles comme dans l'image cible (bleu clair au large des
  côtes) ;
- **Écume côtière** : bande où la profondeur → 0, modulée par une octave
  animée (le temps entre déjà côté jeu via l'advection des nuages — ajout
  d'un `timeSeconds` au UBO) ;
- **Micro-vagues** : perturbation de normale 2 octaves animées → le glint
  solaire s'étale en traînée scintillante au lieu d'un point dur ;
- Fresnel de Schlick correct (remplace l'approximation actuelle) ;
- Glaces polaires : bord de banquise irrégulier + spéculaire léger.

**Acceptation :** glint en traînée sur l'océan à contre-jour, écume visible
le long des côtes en orbite basse, aucune régression au sol (le patch
terrain reçoit les mêmes matériaux).

---

## M28 — Nuages v2 et LEURS OMBRES

C'est le changement qui « vend » le réalisme de l'image cible.

**Contenu :**
- **Couverture par fragment** sur la coquille (le mesh actuel garde son rôle
  de support) : fbm sphérique domain-warpé, seuil net (smoothstep serré) →
  bords de cumulus définis ; 2 couches (cumulus bas opaques, cirrus hauts en
  voiles), advection différentielle (vitesses distinctes déjà supportées par
  `CloudLayerComponent`) ;
- **Ombres portées au sol** : dans le chemin planète du shader, calculer le
  point où le rayon fragment→soleil perce la coquille nuage (intersection
  rayon/sphère analytique, triviale) et échantillonner LA MÊME fonction de
  couverture → atténuation douce de la lumière directe. Coût : ~4 octaves de
  plus par fragment de sol, uniquement en chemin proche ;
- **Silver lining** : wrap + boost du spéculaire sur les bords de nuages à
  contre-jour ;
- Épaisseur simulée : assombrissement du cœur des cumulus (2e échantillon
  décalé vers le soleil dans la coquille).

**Acceptation :** taches d'ombre au sol sous chaque masse nuageuse,
cohérentes avec la direction solaire ; bords nets vus d'orbite ; pas de
Moiré aux pôles (réutilisation du fade polaire existant).

---

## M29 — Atmosphère physique (single scattering analytique)

Remplacer le hack fresnel par un modèle physique LÉGER (pas de raymarch
coûteux) : diffusion simple Rayleigh + Mie avec airmass analytique
(approximation de Chapman), évaluée par fragment.

**Contenu :**
- Paramètres par planète (UBO, déjà extensible) : β_Rayleigh RGB, β_Mie,
  hauteurs d'échelle, rayon atmosphère — Terra bleue, Mars ocre, gratuit ;
- **Trois applications d'une même fonction** :
  1. *perspective aérienne* sur le terrain : extinction + in-scatter le long
     du rayon vue (remplace le fog exponentiel M21 — le terrain lointain
     bleuit et se voile comme dans l'image cible) ;
  2. *limbe depuis l'espace* : sur la coquille, épaisseur traversée réelle →
     dégradé blanc-bleu→bleu profond→transparent, rougi quand le rayon passe
     côté terminateur (couchers de soleil vus d'orbite) ;
  3. *ciel depuis le sol* : même fonction, remplace la couleur de ciel
     calculée côté CPU (levers/couchers exacts, zénith sombre en altitude).
- Le CPU garde une version miroir simplifiée pour `skyAmbient` (cohérence
  éclairage/ciel).

**Acceptation :** limbe conforme à l'image cible (fin, dégradé, rougi au
terminateur) ; transition espace→sol continue en descente ; Mars ciel
beurre/ocre au sol sans aucun code spécifique.

---

## M30 — Perf, LOD, qualité, polish final

- Profil GPU réel (chronos par passe si besoin d'un timer Vulkan simple) ;
  budget cible : < 3 ms de fragment planète en 1440p HIGH sur GPU moyen ;
- Fades de LOD : octaves, ombres de nuages et auto-ombrage coupés
  progressivement avec la distance (aucun pop) ;
- Mode LOW verrouillé pour llvmpipe/CI (captures de non-régression) ;
- Passe couleur finale avec le grade M24 (aucun changement de courbe —
  la cible a exactement ce ton) ;
- Documentation Architecture.md + captures avant/après par phase.

---

## Phase BONUS (hors série, plus tard) — Relief en silhouette

Au limbe, nos montagnes restent dans la sphère (le relief est un ombrage).
L'image cible tolère ça (limbe adouci par l'atmosphère), mais si on veut la
silhouette crochue : déplacement GÉOMÉTRIQUE des sommets des 2 LOD proches
par le heightfield (déjà fait pour le patch local — extension au globe),
puis à terme un quadtree sphérique (CDLOD). C'est le seul chantier qui
touche à la géométrie et il est INDÉPENDANT : aucune phase ci-dessus n'en
dépend. Il déplacera aussi la question du z-fighting patch/globe (résolue
aujourd'hui par le patch local) — à traiter à ce moment-là.

## Garde-fous transverses (toutes phases)

- **Un seul point de vérité** : chaque évolution du bruit se fait d'abord
  dans `Terrain.hpp`/`Noise.hpp` (CPU), puis port GLSL identique, jamais
  l'inverse ; le littoral patch-vs-globe est le test visuel de divergence.
- **Zéro impact simulation** : tout est shader/données ; les seuls
  changements CPU sont le heightfield v2 (M25, testé) et le `timeSeconds`
  du UBO (lecture seule).
- Chaque milestone : 60+ tests verts debug/release, captures headless
  (llvmpipe LOW), archive livrée, docs à jour, réglable par constantes.

## Estimation d'effort relatif

| Phase | Poids | Risque principal |
|---|---|---|
| M25 heightfield v2 | ●●● | divergence CPU/GPU, littoraux modifiés |
| M26 éclairage/pixel | ●●●● | coût fragment, shimmer |
| M27 océan | ●● | faible |
| M28 nuages+ombres | ●●● | Moiré, coût du 2e échantillon |
| M29 atmosphère | ●●● | raccords espace/sol, réglage esthétique |
| M30 perf/polish | ●● | — |


---

## État d'avancement — M25 à M30 livrés

| Phase | État | Où c'est |
|---|---|---|
| M25 heightfield v2 | fait | `Planet/Terrain.hpp`, `Shaders/Terrain.glsl`, `Tests/Source/TerrainTests.cpp` |
| M26 éclairage/pixel | fait | `Shaders/PlanetSurface.glsl` (normales, LOD d'octaves, auto-ombrage, biomes) |
| M27 océan | fait | `planetOcean` dans `PlanetSurface.glsl` + Fresnel de Schlick dans `Mesh.frag` |
| M28 nuages + ombres | fait | `Shaders/Clouds.glsl` (une fonction, deux consommateurs) |
| M29 atmosphère | fait | `Shaders/Atmosphere.glsl` (une intégrale, trois usages) |
| M30 perf / LOD / polish | fait | fondus de LOD dans `PlanetSurface.glsl`, gain de diffusion recalé, docs |
| BONUS relief en silhouette | non fait | reste indépendant, aucune phase ci-dessus n'en dépend |

**Garde-fous tenus.** Le masque continental v1 est intact : un test compare 10 000 directions à la formule v1 et exige ZÉRO écart de classification terre/mer. La parité CPU/GPU n'est plus une revue à l'œil mais une vérification mécanique (`Tools/glsl_parity/check_parity.py` transpile le GLSL en C++ et diffe contre les en-têtes : écart nul sur les trois corps). Zéro impact simulation : les seuls changements CPU sont le heightfield v2, l'horloge monde et le corps atmosphérique du frame, tous en lecture seule côté simulation.

**Écart connu avec la cible.** Le relief reste un ombrage : au limbe, les montagnes ne crochent pas encore l'horizon (phase BONUS). Le patch de terrain au sol garde ses normales géométriques (maillage réel) et ne passe pas par le chemin par pixel.

**Réglages exposés.** `kReliefExaggeration` (PlanetSurface.glsl), les poids `ridgeWeight`/`billowWeight`/`erosion`/`terraceStrength` par corps (Terrain.hpp + son jumeau GLSL), les seuils de biome (pente, ligne de neige, humidité), `kCumulusDrift`/`kCirrusDrift` et les seuils de couverture (Clouds.glsl), les coefficients par planète et `intensity` (Atmosphere.glsl).
