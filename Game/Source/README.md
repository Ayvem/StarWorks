# Game/Source — how the game layer is laid out

One class, `game::StarWorksGame`, implemented across several translation
units — **one theme per file**. The class did not change; only where its
methods live did. If you are looking for a method, the file names are the
index:

| File | Owns |
|---|---|
| `StarWorksGame.hpp` | The class: every member, every method declaration. |
| `StarWorksGame.cpp` | Constructor/destructor, the frame loop (`onUpdate`, `onRender`), and the small shared queries (`controlledEntity`, `bodyRenderPose`, `keyPressed`, mesh registration). |
| `GameInternal.hpp` | What used to be the big anonymous namespace: physical constants (mu, radii, SOI), the HUD palette (`hud::`), warp ladder, procedural mesh builders, small pure helpers. Internal to the Game target — the engine never includes it. Functions are `inline`, constants `inline constexpr`, so any TU may include it without ODR trouble. |
| `GameShell.cpp` | The frame around the game (F13): the boot plan and its bar, the main menu, `newGame`/`continueGame`, shell clicks. |
| `GameScene.cpp` | `buildScene`: the star system, the starting outpost, LOD sets. |
| `GameTerrain.cpp` | The walkable terrain patch and the grass field baked onto it. |
| `GameSaveLoad.cpp` | The save schema, quicksave/quickload, and the named saves in `Saves/`. **A component added to the world must be registered here** — `Snapshot` throws on save otherwise (by design), and `bootPrepareSaves` reports any omission on frame one. |
| `GameFlight.cpp` | Flying: ship controls, warp and its gate, maneuver nodes, prediction refresh, reentry effects, EVA toggle, chase camera. |
| `GameHangar.cpp` | The design office: blueprint editing, ghosts, symmetry, subtree grab, `.swship` save. |
| `GameFactory.cpp` | Ground construction and the factory: placing buildings, hulls, cables, belts, launch pads, vehicle orders. |
| `GameFactoryUi.cpp` | What the factory shows: build ghosts and overlays, the `F` catalogue, the `E` panel. |
| `GameNetwork.cpp` | Multiplayer session: host/join, the `F3` panel, per-player clocks and sync. |
| `GameHud.cpp` | The HUD: panels, text, navball, buttons, and `handleHudClicks`. |
| `GameMap.cpp` | The map view: trajectory lines and the per-frame draw item list. |
| `Components.hpp` | The game-side ECS components (all trivially copyable, as everywhere). |
| `Systems.cpp/.hpp` | The game-side simulation systems (thrust, SAS…) that run on the lanes. |

Two rules keep this healthy:

1. **A new method goes in the file whose theme it belongs to**, not in the
   file that happens to be open. If no theme fits, that is the signal a new
   `Game<Theme>.cpp` is due — add it to `Game/CMakeLists.txt` and this table.
2. **A helper needed by two files moves to `GameInternal.hpp`** (inline),
   not copy-pasted. A helper needed by one file stays in that file's own
   anonymous namespace (see the hangar's quaternion helpers, or the
   factory's `resourceMined`).
