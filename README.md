# StarWorks

An industrial space simulation game — mine, automate, build ships and stations, and grow a civilization that spans a star system. Powered by a bespoke C++20 / Vulkan engine written exclusively for this game.

## Current state — F1: The factory is data now

The mining outpost was three hard-coded machines. It is now a **SITE**: a hub, a miner, a smelter, a silo and a solar field standing on real ground, each one a `.swpart` building running a `.swrecipe` production chain. Nothing about industry lives in C++ any more — rates, power, losses, footprints, the slope a building tolerates and the ore grade a mine refuses are all files you can edit, and Part Studio already saves them.

Two rules hold the whole thing up. **Matter is conserved** — every recipe is weighed at load and again in the tests, outputs may never exceed inputs, and the difference must be the loss the recipe declares; a balance pass that quietly invents iron fails the build. And **production is warp-exact** — everything is units per second, so eight hours of time warp produce exactly eight hours of goods, to the unit, whether the lane hands the executor one tick or one hour.

**Ore is analytic, like the terrain.** There is no deposit table and nothing about ore in the save file: `oreDensity(direction)` is a pure function of where you are standing, so the scanner shows the very field the miner is paid on and a world reloaded a year later has its ore in the same rocks. A mine's yield is its recipe rate times the density under its feet — siting IS the gameplay.

A sixth building exists for one reason: **a site you cannot find is not a site.** The BC-1 nav beacon produces nothing but visibility — a lit 25 m mast, a marker on the star map, and inside 100 km a reticle in the cockpit with the site name and the live distance under it. When the beacon is behind you or off the edge of the screen the pointer clamps to the border and points the way you would turn, because that is exactly when you need it.

Which caught something the old constants had been hiding for a dozen milestones: **the outpost and the launch pad were in the sea.** Both were nailed to `+Z` on Terra's equator because it was a convenient number, and `+Z` is open ocean — but the heightfield clamps at sea level, so nothing ever complained. The starting site is now SURVEYED from the same two analytic fields the game already trusts — and the search runs along **the equator**, because latitude is a tax a base pays on every launch it ever makes: rotation speed it is not given, and a plane change it has to fly. A full sweep of the ring finds a continent with ore; a fine pass finds ground flat enough to stand the whole factory on one plateau. It lands at latitude 0.000000, 526 m up, over ore grading 0.85, nearly 800 km inland, with the pad 120 m east.

**And the surface stopped vibrating.** Everything anchored to the ground shimmered by up to 0.77 m between frames, and the cause was one word in a struct: the body's rotation was an `f32` quaternion, and a surface anchor rotates a vector 6,371 km long. Seven digits of precision quantise that lever arm to about a metre, and the error moves as the planet turns. The spin is now kept in `f64` and used for every position that far from the axis; the f32 quaternion goes on orienting meshes, where it is worth a nanometre.

Also: the base beacon now carries 1000 km and gets out of the way under 500 m, and on EVA you walk where you look — the mouse turns your body, the camera follows from behind, and the side keys strafe instead of steering.

**And a bug that had been hiding in plain sight since the first landing:** ground contact put a body's ORIGIN on the terrain. That is right for an object of zero size and wrong for everything else — the rocket sank 7.2 m into the rock, and on EVA you walked with your waist at ground level. Objects now carry the box their collision hull fills, and rest on it; the box is projected onto the local vertical, so a rocket standing on its tail rests on its engine bells and one lying on its side rests on its flank. The rocket's box comes from the same collider shapes the VAB validates against, so it shrinks when a stage separates.

## Previously — Milestones 25–31: Planetary realism

Planets became places. Per-pixel mountain relief from sixteen warped ridged/billow octaves with erosion, terraces and a soft ceiling instead of a clamp; credible biomes; volumetric clouds that cast their own shadows on the ground; a physical Rayleigh/Mie limb that actually goes dark behind the planet; a specular ocean. All of it under the founding rule: **what the renderer shows is exactly what the physics collides with** — one analytic terrain function, sampled by the CPU and by the GPU, with a transpiler that proves the two bit-exact on every build. The bodies turn on their axes in closed form (so a day survives time warp and a save), the terrain patch and the collider agree to 0.000 m at landing altitude, and the fragment path got about 40% cheaper along the way.

## Previously — Milestone 24: The cinematic grade

The light calmed down. The harsh ACES curve gave way to a Hable filmic tone curve with an explicit grade — gentler contrast, softly desaturated color, raised cool blacks — all tunable constants at the top of `Mesh.frag`. The day/night terminator is a soft wrap-lit gradient instead of a razor cut, planetary eclipse shadows fade through a real penumbra, and a subtle dither wipes out gradient banding. Space finally looks like the films.

## Previously — Milestone 23: Pixel-sharp planets

Close orbit no longer blurs the world below: when you are within four radii of a planet, its surface switches from vertex colors to PER-FRAGMENT procedural shading — the very same noise functions the physics terrain collides with, ported bit-for-bit to the GPU. Coastlines, turquoise shallows, beaches, lunar maria and martian rust resolve at pixel scale, with a fine albedo grain no vertex mesh could carry, and it costs nothing when you're far away. Also gone: the stray colored circles in deep space (over-eager lens-flare ghosts, and the faint nebula discs which are simply removed).

## Previously — Milestone 22: Planets rendered right

Every UV sphere in the engine was wound INSIDE-OUT since Milestone 2 — from orbit you were seeing the far hemisphere through the near one, mirrored, which is why both polar caps could show at once. The winding is fixed (planets, moons, tank domes, the EVA capsule), and on top of it the globes gained RELIEF SHADING: vertex normals tilted by the exact analytic heightfield physics collides with, so mountain ranges catch the light from orbit exactly where you'll land on them. Top-detail spheres are sharper (150x225), and the atmosphere limb now wraps a correct silhouette.

## Previously — Milestone 21: The visual overhaul

The game finally looks like the sim deserves. Filmic tonemapping and real specular materials (oceans glitter under the sun, hulls catch highlights, regolith stays matte); AERIAL PERSPECTIVE everywhere — the horizon fogs into a sky whose color follows the local sun, from noon blue through amber terminator to black night, sunrises included; the atmosphere is a fresnel LIMB GLOW: a blue rim around the planet from orbit, a glowing horizon band from inside. 3400 stars with real color temperatures and a galactic band fade out in daylight; the sun throws a proper lens flare, occlusion-tested behind planets. Engine plumes and reentry plasma are soft billboard glows now — a launch looks like a launch.

And because making things beautiful means LOOKING at everything: two real, pre-existing simulation bugs got caught and fixed. A craft standing on a planet is no longer flung skyward by time warp — landed vessels become surface anchors (frozen in the planet's rotating frame, woken up co-rotating, attitude preserved, unit-tested). And on slow machines, physics warp no longer desynchronizes the analytic world from the integrated one: the simulation now slows honestly instead of dropping time it never simulated. Save v8.

## Previously — Milestones 19 & 20: Part Studio & the real VAB

Parts are DATA now. Every part is an `.swpart` JSON file: composed primitives (boxes, cylinders, cone frustums, ellipsoids, tubes) each with its own color and emissive glow, a separate collision hull built from the same primitive list, and attach nodes that sit exactly ON those surfaces. **PART STUDIO** (`PartStudio.exe`, built next to the game) is the new authoring tool: orbit around your part, click-select any primitive, move/rotate/scale it Blender-style (`G`/`R`/`S` + `X`/`Y`/`Z`, grid snap), paint it from the palette, flag what collides, place the attach nodes with surface snapping — SAVE, launch the game, fly it. The nine stock parts were rebuilt this way (striped tanks with domes, an engine bell with a glowing throat, a capsule with lit windows).

And the hangar became a true **VAB**: parts follow the MOUSE. Stack nodes snap magnetically under the cursor, radial parts glue to any point of any hull the cursor touches (real ray cast against real colliders), `X` cycles symmetry x2/x3/x4/x6/x8 with live group previews, clicking a placed part grabs its whole subtree to move or discard, and yellow/violet flags track center of mass and thrust while you build. The green/red ghost is honest — it runs the same compound-collider overlap test the game itself trusts.

## Previously — Milestone 18: The hangar

Press `B` and you leave the world entirely: the HANGAR is its own view — a quiet floor grid, your rocket standing nose-up in front of you, right-drag to walk around it, wheel to zoom. The design you edit there is a BLUEPRINT (plain data, not the live ship): click parts in the palette, watch the green/red ghost, PLACE or UNDO freely. NEW starts a fresh design from a command core; BUILD stands it on Terra's launch pad, co-rotating with the planet, ready to fly. LOAD cycles through every part-built vessel in the world and pulls it into the editor — modify your orbiter's design and BUILD rewrites it in place, same orbit, same identity. And since there can now be several ships, `P` in flight switches which one you're piloting.

## Previously — Milestone 17: Build it, stage it, dock it

Every connection is a real JOINT ENTITY with its own strength and break force — the structural skeleton future crashes will tear along. Press `Z` in flight and the decoupler fires: the lower stage drifts away as its own vessel and your fuel gauge drops to what's actually still bolted to you. Bring two docking ports of different vessels within a few meters and they merge into one.

## Previously — Milestone 16: Everything is made of parts now

The part system has landed — the foundation the rest of the game will be built on. Nine part types (fuel tank, engine, wing, battery, solar panel, docking port, decoupler, cargo bay, structural) live in a catalog where every part exposes its mass, cost, volume, resource capacities, connexions/attach points, strength and aerodynamics. Every part instance is its own entity attached to a vessel — which is what will make staging, docking and damage natural later. Your ship IS one now: an 11-part rocket (37.8 t wet) whose mass, thrust, drag and cost are aggregated from its parts every tick. The engine burns real fuel out of the real tanks — watch FUEL fall on the HUD and the rocket get lighter (the rocket equation isn't coded anywhere: it emerges) — and the solar wing recharges the battery.

## Previously — Milestone 15: Real ground under your landing gear

Planets have TERRAIN now: an analytic procedural heightfield (mountains to 9 km on Terra, 16 km on Mars) that physics and rendering sample from the same function — so where you see a mountain, you can land on it, walk on it, and build on it, at its real elevation, even after the planet has rotated it around. Near the ground a detail patch renders the landscape with true slope lighting, its extent scaling with your altitude (the LOD). One mouse CLICK on the new [SAS] [PGD] [RTG] buttons points your ship prograde or retrograde and holds it there — the first pieces of clickable UI. The map view rotates with right-drag. And two old annoyances are dead: particles no longer float beside the ship (they were anchored to the wrong time frame), and the polar "double cap" glitch is gone.

## Previously — Milestone 13: Plan your burns

The map now plans like KSP: press `N` for a **maneuver node**, dial in prograde/normal/radial dv (`I`/`K`, `U`/`O`, `Y`/`H`) and slide its time (`J`/`L`) — the post-burn trajectory draws in white-violet through every SOI it crosses, a violet marker sits on the navball where the burn vector points, and the HUD counts the remaining dv down while you burn. **Physics warp**: up to ×5, everything stays truly simulated — engines fire, drag brakes, reentries happen five times faster. The HUD gained real orbit data (apoapsis, periapsis, period, time to each). The chase camera is yours now: right-drag orbits the craft, wheel zooms, `C` snaps back. Engines shoot a blue-white exhaust jet, reentry plasma became proper streaks stretched along the airflow, and the Sun finally looks like one — a blinding core wrapped in a warm halo, properly eclipsed by planets.

## Previously — Milestone 12: A universe you can SEE

The sky is full of stars — 1700 of them, procedurally placed once and forever fixed, parallax-free: they are your compass. Terra is no longer a blue ball: procedural continents with lowlands, highlands and snow-capped ranges rise out of deep and coastal oceans, polar ice caps the poles, and two translucent shells wrap it all — a blue atmospheric veil and a layer of blobby white clouds that drifts slowly over the ground (a new blended render pass handles translucency, visible from orbit AND from underneath: the sky is blue down there). Luna got basalt maria, Mars got rust and CO2 caps. And the reentry plasma now streams exactly opposite your motion through the air, not your motion through the solar system.

## Previously — Milestone 11: Reentry fire, walkable worlds & the navball

Light now behaves: every fragment is lit from Sol's true position and planets cast real shadows — fly behind Terra and the station goes dark. Dive into an atmosphere too fast and your ship glows red, then turns into a fireball shedding a wake of plasma particles (the whole descent was flight-tested: deorbit burn, warp descent with automatic downshift, reentry fireball, crash at 91 m/s… and the wreck sits perfectly still afterward, co-rotating with the planet at exactly ω×r). Every body's surface is solid and WALKABLE: ground friction and EVA movement work in each planet's rotating frame, on Terra, Luna or Mars. When your trajectory stops being an orbit and you get low (relative to the body's size), the chase camera levels itself on the horizon — KSP surface mode. And a proper **artificial horizon** sits bottom-center: roll/pitch horizon line, prograde and retrograde markers projected on the ball. Under the hood, this milestone also fixed a deep simulation bug: analytic celestial positions were evaluated up to one physics step ahead of integrated bodies (600 m at Terra's orbital speed) — everything now shares the lane's per-tick "present".

## Previously — Milestone 10: The star system & the KSP-style flight plan

The universe is now a real hierarchy: **Sol → Terra (→ Luna) / Mars**, every orbit at true scale (Terra rides 1 AU from the Sun at 30 km/s — and the station, the ship, the asteroid and the ground outpost all ride with it, because rails, anchors, atmosphere and ground friction are measured in their planet's moving frame). Each body's sphere of influence decides what you orbit: near Luna you orbit Luna, not the heavier Sun.

And the map earned its KSP stripes: a **patched-conics flight plan** predicts your trajectory across spheres of influence — elliptic AND hyperbolic arcs, up to 5 patches, each drawn in its own color around its primary with markers where things happen. Burn toward the Moon and the plan shows the Luna encounter before you commit; aim too low and the HUD warns **`IMPACT TERRA T-403 S`** while the map plants a red marker on your crash site. Events (encounter / SOI exit / impact) are found by scanning the conic and refined by bisection to the millisecond. The sun now actually shines from Sol: Terra shows a live day/night terminator.

## Previously — Milestone 9: Save/Load & surface anchoring

`F5` saves, `F9` loads — the whole universe: every entity and component (columns are memcpy'd thanks to the trivially-copyable rule), simulation clocks (on-rails orbits resume to the exact position), factory state mid-production (bitwise deterministic afterwards, unit-tested), warp, camera and EVA state. Saves identify components by stable names with strict version/size checks. And the first SURFACE BASE exists: a mining outpost anchored to Terra's equator in the planet's rotating frame — it rides the rotation and reloads exactly where it was built, the model every future planet/asteroid factory will follow.

## Previously — Milestone 8.5: Flight information & accessibility

A flight HUD (procedural bitmap font, no assets) shows your control mode, speed — `V` toggles orbital vs surface-relative — altitude, throttle and warp factor. The star map now draws every tracked orbit as a dotted Kepler ellipse at real scale. Terra gained an exponential atmosphere (real drag) and a solid surface with friction: descend gently and you land, arrive hard and it's logged as the crash it is. `G` steps out into an EVA capsule that walks on the ground and falls ballistically off it. `Shift`/`Ctrl` ramp the main-engine throttle limiter.

## Previously — Milestone 8: Factory foundations & time warp

The industrial loop is alive: a drill on the asteroid mines iron ore, a station refinery converts it to iron at 90% yield, a depot stores the output — all volume-bounded with real material densities, matter-conserving, running in the Automation/Logistics simulation lanes. Time warp (`,` slower / `.` faster) climbs ×1, ×2, ×5, ×10, ×50, ×100, ×1000, ×10⁴, ×10⁵: during warp everything rides analytic Kepler rails (no integration, exact at any speed), factories keep producing at exactly warped rates thanks to bulk catch-up ticks, and the warp factor is capped by your altitude above the nearest body (auto-downshift included).

## Previously — Milestone 7: Pilotable ship & star map

You now fly. The game boots in the pilot seat of a 50-ton vessel in 400 km orbit: `W/S` main engine (400 kN, F=ma on the dynamic body, on top of real gravity), `A/D` yaw, arrows pitch, `Q/E` roll, `X` kills rotation, chase camera follows the interpolated ship pose; `Tab` switches to the free camera. `M` opens the star map: a top-down system view at REAL scale — Luna sits at its true 384,400 km — with color-coded beacons of constant on-screen size marking Terra, Luna, the station, the asteroid and your ship (mouse wheel zooms from 20,000 km to 1.6M km). The debris belt is gone (it was pure load); the instanced mass-rendering path remains for real content.

Milestone 6 foundations: two-regime physics — distant objects on analytic Kepler rails (never integrated), objects near the player truly simulated under f64 Newtonian gravity at 50 Hz, continuous conversions at the bubble boundary, real GM values.

Milestone 5 foundations: real astronomical scale (f64 world positions, f32 camera-relative rendering), angular-size LOD for celestial bodies, CPU frustum culling + one instanced draw per mesh, RenderStats with CPU timings.

Milestone 3–4 foundations: archetype ECS (SoA storage, generation-checked handles, parallel stage scheduler), multi-rate fixed-step simulation lanes (Physics 50 Hz … World 1 Hz) with pause/time-scale and render interpolation, deferred structural command buffer, CTest suite (20 tests).

Milestone 2 foundations: VMA-backed GPU memory with RAII wrappers and staging uploads, reverse-Z depth buffer, per-frame camera UBO via descriptor sets, procedural primitives (cube/sphere/grid), cgltf-based glTF importer. `--cpu` selects a software Vulkan device (llvmpipe) with zero code differences — the whole engine runs CPU-only today and will use the GPU the moment one is preferred.

Milestone 1 foundations: logging, assertions/error handling, frame clock, GLFW window layer, input snapshots, camera + controller, Vulkan 1.3 (dynamic rendering, synchronization2), swapchain with live resize, build-time GLSL → SPIR-V compilation.

## Requirements

- CMake ≥ 3.24
- A C++20 compiler (Visual Studio 2022 on Windows, GCC 13+/Clang 16+ on Linux)
- The [Vulkan SDK](https://vulkan.lunarg.com/) (headers, loader, glslangValidator, validation layers)
- A GPU + driver supporting Vulkan 1.3

GLFW 3.4 and GLM 1.0.1 are fetched and built automatically by CMake.

## Building (Windows / Visual Studio 2022)

```powershell
cmake --preset windows-msvc
cmake --build --preset windows-debug
build\windows\bin\Debug\StarWorks.exe
```

Or open `build/windows/StarWorks.sln` in Visual Studio and run the `StarWorks` startup project (F5).

## Building (Linux)

```bash
cmake --preset linux-debug
cmake --build --preset linux-debug -j
./build/linux-debug/bin/StarWorks
```

## Running the tests

```powershell
ctest --test-dir build/windows -C Debug --output-on-failure   # Windows
ctest --test-dir build/linux-debug --output-on-failure        # Linux
```

## Controls

**Ship (default mode):** `W`/`S` main engine forward/retro, `A`/`D` yaw, `↑`/`↓` pitch, `Q`/`E` roll, `X` kill rotation, `Shift`/`Ctrl` throttle up/down. `Tab` switches between pilot and free camera. `G` goes EVA (capsule: `W`/`S` walk, `A`/`D` turn — grounded walking on any body, ballistic otherwise). `V` toggles the HUD speed between orbital and surface-relative. The artificial horizon (bottom center) shows attitude vs the local horizon plus prograde/retrograde markers; the chase camera auto-levels on the horizon when you are low and suborbital.

**Free camera:** hold the right mouse button to look around, `WASD` to move, `Q`/`E` down/up, `Shift` to boost, mouse wheel to change speed.

**Star map:** `M` toggles the system view (real scale, beacon markers, patched-conics flight plan with encounter/impact/SOI-exit markers); mouse wheel zooms from LEO out to the whole Sol system. The map centers on your current SOI primary.

**Maneuver nodes (in map view):** `N` create/delete, `J`/`L` node time −/+, `I`/`K` prograde +/−, `U`/`O` normal −/+, `Y`/`H` radial +/− (`Shift` ×10, `Ctrl` ×0.1). Fly the burn by pointing at the violet navball marker and burning until DV reaches zero.

**Chase camera:** hold the right mouse button to orbit around the craft, mouse wheel to zoom, `C` to reset behind it.

**Hangar (VAB):** `B` opens/closes (sim paused; separate view). Right-drag orbits, wheel zooms. Click the palette to take a part IN HAND — it follows the mouse: cyan stack nodes snap it magnetically, or it glues to any hull surface under the cursor (radial parts). `W`/`S`/`A`/`D`/`Q`/`E` rotate the held part in 90° steps, left-click places, `ESC` puts it back, `DEL` discards. Click a placed part to grab its whole subtree. `X` cycles symmetry (x1/2/3/4/6/8, radial placements), `C` toggles the CoM (yellow) / thrust (violet) flags, UNDO removes the last placement (symmetry ring included). NEW starts a fresh design (built on the launch pad), LOAD cycles the world's vessels into the editor, BUILD makes it real. `P` in flight switches the piloted vessel. `Z` fires the decoupler (staging).

**Part Studio (`PartStudio.exe`):** the part authoring tool. Right-drag orbits, left-click selects a primitive or node, `G`/`R`/`S` move/rotate/scale with `X`/`Y`/`Z` axis constraint (`Shift` = fine, grid-snapped), `K` toggles the orange collision overlay, `DEL` removes. Buttons add primitives (box/cylinder/cone/sphere/tube), duplicate, set colors/emissive/segments, flag visible/collider, add stack/radial nodes (X/Y/Z sets a node's direction, SNAP SURF projects it onto the hull). SAVE writes the `.swpart` into the build AND the source `Assets/Parts/` — the game loads it at next launch.

**Time warp:** ×2 and ×5 are PHYSICS warp — everything stays simulated and the engines still work. Above ×5 the world rides analytic rails and engines cut out.

**Time warp:** `.` faster / `,` slower (×1 → ×100,000, altitude-limited; engines only work at ×1). `Space` pauses. `F5` saves, `F9` loads. `Esc` quits.

## Useful flags

`--frames N` exits after N frames (soak testing); `--log-file path.log` mirrors the log to a file; `--cpu` prefers a software (CPU) Vulkan device over hardware GPUs (and implies `--quality low`); `--quality low|medium|high` sets the shading tier — it drives the planet shader's octave budget, terrain self-shadowing, cloud shadows and the number of atmospheric scattering steps.

## Planetary rendering tools

The planet surface is one analytic function shared by the CPU (collision, terrain patch, site placement) and the GPU (`Shaders/Noise.glsl`, `Terrain.glsl`, `PlanetSurface.glsl`, `Clouds.glsl`, `Atmosphere.glsl`). Two scripts keep that honest:

```bash
# Prove the GLSL twins are exact ports of the engine headers (they are
# transpiled to C++ and diffed over 20,000 directions per body).
python3 Tools/glsl_parity/check_parity.py --glm build/linux-release/_deps/glm-src

# Render the planet shader on the CPU, without a GPU: terrain, biomes,
# self-shadowing, clouds, ocean, atmosphere. Writes PNGs of a fixed set of
# viewpoints so a shading change can be reviewed as a diff.
python3 Tools/planet_preview/render_preview.py \
    --glm build/linux-release/_deps/glm-src --out captures/
```

On Windows point `--glm` at `build/windows/_deps/glm-src` and make sure a C++20 compiler is on the PATH.

## Repository layout

See `docs/Architecture.md` for the module map, engineering conventions, and the development log.
