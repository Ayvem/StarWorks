# StarWorks — Engine Architecture

This document is the living reference for the engine's structure and the reasoning behind it. It is updated at every milestone.

## Vision

A single-purpose engine for an industrial space simulation: hundreds of thousands of rendered objects, a multi-rate simulation that never depends on rendering, seamless space-to-planet transitions, and save files with no memory-pointer dependencies. The engine is not a general-purpose product; every design decision is allowed to exploit that.

## Ground rules

Modularity with minimal dependencies; no God objects; strong cohesion inside a module, weak coupling between modules. Game code talks to the engine only through `Engine.hpp` and the `Application` hooks — never to GLFW or Vulkan directly. GLM is quarantined behind `Math/Math.hpp` so a future in-house math library (notably f64 large-world coordinates) is a one-module change.

## Module map

| Module | State | Responsibility |
|---|---|---|
| `Core` | active | types, log, assert, errors, clock, filesystem, Application/main loop |
| `Platform` | active | GLFW window, surface creation, raw event callbacks |
| `Input` | active | keyboard/mouse snapshots (down / just-pressed / just-released) |
| `Math` | active | math façade over GLM; f64 world coordinates (WorldVec3); frustum culling; engine-wide conventions |
| `Scene` | active | camera, free camera controller (scene graph later) |
| `Renderer` | active | Vulkan 1.3 front-end: instance, device, swapchain, pipelines, frame loop |
| `RenderGraph` | planned | pass scheduling, automatic barriers, transient resources |
| `Assets` | active | MeshData/Vertex, procedural primitives, glTF import (cgltf); streaming later |
| `ECS` | active | archetype ECS: generation-checked entity handles, contiguous SoA columns, access-declared systems, parallel stage scheduler |
| `Physics` | active | Kepler conics (elliptic + hyperbolic, primary-relative), f64 Newtonian gravity integration, simulation bubble (rails↔dynamic, SOI-based primaries), AERODYNAMICS (tabulated per-part force/moment, atmosphere + wind + Mach, occlusion, damping); rigid bodies/docking later |
| `Simulation` | active | fixed-rate lanes (Physics 50 Hz, Logistics 10, Automation 5, Economy 2, World 1), pause/time-scale, catch-up bounds, interpolation alpha |
| `Resources` | active | resource catalogue with real mass/volume per unit (ores, metals, water, gases) |
| `Factory` | active | volume-bounded inventories, DATA-DRIVEN recipes (`.swrecipe`) + one generic production executor, buildings/sites/power books, matter conservation |
| `Space` | active | hierarchical star system (CelestialBody tree), CelestialIndex (analytic world states at any time, SOI queries), CelestialMotionSystem, patched-conics trajectory prediction |
| `Planet` | active | analytic planetary heightfield (mask + orogeny belts + warped ridged/billow relief + erosion + bathymetry), shared bit-exact with the GPU; analytic ORE DEPOSITS + the survey that sites a mine on them |
| `UI` | active | procedural 5x7 bitmap font + renderer screen-space path (flight HUD); retained UI later |
| `Gameplay` / `Audio` | planned | game rules, sound |
| `Network` | active | authoritative-host multiplayer, PER-PLAYER CLOCKS and a stamped Timeline (an action carries the instant it happened at; a player who has not reached it holds it until they do): UDP transport (+ a simulated wire for tests), two delivery services over one socket (unreliable-sequenced state, reliable-ordered everything else), baseline/delta replication of ECS component columns into an INDEX-EXACT mirror world |
| (game layer) | active | pilotable ship (thrust/RCS via ThrustSystem), chase camera, star-map view with constant-screen-size beacons. One class across themed translation units — `Game/Source/README.md` is the index |
| `Serialization` | active | bounds-checked binary writer/reader (throws on corruption) |
| `Save` | active | schema-named world snapshots, entity index+generation preserved, simulation clocks |
| `Tools` / `Editor` | active | Part Studio (`.swpart` authoring), **AeroForge** (offline wind tunnel: geometry → `.aero.json`), **NetProbe** (headless host+client over real sockets), GLSL parity checker, planet preview renderer |

Dependency direction (only downward):
`Game → {Scene, Renderer, Input, Core} → {Platform, Math, Core}`. `Platform` knows nothing about `Renderer`; `Input` knows nothing about GLFW; `Scene` knows nothing about Vulkan.

## Key decisions & rationale

**Vulkan 1.3 + dynamic rendering + synchronization2, no legacy render passes.** The future RenderGraph will compute image layouts and barriers itself; VkRenderPass objects would fight it. Dynamic rendering removes framebuffer/renderpass lifetime management entirely. Minimum driver requirement is explicit and enforced at device selection.

**Frame model.** 2 frames in flight; per-frame transient command pool (whole-pool reset each frame — cheapest reuse pattern); image-available semaphore + in-flight fence per frame; render-finished semaphores indexed *per swapchain image* (required for correct present semantics when image count > frames in flight).

**Error policy.** Initialization and asset failures throw `sw::Exception` (caught once in `main`). Per-frame code never throws for recoverable states — `VK_ERROR_OUT_OF_DATE_KHR`/`VK_SUBOPTIMAL_KHR` are handled as values. Programmer errors use `SW_ASSERT`.

**Logging.** Static facility with level check before message formatting, category tags, ANSI colors (enabled on Windows via VT processing), optional file sink. Validation-layer output is routed through the same log.

**Coordinates & conventions.** Right-handed, +Y up, -Z forward; radians everywhere; depth [0,1] (`GLM_FORCE_DEPTH_ZERO_TO_ONE`); projection Y-flip done once in `Camera`; geometry authored CCW with `VK_FRONT_FACE_COUNTER_CLOCKWISE` and back-face culling (verified by rendering test). sRGB swapchain: shaders write linear color.

**GPU memory via VMA behind an engine-owned façade.** `VulkanMemory` is the single entry point (device buffers, host-visible mapped buffers, attachment images, staging uploads); RAII wrappers (`VulkanBuffer`/`VulkanImage`) make leaks structurally impossible. VMA/cgltf implementations live in dedicated warning-silenced TUs; engine code keeps the full warning set.

**Reverse-Z depth.** Projection maps near→1, far→0 (`glm::perspective` with swapped planes), depth clear is 0.0, comparisons are `GREATER_OR_EQUAL`. This is the only depth scheme that keeps f32 precision at planetary/orbital distances; adopted before any content existed so nothing ever depends on standard-Z.

**CPU/GPU device choice is a config bit, not an architecture.** `--cpu` (RendererConfig::preferCpuDevice) inverts the device-type scoring so a software implementation (llvmpipe/SwiftShader) wins; every rendering feature runs identically on both. CI runs the engine on CPU; players run the same binary on hardware.

**ECS design.** Archetype-based: entities with identical component sets share one archetype whose components live in contiguous per-type byte columns (SoA) — queries are linear walks of packed arrays. Entity handles are index+generation (stale handles detected, no pointers, serializable as-is). Components must be trivially copyable/destructible and ≤16-byte aligned (compile-time enforced): rows migrate with `memcpy`, columns never run constructors, and component memory is directly serializable — non-trivial data belongs in assets or ID-referenced side tables. Signatures are 64-bit masks (cheap compare/hash; limit is one static_assert away from widening). Row removal is swap-remove O(1).

**System parallelism by declaration, not by luck.** Every system declares read/write component masks; the scheduler builds order-preserving stages where no two systems conflict (write-write or read-write), then runs each stage's systems concurrently on the Core ThreadPool. Determinism holds regardless of thread timing because concurrent systems have disjoint access by construction. Structural changes (create/destroy/add/remove) are owner-thread only — deferred command buffers arrive with the Simulation milestone.

**Real astronomical scale via f64 world + f32 camera-relative rendering.** The game world uses REAL dimensions (Terra 6,371 km radius, Luna at 384,400 km). f32 breaks at that scale (~0.5 m ULP at Earth radius), so all world POSITIONS are `WorldVec3` (f64): ECS transforms, orbits, the camera. The GPU only ever sees f32 *camera-relative* values: positions are subtracted from the camera position in f64, then narrowed — precision is highest exactly where the player looks. The view matrix carries rotation only. Combined with reverse-Z, near=0.5 m and far=1e9 m coexist in one frame.

**Distance-based LOD for celestial bodies.** A body's LOD level derives from its ANGULAR diameter relative to the FOV (5 sphere levels, 64×96 down to 6×9). A moon at true distance costs a few dozen triangles; the planet you orbit gets full silhouette geometry; nothing ever renders surface detail from far away. The Planet module's quadtree terrain will later replace level 0 near the ground, behind the same component.

**Mass rendering: cull, then instance.** Frame flow: game submits `DrawItem`s (mesh + camera-relative transform + bounding sphere) → CPU frustum culling (6 planes extracted from the actual VP matrix, so reverse-Z/Y-flip are automatic; unit-tested at planet-scale radii) → survivors sorted by mesh → transforms written to a per-frame storage buffer → ONE `vkCmdDrawIndexed` per distinct mesh, vertex shader indexes the SSBO via `gl_InstanceIndex`. 12k+ submitted items cost 0.1–0.2 ms of prepare (release build). `RenderStats` exposes submitted/culled/instances/draw-calls plus CPU timings (fence wait, prepare, total) — measured, not guessed.

**Two-regime physics: simulate only what is near the player.** Every distant object is ON RAILS: it stores nothing but Kepler orbital elements (with a precomputed perifocal basis), and its position is a closed-form function of time — no integration, no drift, evaluated at 10 Hz by the RailsSystem (and skippable entirely when invisible). Only objects inside the simulation bubble around the camera (10 km enter / 15 km exit hysteresis) carry a DynamicBody and get true f64 Newtonian gravity, integrated with symplectic (semi-implicit) Euler at 50 Hz. The BubbleSystem converts between regimes with position/velocity continuity in both directions (state vectors from elements on entry, elements from state vectors on exit; unbound trajectories stay dynamic), via the deferred command buffer. Per-tick cost therefore scales with the bubble population (~hundreds), not the world population (tens of thousands) — the architecture that lets a whole star system exist at once.

**Time warp = rails + bulk catch-up + altitude limits.** The warp ladder (×1 … ×100,000) is a Simulation time scale plus two guarantees. First, warping forces EVERYTHING onto rails (BubbleSystem force-rails): analytic orbits are exact at any speed while numerical integration is not, so integration simply doesn't run above ×1 (thrust is disabled — you burn at ×1, you coast at warp). Second, rate-based lanes (Logistics/Automation/Economy/World) use *bulk catch-up*: backlog beyond their tick bound is consumed in one big-dt tick — exact for `amount = rate × dt` systems, so factories produce precisely warp-scaled output; the Physics lane instead keeps strict fixed steps and drops backlog. Warp is capped by the controlled ship's altitude above the nearest body (e.g. ×100 at 400 km LEO) with automatic downshift on approach — warping hard near a planet is how you tunnel through it.

**Factories move real matter.** Inventories are bounded by VOLUME with real densities (1 m³ holds 2,500 units of ore but 7,870 of iron); refineries reserve output space before consuming input, so a full tank stalls the machine and matter is conserved by construction (unit-tested end-to-end across the real simulation lanes). Machine graphs use entity links (`ItemLinkComponent`) — the pull-based abstraction that real conveyor entities will later implement.

**Saves: stable names, exact identities, strict failures.** Component columns are written under STABLE string names registered in a `save::Schema` (never runtime type ids — those depend on registration order). Loading restores entity indices AND generations exactly, so entity references inside components (factory links, surface anchors) survive untouched — the day-one "no pointers in components" rule exists for this moment. Trivially-copyable columns memcpy in and out; the free list is rebuilt canonically so entity creation stays deterministic after a load; unknown names or size/version mismatches abort with precise errors (migrations later, silent reinterpretation never). Simulation clocks (master time, per-lane tick counts and accumulators) are part of the snapshot: on-rails orbits — pure functions of time — resume to the exact millimeter. Round-trip factory determinism is asserted bitwise in tests.

**Surface bases live in their body's rotating frame.** Factories built on planets or asteroids carry a `SurfaceAnchorComponent` (parent body entity + body-fixed local position); the `SurfaceAnchorSystem` recomputes their world pose from the body's current rotation each tick. Saving stores the ANCHOR, not a world position — the planet may rotate arbitrarily far, the base reloads exactly where it was built. This is the foundation every surface factory will stand on.

**Simulation never driven by rendering.** The frame loop feeds wall-clock time into `Simulation::advance`; each lane accumulates it and runs zero or more ticks with its *exact* fixed step (systems always see 1/Hz seconds — time scale changes tick count, never tick size). Tick counts are provably independent of frame slicing (unit-tested). Catch-up per frame is bounded per lane; excess backlog is dropped with a throttled warning instead of spiraling. Pause simply stops feeding time — the renderer keeps running. Rendering reads simulation state through interpolation: `PreviousTransform` is snapshotted at the start of each Physics tick and the render list lerps/slerps with `lane.alpha()`.

**Deferred structural changes.** Systems must not create/destroy entities or add/remove components while stages run in parallel; they record into a thread-safe `EntityCommandBuffer` and the owner thread plays it back at a defined point. Commands on entities that died before playback are skipped (standard deferred-destruction semantics).

**Unit tests from Milestone 3 onward.** `StarWorksTests` (in-house harness, CTest-integrated, no window/GPU needed) covers entity lifetime/generations, archetype migration, swap-remove integrity, query semantics, a 10k-entity stress test, ThreadPool completeness and scheduler staging/parallel correctness. Every future engine module ships with tests.

**Explicit source lists in CMake** (no GLOB): adding a file is a deliberate, reviewable act.

**`--frames N` headless soak mode** from day one, so every milestone can be smoke-tested in CI without a human.

## Development log

### Milestone 1 — Bootstrap (done)
Window + main loop + logging + error handling + clock + input + free camera + Vulkan 1.3 init (instance/validation, device selection & scoring, swapchain + live resize/minimize handling, graphics pipeline with push constants) + first triangle via build-time-compiled GLSL.

### Milestone 2 — GPU memory & geometry (done)
VMA-backed memory layer (`VulkanMemory` + RAII buffer/image wrappers, staging uploads), depth buffer with reverse-Z, vertex input pipeline (position/normal/color/uv), per-frame camera UBO through descriptor sets, model matrix via push constants, Lambert+ambient shading, procedural primitives (cube, UV sphere, grid), glTF import via cgltf (node transforms applied), `Tools/generate_asteroid.py` producing the deterministic test asset, `--cpu` device preference. Verified: 240-frame soak on llvmpipe, screenshot-checked scene (occlusion, lighting, glTF geometry).

### Milestone 3 — ECS core (done)
Archetype ECS (`ECS/`): Entity index+generation, Component registration with signature masks, Archetype SoA columns with swap-remove and memcpy migration, World API (create/destroy/add/remove/get/forEach/count), System + SystemAccess declarations, SystemScheduler with conflict-free parallel stages, Core ThreadPool. Game scene fully migrated to ECS: 150 entities (asteroid, station ring, deterministic debris field) driven by OrbitSystem/SpinSystem through the scheduler; render list collected from a `<Transform, Mesh>` query. Verified: 8 unit tests green (CTest), 240-frame soak at 37 FPS on llvmpipe, screenshot check.

### Milestone 4 — Multi-rate simulation (done)
`Simulation` module: configurable fixed-step lanes with per-lane SystemScheduler, bounded catch-up with backlog dropping, pause and time scale (0–16×), interpolation alpha. `EntityCommandBuffer` for deferred, thread-safe structural changes. Game fully migrated: Snapshot/Orbit/Spin on the Physics lane (50 Hz), DebrisFlow on Logistics (10 Hz), Stats on World (1 Hz); render transforms interpolated (lerp/slerp) between Physics ticks; Space = pause, 1/2/3 = time scale. Verified: 17 unit tests green (exact tick counts on power-of-two lanes, frame-slicing independence, catch-up bound, pause/scale semantics, deferred playback, parallel recording), 300-frame soak at 38 FPS on llvmpipe.

### Milestone 5 — Real scale & mass rendering (done)
f64 world coordinates + camera-relative rendering; angular-size LOD for celestial bodies; CPU frustum culling + per-mesh instancing through a per-frame SSBO; RenderStats with CPU timings. Scene: Terra (6,371 km, real sidereal spin) + Luna (1,737 km at 384,400 km) + 12,000-object debris belt in 400 km LEO at true orbital angular velocity. Verified: 20 unit tests green (incl. frustum at planet-scale radii), release soak at 27 FPS on a 2-core llvmpipe container (culling 12k items in 0.15 ms; cost dominated by software rasterization of the full-screen planet). Debugging note: a "progressive slowdown" turned out to be a Clock FPS-smoothing artifact (EMA seeded by the meaningless first-frame delta) — fixed and the lesson kept: never trust a smoothed metric without a raw counterpart.

### Milestone 6 — Two-regime Newtonian physics (done)
Physics module: `KeplerOrbit` (fromElements / fromStateVectors / evaluate, Y-up convention, Newton-solved Kepler equation, precomputed perifocal basis), `GravityIntegrationSystem` (symplectic Euler, multiple point-mass sources with real GM values), `RailsSystem` (10 Hz analytic refresh), `SimulationBubbleSystem` (hysteresis, continuous rails↔dynamic conversion through the command buffer). TransformComponents promoted to the engine. Game: debris belt on real orbital elements (a/e/i/Ω/ω), asteroid as a true dynamic body held in orbit by gravity alone, per-tick work restricted to bubble contents (Snapshot/Spin require DynamicBody). Verified: 25 unit tests green (circular geometry & period, element↔state round-trip, unbound rejection, 100 s integrator energy/radius drift < 0.05%, bubble conversion continuity both ways); release soak shows live regime migration (284 dynamic → 0 as the belt leaves the bubble) at stable FPS.

### Milestone 7 — Pilotable ship & star map (done)
Debris belt removed (performance request — the mass-rendering path stays, ready for real content). Player ship: compound mesh assembled from box primitives (`PrimitiveFactory::makeBox`/`append`), `ThrustSystem` in the Physics lane applying F=ma main thrust and RCS attitude control (body-frame angular velocity, kill-rotation) on top of gravity; chase camera follows the interpolated ship pose; Tab toggles pilot/free camera (with pose hand-off); bubble focus follows the ship. Star map (M): top-down system camera at REAL scale (Luna at its true 384,400 km), wheel zoom (20,000 km → 1.6M km height), and octahedron beacons on marked entities whose scale grows with distance for constant on-screen size, lifted off surfaces so planets don't occlude their own marker. Renderer gained per-instance color tint (InstanceData mat4+vec4). App now boots in the pilot seat: a static camera would watch the whole scene recede at 7.7 km/s of orbital velocity. Verified: 28 unit tests green, release soak with live regime migration, screenshot checks of both views.

### Milestone 8 — Factory foundations & time warp (done)
Resources module (real mass/volume catalogue), Factory module (volume-bounded inventories, MinerSystem/RefinerySystem in the Automation lane, TransferSystem links in Logistics), first production chain live: asteroid drill → station refinery (90% yield) → storage depot. Time warp ladder ×1–×100,000 (`,`/`.`): rails-only above ×1 (bubble force-rails), bulk catch-up for rate-based lanes (warp-exact production), altitude-capped with auto-downshift. Also fixed a real Input defect the warp tests exposed: sub-frame key taps (press+release within one frame) were lost — press events are now latched. Verified: 32 unit tests green (inventory bounds, refinery conservation/stall, full chain through real lanes, bulk catch-up exactness, force-rails conversions), release soak climbing the ladder to the ×100 altitude cap with warp-scaled factory output in the logs.

### Milestone 8.5 — Flight information & accessibility (done)
HUD through a new renderer screen-space path (`DrawItem::screenSpace`: NDC transforms, unlit, alpha-blended, no depth, drawn after the world in the same dynamic-rendering pass) fed by a dependency-free procedural 5x7 bitmap font (`UI/HudFont`, one mesh per glyph, one instanced item per character). Readouts: control mode, speed with `V` toggling ORBital vs SuRFace-relative (planet-rotation aware), altitude, throttle, warp. Star map draws sampled Kepler ellipses as dotted trajectories for every marked object (dynamic bodies via fromStateVectors), default zoom framing LEO. `SurfaceInteractionSystem` (engine): exponential-density quadratic drag inside atmospheres, solid body surfaces with position clamping, impact classification (crash > 8 m/s vertical), tangential friction to rest, `isGrounded` flag (surface co-rotation deliberately deferred). EVA (`G`): capsule (new `makeCapsule` primitive) spawned co-moving beside the ship, kept upright along the local vertical, walks on the ground when grounded. Ship throttle limiter ramped by Shift/Ctrl, applied by ThrustSystem. Input hardening: sub-frame key taps latched (found by injected-input testing). Verified: 35 unit tests green (drag/landing/rest, capsule and glyph geometry), screenshot checks of HUD, map trajectories and EVA.

### Milestone 9 — Save/Load & surface anchoring (done)
Serialization module (bounds-checked binary writer/reader), Save module (schema-named world snapshots with exact entity identity restoration, simulation clock persistence), `SurfaceAnchorComponent`/`SurfaceAnchorSystem` (body-frame attachment for surface factories — co-rotation with the planet, save-proof by construction), ground mining outpost on Terra's equator as the first surface base. In-game: `F5` save / `F9` load (world + simulation + player state incl. warp, camera, EVA). ECS gained a documented serialization-support API (column introspection, exact-identity restore, canonical free list). Verified: 39 unit tests green — including BITWISE factory determinism after a mid-production save/load and anchor persistence across snapshots — plus a live in-game save → warp ×10 → load cycle returning to the exact saved second.

### Milestone 10 — Hierarchical star system & patched-conics flight plan (done)
The universe became a tree: Sol → Terra/Mars → Luna, all real values (a, e, i, GM, radii, SOI radii = a·(mu/mu_parent)^(2/5)). Kepler v2: orbits are PRIMARY-RELATIVE (no stored center) and support HYPERBOLIC conics (e>1: unwrapped mean anomaly, `M = e·sinh H − H` Newton solve, asinh start; near-parabolic band |e−1|<1e-6 rejected) — a lunar flyby IS hyperbolic relative to the moon. New Space module: `CelestialBodyComponent` (parent + parent-relative orbit + fixed-size name, trivially copyable ⇒ saves for free), `CelestialIndex` (topologically sorted snapshot; recursive analytic world state at ANY time — past or future; SOI primary = deepest containing sphere of influence), `CelestialMotionSystem` (Physics lane, first: moves celestials with REAL interpolation and stamps `GravitySource::worldVelocity`). Everything became frame-aware: atmosphere/ground friction and capsule walking measure against the body's velocity (a ship landed on a planet racing at 30 km/s stays landed), rails compose primary position + relative conic (station follows Terra around Sol for free — and moved to the Physics lane, since a 10 Hz refresh would step visibly at heliocentric speeds), the bubble picks SOI primaries (near Luna you orbit Luna, not the heavier Sun) and preserves the dynamic payload across regime flips (a 50 t ship no longer comes back from warp weighing the 1000 kg default — found by writing the round-trip test). THE feature: `space::predictTrajectory` — KSP-style patched conics. Fit conic around current primary → coarse scan (32 s steps) for the FIRST of impact (r < body radius) / SOI exit (r > SOI) / encounter (distance to a child evaluated on ITS orbit < child SOI) → bisection-refine to ~1 ms → transform state into the new primary's frame → repeat (≤5 patches). The map draws each patch as colored dots around its primary's CURRENT position with event markers; the HUD calls out "ENC LUNA T-…" / "IMPACT TERRA T-…" / "EXIT TO SOL T-…". Emissive rendering convention (tint alpha > 1.5 = unlit) for the star, beacons and trajectory dots; the sun direction is computed from Sol's position every frame (day/night terminator on Terra). Save format v2 (OnRails v2, GravitySource v2, space.CelestialBody v1). Verified: 45 unit tests green — including a Hohmann-transfer Luna encounter whose refined event time puts the ship exactly ON Luna's SOI sphere, impact/SOI-exit refinement, hierarchy recursion, rails-follow-primary — plus a live blind-piloted deorbit burn showing "IMPACT TERRA T-403 S" on the HUD and the red impact marker on the map, and a save→warp→load cycle at warp ×10.

### Milestone 11 — Reentry, surfaces, instruments & the lane-time fix (done)
**Realistic lighting.** The camera UBO now carries Sol's camera-relative POSITION plus up to 8 occluder spheres; the fragment shader computes the light direction per fragment (Terra and Mars genuinely lit from different directions) and tests the fragment→sun ray against every occluder for analytic eclipses — no sunlight behind a planet. The intersection uses the fragment-to-center vector (small numbers) instead of sun-distance math, which f32 could not survive at 1e11 m. Emissive tint convention (alpha > 1.5) covers the star, beacons, trajectory dots and reentry plasma.
**Reentry heating.** Heating proxy q = rho·v_rel³ against the co-rotating atmosphere; the craft's tint ramps to red and flips emissive when severe, while a capped CPU particle pool sheds glowing plasma along the wake (white-hot → deep red, expanding as it cools). Purely visual, render-frame rate.
**Walkable, co-rotating ground everywhere.** `GravitySourceComponent` gained `angularVelocity` (schema v3): surface & atmosphere velocity at a point = worldVelocity + ω×r. Ground friction and EVA walking act in that ROTATING frame — a landed ship sits at exactly ω×r world speed (465 m/s on Terra's equator, unit-tested) instead of having the planet slide underneath it, and the capsule walks on any body's surface (SOI-selected).
**THE BIG ONE — the lane-time fix.** Blind-flight testing (deorbit → land → EVA) exposed a landed capsule being flung kilometers into the air. Root cause: `Simulation::simulatedSeconds()` advances by the whole frame BEFORE lanes tick, so analytic celestial evaluation ran up to one full step (600 m of Terra's motion!) AHEAD of the integrated bodies — the ground itself jittered under everything landed, pumping energy through the surface clamp. Fix: `SimulationLane::presentSeconds()` = master seconds − accumulator, the exact time of the CURRENT tick (the accumulator is now consumed before the scheduler runs); CelestialMotion, Rails, the bubble, and every game-side query (HUD, prediction, map, navball, chase camera) evaluate at the lane's present, never at the master clock. Dropped backlog moves the lane's whole world forward consistently. Lesson recorded: fixed-step worlds must never read the accumulating master clock mid-frame.
**Ground-locked chase camera.** When the predicted trajectory is no longer an orbit (impact predicted / grounded) AND altitude < 3% of the primary's radius, the camera blends its up-vector to the local vertical and levels its orbit frame on the horizon — KSP surface mode. Smoothly blended both ways.
**Artificial horizon.** Bottom-center navball: outer ring, roll-rotated & pitch-offset horizon line (chord-clipped to the instrument, ±30° pitch ticks), fixed craft reference, and prograde/retrograde markers — the HUD-frame velocity direction projected into the craft's body frame, front hemisphere only. Verified live: walking EVA shows the prograde diamond dead on the leveled horizon at 4.0 m/s SRF.
Verified: 46 unit tests green in Debug and Release; a full blind-piloted cycle — deorbit burn, warp descent with auto-downshift, plasma reentry, 91 m/s crash-landing, stable co-rotating rest (ALT −0.0 km, SPD ORB 464.6 = ω×r), EVA walking at 4.0 m/s — captured frame by frame.

### Milestone 12 — Visual update: stars, living planets, atmosphere & clouds (done)
**Transparent world pass (renderer).** Third pipeline sharing the Mesh shaders: alpha-blended, depth-tested but not depth-written, no face culling (shells are seen from inside — the sky IS the far side of the shell). `DrawItem::transparent`; items sorted back-to-front, drawn between opaques and HUD.
**Static starfield.** One mesh of 1700 emissive octahedra on a unit sphere (FIXED seed — the sky never changes), drawn CAMERA-CENTERED at 1e12 m: zero parallax, an infinitely-distant orientation reference, one draw call. Sizes/colors vary by procedural magnitude/temperature.
**Living planet surfaces.** Deterministic 3D value-noise fBm colors the celestial LOD spheres per vertex: Terra gets deep/shallow oceans, lowland→highland→peak land bands and noisy polar ice; Luna gets basalt maria over cratered highlands; Mars rust bands and CO2 caps. LOD0 raised to 96×144 for readable coastlines. No geometry displacement (relief is meters-scale vs a 6371 km sphere; collision stays the analytic sphere).
**Atmosphere + clouds.** Two transparent shells around Terra: a uniform blue veil at ×1.012 (blue limb from orbit, blue sky dome from the ground) and a cloud sphere at ×1.005 whose per-vertex alpha comes from thresholded fBm (blobby systems with clear gaps), spinning slightly FASTER than Terra so clouds drift over the ground. `CloudLayerComponent`/`CloudLayerSystem` (game) keep shells centered on their body with lockstep previous-transforms.
**Reentry wake fix.** The plasma trail direction now uses the AIR-relative velocity (world velocity is dominated by the planet's 30 km/s orbital motion — the old wake pointed along Terra's orbit, not the craft's motion through the atmosphere).
Save format: game.Mesh v2 (transparent flag), game.CloudLayer v1. Verified: 46 tests green both configs; flight-tested — orbital view with continents/clouds/terminator, star navigation view, night-side reentry with a correctly-trailing plasma wake.

### Milestone 13 — Maneuver nodes, physics warp & flight polish (done)
**Maneuver nodes (KSP-style).** In map view: `N` creates/deletes a node on the flight plan; `J`/`L` slide its time, `I`/`K` prograde, `U`/`O` normal, `Y`/`H` radial (Shift ×10, Ctrl ×0.1 — no key conflicts: node editing exists only in map view where flight controls idle). The pre-burn prediction is capped at the node; the dv is applied in the orbital frame there (prograde = v̂, normal = r̂×v̂, radial = prograde×normal) via the new engine helper `space::stateOnPrediction` (world state at any time ON a prediction — segment lookup + conic evaluation + primary composition); a SECOND patched-conics prediction from the post-burn state draws in white-violet with its own event markers. Node marker on the map, violet burn-direction marker on the navball (direction of the live remaining-dv vector — point at it and burn), HUD shows planned dv then switches to the live remaining dv inside the 30 s burn window. Node persists in saves (v4). Unit-tested: stateOnPrediction equals direct composition; a prograde dv raises apoapsis and keeps the burn point as periapsis.
**Physics warp.** Time scales up to ×5 keep the world FULLY simulated (Physics catch-up raised to 16 ticks/frame): drag, thrust — engines now work while warping ≤×5 — and ground contact stay live; the atmosphere altitude cap allows ×5 (fast reentries, the point of the feature). Above ×5, rails warp as before.
**Orbit readouts.** APO/PER altitudes with T-apo/T-peri and orbital period (hours formatting), from the live conic around the SOI primary; hyperbolic shows periapsis + ESC.
**Free chase camera.** Right-drag orbits the craft (sticky yaw/pitch, KSP-style), wheel zooms 0.35×–12×, `C` resets — composed on top of the ground-lock blend.
**Particles v2.** Streaks: every particle now elongates along its motion axis (oriented non-uniform scale, ×5–11 for plasma by heat). Engine exhaust jet: blue-white streaks out of the correct nozzle end (rear for prograde thrust, front for retro), rate scaled by throttle.
**Better sun.** The emissive encoding grew opacity: alpha ∈ (1,2] = self-lit with opacity α−1 (2.0 stays opaque — fully backward compatible), giving free radial falloffs by vertex-alpha interpolation. Two billboarded glow discs (wide warm halo + tight white core) draw at Sol in the transparent pass, correctly occluded by planets.
Verified: 47 tests green in both configs; in-game — orbit data live, exhaust jet visible, node created/edited on the map with post-burn plan and markers, engines burning at warp ×2 (apoapsis climbing in fast-time), camera orbited around the craft by mouse.

### Milestone 15 — Procedural terrain, SAS & clickable UI (done)
**Planet module.** Terrain is an ANALYTIC HEIGHTFIELD: `planet::terrainElevation(TerrainComponent, bodyFrameDir)` — a pure function (engine `Math/Noise.hpp` fBm, moved from the game so ONE noise feeds terrain, globe colors and clouds; Terra's coastlines match its collision exactly). Elevation is meters above the sea-level sphere; oceans clamp to 0; land fraction squared → coastal plains + steep peaks. Terra 9 km / Luna 8 km / Mars 16 km amplitudes.
**Terrain collision & landing.** SurfaceInteraction samples the heightfield at the contact direction in the body's ROTATING frame: ground radius = sea level + elevation. Landing, EVA walking, and anchoring all work on mountains through the existing clamp; atmosphere density stays sea-level-referenced. Flight-tested: the ship now rests at ALT 0.4 km — on a hill. Unit test drops a probe onto a located mountain and checks rest radius = R + elevation(rest dir) within 1 m.
**Terrain rendering + LOD.** A 64×64 tangent-plane patch projected onto the sphere with real heights, finite-difference normals (true slope shading) and an elevation palette, centered under the craft; its EXTENT scales with altitude (4 km–400 km — that scaling is the LOD), rebuilt at most 1/s when the craft moves 30% of the patch or the scale doubles (renderer idled for the swap — bounded hitch). Visible below 300 km; the colored globe (whose inter-vertex sagitta sits ~800 m below sea level at LOD0) serves as ocean/far view underneath.
**Bug fixes.** Particles spawned at the raw physics-tick position while the craft renders at the INTERPOLATED pose — the whole cloud shared one offset (up to ~600 m of planetary travel); they now spawn from the interpolated pose. Pole glitch: cloud coverage fades above 90° of latitude (the UV-fan pinch read as a star artifact), ice caps tightened to >66° with a soft edge, mountain peaks recolored rocky gray-brown so they never read as polar ice.
**SAS + first clickable UI.** `SasSystem` (before ThrustSystem): proportional shortest-arc controller commanding body-frame angular velocity toward ±prograde (SOI-relative), pausing under pilot input, settling when aligned. Clickable HUD buttons [SAS][PGD][RTG] bottom-left — NDC hit-testing of the mouse against per-frame button rects — plus `T` cycling. Flight-tested: one CLICK on RTG flipped the ship retrograde and held it through the whole deorbit burn. Saved (game.Sas v1, save v5).
**Map rotation.** Right-drag orbits the map camera (yaw + tilt clamped 7°–88°) around the focus primary.
Verified: 48 tests green in both configs; full mouse-clicked SAS deorbit → ×5 physics-warp reentry (atmosphere cap now allows it) → terrain touchdown at real elevation.

### Milestone 16 — The part system (done)
**The foundation for all future construction — three layers in the new Gameplay module.** (1) STATIC DATA: `PartDefinition`/`PartCatalog` — 9 part types (FuelTank, Engine, Wing, Battery, SolarPanel, DockingPort, Decoupler, CargoBay, Structural), each exposing mass, cost, volume, resource capacities, strength (crash tolerance + breaking force), aerodynamics (CdA + lift coefficient, ready for the aero pass) and attach points (stack/radial joints, positions + normals) — addressed by STABLE ids, code-defined today, data files later behind the same lookup. (2) INSTANCES: one ECS ENTITY PER PART (`PartComponent`: definition id + vessel handle + vessel-local pose + integrity) — the property that makes staging (reparent to a new root), docking (point at a merged root) and damage (destroy one entity) cheap later. Resources carried by parts are ordinary `factory::InventoryComponent`s ON the part — tanks are cargo, and ElectricCharge's per-unit volume is tuned so a battery's inventory volume IS its capacity. (3) SYSTEMS: `VesselAssemblySystem` (Physics lane, early) re-aggregates every tick — dry mass + carried resource mass → DynamicBody.mass (the ROCKET EQUATION simply emerges as fuel burns), engine thrust + mass flow (F/(Isp·g0)), ΣCdA → ballistic factor, solar rate, total cost; `PartAttachmentSystem` (late) rides parts on their vessel with lockstep previous-transforms.
**The player ship is now a rocket of 11 catalog parts** (all 9 types: dock ring, command core, cargo bay, decoupler, 2×16t fuel tanks, V-400 engine, 2 fins, battery, solar wing — ~5.8 t dry, 37.8 t wet, ~6.3 km/s dv). ThrustSystem burns real Fuel from the vessel's tanks (dry tanks = dead engine, partial tank = partial thrust); SolarChargeSystem trickles ElectricCharge into batteries; the HUD shows FUEL/ELEC (fuel readout turns red under 3 t); the whole stack glows on reentry. Save v6 (parts.Part v1, parts.Vessel v1; new resources Fuel + ElectricCharge).
Verified: 51 tests green both configs (catalog integrity — all types, unique ids, full property sheets; aggregation incl. cargo mass and the lighten-on-burn check; rigid attachment with lockstep previous) and in flight: 20 s full burn consumed 2355 kg = exactly F/(Isp·g0), solar recharging live.

### Milestone 17 — Vessel construction: editor, joints, staging, docking (done)
**Joint entities.** A joint is its OWN ENTITY (`JointComponent`: partA/partB + attach point indices + type Stack/Radial/Docking + strengthN + breakForceN) — not an implicit parent pointer. This is the structural truth of a vessel: an impact beyond breakForceN will one day destroy the joint entity and the ship falls apart along real lines; decoupling and undocking are the same operation triggered politely. The starter rocket is now wired with 10 real joints.
**Vessel operations (engine, `sw::parts`).** `connectParts` (joint creation), `splitVessel` (clones the root frame so detached parts keep their local poses, inherits velocity + separation shove), `decoupleAt` (severs the decoupler's tail-side joint, BFS connectivity from the nose-most part, splits the disconnected component into a new vessel), `dockVessels` (re-localizes every part of vessel B into A's frame, destroys B's root, joins the ports with a Docking joint — parent rerouting, exactly). All structural, called between ticks.
**The editor (B).** Pauses the simulation. Clickable part palette (all 9 types) + PLACE/UNDO/DONE buttons; arrows cycle the target attach node and part type; Enter places. The ATTACH GRID is the attach points themselves: open nodes (not referenced by any joint) shown as cyan diamonds, the target in white; a GHOST of the selected part renders translucent green when valid, red when not. Validation: attach-point compatibility (the child needs a node facing back along the parent node), coarse sphere collision against every part, and JOINT LOAD (child wet mass x 12 m/s^2 must not exceed the weaker breaking force). Live mass/cost/part-count readout while building.
**Staging (Z).** Fires the first decoupler: the lower stage separates as an independent vessel (map marker, bubble-managed, railifies when far) — and the HUD fuel gauge collapses to what's left on YOUR side, because the tanks left with the stage. **Docking**: every half-second, docking ports of different vessels within 4 m merge (player's vessel absorbs).
Bug found by flight-testing: the staging handler re-added PreviousTransform to the freshly split root (splitVessel already had) — assert fired; the ECS's already-has-component check did its job.
Verified: 53 tests green both configs (decoupling partitions the joint graph and keeps the decoupler root-side; docking merges, re-localizes into A's frame at the right local z, destroys B's root and creates the port joint) + in-game: editor open/place/close, stage separation drifting away with the fuel. Save v7 (parts.Joint v1).

### Milestone 18 — The hangar: separate build view, loading, pilot selection (done)
**Construction moved into its own VIEW.** `B` no longer edits the live ship in-place: it opens the HANGAR — a fully separate scene (own camera with right-drag orbit + wheel zoom, neutral floor grid, fixed light, no eclipse shadowing) rendered by its own path (`collectHangarItems` → `renderFrame`, the world's draw path never runs). The simulation pauses underneath and resumes on exit.
**The design is a BLUEPRINT, not live entities.** `BlueprintPart{definitionId, localPosition, parentIndex, parentPoint, childPoint}` — plain data edited freely with zero ECS churn; UNDO is a pop_back. The rocket displays vertically (nose up) via a display-only rotation; ghost preview + validation (node compatibility, sphere collision, joint load) run against the blueprint. UI: clickable 9-part palette with the selected part's property sheet (mass/cost/volume, thrust/Isp for engines), PLACE / UNDO / NEW / LOAD / BUILD buttons, live wet-mass/cost/part-count, and a mode banner — NEW BUILD → LAUNCH PAD vs MODIFYING LOADED VESSEL.
**Loading rockets to modify them.** LOAD cycles every part-built vessel in the world into the blueprint — parts map to indices, `JointComponent`s are decompiled back into parent links. Opening the hangar auto-loads the vessel you were flying. BUILD on a loaded vessel tears out its old part/joint entities and rebuilds it in place (same root: the craft keeps its orbit, save identity and map marker).
**NEW builds spawn on the launch pad.** `instantiateBlueprint` places a fresh vessel standing on Terra's pad: terrain elevation sampled at the pad direction, co-rotating surface velocity, nose to the sky — flight-tested: a core/tank/engine rocket built from scratch sat stable on the pad (ALT 0.0 km, ORB speed = ω·r = 464.6 m/s) and stayed put.
**Pilot selection.** `P` in flight cycles which ShipComponent vessel you control (camera, HUD, prediction and controls all follow) — verified switching between the pad rocket (16 t fuel, 0 elec) and the 400 km orbiter (32 t fuel).
Link lesson: replacing the old inline-editor block wholesale also deleted `collectEditorUi`'s definition while its declaration and call sites survived — caught by the linker, rebuilt as the hangar UI collector that now owns the whole per-frame button set.
Verified: 53 tests green both configs + in-game: open (auto-load, 11 parts), NEW, palette + PLACE assembly, BUILD to pad, P-cycling.

### Milestone 19 — Data-driven parts & PART STUDIO (done)
**Parts became FILES.** `.swpart` (JSON — parsed by the new dependency-free `sw::json` module, ~450 lines we own: ordered objects for clean diffs, scalar arrays inline) now carries EVERYTHING about a part: physical properties, resource capacities, and — new — its GEOMETRY. A part is COMPOSED PRIMITIVES (`PartShape`: box / cylinder / cone-frustum / ellipsoid / tube, each with pose, per-shape color, emissive level, tessellation) with two flags: `visible` (rendered) and `collider` (part of the collision hull). `AttachNode`s are named, typed (STACK vs RADIAL) and sit ON the collider surfaces. `parts::loadCatalog(dir)` replaces the registry at boot (built-in box-catalog fallback keeps tests/game alive without assets); the 9 stock parts were re-authored as real shapes — striped tanks with ellipsoid domes, engine bell with a glowing throat, capsule cone with emissive windows, ribbed solar wing.
**One geometry, three consumers (`Gameplay/PartGeometry`).** `buildPartMesh` triangulates the visible shapes (per-vertex colors, correct normals — including slanted frustum sides and ellipsoid gradients); `raycastPart` intersects the collider shapes analytically (slab OBB, quadric cylinder side+caps, squashed-space ellipsoid); `partsOverlap` does compound OBB SAT with a contact margin. What you see, what the mouse picks and what placement validates are THE SAME DATA — this is the fix for the old "attach points inside the part / collisions wrong" complaint, and the test suite enforces it: every shipped node must raycast onto its own collider surface within 15 cm.
**PART STUDIO (`Tools/PartStudio`, third engine executable).** The authoring tool: orbit camera, click-select via real ray casts, Blender-style modals (G/R/S with X/Y/Z axis constraint, Shift fine snap; 5 cm grid, 5-degree angles), five add-shape buttons, duplicate/delete, 12-swatch palette + RGB nudges, emissive and tessellation controls, per-shape visible/collider flags with a translucent orange hull overlay (K), attach-node editing (add stack/radial, X/Y/Z sets the direction, SNAP SURF ray-projects the node onto the hull), NEW/PREV/NEXT/SAVE. Saves write the build mirror AND the source `Assets/Parts/` so a rebuild never clobbers work. The game loads the same files on the next launch — no export pipeline.

### Milestone 20 — The VAB: KSP-style mouse construction (done)
**The blueprint got a full pose model** (`localRotation` quaternion, `symmetryGroup`, `parentPoint == 255` = SURFACE attachment) and the editor became a real VAB:
**Part in hand.** Click the palette — the part follows the CURSOR (ray built by unprojecting two reverse-Z depths through the inverse view-projection, then un-rotating into the blueprint frame). W/S/A/D/Q/E rotate it in 90-degree steps. ESC puts a grabbed assembly back exactly (full blueprint backup), DEL discards.
**Stack magnet.** Open STACK nodes (cyan diamonds) attract the ghost when the cursor ray passes within the node's snap radius; the child node that OPPOSES the target under the current held rotation is chosen automatically, so flipped/sideways stacking works.
**Radial surface attach.** No node under the cursor? The cursor ray is cast against every placed part's COLLIDER hull; the ghost's radial node is glued to the exact hit point, oriented by the exact surface normal (shortest-arc alignment) — parts stick anywhere on a tank wall, like KSP.
**Symmetry x1/2/3/4/6/8** around the stack axis for radial placements: all clones preview live, are validated together, placed as a group (shared symmetryGroup — UNDO removes the whole ring).
**Subtree grab.** Clicking a placed part lifts it AND everything attached below it into the hand (poses re-rooted, parent links remapped, survivors' indices fixed up); the whole assembly re-places through the same magnet/surface/validation pipeline. Root parts refuse to be grabbed.
**Validation is the real thing now:** every candidate (held part + subtree + every symmetry clone) runs compound-OBB overlap against every placed part, plus the joint-load check. Green ghost = it will actually fit.
**CoM / thrust flags** (C): wet-mass centroid (yellow) and thrust centroid (violet) as side flags with pointer lines — outside the hull, where depth testing cannot hide them.
Instantiation writes rotations into `PartComponent.localRotation` (already in save schema v7 — no version bump) and surface attachments become Radial joints. Flight-tested end-to-end on the 400 km orbiter: LOAD -> add a nose tank via stack magnet -> 4 fins in x4 symmetry -> grab the tail stage (tank+engine+fins moved as one) -> re-snap it -> BUILD: same entity, same orbit, FUEL 32000->48000 kg live.
ESC now quits the game only OUTSIDE the hangar (it belongs to the editor there).

### Milestone 21 — The visual overhaul (done)
**Lighting & materials (Mesh.frag v2).** Blinn-Phong specular + fresnel sheen driven by a per-vertex MATERIAL channel (the unused uv: x = specular strength, y = gloss): oceans get sun glitter (globe AND terrain patch vertices tagged), part primitives carry authored `specular`/`gloss` fields in the .swpart format (optional keys, sane defaults; emissive shapes stay pure light sources). ACES filmic tonemap with a post-grade chroma restore, in linear space ahead of the sRGB swapchain.
**Aerial perspective.** The frame UBO gained `fogColorDensity` + `skyAmbient`, fed per frame by the game from the camera's position inside an atmosphere: exponential distance fog toward the horizon color, and sky-scattered ambient lifting shadows in daylight. The color tracks the LOCAL SUN ELEVATION — blue at noon, amber through the terminator, near-black at night — so sunsets and dawns come out of the same three lines of shader. The distant globe fogs into the sky color from the ground: the horizon finally looks like air, not vacuum.
**Atmosphere shells** route through a dedicated material (instance tint alpha > 2.5): fresnel LIMB GLOW — the blue rim of the planet from orbit, a glowing horizon band from inside — day-lit with a soft wrap by the true sun direction (twilight ring included).
**Sky & sun.** Starfield v2: 3400 stars, power-law brightness, blue-white..orange-red temperatures, a fixed GALACTIC BAND with faint nebulosity fans — and the whole dome FADES OUT IN DAYLIGHT (the emissive tint rides the game's day factor). Screen-space LENS FLARE: five colored ghosts along the sun-center axis, occlusion-tested against every planet, edge-faded.
**Particles** became soft round BILLBOARDS (radial-falloff glow discs) stretched along the motion projected into the view plane — the launch plume reads as a column of fire and smoke, reentry plasma as glowing gas, no more hard-edged boxes.
**Two REAL simulation bugs surfaced during validation (both pre-existing, both fixed):**
1. *Landed craft flung by rails warp.* Railifying a craft standing on a planet converts its ground state into a degenerate ellipse that launches it. Landed bodies (altitude < 25 km, ground speed < 5 m/s) now become SURFACE ANCHORS instead — `SurfaceAnchorComponent` gained a body-frame rotation + preserved mass/ballistics + an autoRelease flag — frozen in the body-fixed frame, released back to a co-rotating dynamic body when the bubble focus returns. Unit-tested end to end (anchor under forceRails, zero drift across warp ticks, exact co-rotation and attitude on release). Save v8 (phys.SurfaceAnchor v2).
2. *Fixed-step backlog drops desynchronized the world.* When a frame couldn't afford the Physics lane's catch-up ticks (physics warp x5 on a slow machine), the dropped backlog made `presentSeconds()` jump while integrated bodies stood still — analytic Terra teleported INTO the landed rocket every frame and battered it into the sky. Lanes can now be STRICT: `Simulation::advance` slows the MASTER clock to what a strict lane can actually consume (the sim runs below the requested time scale on weak hardware instead of desyncing). The game keeps Physics strict during physics warp and relaxes it for rails warp, where drops move the whole analytic world coherently and x10000 needs them.
Verified: 60 tests green both configs + in-flight: orbit day/terminator shots, sunrise with the limb glow, pre-dawn pad, a full liftoff with the new plume, x5 warp ON the pad (stays put), rails warp with the pad rocket anchored (`LANDED -> surface anchor` / release on return).

### Milestone 22 — Planets & moons rendered right (done)
**The inside-out sphere, finally caught.** `PrimitiveFactory::makeUvSphere` wound its triangles with the right-hand normal pointing INWARD — opposite to the engine's actual front-face reference (the cube, correct since M2). With back-face culling, every planet rendered its INTERIOR: from orbit you were looking through the near hemisphere at the far one, mirrored — which is why BOTH polar caps could be seen at once (the M15 "pole glitch" was this bug wearing makeup; the color-only fix is now redundant but harmless). The winding is flipped to outward; the map view now shows exactly one cap from above a pole, continents un-mirrored, and the atmosphere limb wraps the correct silhouette. The same inversion existed in `PartGeometry`'s ellipsoid (tank domes) and in `makeCapsule` (inherited) — both heal with the flip. The M21 nebulosity fans are wound toward the dome center so they face the camera.
**Relief shading from the REAL heightfield.** The two closest LOD spheres tilt their vertex normals by the gradient of `planet::terrainElevation` — the exact analytic function physics collides with — sampled per vertex with a ~25 km arc and an exaggeration factor (real slopes are 9 km over thousands; silhouette and collision stay untouched). Mountain ranges catch morning light and throw shadowed flanks from orbit, on Terra, Luna and Mars, precisely where the terrain patch will put them when you land. Oceans clamp to elevation 0, so their gradient vanishes and they keep the flat mirror specular from M21.
**Sharper globes.** Top LOD raised 96x144 -> 150x225 rings/segments (second level 80x120): the horizon curve and coastlines stop showing polygon corners at 400 km.
Verified: 60 tests green both configs; map view from above the pole (one cap, limb ring), orbit view, hangar domes shading correctly.

### Milestone 23 — Per-fragment planet surfaces (done)
**The blur, killed at the root.** In close orbit the globe fills the screen and per-vertex colors (even at 150x225) interpolate into mush. The fix moves the surface to the FRAGMENT shader: `Math/Noise.hpp` (hash, value noise, fbm) is ported to GLSL **bit-for-bit** — same lattice hashes, same integer arithmetic — so the fragment-shaded coastlines sit exactly where the CPU terrain collides. The game routes a styled globe into the procedural path only when the camera is closer than 4 radii (instance tint alpha = 3.6 + style/10; `CelestialLodComponent` now carries its `surfaceStyle`, v2): far planets keep the cheap vertex-color meshes, near ones get pixel-sharp shores, turquoise shallows, beach lines, Luna maria and Mars rust — plus a sub-vertex albedo detail octave the vertex path could never carry. Model-space position IS the body-frame direction (the mesh spins with the planet), so features stay glued to the ground. Ocean/ice specular values are decided per fragment now, not per vertex. Save v9.
**Deep-space colored circles, removed.** Two culprits: the M21 nebulosity fans (flat colored discs once the winding fix made them face the camera — deleted outright) and lens-flare ghosts firing while the sun was off-screen or behind the camera (`clip.w > 0` alone is not "visible"). The flare now requires the sun core ON screen (|ndc| < 0.98) AND in front (view-forward test), with a tighter edge fade.
Verified: 60 tests green both configs; day-side close orbit (sharp coastline + shallows), night-side silhouettes, deep-space view clean of stray discs.

### Milestone 24 — The cinematic grade (done)
**Tone curve.** The ACES fit (contrasty, saturation-pushing) is replaced by the Hable/Uncharted-2 filmic curve normalized to a 4.0 white point — highlights ROLL OFF instead of clipping toward primaries. On top of it, an explicit grade with named knobs at the top of Mesh.frag: `kContrast 0.90` (pivot 0.42 in the tonemapped domain), `kSaturation 0.90` (mix toward luminance), `kBlackLift` (faintly cool raised blacks — shadows breathe instead of crushing). One `gradeCinematic()` shared by the lit, emissive, planet and atmosphere paths keeps the whole frame consistent.
**Softer light, better shaders.** WRAP DIFFUSE (`kDiffuseWrap 0.22`, renormalized): the terminator became a gradient, standing in for atmospheric scattering and bounce. Eclipse shadows gained a PENUMBRA: the binary ray-sphere hit is now a smoothstep over the ray's closest-approach distance across `kPenumbra` of the occluder radius — planet shadows fade like a real sun's angular size dictates (the formulation also drops the old day-side special case: occluders behind the fragment are rejected by the sign of the ray projection). A screen-space hash DITHER (+-0.5/255) removes the 8-bit banding that smooth sky and terminator gradients otherwise show.
Verified: 60 tests green both configs; day-side orbit (ocean sun glint, pastel grade, soft limb), twilight and deep-space shots clean.

### Milestone 25 — Heightfield v2: a terrain worth lighting (done)
**Why first.** Everything the planetary-realism plan does afterwards (per-pixel normals, cloud shadows, aerial perspective) LIGHTS this terrain. v1 was a single 5-octave value-noise fBm: round continents, round hills, no crest, no valley, nothing an oblique sun can carve. No amount of shading fixes that, so the function came first.

**What v2 is.** `Planet/Terrain.hpp` is now a composed, still 100% analytic and seed-deterministic function: the v1 fBm survives UNTOUCHED as the *continental mask* (so every coastline, the world map and every placed site are bit-identical — a unit test compares 10,000 directions against the verbatim v1 formula and requires ZERO land/sea mismatches), and relief is added on top of it: an *orogeny belt* mask (low-frequency fBm, thresholded) decides where ranges run; one *domain-warp* pass (2 octaves, 3 axes) kills the value-noise grid signature; a *ridged multifractal* (folded, squared, octave-weighted) carves crests, spurs and cols; *billow* noise rolls the foothills; an *erosion curve* (`mix(h, h*h, erosion)` — no `pow`, so the GLSL twin stays exact) deepens valleys; soft *terraces* bench the slopes inside the belts; a *coastal envelope* fades relief to exactly 0 at the shoreline (flat beaches, no cliff at the seam). Terra's hypsometry came out Earth-like by construction: 81% of land below 900 m, mean 704 m, rare summits at the full 9 km amplitude.
**Real bathymetry.** The ocean is no longer a flat clamp: `terrainElevationSigned` returns the NEGATIVE sea floor (continental shelf -> abyssal plain -> mid-ocean ridges, `oceanDepth` per body), which M27 needs to color water by depth. `terrainElevation` — what physics, the patch geometry and every landing touch — still clamps to 0: **the sea-level sphere is the surface you collide with**, unchanged.
**Per-body geology.** New `TerrainComponent` fields (ridge/billow weights, warp strength, erosion, terraces, belt threshold, ocean depth) turn the same function into different worlds: Luna billow-dominated and un-eroded (basin rims, no terraces), Mars strongly warped (canyon systems), Terra folded belts. The three presets moved INTO the engine (`presetTerra/presetLuna/presetMars`) — the game, the collider, the patch and the shader now read one table instead of four copies. Save schema `planet.Terrain` v2.

**Shaders became modules.** `Shaders/Noise.glsl` and `Shaders/Terrain.glsl` are the GLSL twins of the two headers, `#include`d (GL_GOOGLE_include_directive) by `Mesh.frag`; `cmake/Shaders.cmake` gained the include path and makes every `.glsl` module a build dependency of every compiled shader.
**The divergence detector.** `Tools/glsl_parity/check_parity.py` mechanically TRANSPILES both GLSL modules into C++ (the shared subset is deliberately small), compiles them next to the real headers and evaluates both over 20,000 directions per body: mask delta, elevation delta, land/sea mismatches, and a field-by-field preset comparison. Current result: **bit-exact, zero mismatches on all three worlds**. Line-by-line review does not scale; this does.
**One palette everywhere.** `colorizeSurfaceVertex` (game) is the CPU twin of `planetSurface` (Mesh.frag) and both are driven by the v2 elevation — far LOD vertex colors, the close-orbit per-fragment path and the ground terrain patch now agree, so crossing a LOD boundary changes sharpness and nothing else. Globe LOD build does ONE elevation sample per vertex feeding both the palette and the relief normal (v2 costs ~22 noise samples per point; paying it twice was a visible build hitch).
Verified: 67 tests green (7 new terrain contract tests), parity check bit-exact, shaders compile with glslangValidator, terrain cost measured at 0.67 us/sample release (65x65 patch rebuild ~2.8 ms, throttled to once per second).

### Milestone 26 — Per-pixel terrain lighting (done)
**The point of the milestone.** M25 built a terrain worth lighting; this one lights it. The globe's shading normal is no longer interpolated from 150x225 vertices — it is the GRADIENT OF THE HEIGHTFIELD, taken per fragment, at that fragment's own screen footprint. Mountain flanks catch the sun, valleys fall away, and because the gradient comes from the same analytic function physics uses, a ridge that shades is a ridge the ship hits.

**Body-frame shading.** The vertex shader now hands the fragment the sun direction, the view direction and the body radius expressed in the BODY FRAME (`transpose(mat3(model))`, uniform scale being an engine invariant) — no new uniform, no CPU work. Dot products do not care which orthonormal frame they are taken in, so the planet path shades entirely in the frame where the heightfield gradient is naturally expressed. The eclipse test stays in world space (it is about positions, not normals).

**The octave LOD — the anti-shimmer that makes the rest possible.** Detail finer than a pixel cannot be drawn, only crawled. `terrainLodOctaves` converts the fragment's `fwidth`-derived footprint into a FRACTIONAL octave count, and `ridged3f`/`billow3f` fade the last octave in and out so detail arrives continuously (an integer count pops). At ground level the count saturates at the body's full budget and the sampler reduces EXACTLY to `terrainElevationSigned` — the GPU converges to the collider as you land. Relief octaves went 5 -> 10 on Terra and Mars (9 on Luna) now that they are LOD-gated: the finest crest is ~2 km wide instead of ~60 km.
**Cheap extra taps.** The continental mask and the orogeny belt vary over thousands of kilometres, so the fragment samples them ONCE and passes them to every gradient tap and shadow step (`terrainElevationFast`) — 8 of the ~22 noise evaluations disappear from each extra sample.
**Terrain self-shadowing (HIGH).** Six quadratically-spaced steps march the ground toward the sun and ask whether anything rises above the light ray; a ridge 4% of the amplitude above it shadows fully, with a soft edge for the sun's angular size. Direct light only — the sky still fills the valleys. MEDIUM skips the march and keeps the gradient shading (already most of the effect); LOW drops per-pixel normals entirely.
**Roughness and shape, two honest tunings.** The ridged multifractal's gain went 0.5 -> 0.55: with lacunarity 2.07, halving amplitudes make an almost BROWNIAN surface (H ~ 0.95) whose slope barely grows with detail — mountains without steepness. And the ridged term is now SQUARED before weighting: the raw multifractal averages 0.46, which turned every belt into a high plateau; squaring keeps the crests and drops the mass between them (ridge weights raised to compensate: Terra 1.35, Mars 1.15, Luna 0.45). Terra's hypsometry stays Earth-like (87% of land under 900 m, summits at 7.7 km). `kReliefExaggeration = 7` tilts normals only — geometry, silhouette and collision are untouched, exactly like the M22 vertex exaggeration.
**Biomes.** Albedo is now a function of altitude, SLOPE, latitude and humidity: a dedicated low-frequency fBm plus coastal proximity drives desert -> steppe -> grass -> forest; beaches appear only where the ground is flat and just above the water; soil thins with altitude and gives way to bare rock on steep ground; snow follows a quadratic-in-latitude line (~6.7 km at the equator, ~4 km at 37 deg, sea level at the poles) and does not stick to cliffs. Luna exposes brighter regolith on crater walls, Mars dark basalt on its scarps and a dusty high plateau. `colorizeSurfaceVertex` (CPU, far LODs) mirrors the same formulas with a slope measured at its own sampling scale, and the globe LOD builder now takes 6/5/4 relief octaves by level — the vertex path and the fragment path meet in the middle instead of popping.
**Quality tiers.** `ApplicationConfig::renderQuality` (`--quality low|medium|high`, and `--cpu` implies LOW) travels to the shaders in the camera UBO alongside a wrapped world clock the later milestones animate with.

**Seeing the shader without a GPU.** `Tools/planet_preview/render_preview.py` transpiles Noise/Terrain/PlanetSurface with the same machinery as the parity check and RAY-CASTS the planet on the CPU through the real `planetShading` entry point — footprint LOD, self-shadow, biomes, the M24 grade and the sRGB encode the swapchain performs. It writes PNGs of a fixed set of viewpoints (Terra orbit at three quality tiers, raking light over the ranges, a coast, a polar cap, Luna, Mars). Reviewing a shading change no longer means building, launching and flying there; it means running one script and looking at the diff.
Verified: 67 tests green, parity still bit-exact on all three bodies, shaders compile, preview captures reviewed at every tier.

### Milestone 27 — A living ocean (done)
**Depth, at last.** M25's bathymetry now has a consumer: water is coloured by its REAL depth (abyssal plain -> open ocean -> turquoise shelf), so the pale continental shelves of the reference frame appear exactly where the sea floor rises, and the deep basins go properly dark.
**Surf.** A foam band keyed to the depth in METRES (full under ~2 m, gone by ~12 m), modulated by an animated 2-octave noise. Its width is therefore decided by the sea floor, not by a constant: narrow along a steep coast, wide over a flat shelf — for free, because the bathymetry already knows.
**Micro-waves, and the honest way to skip them.** A 260 m swell plus a 90 m chop, animated by the world clock, perturb the surface normal — but only while a pixel can resolve them (`resolved` fades between 60 m and 260 m of footprint). Beyond that the waves are folded into a WIDER specular lobe instead of being drawn, which is the physically correct way to render a sub-pixel rough surface and is precisely what turns the glint from a hard dot into a scintillating trail.
**Fresnel, for real.** Water switches to Schlick with its true F0 = 0.02: face-on the sea barely reflects, at grazing angles it becomes a mirror. On its own that made the glint invisible (2% of a light whose radiance we do not model), so the water lobe is ENERGY-NORMALIZED — `(exponent + 8) / 8pi` — concentrating the same energy into a brighter, tighter highlight. Authored materials (hulls, ice, parts) keep the M21 response curve untouched.
**Sea ice** shares Terra's cap edge with the land biome, so the polar cap is one coherent shape: ragged noisy boundary, flat normal, low sheen.
The camera UBO's world clock (added in M26) is the only new input; nothing here touches the simulation.
Verified: 67 tests green, parity bit-exact, preview captures of the glint trail (camera tilted toward the sun's azimuth by 90 deg minus its elevation — the shot a mirror sea actually returns), the surf line and the shelves.

### Milestone 28 — Clouds v2, and the shadows they throw (done)
**One function, two consumers.** The cloud deck is now a single analytic function of (body-frame direction, world time) in `Shaders/Clouds.glsl`. The shell fragment shader evaluates it to DRAW the clouds; the ground fragment shader evaluates it where its ray to the sun PIERCES the shell, to know whether it stands in shade. There is no shadow map, no second pass and nothing to keep in sync — a cloud physically cannot cast a shadow that disagrees with its own shape.
**Per-fragment weather.** Two layers: domain-warped, hard-thresholded CUMULUS (defined edges instead of the old per-vertex blur) and stretched CIRRUS veils, drifting at different rates. Detail follows the same footprint LOD as the terrain (6 octaves close, 3 for a marble), the polar fade of the UV sphere is kept, and clear sky `discard`s so the planet shows through untouched.
**Thickness and silver lining.** A second coverage sample offset toward the sun tells a fragment whether it sits in the shaded core of a mass or on a lit flank; and a forward-scattering phase term lights thin edges when the camera looks toward the sun — the rim that reads as backlit cloud.
**The drift moved into the shader.** `CloudLayerSystem` now GLUES a shell with no spin of its own to the body's rotation, and the deck's advection happens in the coverage function from the world clock. That is precisely what lets the ground reproduce the deck exactly. `MeshComponent::kCloudDeck` (value 2 of the existing transparency flag — no save migration) routes the shell to instance tint 3.2, the cloud material, while 3.0 stays the atmosphere limb.
**Where the shadow lives.** Inside `planetShading`, not in `Mesh.frag`: the CPU preview then renders it too, which is how it was reviewed.
**Preview fix.** The preview camera basis went degenerate at zero tilt (`cross(forward, centre)` vanishes when looking straight down) and produced NaN frames; it now falls back to a local tangent. The full-planet capture — continents, shelves, snow, cumulus masses and their shadows — is the first end-to-end look at M25 through M28 together.
Verified: 67 tests green, parity bit-exact, shaders compile, captures at every quality tier.

### Milestone 29 — Physical atmosphere (done)
**One integral, three uses.** `Shaders/Atmosphere.glsl` computes analytic SINGLE SCATTERING — Rayleigh (the blue, wavelength-dependent) plus Mie (aerosols, the white glare) — along a view ray, with the optical depth toward the sun taken from the Kasten-Young airmass formula instead of a nested integral. Six steps, no lookup table, no precomputation, no extra pass. That one function now produces:
1. **Aerial perspective** on every lit fragment: the ray from the camera to the fragment is extinguished and veiled by the air in front of it. Distant ground goes blue from orbit and amber at sunset for the same reason the sky does — because it is the same air. This replaces the M21 exponential fog (kept as a fallback for bodies with no air parameters).
2. **The limb from space**: the atmosphere shell no longer runs a fresnel trick. It marches the real air column and returns what the ray gathers — thin, graded, bright where the column is long, reddened where it crosses the terminator.
3. **The sky from the ground**: the same shell seen from inside IS the sky. Zenith deep blue, horizon pale, sunsets and dawns for free — no special case, no CPU-side sky colour.
Because it is one function, the descent from orbit to the pad is continuous by construction: nothing switches models, the ray simply gets shorter.
**No double counting.** The shell DISCARDS wherever the planet blocks the ray: that fragment's own aerial perspective already accounts for the air in front of it. Painting both is how atmospheres end up looking like fog banks.
**Per-body air.** `atmospherePreset` mirrors the terrain-preset pattern (style id 0 Terra, 2 Mars): real coefficients for Terra (beta_R 5.8/13.5/33.1e-6, H_R 8 km, H_M 1.2 km); a thin, dust-reddened Mars whose Rayleigh term is INVERTED — the dust absorbs blue rather than scattering it, which is the whole reason its sky is butterscotch. The camera UBO gained the camera-relative centre and radius of the body whose air we are looking through, chosen per frame as the nearest body with an `AtmosphereComponent` (always, not only when inside it — the limb needs it from orbit). The atmosphere shell mesh grew to 1.0130 R so it encloses the 80 km column the shader marches.
**Quality dial.** 8 marching steps for the sky at HIGH, 5 below; 6 / 4 / 3 for aerial perspective. The CPU keeps its simplified sky model for `skyAmbient` (the light the atmosphere puts back into the scene) and for the HUD's day factor.
Verified: 67 tests green, parity bit-exact, shaders compile; captures of the limb from 500 km, a ground-level sunset (golden horizon, blue zenith, terrain fading into haze) and the full planet with its halo.

### Milestone 30 — LOD fades, budget and polish (done)
**Nothing pops.** Every shading term that depends on detail now fades on its OWN scale, driven by how much of the world one pixel covers: terrain self-shadowing between 800 m and 3 km per pixel (a shadow whose caster is smaller than a pixel is not a shadow, it is noise), cloud shadows between 12 km and 40 km, with the shadow's cloud octaves following the pixel exactly like the deck's do. The relief octave count was already continuous (fractional octaves since M26), so the whole descent from a marble to a landing is fade-free.
**The scattering gain, corrected.** M29 shipped with an in-scatter gain of 22, which washed the ground out from orbit: the engine's sun is normalized to 1 for direct lighting, so a physically scaled sky lands at a few PERCENT of a sunlit surface — which is correct. The gain is now 6 (Terra) / 4.5 (Mars), which stands in for the multiple scattering a single-scattering model omits (a third to a half of a real sky) and nothing more. Ground detail and colour survive the trip through 80 km of air; the limb and the sunset are unchanged.
**Map view** no longer receives an atmospheric body: the star map is a schematic and had no business hazing over its own orbits.
**Measured cost.** Relative fragment cost of the planet path, measured on the CPU preview (same arithmetic the GPU runs, single core, 360x360 nadir view from 400 km): LOW 2.8 us/px, MEDIUM 5.2 us/px, HIGH 7.4 us/px — a 1.0 / 1.85 / 2.6 ratio between tiers. LOW is the CI and llvmpipe tier (`--cpu` selects it automatically): no per-pixel normals, 3 relief octaves, no terrain self-shadowing, 3 scattering steps.
**Documentation.** README gained the `--quality` flag and a section on the two planetary tools; `docs/Architecture.md` carries the full M25-M30 log; `captures/M30/` holds the reference set (Terra at three tiers, raking light with and without self-shadowing, coast, surf, glint, polar cap, full planet, limb, ground sunset, Luna, Mars).
**The grade is untouched.** The M24 Hable curve, contrast, saturation and black lift are exactly as they were — the target look has that tone, and everything above was built to land inside it.
Verified: 67 tests green, parity bit-exact on all three bodies, all shaders compile, full capture set reviewed at every tier.

### Milestone 30b — the grey planet, diagnosed (done)
In-flight review after M30 showed the planet from 400 km as a featureless grey-blue film: no terrain, no clouds, no water. Three causes, found by reproducing the exact frame in the CPU preview and printing the contribution of each layer at one pixel.

1. **Cloud cells were the size of continents.** `cloudCumulus` inherited the base frequency 3.6 of the old per-vertex shell — cells roughly 1,800 km across. From a 400 km orbit that is ONE cloud filling the entire screen, which reads as a film, not as weather. The base frequency is now a named constant at 22 (cells ~290 km, detail down to a few km), the domain warp scales with it, and the octave budget comes from a proper `cloudLodOctaves(footprint)` mirroring the terrain's rule (it was a hand-rolled expression tuned for the old frequency). This was the dominant term: at the measured pixel, coverage went from 0.59 (opaque veil over deep water) to broken fields with gaps.
2. **Cirrus were counted twice.** The shell's alpha added `cirrus * 0.25` on top of a `coverage` that already included the cirrus term, and the layer itself was far too generous (visible over 43% of the sky at a quarter opacity). Cirrus are now rare and thin (13% of the sky), and the alpha uses coverage alone.
3. **The scattering gain was a single number for both species.** A gain that makes the sky right amplifies the Mie forward lobe into a sheet of haze whenever the sun is behind the camera. Rayleigh and Mie now carry SEPARATE gains (3.0 / 1.0 on Terra): Rayleigh keeps the allowance for the missing multiple scattering, Mie gets none — it is already a narrow lobe.

**Method worth keeping:** the CPU preview reproduced the reported frame exactly (400 km, sun high behind the camera, open water), which turned "it looks grey" into three measured numbers — ground 0.039/0.089/0.174, cloud alpha 0.589, air 0.015/0.033/0.073 — and pointed straight at the layer responsible. Add a shot to `SHOTS` and the frame becomes a regression test.
Verified: 67 tests green, parity bit-exact, full capture set re-rendered.

### Milestone 30c — in-flight review: light, relief, cost (done)
Second pass on real footage from a 400 km orbit. Four reports, four causes.

**1. "Behind the planet it is broad daylight."** A real bug, and a nasty one. The sun's optical depth used the Kasten-Young airmass formula, which is only defined for a sun ABOVE the horizon: fed a negative cosine it returned a value near ZERO — "no air in the way" — so the night hemisphere's atmosphere was lit as if at noon. It is replaced by a rational fit (exactly 1 at the zenith, ~38 at the horizon, no acos, no pow) plus an explicit `sunVisibility` test: a parcel of air is in shadow when its ray to the sun points below the local horizon AND passes inside the planet. That test is what actually draws the terminator, and it costs one dot product and one sqrt.

**2. Black holes punched through the terrain.** Introduced by an optimisation in this same pass: the gradient's centre sample was taken from the caller (a real saving — one tap in three) but with a DIFFERENT octave and warp count than the two tangent taps. The difference then measures the offset between two functions rather than a derivative: slopes came out at 4.5 instead of 0.037, normals swung past the terminator, and neighbouring pixels rendered fully lit and fully black. The centre now always comes from an identical sampler, and `terrainNormalBody` carries a comment saying why that is not optional. Belt and braces: `kMaxNormalTilt` caps the exaggerated tilt at 45 degrees, so no future tuning can swing a normal past the light again, and the exaggeration itself drops 7 -> 4.

**3. Flat ground was too flat.** `plainsWeight` (0.20 Terra / 0.26 Luna / 0.28 Mars) applies the ridged field OUTSIDE the orogeny belts, where it was previously multiplied to zero. There is no such thing as flat ground at this scale, and the plains were the flattest thing in frame. It reuses the ridged value already computed — the cost is one multiply. Save schema `planet.Terrain` v3.

**4. Too saturated / too slow.** Grade saturation 0.90 -> 0.82. And the fragment cost of the planet path, measured on the CPU preview (same arithmetic, single core, 250 m/px): **HIGH 2580 -> 1550 ns/px (-40%), MEDIUM -> 1240 (-52%), LOW -> 850 (-67%)**, from:
- the water branch no longer computes a heightfield gradient it then throws away (a third of the planet);
- the night hemisphere skips the gradient, the sun march and the cloud shadow entirely — the gate sits at -0.42 of the sun dot, deep enough that no tilted normal can still catch light (at -0.25 it drew a visible seam down the planet);
- the sun march is skipped when the sun is high enough that no slope here could shadow anything;
- the polar cap edge is only sampled near the poles (three noise samples saved on every other pixel);
- the airmass rational fit replaces acos+pow, six times per fragment;
- the atmosphere march scales its step count with the air it actually crosses — a rocket 30 m away no longer pays six samples;
- the quality tiers became genuinely far apart (the octave count is paid three times per fragment, which makes it the most effective dial there is): HIGH full stack, MEDIUM 6 octaves and a single warp octave, LOW 3 octaves and sphere normals. `--quality medium` is now a real lever.
Cloud shadows also softened (strength 0.75 -> 0.5) and the self-shadow threshold widened (4% -> 8% of amplitude): under a real deck the ground still gets plenty of light from the sky and the neighbouring clouds.
Verified: 67 tests green, parity bit-exact, full capture set re-rendered including new night and terminator shots.

### Milestone 30d — the flat grey rocket (done)
In-flight report: hulls in orbit read as flat grey tubes, with almost no difference between their lit and shadowed sides. The cause was M24's WRAP DIFFUSE. It exists to stand in for the atmospheric scattering and bounce light that blur a PLANET's day/night line — and it was being applied to every mesh in the scene, which lifted any surface facing 90 degrees away from the sun to 18% brightness. On a cylinder that is the difference between a lit tube and a shaded one.
The wrap is now split: `kDiffuseWrap` 0.22 for the procedural planet path, where it represents something real, and `kDiffuseWrapObject` 0.03 for everything else — a nearly hard Lambert. Grade contrast also went 0.90 -> 0.98. Measured through the full pipeline (Hable, contrast, black lift, sRGB) on a 0.62-albedo hull:

| dot(n, L) | before | after |
|---|---|---|
| +1.00 (noon) | 0.598 | 0.590 |
| +0.40 | 0.486 | 0.431 |
| 0.00 (grazing) | 0.370 | **0.213** |
| -0.15 (shadow) | 0.308 | **0.183** |

The fully lit side is untouched; the lit-to-shadow ratio goes from 1.9 to 3.2.

### Milestone 30e — detail range and hitches (done)
In-flight report: substantial stalls, and the ground still not textured enough away from the ranges.

**Where the detail starts.** The per-fragment planet path switched on at 4 body radii — 25,000 km on Terra, from where the globe is a small disc the vertex path draws just as well. It now starts at **1.6 radii** (about 3,800 km of altitude), which still covers every orbit worth flying, and hands everything beyond that back to the cheap path.

**The terrain patch, three ways.** It was the source of the periodic hitches:
- it built below 300 km; now **120 km** — above that the globe's own per-fragment surface is what you are looking at anyway;
- it rebuilt on a fixed 1 s cadence; the interval now scales with altitude (1 s near the ground, up to 8 s at 100 km) — high up the patch spans hundreds of kilometres and nothing moves relative to it;
- it sampled the FULL octave stack onto a 65x65 grid whose cells can be kilometres wide. The octave count is now derived from the cell size, exactly as the shader derives it from the pixel footprint: detail a mesh cannot represent only aliases between its vertices, and it was being paid 4,225 times per rebuild.

**And the stall itself.** Replacing the patch mesh destroyed buffers that up to two frames in flight might still reference, so the code called `renderer().waitIdle()` — a guaranteed pipeline bubble on every rebuild. The patch is now **double buffered**: two slots used in turn, so the buffers being destroyed are the previous rebuild's, seconds old and long retired. No device idle, no bubble.

**More relief, everywhere.** `plainsWeight` 0.20 -> 0.34 (Terra), 0.42 (Luna), 0.46 (Mars), and the shading exaggeration 4 -> 5.5 (still capped at 45 degrees of tilt). Terra's mean land elevation lands at 853 m — Earth's is 840 — with the hypsometry spread out rather than piled into the lowest tenth: 67% under 900 m instead of 87%. Measured tilt: 12 degrees mean, 27 at the 90th percentile, and 0.6% of land steep enough to fall fully into shadow.

The cloud deck — a second full-screen pass over the planet disc — now follows the quality tier as well (6 / 5 / 4 octaves).
Verified: 67 tests green, parity bit-exact, full capture set re-rendered.

### Milestone 31 — Ground you can land on (done)
The approach looked right from orbit and turned into a billiard table on final. Measuring a 3 km site sampled every 30 m said why, in three separate ways.

**1. Summits were literally flat.** `clamp(h, 0, 1)` pinned every point where the weights overshoot to exactly the amplitude: a 9 km peak came out as a level mesa, gradient zero — unlandable and unlit. It is now a SOFT CEILING (`h > 0.75 -> 0.75 + (h-0.75)/(1+4(h-0.75))`) which compresses toward 1 without ever reaching it. Pure arithmetic, so the GLSL twin stays bit-exact.

**2. The octave cascade cannot supply landing-scale relief.** With gain 0.55 and lacunarity 2.07, an octave's SLOPE only grows 14% per step, so by the time the wavelength is a few hundred metres its amplitude is a metre or two. Raising the stack from 10 to 16 octaves (finest ridge ~30 m) moved a mountain site from 0.6 to 1.4 degrees of average slope — still a table. The fix is a dedicated `detailWeight` / `detailFrequency` term: a separate ridged field at ~800 m wavelength with its own budget (2.2% of amplitude on Terra, 3.0% on Luna where nothing erodes, 1.8% on Mars), four times stronger inside the mountain belts than on the plains, and applied AFTER the soft ceiling — running it through the knee would flatten it exactly on the summits where it matters most.

Measured on the same 3 km sites, before and after:

| site | before | after |
|---|---|---|
| piedmont | 183 m of local relief, 1.9 deg | **256 m, 10.8 deg** |
| mountain | flat mesa, 0.6 deg | **212 m, 11.4 deg** |

Global hypsometry is untouched (mean land 859 m, Earth's is 840). Detail fades in over octaves 11-13, so it only exists where a pixel — or a mesh cell — can actually resolve it, and never aliases into the distance.

**3. The patch could not have shown it anyway.** It bottomed out at 4 km of extent on a 65x65 grid: 125 m cells, which cannot represent anything smaller than a hill. It now goes down to 1.5 km on a 97x97 grid — **31 m cells** — and derives its octave count from that cell size exactly as the shader derives its own from the pixel footprint.

**And the build moved off the main thread.** At that density a rebuild is ~9,400 evaluations of a 16-octave heightfield: a visible stutter every time the craft crossed its own patch. `buildTerrainPatch` now runs on the engine ThreadPool, captures everything it needs BY VALUE (it never touches the world, the renderer or a component), and writes into a pending mesh the main thread uploads on a later frame — so the patch on screen is never the one being written. Combined with the double buffering from M30e there is no `waitIdle`, no stall and no hitch. The game's destructor waits for the job, since the pool lives in the base class and outlives these members.

Collision samples the full 16 octaves plus the detail: what the lander touches is what the patch draws, still, to the metre.
Verified: 67 tests green, parity bit-exact, CPU cost 0.97 us/sample (16 octaves + detail, up from 0.77 at 10), new landing-scale captures.

### Milestone 31b — the day cycle, on rails (done)
The bodies already carried real rotation rates (Terra 7.2921e-5 rad/s — a sidereal day; Luna tidally locked at 2.66e-6; Mars 7.088e-5) and a system that applied them. What they did not have was a rotation that SURVIVED TIME WARP: `CelestialSpinSystem` integrated the angle tick by tick, and the Physics lane keeps strict fixed steps and DROPS the backlog it cannot afford. Above physics warp the planet's position jumped analytically around the star while its surface barely turned — the world moved and the day did not.

The spin is now a closed form, for exactly the reason the orbits are: `angle = rate x presentSeconds`, evaluated at the same lane present the POSITIONS use (so a body's spin and its orbit can never disagree by a tick), with an `fmod` because 7.29e-5 rad/s reaches 2,300 radians in a single orbit and f32 would grind that to a stutter. It is also exact across a save: a base goes to sleep and wakes up under the same star, instead of wherever the tick budget happened to leave it.

Everything downstream follows for free, because they all read the body's rotation: the terrain patch centre (computed through the inverse body rotation), surface anchors, the atmosphere and cloud shells. And the shader's animation clock moved from the WALL clock to the simulation's — the cloud deck drifts over a ground that now turns with the same time, so warping speeds the weather up exactly as it speeds the day up.

### Milestone 31c — the ground under your feet, and not drawing what is under it (done)
**Collision sat below what was drawn.** Not a physics bug: the patch mesh chose its octave count from its cell size with a Nyquist-ish rule (drop an octave once its wavelength falls below twice a cell), while COLLISION always samples the full stack. The two surfaces therefore differed by the octaves the mesh had dropped — up to about 3 m at landing altitude, which is exactly "slightly too low" when you are standing on it. The rule is now a quarter of a cell instead of twice one, which costs one or two octaves on a job that runs off the main thread and buys this:

| altitude | cell | octaves | patch vs collider |
|---|---|---|---|
| 80 m | 31 m | 16 | **0.000 m** |
| 400 m | 50 m | 16 | **0.000 m** |
| 2 km | 250 m | 14 | 2.4 m |
| 20 km | 2.5 km | 11 | 114 m |

Where you can touch the ground, the drawn ground and the collided ground are the same surface. Further up the difference stays well under one mesh cell.

**Not drawing what cannot be seen.** Two changes, both about the depth buffer:
1. Opaque draws are now submitted FRONT TO BACK. Sorting the BATCHES (not the instances) leaves every batch's slice of the instance buffer exactly where it was written, so the cost is one sort of a few dozen entries — and the terrain patch and the craft are drawn before the planet behind them.
2. `Mesh.frag` declares `layout(early_fragment_tests) in;`. The shader contains `discard` (the cloud deck and the atmosphere shell), and a shader that MAY discard normally forces the driver to postpone the depth test until AFTER shading — so every fragment of planet hidden behind the terrain patch, the rocket or a part still evaluated its noise octaves before being thrown away. The mode is safe here: the only depth-WRITING pipeline (opaque) never reaches a discard, and the transparent and HUD pipelines do not write depth at all.

Standing on a planet, those two together reject most of the screen before it costs anything.

### F1 — Fondations données de l'industrie (done)
**The mining outpost stopped being code and became data.** Three files decide what industry is now, and none of them is a `.cpp`:

* **`.swrecipe`** (Assets/Recipes, 8 shipped) — inputs, outputs, power, declared mass loss, addressed by stable id. Rates are UNITS PER SECOND on purpose: the Automation lane hands the executor either one 0.2 s tick or eight hours of warped catch-up, and units-per-second is the only formulation where both give the same goods. `IndustryTests` proves that equality rather than assuming it.
* **`.swpart` + an industrial block** — a building is a part with a `BuildingSpec` (category, ground footprint, steady power balance, storage volume, the steepest slope and the poorest ore it accepts). Same file format, same stable-id space, same Part Studio; the VAB palette simply filters the ones that have the block, because a refinery is not something you stack on a rocket.
* **`Planet/Deposits.hpp`** — ore as an ANALYTIC FIELD, exactly like the terrain: `oreDensity(deposits, direction, resource)` is a pure function of where you are. Nothing about a deposit is saved, so a world reloaded a year later has its ore in the same rocks, the scanner shows the very function the miner exploits, and a survey cannot lie.

**One executor, not one system per machine.** `ProductionSystem` runs every building in the game from its recipe. The order of operations is the part that matters: it works out what fraction of a second it can actually run — limited by the inputs present AND by the room its products need — and only then moves any matter. Writing that test found a real bug in the first draft: each output asked *independently* whether it fitted, so electrolysis' two gases both got a yes, the second silently did not fit, and the water that became it was already gone. Output room is a VOLUME question about the whole batch, minus what the inputs free. Matter had been dying in that gap.

**MATTER IS CONSERVED, and it is the build that says so.** Every recipe in the catalogue — built-in or loaded from disk — is weighed at load time and again in the tests: outputs may never exceed inputs, and the difference must be the loss the recipe declares. A balance pass that quietly invents iron fails the build, not the playtest.

**The outpost is now a SITE.** A hub owns it, and a miner, a refinery, a silo and a solar field stand around it — each an entity carrying `BuildingComponent` (what it is) + `RecipeStateComponent` (what it is doing) + `PowerComponent` (what that costs), anchored in Terra's rotating frame, and all four registered in the save schema. The miner's yield is the recipe rate times the analytic ore density under its feet, so siting IS the gameplay.

**Which exposed something the old numbers had been hiding: the outpost was in the sea.** The rig and the launch pad were both nailed to `+Z` on the equator because it was a convenient constant — and `+Z` on Terra is open ocean. `terrainElevation` clamps at sea level, so nothing ever complained: the pad floated, and a mine there would have dug water.

The starting site is now SURVEYED, and the survey searches **the equator**. Latitude is a permanent tax on every mission a base ever flies: an equatorial pad is handed the planet's full rotation for free (465 m/s on Terra) and needs no plane change to reach the equatorial orbits the rest of the system uses. A richer site 24 degrees north is not a better site. So `planet::surveyEquatorialSite` sweeps the whole ring at ~28 km for a continent carrying ore, then refines at ~350 m for ground flat enough to stand the WHOLE factory on one plateau — a slope limit measured across the entire 120 m footprint, far stricter than any single building's own. It lands at longitude −137.6, latitude **0.000000**, 526 m up, over ore grading 0.85, 768 km from the nearest coast east and thousands north and south. Every plot in the shipped layout, plus the pad 120 m east, comes out under a 0.08 slope — and the test checks all seven rather than trusting the centre. The survey is one shared function, so the ground the scene founds the base on, the ground F2's build cursor will validate, and the ground the tests check are the same arithmetic.

**And a sixth building, because a site you cannot find is not a site.** The BC-1 nav beacon produces nothing but VISIBILITY: a 25 m lit mast to see from the ground, a marker on the star map, and — inside the range it declares (100 km) — a reticle in the cockpit carrying the site name and the LIVE distance under it. Distance is measured from the craft you CONTROL, not from the camera, because in free-cam or on the map the viewpoint can be parked anywhere and "how far am I" has to mean the pilot.

Where that pointer goes when the target is behind you or past the edge of the screen is `UI/ScreenMarker.hpp` — one pure function, unit-tested, shared by every world-anchored overlay the game will grow. Doing nothing there is the wrong answer twice over: you are hunting for the beacon precisely BECAUSE you cannot see it, and the perspective divide with `w < 0` does not merely fail, it mirrors the point through the origin and points the pilot 180 degrees off course. Behind the camera the direction is rebuilt from the camera basis instead, and off-screen markers are clamped to the border in the direction you would turn to find them. The tests sweep a target from dead ahead to dead astern and check the pointer never leaves the border box, never goes NaN, and never flips side.

Verified: 82 tests green (recipe conservation over the shipped files, deposit determinism and bounds, polar ice, warp-exactness to the unit, blocked-instead-of-destroying, a survey that stands on buildable ore and says "nothing" when it finds none, the whole mine → smelter chain weighed end to end, and five screen-marker cases including behind-the-camera and the dead-astern degenerate).

### F1b — the buried-to-the-waist bug (done)
**Ground contact snapped a body's ORIGIN onto the terrain.** That is only correct for an object of zero size, and everything in the game has a size: the stock rocket, modelled around its middle, went 7.2 m into the rock; the EVA capsule — a 2 m body centred on itself — put the player's waist at ground level. It had been hidden by a hand-tuned `+11.0` in the launch-pad spawn, a guess at one particular rocket's half length, which meant a NEW rocket looked roughly right on the pad and everything else sank.

The missing half of the question is now a component. `GroundHullComponent` is an axis-aligned box in MODEL space, and `groundClearance(hull, rotation, up)` projects it onto the local vertical to answer "how far below its origin does this thing reach, at this attitude". The resting radius is the terrain height plus that.

A box rather than a radius, deliberately. A bounding SPHERE gives the right answer for a rocket standing on its tail purely by accident — a rocket's bounding radius IS half its length — and then floats it ten metres in the air the moment it tips over. The box knows the difference between the direction a vehicle is long in and the direction it is wide in, which is the whole question when it is standing on its engine bells. The projection is done in model space, so it is exact at any attitude; the tests pin tail-down (10 m), on-its-side (1.5 m) and off-centre hulls.

The vessel's hull is accumulated by `VesselAssemblySystem` from the same COLLIDER shapes the VAB validates placement against — so what a rocket rests on is what a rocket is made of, and the day a stage separates the hull shrinks with it. The pad spawn reads the same box off the blueprint before the vessel exists, so the spawn pose and the resting pose agree by construction instead of by a constant somebody has to maintain. A body with no hull keeps the old behaviour exactly, which is what the asteroid and any bare probe want.

Verified: 84 tests green (including a shapeless probe and a 2 m body dropped side by side — the first rests its origin on the ground, the second rests its FEET on it), parity still bit-exact.

### F1c — the shimmer, the beacon range, and walking (done)
**Everything standing on the planet vibrated.** Not the renderer, not interpolation: `TransformComponent::rotation` is an `f32` quaternion, and a surface anchor is positioned by rotating a vector one PLANET RADIUS long. f32 carries about seven digits, so 6,371 km is quantised to roughly 1.2 m — and because the quantisation error is a function of the angle, it MOVES as the planet turns. Measured on Terra's real rate at 50 Hz: up to **0.77 m of displacement between consecutive frames**. Every building, the launch pad and the terrain patch swam by that much while the camera — an honest f64 world position — held perfectly still.

The body's spin is now kept at full precision on its `GravitySourceComponent` (axis, angle, previous angle, all `f64`), stamped by `CelestialSpinSystem` alongside the f32 quaternion it already wrote. The rule that follows is simple enough to state in one line: **position math uses the f64 spin, orientation math keeps the f32 quaternion.** A ten-millionth of a radian across a 20 m building is a nanometre; across 6,371 km it is a metre. Five call sites moved over — surface anchors (current and previous pose), the terrain patch's centre direction and its draw origin, the terrain sample inside ground contact, the landed-craft-to-anchor conversion, and the launch pad. The patch is also now oriented by the very rotation that placed it, rather than by a second one a fraction of a radian away.

`SaveTests` no longer just asserts "sub-metre": it turns a planet at Terra's rate for 200 ticks and requires the anchor's frame-to-frame excursion to stay under a **micrometre**.

**The beacon reaches 1000 km, and steps aside under 500 m.** Range was the point of the component being data; both numbers are fields, so F2's placed beacons can carry their own. The map still ignores both — a map that hid the thing you were looking for would not be a map.

**On foot, you walk where you look.** The suit's heading used to be integrated from the turn keys while the camera orbited independently, so the body and the view could point in different directions and reaching a spot meant fighting two controls. Now the horizontal mouse drag turns the BODY (the camera simply follows from behind), and the left/right keys SIDESTEP — you can circle a building while still watching it. Diagonal walking is normalised, so strafing forward is not 41% faster than walking forward.

### F1d — walking into mountains, Luna's weather, and the fleet switch (done)
**You fell through slopes but not through flat ground.** Ground contact samples the heightfield in the body's ROTATING frame, and it was doing so with the attitude from the PREVIOUS tick: `CelestialSpinSystem` ran near the end of the Physics lane, after the surface systems. One tick of Terra's spin is 1.46e-6 rad — which at 6,371 km is **9.3 m of ground**. On the flat that is nothing. Measured across the real heightfield, on ground steeper than 0.25 that offset is worth up to **13.6 m of elevation**, and you walked straight into the hillside. The spin is analytic, so it now runs at the HEAD of the tick, right behind the celestial positions: everything downstream reads one consistent attitude. (The shimmer fix is what made this visible — a metre of jitter had been hiding a hillside.)

The second half was the footprint. Contact sampled the terrain under the body's CENTRE only, so on a slope the uphill edge of a wide craft was buried — 1.2 m for an 8 m-wide rocket on a 0.3 slope. It now samples the four horizontal extremes of the ground hull as well and takes the HIGHEST: an object may rest above the ground, never inside it. `PhysicsTests` drops a craft onto the steepest dry ground the heightfield makes and checks all four corners against the analytic surface.

**Luna looked like it had weather.** Two causes, one of them structural. The globe's far LODs are coarse spheres — the lowest is 96 vertices — and the vertex palette was sampling noise at frequency 11 and 42 on them. That is not detail, it is one random number per vertex smeared across enormous triangles by Gouraud, and on a grey world it reads exactly like cloud. Every term in the palette is now faded toward its mean as its frequency approaches the mesh's Nyquist limit (rings / 2pi), so a mesh only ever carries frequencies it can hold. Measured on Luna, the neighbour-to-neighbour albedo step at the two coarsest LODs drops from 0.119 to 0.016 — **7x** — while the maria, a legitimate frequency-3 feature, survive untouched at the LODs that can draw them. The terrain patch passes its own (enormous) limit, so close-up detail is unaffected.

The other cause was a hard `m < 0.47` step between mare and highland. On a fractal field that draws a crisp wandering bright edge, and a crisp wandering bright edge on a grey world is a weather front. It is a smoothstep now, in the shader and in its CPU twin, with the fine-grain and crater-brightening amplitudes pulled down.

**And the map can change which ship you fly.** `P` always cycled; now there is a NEXT SHIP button with a `SHIP n/N` readout in the map view, where you are already looking at your fleet. It appears only when there is more than one vessel to cycle between.

### F1e — the ground you build from (done)
Three changes, all of them shaped by what F2 is about to need: **you place buildings by walking up to a spot and putting one there.**

**EVA is first person.** A factory is built at arm's length, and a third-person camera puts the thing you are aiming at behind your own shoulders. The view now sits at the suit's eyes: the body's heading IS the look direction (the mouse already turned the body since F1c), the pitch is a free look about the local right axis, and the horizon stays level because both are applied about the LOCAL vertical rather than a world axis. The suit mesh is skipped while you are inside it — the map still draws it, because there you are looking at the world rather than out of your own eyes.

**`F` opens the building catalogue.** Satisfactory's lesson applied: the list of things you can put on the ground is a first-class screen, not a submenu of the vehicle editor. It reads the same `.swpart` catalogue the VAB filters buildings OUT of, shows each one's category, footprint, power balance, storage and ore requirement straight from its industrial block, and arms one. `m_heldBuilding` is the whole contract with F2 — the armed id is what ground placement will read, and what a hotbar will one day save. Nothing here moves when placement lands.

**Conveyors are visible, and so is what they carry.** The belts are real geometry now: a deck with rails and legs, laid between two buildings along a path that samples the SAME heightfield the collider reads, so a belt crossing a rise climbs it instead of disappearing into it. `IndustryTests` checks that no point of a deck — including between its sample points — is ever below the ground it crosses, on the roughest dry terrain the heightfield makes.

The cargo is the part worth explaining. There are no item entities and no second simulation: a crate's position is a closed-form function of the lane's present time and the link's throughput, the same discipline the orbits and the planet's spin already follow. And it is the MEASURED throughput, not the rating — `TransferSystem` now records what actually moved (`ItemLinkComponent::flowUnitsPerSecond`), so crate spacing IS the flow. A belt fed by a starving mine visibly thins out; a blocked one empties. On the shipped outpost the ore belt carries ~9 crates and the iron belt ~5, because the smelter really is producing 0.51 units for every 0.85 it receives. `Factory/Conveyor.hpp` owns both the path and the "where is a crate at arc length s" query, because F2 draws these by hand and F6 turns them into transport — a second implementation would be a belt whose items are somewhere its rails are not.

### F1f — the camera stops flying the rocket (done)
**A chase camera that inherits the vehicle's attitude turns every roll, every RCS twitch and every SAS correction into a camera move.** The world swings around you while you are trying to read it, and you cannot look at anything for longer than the autopilot leaves the nose still. That is what the ship camera had been doing since M11: its offset frame WAS the craft's rotation, blended toward a "ground frame" that was itself built from the craft's heading — so even levelled, it still tracked the nose.

The craft's rotation now appears nowhere in the chase camera. The view has its own orientation and only the mouse changes it. The reference frame it orbits in is:

* **away from a body — INERTIAL**, the world axes. They do not drift, they do not spin, and a camera parked in them stays parked.
* **close to one — the local HORIZON frame**, whose "up" is the radial and whose heading is NORTH: the body's own spin axis projected onto the local horizontal, with a continuous fallback directly over a pole. North is the point — it is the one heading available that does not move with the craft, so "level" cannot quietly become "level, and also pointing where the rocket points".

Low means 3% of the body's own radius, so it means the same thing on a moon as on a planet, and the two frames are slerped through `m_groundCamBlend` so crossing the threshold levels the view rather than snapping it. Verified numerically: the frame's +Y is the radial to 9e-8, its −Z is north, and a half blend is a unit quaternion sitting symmetrically 29.4 degrees from each of the two ups.

The navball still reads the craft's attitude, obviously — that is an instrument, and reading the vehicle is its whole job.

### F1g — the cargo that flew away (done)
The crates on the belts were being flung hundreds of metres off, and the diagnosis was the second of the two the report offered: **the wrong tick.**

Every pose on screen is INTERPOLATED between the last physics tick and the next — it has to be, because the planet these belts are bolted to moves 29.78 km/s around its star. The belt deck went through that path like every other mesh. The cargo did not: it read the belt's raw TICK pose. The two are up to one full physics step apart, and one step of Terra's orbit is **595.6 m**. So the crates were drawn where the belt had been (or would be) one tick away, in whatever direction the orbit happened to point that day, snapping back to zero at every tick boundary.

The cargo now reads the same `mix(previous, current, alpha)` the deck is drawn with, so it can only ever be ON the deck. Two smaller things fell out of the same read: the crates' phase used `presentSeconds()` raw, which is quantised to the tick and made them advance in 5 cm hops instead of gliding (it now carries the sub-tick residue), and the 30 cm ride height was being added to a BODY-frame position after it had already been rotated into the world — a 0.27 m error, in the wrong direction, on top of everything else.

The general rule this cost us is worth stating plainly: **anything positioned relative to a rendered entity must use that entity's rendered pose, not its simulated one.** At planetary velocities the difference is not a rounding error, it is half a kilometre.

### F1h — belts are parts, and machines have mouths (done)
**Everything on a conveyor is now a `.swpart`.** The deck was procedural geometry built in C++, which meant the one thing in the factory nobody could redraw in Part Studio was the thing you look at most. Two definitions replace it:

* **CV-1 Belt Segment** (id 106, category `conveyor`) — one 2 m tile of deck, rails and roller. The game reads its LENGTH and its DECK HEIGHT off its own collider box and tiles it along the path, stretching each tile so the run divides exactly — no gaps, no overlaps. Author a longer CV-1 and you get fewer, longer tiles with no code change; raise its deck and the cargo rides higher, because the crate's height is that measurement and not a constant.
* **CR-1 Cargo Crate** (id 107) — what rides the belt, tinted per resource.

The crate needed a third family. It is not a vessel part (it must not appear in the VAB) and it is not a building (you do not plant crates), so `PartDefinition` gained `prop`: authored geometry the GAME places, never the player. `isVesselPart` / `isBuilding` / `isProp` now partition the catalogue, and `PartsTests` asserts the partition is exactly that — no definition in two families, none in none.

**And machines have MOUTHS.** `NodeType` gained `ConveyorIn` and `ConveyorOut`. A belt does not attach to a hull the way a fin does: it arrives at a specific place, facing a specific way, and it has a DIRECTION — goods leave through an out and arrive at an in. Making that a node type rather than a convention puts the mouths on the geometry, in the tool, where they belong: Part Studio has `+CONV IN` / `+CONV OUT` buttons, draws the two in different colours (amber in, orange out — the direction is the thing you have to get right), snaps them to the hull with the existing SNAP SURF, and its type cycler walks all four types instead of toggling two.

The shipped machines declare theirs: the miner ships out, the silo and the hub only receive, the refinery does both, and the belt segment itself has one of each — a belt is a chain link. `PartsTests` checks that table, and the existing "every node sits ON the collider surface" test now covers the ports too, which is how the first placement attempt was caught sitting 2.2 m inside the miner.

Belts run **port to port**, not centre to centre, and each machine is YAWED so its mouth faces the belt it feeds — `yawToFace` solves for the spin about the local vertical that points a model direction a given way on the ground, which is the same arithmetic F2's placement cursor needs when the player rotates a building. Re-author a mouth in Part Studio and the machine re-aims itself. Measured on the shipped outpost: 31.76 m and 33.68 m of deck, 16 and 17 tiles at 1.98 m, stretch 0.99, and no part of either deck below the ground it crosses.

### F1i — the blinking belts (done)
The conveyors and their cargo vanished and came back on a regular beat. One root cause, showing up twice.

**`ItemLinkComponent::flowUnitsPerSecond` was a SNAPSHOT of the last tick**, and a snapshot of that link is meaningless. The belt is rated 3 units/s and the mine behind it makes 0.85: the link empties its source on one Logistics tick and finds it bare on the next, so the raw rate alternates between "everything" and "nothing" at 10 Hz. Everything reading it flickered in step — and the cargo spacing reads it, which makes this a RENDERING input, not just a statistic.

It is an exponential moving average now, with a four-second memory, dt-weighted so a bulk-catch-up step longer than the memory simply lands on the instantaneous value (which over that step IS the average). Measured on the shipped chain: it settles at 0.839 units/s and does not move. `IndustryTests` runs the two lanes at their real relative rates — Automation 5 Hz, Logistics 10 Hz — and requires the reading to stay within a few percent of what the mine actually produces, plus decay to zero when the source dries up. (Getting that lane ratio wrong is its own way of lying about a factory: the first draft of the test measured against the wrong clock and "proved" 1.7.)

**And the DECK was behind the same gate as the cargo.** A belt is a structure — it exists whether or not goods are on it — but the whole conveyor sat after the `flow <= 0` early return, so a single dry tick took the rails with it. The deck is drawn unconditionally now; only the crates depend on the flow.

### F2 — Ground build mode (done)
**You build a factory by walking around it.** Arm a machine in the `F` catalogue, look at the ground, click. The ghost lands where your gaze meets the heightfield — the real one, marched, not a plane at sea level — inside a thirty-metre reach you have to walk to extend. The wheel spins it, `R` razes what you are looking at. Free, for now: matter cost waits for F5/F6, when there is a fabrication loop to close.

**The rules come from the .swpart, and there is one copy of them.** `build::validatePlacement` answers "may this stand here" from fields the file already carried: on land (`terrainElevation` clamps at sea level, so an ocean is water, not flat ground at altitude zero), flat enough (`maxSlopeTangent`, measured across the building's OWN footprint rather than its centre pixel), on the ore (`minOreDensity`, from the same analytic field the miner will be paid on), and with room for it. Three callers need that answer — the ghost, the commit, and the scene builder — and three plausible implementations would be three different games, so it is one tested function in the engine. `placeBuilding` went the same way: the starting outpost and a machine you put down are now made by the same code, so there is no scripted variant that quietly differs.

Writing the test caught a real one. `groundDistance` measured the separation of two footprints through an f32 dot product, and two buildings fourteen metres apart on a 6,371 km sphere subtend 2.2e-6 radians — a dot product of 1 - 2.4e-12, which in f32 is exactly 1.0. Every building overlapped every other one, everywhere, and the ghost would never have turned green. It goes through the chord in f64 now.

**Conveyors are buildings, and the network is DERIVED.** You place CV-1 segments one at a time like anything else. What turns a row of them into a working link is not an intention the game recorded — it is that their `conveyor-out` and `conveyor-in` mouths MEET. `factory::traceConveyorChains` walks that graph after every build and demolition and rebuilds every complete run it finds: machine, some belts, machine. Demolish a tile from the middle and the chain simply is not there next frame, because the ports no longer meet. It is pure graph work over body-frame positions, so it is tested without a world: a run that connects, a run with a hole, a stub, a ring (which must terminate, not hang), two independent runs, and two machines set mouth to mouth with no belt at all.

Two constants earned their comments. The port snap has to stay UNDER one segment's length, or removing a tile leaves a gap the snap bridges anyway and the run keeps conducting through a hole you can see through — 1.5 m against the shipped 2 m CV-1; at 2.5 m the hole test failed exactly as it should have. And belts are exempt from the overlap rule in both directions: a conveyor is a linear structure a metre wide, a footprint circle is the wrong shape for one, a row of 2 m segments overlaps itself by that measure, and a belt has to reach a mouth that sits inside its machine's own circle. Without the exemption you cannot lay a belt at all.

The starting outpost is now built the way the player builds: 3 machines and 24 belt tiles, placed by the same call, with the two links derived from the ports rather than declared. Measured: 9 tiles from the mine to the smelter, 15 from the smelter to the silo, worst mouth gap 0.57 m against a 1.5 m snap.

### F2a — the ghost that lagged the crosshair (done)
The same bug as the conveyor cargo, for the same reason, in a new place — so this time it became a function.

**The ghost was placed from the planet's TICK pose while the camera sits in the RENDERED world.** One physics step of Terra's orbit is 595.6 m, and the error resets every tick, so the ghost swung hundreds of metres away from where the player was aiming and snapped back, over and over. Worse, the aiming RAY had the same fault at its origin: the eye was transformed into the body frame through the tick pose, so the 30 m ray could start half a kilometre from where the player actually stood.

`bodyRenderPose` now answers "where is this planet, as it is being drawn" — interpolated position, f64 spin at the frame's alpha — and the cursor and the ghost both go through it. An audit of the remaining `spinRotation` calls says the rest are right where they are: the physics systems want the tick pose (that is the pose they are simulating), the terrain patch's LOD centre is self-consistent with the tick craft pose it is derived from, and the launch pad is a one-off spawn.

**And the cursor was aiming a whole frame early.** `updateBuildCursor` sat up with the key handling, before `m_simulation.advance` and before `updateChaseCamera` — so it cast its ray from LAST frame's camera at a planet that had not moved yet. It runs last now, after the camera, which is the only place it can run: it is a query against the rendered world, and the rendered world is not finished being decided until then.

The general rule, restated because it has now cost two features: **anything positioned relative to a rendered entity must use that entity's rendered pose, and must be computed after that pose is known.**

### F2b — the belt tool: pick an output, pick an input (done)
Laying a conveyor tile by tile was the wrong verb. The player's operation is **"feed THIS from THAT"** — the segments are the RESULT of that decision, not its input.

So with a CV-1 armed, the cursor becomes a two-click tool. Look at a machine and click: that is the source, and it must have a `conveyor-out` mouth or the pick is refused with a reason. Look at another and the whole run appears as a green ghost, tile by tile, with the segment count and the destination's name on the HUD. Click again and it is built — and it is **carrying goods from that frame on**, because the network is derived from where the ports ended up and there is nothing else to tell. `R` cancels a pending pick.

`planBelt` is the single routine behind all three uses: the preview, the commit, and the scene builder's starting outpost. What you are shown in green is what `placeBuilding` will be handed — the same tiles, from the same call — so a belt cannot come out different when you click, and the starting outpost gets no shortcut the tool does not have. It refuses a run over 250 m (a belt across a continent is a thousand entities and almost certainly a misclick) and reports the FIRST reason any tile could not be laid, so "UNDERWATER" means one specific tile in the middle, not a vague failure.

What did NOT change is the point: the tiles it lays are ordinary CV-1 buildings, indistinguishable from ones placed by hand, and `traceConveyorChains` still derives the link from geometry afterwards. The tool is a convenience over the model, not a second model.

### F3 — Generic production and ENERGY (done)
**Sunlight is geometry, and a factory runs on it.** `factory::solarFactor` is the whole premise: Lambert against the REAL star direction at the site's REAL local vertical, zero at and below the horizon, zero behind any body that gets in the way. It is the ships' `SolarCharge` idea generalised and made pure — no world, no components, no clock — because a fourteen-day lunar night has to be testable in a millisecond. One subtlety earned a comment and a test: the grid hands it EVERY gravity source including the body the site is standing on, and that is safe, because with the star above the local horizon that body's closest approach to the ray lies behind the panel. Night is the elevation term's job, and only its job.

**A brownout stops the factory at a readable place.** `allocatePower` serves strict priority BANDS and splits proportionally only inside the band that runs out; everything below it gets nothing. The default order is `defaultPowerPriority`: hub and beacon first (keeping the lights on is cheap and keeps you oriented), then miners (ore keeps overnight), then refineries, then fabricators. It is a per-building field, cycled from the `E` panel, so a player who wants their electrolyser to win can say so.

**`PowerGridSystem` runs FIRST in the Automation lane**, ahead of the executor — the other order would run the factory on last tick's weather. Per site it sums what the panels are actually making, subtracts demand, charges or discharges the banks (bounded by their rated kW and by what they actually hold), and writes each building's `satisfaction`. `BatteryComponent` plus an `ElectricCharge` inventory is the whole battery: 1 unit is 1 kJ, so kW·s is units directly and there is no conversion constant to get wrong. **BT-1 Battery Bank** (id 108, 150 m³ = 1 GJ) ships with the starting outpost, half charged — which makes the first night a decision rather than a scripted blackout.

**The executor distinguishes its four states honestly.** `NO POWER` now means STOPPED — a machine on half a grid is `RUNNING`, at half speed, and telling the player otherwise sends them hunting a fault that is not there. The material limit and the electrical limit are computed separately for exactly that reason.

**Logistics links carry more than one good.** `ItemLinkComponent` is an array of four channels (source, resource, max rate, measured flow), because the fuel chain demands it: the synthesiser takes hydrogen AND oxygen, from the same electrolyser, and a machine has ONE output mouth. So a belt now carries everything its source makes, one channel per product, and `linkFlowFrom` gives the renderer that belt's own total (several runs can feed one machine). A single-link component made the chain the whole of F3 exists to enable unbuildable.

**The `E` panel** is where a building's recipe is chosen: stand within 18 m on foot, press `E`, and you are looking at the machine's front plate — state, its share of the grid, the site's books, what is in the bin, and every recipe its category can run, each read as a sentence (`1.00 WATER > 0.11 HYDROGEN + 0.89 OXYGEN   480 KW`). Nothing on it is cached; a panel that agreed with the simulation only at the moment it opened would be worse than no panel. Choosing a recipe moves three things together — what the machine runs, what it draws, and what its outgoing belt carries — because any two of those three alone is a site that lies about itself.

**The fourteen-day night, measured.** `PowerTests` spins an equatorial lunar outpost (180 kW of panels, a 2 GJ bank, a 30 kW ice mine) through a full 28-day rotation and pins the shape of it. The banks fill by day and reach NIGHTFALL only half full — a panel makes 180 kW × cos(elevation), so it drops below the mine's draw about ten degrees before the sun actually sets, and the last three quarters of a "daylight" day is already run on stored charge. After that the arithmetic is exact: 1.03 GJ at 30 kW buys 41,041 units of ice at 1.2 u/s and not one unit more, to within a single tick. By midnight the banks are flat, the mine reads NO POWER and the site produces 0 kW; it resumes at dawn. And the same night run at one-minute ticks and at one-hour ticks reaches the same books, within the two ticks that straddle dawn and dusk — which is the whole reason rates are per-second and the lanes catch up in bulk.

**The fuel chain works end to end**, and is HYDROGEN-limited, which is a real fact about it rather than a bug: electrolysis splits water 0.111/0.889 while the synthesiser wants 0.25/0.75, so oxygen piles up in the synthesiser's bin and a player reading STARVED off its panel is being told the truth about their factory's shape. The test weighs every gram from ice to propellant against the losses the recipes declare.

No perf regression: 600 buildings through the grid AND the executor cost 24 µs a tick — 0.012% of the Automation lane's 5 Hz budget.

### F3a — the grid is the cables, and the menus are readable (done)

**One grid per site was a placeholder, and it read like one.** Every building you planted near the hub was silently wired to it. Electricity is a network, and a network you cannot see or route is a rule rather than a mechanic — so the grid is now the CABLES. `factory::traceGrids` is a union-find over the cable graph; a grid is a connected component; `PowerGridSystem` keys its books on `power.gridId` instead of `building.site` and never walks the graph itself. A solar farm you forgot to hook up powers nothing, visibly, because there is no wire.

Two rules give the network its shape, and they are the whole design. **A building takes ONE cable. A pole takes as many as you like.** That is what makes routing a decision — you cannot daisy-chain a factory, you have to distribute it — and it is what makes PL-1 worth building. The starting outpost gets no exemption from the rule it teaches: it ships with one pole in the middle of the yard and seven spans radiating from it, because seven machines that each take one wire cannot form a chain. Measured on the shipped site: spans of 17.3 to 50.4 m, sag 0.8 to 2.3 m, one grid.

Grid ids are the SMALLEST member index rather than whatever the union-find happened to reach first, so demolishing an unrelated span on the far side of the base does not renumber the grid you were looking at.

**`NodeType::Power`** joins the two conveyor mouths: where a cable hooks on, authored on the geometry in Part Studio. A definition without one takes no cable at all, which is the honest way to say "this does not touch the grid" — a belt tile has no power node and can never be wired. The seven existing buildings got theirs by raycasting their real colliders, the same throwaway-tool method the conveyor ports used.

**A cable is DECLARED, not derived**, and the difference from the belts is real. A belt is a row of tiles you can see, and what they connect follows from where their mouths ended up — the geometry IS the statement. A cable has no intermediate object, so the span is what gets stored (`PowerLinkComponent`, two entities) and the GRID is what gets derived from it. `rebuildPowerNetwork` runs after every build, demolition and load: it drops spans whose endpoint has gone, renumbers the components, and re-hangs every surviving curve from its endpoints' current nodes, so a wire can never be left pointing at where a machine used to be. The curve itself is a parabola — the small-sag limit of a catenary, indistinguishable from one at the sags a base uses — drooping along the LOCAL vertical, which on a sphere is not a world axis. Cables on a grid that is short dim to a dull red: the cheapest possible answer to "why is my smelter stopped" that does not involve opening a panel.

**The menus were unreadable, and the cause was that each one picked its own colours.** The build menu and the machine panel were each choosing near-black rows on a near-black translucent panel. There is now ONE palette (`hud::`) and one set of rules behind it: a panel is nearly opaque (a list read through a landscape is not a list), a row is clearly lighter than its panel and every other row lighter still, selection is a hue change because brightness is already carrying the zebra, and secondary text is a desaturated blue rather than the panel colour with the alpha turned down — which is exactly how the old menus disappeared. Rows light up under the cursor, every row carries a category colour chip, and the machine panel's state gets its own coloured tab because it is the one thing you should be able to read from across the room.

### F3b — the menu that went blank (done)
The build menu rendered as an empty box, and flickered. One root cause, and it was in the renderer, not the menu.

**The HUD was sorted by mesh POINTER.** `prepareBatches` did `std::sort(m_hudIndices, by mesh)` with the comment "their draw order has no meaning" — true when the HUD was glyphs plus a few isolated markers, false the day panels got rows. A panel and the rows on it are the SAME unit-quad mesh, so the sort scrambled them against each other; the glyph meshes sit at unrelated addresses, so whether text landed above or below its own panel depended on where the mesh table happened to put them. Adding PL-1 and CW-1 to the catalogue moved the table, and the text went under the panel. `std::sort` being unstable, the panel-versus-row order was also decided afresh whenever the frame's content changed — a distance readout ticking over was enough — which is what made it flicker rather than simply being wrong.

The fix is a LAYER, and it lives in a pure function: `ui::hudDrawOrder` (UI/HudOrder.hpp). Backgrounds are layer 0 and text is layer 1, so a glyph can never be painted over by a panel whatever the mesh table looks like. Within the background layer submission order is preserved exactly — panels, rows and chips overlap on purpose, so their order IS the layout. Within the text layer, grouping by mesh is still allowed and still done, because glyphs do not overlap one another: that is where the batching actually mattered, and a hundred characters of two letters is still two draw calls.

`HudOrderTests` owns the rule, and every case in it is written to fail against the old comparator — verified by re-running the old sort against the same assertions: text-after-panel `NO`, frame-to-frame stability `NO`, both `yes` after. The bug was invisible in review and glaring on screen, which is exactly the kind a test should hold.

Two layout faults surfaced once the pixels were actually checked (by rendering the menus' real arithmetic — real catalogue, real glyph advance — to SVG and looking at them): the machine panel's RECIPE label was drawn straight through the row beneath it, and `"{:.0f} KW"` turned the belt segment's half-kilowatt into `-0 KW`, which states something false with total confidence. Below 0.05 kW the catalogue now says PASSIVE.

### F3c — the hull is authored, machines have several mouths, and the player has a body (done)

**A part's collision hull is no longer inferred from what it looks like.** It was the shapes flagged `collider` — the same primitives that draw the thing — which conflates two jobs. Geometry wants cones, tubes, greebles and forty segments; collision wants as few boxes as will do, because every one of them is tested against every other part. Worse, you could not fix a hull without redrawing the model. So `PartDefinition` carries `hitboxes`: a list of boxes, axis-aligned in the part's own frame — that is what makes it an AABB and what makes it cheap — and their union is the hull. `expandPartHullBounds` (renamed from `expandPartColliderBounds`, because it no longer means what the old name said) and `partsOverlap` both prefer them. **A definition with none falls back to the collider shapes exactly as before**, so every `.swpart` written before this loads and behaves unchanged; the shipped catalogue was then seeded with `hitboxesFromColliders`, which is the same call Part Studio's `FIT HULL` button makes.

**Part Studio edits them like anything else.** `+HITBOX` (fitted to the part's current bounds, because the first thing you want is "that, but tighter"), `FIT HULL`, `HB DEL`, a list column, `G`/`S` to move and size, and `H` to toggle the overlay. Rotate refuses, as it does for nodes — an AABB has no orientation, by definition. The overlay is drawn in a colour nothing else uses: the point of looking at a hull is to see where it does NOT match the model, and one that reads as part of the geometry is one you stop checking.

**Machines have several mouths.** One `conveyor-in` and one `conveyor-out` meant an electrolyser's hydrogen and oxygen were stuck sharing a belt. `PortNode` carries arrays now, `Chain` records WHICH mouth each end used, and the rule is: **one out mouth ships everything, several ship one product each — mouth i carries the recipe's product i.** RF-1 gained a second pair on its side faces, so hydrogen leaves by the back and oxygen by the left, down separate runs, to wherever the player wants them. An IN mouth takes one belt: `traceConveyorChains` claims mouths as it traces, which is what makes a second in port useful rather than decorative — the second run is pushed onto it instead of piling onto the first.

Choosing a mouth needed no UI: the player is already aiming at the machine, so the tool takes the free mouth **nearest their aim**, and clicking the same machine twice lays two different runs. Mouths that already have a belt on them are skipped, which is what makes the second port reachable once the first is wired.

**And the player has a body.** EVA was a capsule primitive — a placeholder visible in the map, in every screenshot from the ship, and to a future second player. EV-1 is an ordinary `.swpart` prop: helmet and visor, life-support pack, orange collar and knee bands, boots. Its ground hull is its own hitbox rather than a constant in the game file, so there is one description of how big the player is instead of two, and the suit is redrawn in Part Studio like everything else.

### F3d — solid objects (done)
The hitboxes stopped being a description and became a fact: **you cannot walk through a refinery, a fuel tank or a power pole.**

`phys::HullComponent` carries a part's authored boxes onto the entity at spawn, so collision never goes back to the catalogue and an entity's solidity is a property of the entity. `HullMoverComponent` tags the things that get pushed OUT of the others — the player is one, a building is not, because a building does not move for anybody. `HullCollisionSystem` runs LAST in the Physics lane, after the surface anchors and the part attachment: any earlier and it would be resolving against last tick's building positions, which on a spinning planet is 595 m of lie.

**The broad phase is the point.** Every hull carries the bounding-sphere radius around its own origin, so a pair is rejected with one f64 subtraction and one comparison. Measured: 600 solid buildings and one walker cost **10.5 µs a tick — 0.05% of a 50 Hz step**, with 601 of 601 pairs rejected before anything expensive happened. The narrow phase is box-against-box, and only ever for the handful within arm's reach.

**The push is the SHORTEST one, not the deepest.** Pushing along the deepest axis instead would send a player standing beside a wall out through its roof — three metres up rather than ten centimetres back. Pushes from several blockers accumulate per direction rather than summing, so standing in a corner where two walls each want 10 cm is one 10 cm step out and not twenty. And only the velocity heading INTO a surface is removed: walking away is never resisted, or a player could not leave.

All fifteen separating axes are tested, including the nine cross products. Dropping them is the classic shortcut and the classic bug: two boxes can miss on all six face axes and still overlap edge to edge, which is exactly a player wedged into the corner of a diagonal building.

**Belts and cables are not solid**, and that is a property of the CATEGORY (`parts::isSolid`) rather than of whether someone remembered to leave their hitboxes out. You step over a conveyor deck and duck under a wire; they are the two things a player walks among most, and making them obstacles would turn a factory floor into an assault course. They still HAVE hitboxes — the renderer and the build validator want them. Solidity is a separate question from shape.

**And `E` asks a ray now.** The old rule took the nearest building CENTRE within 18 m, which is wrong twice over: a 16 m solar field you are standing on has its centre 8 m away and loses to a silo behind your shoulder, and a machine you are touching can have its centre out of range entirely. "Near enough and in front of me" is a question a ray answers exactly — against the very boxes you cannot walk through, through the same broad phase.

### F3e — the wall that fired the player off the planet (done)
Walking into a building launched the player about a hundred metres. One line, and it is the same class of mistake that has now cost this project three features.

`HullCollisionSystem` finished by removing "the component of the velocity heading into the wall". That reads as textbook collision response and is completely wrong here, because `DynamicBodyComponent::velocity` is a WORLD velocity: standing on Terra it already carries ~30 km/s of orbital motion around the Sun plus 465 m/s of the planet's own rotation. Projecting THAT onto a wall normal and subtracting it is not a small correction — it is a kilometres-per-second impulse, and the wall works as a catapult.

What would have been correct is the velocity RELATIVE to the blocker, and a blocker's carrier velocity is a question about the planet it is anchored to — which this system deliberately knows nothing about. It does not need to: the walker SETS its tangential velocity from input every tick in the local surface frame, so nothing accumulates, and pushing the position back out is the whole job. **Resolution is position-only**, and the system's access declaration no longer even asks to write velocities.

The general rule, restated because it keeps arriving in new clothes: **on a planet, a world velocity is mostly carrier motion.** Anything reasoning about how fast something moves relative to the ground has to subtract that carrier first, or it is doing arithmetic on 30 km/s it never meant to touch. `HullTests` pins it with a mover carrying exactly Terra's real orbital and rotational velocity: it must be pushed clear by the 10 cm it overlapped, and come out the other side with its velocity untouched to the last bit.

**`F2` draws the hulls.** Green for the things that stand still, amber for the things that get pushed out of them, so the player's own box is never confused with the world's. Belts and cables draw nothing, which is itself the fastest way to confirm they are walk-through. Not a flag behind a rebuild: the hitboxes are hand-authored now, and an authoring mistake and an engine mistake look identical until you can see the boxes.

The overlay reads the RENDERED pose, not the tick pose — `mix(previous, current, alpha)`, the identical interpolation the mesh pass uses. The first version read `TransformComponent` raw and every box swam beside its own machine and snapped back once a tick, because a building's transform is where it was at the last physics step and one step of Terra's orbit is 595 m. **That is the fourth feature this rule has cost** (conveyor cargo, the build ghost, the cables, now this), so it is worth stating flatly: on this project, anything drawn relative to a body on a moving planet must go through the same interpolation as the meshes, and "close enough" is 595 metres.

### F3f — the editor was drawing every box at half size (done)
Hitboxes looked smaller in Part Studio than in the game, and the game was right.

`PrimitiveFactory::makeCube(1)` is a cube ONE METRE ACROSS — its half extent is 0.5, not 1. Drawing a box of half extents `h` with it therefore means scaling by `2h`. The game's `F2` overlay did; Part Studio scaled by `h`, so **every collider overlay and every hitbox in the editor was drawn at half its real size** — the collider overlay since the day it was written.

The consequence is worse than a cosmetic one, because the hulls are authored against that picture: a box drawn at half size is a box you make twice as big to look right. EV-1's hull came out 3.63 m tall around a 1.9 m body for exactly this reason. **Hulls authored before this fix are likely about twice the size they should be** and are worth re-checking against the now-honest overlay.

The 2 is a named constant next to the mesh it belongs to, with the reason written down, so it is not re-derived at the next call site.

### F3g — E stopped working, and the reason was 595 metres again (done)
The machine panel opened on nothing. Not intermittently — never.

`hullUnderCrosshair` cast an 18 m ray from the camera, which sits in the INTERPOLATED world, at each building's `TransformComponent`, which is its TICK pose. One physics step of Terra's orbit is 595 m, so the boxes were never within thirty times the ray's length of where the ray was looking. **Fifth time.** The rule now has its own line in this file and in the code: on this project, anything that compares a camera-space quantity against a body on a moving planet goes through `mix(previous, current, alpha)` — the same call the mesh pass makes, not something that merely looks equivalent.

Two neighbours of the same bug went with it:

**E now resolves AFTER the camera.** The key handler runs before `updateChaseCamera`, so opening the panel there aimed a ray from where you were looking one frame ago — the exact reason the ground cursor was moved last. Pressing E sets a flag; the flag is answered next to `updateBuildCursor`. Closing still happens immediately, because closing needs no camera.

**Hulls are rebuilt after a load.** They are derived from the `.swpart` and deliberately not saved — a hull in a save file is a second copy of the model's own answer, free to drift the moment a part is redrawn. But nothing rebuilt them, so a loaded world had no solid objects at all: you walked through everything and E hit nothing. `rebuildHulls` now runs beside `rebuildConveyorNetwork` and `rebuildPowerNetwork`, which is where it always belonged.

### F5 — the VAB, and the bridge from factory to rocket (done)
Until now the factory made metal that nothing consumed, and the hangar made rockets out of nothing. F5 joins the two ends: **a design is a file, a rocket is a bill of materials, and the metal that pays it arrives on a belt.**

**A design is a file.** `Gameplay/Blueprint.hpp` gives the hangar's part list the same contract `.swpart` and `.swrecipe` already have — a `.swship` JSON, a per-file loader that refuses garbage, a catalogue read from `Assets/Ships` at startup, stable ids. The record carries the JOINTS as well as the poses: parent index, parent node, child node. A file that forgot them would load as a pile of parts flying in formation, which looks right until something pushes on it. SAVE in the hangar writes one and registers it live, so a design is orderable at the VAB without a restart; BUILD stays beside it as the one-click test shortcut it has always been.

**A rocket costs what it is made of.** `partCost` splits a part's dry mass into copper and iron by what the part IS — a battery is mostly plate, a solar wing mostly conductor, an engine a steel bell wrapped in pumps and harness, everything else steel with a loom run through it. Iron is computed as the REMAINDER, never as an independent number, which is what makes iron + copper equal the dry mass to the last gram whatever anyone does to the fractions later. It is also the first reason copper exists in the chain at all.

**The VAB is a machine like any other.** `AssemblyComponent` holds an order and the metal paid so far; `AssemblySystem` sits in the Automation lane beside the recipe executor, on the same power and the same bulk catch-up. It pours iron and copper TOGETHER in the bill's own ratio — pouring the iron first would let a hall with no copper drain every smelter in the base and then stop one gram short, which reads as a supply fault where there is none. Room for the finished hull is checked before a gram is worked, for the same reason the executor reserves its output volume: a machine that consumes its input and then finds nowhere to put the result has destroyed matter.

**A crate has no identity, so the name travels beside it.** One `Resource::Vehicle` unit is one rocket, and a unit cannot say which design it is. The hall keeps a small FIFO of names; the pad reaches it through the belt's own link channel, which already records the machine at the far end. It works because a belt is first-in first-out and the crates on one belt all came from one hall — route vehicles through a silo and the identity is lost, which is the honest consequence of a unit not being a thing.

**The pad is a building now.** `LP-1` has a deck, a tower, a power node and two mouths — one for vehicles, one for fuel. `instantiateBlueprint` takes a pad and stands the vessel on THAT pad's deck, co-rotating with the planet under it; with no pad it still falls back to the surveyed place 120 m east of the hub, which is where new vessels have always appeared. The deck height is read from the hull boxes UNDER THE PAD'S AXIS, not from the bounding box — a probe caught the first version standing the rocket 18 m up, on top of the service tower.

**And the belt out of a hall carries rockets.** A hall runs no recipe, so the "a silo ships what it is holding" rule would have exported the iron it was standing on: the belt to the pad running backwards, carrying metal away from the machine that needs it. A hall's product is the one thing it makes. `CR-2` is the cradle they ride in, four times the length of a CR-1 crate, because a belt of rockets should not look like a belt of ore.

The starting outpost ships with the whole loop standing: a VB-1 at 75 m east, an LP-1 at 120 m (where the spawn place used to be computed), the 21 m belt between them, a second PL-1 because a 120 m span cannot reach the launch complex in one hop, and enough metal in the hall for the first hull. `BlueprintTests` weighs all of it — the bill against every part in the catalogue, the round trip through disk with its joints, the ledger across a hall that keeps building, the stall with no copper, the block with no room, and one crate riding a link from hall to pad.

### F5a — one rocket per pad, trajectories that reach, and warp to ten million (done)
Three things the launch loop needed the moment it worked.

**A pad holds ONE rocket.** The second crate to arrive was unpacked on top of the first, which put two vessels in the same cubic metre and left the hull solver to resolve it — by throwing one of them off the pad. `padIsOccupied` asks the vessels, not the hulls: one distance per vessel against the deck's own footprint plus sixty metres of launch clearance, cheap enough to ask every frame. An occupied pad simply does not unpack; the crate sits on the belt, which is where a crate belongs, and the panel says `DECK OCCUPIED` rather than leaving the player to wonder.

**A trajectory now ends where the physics ends it.** The prediction ran for a fixed six days. On a parking orbit that was ninety-six revolutions drawn on top of each other; on a heliocentric transfer it was one and a half degrees of arc — a stub hanging in space. A segment now runs until something HAPPENS to it: an impact, an encounter, an escape, or a full revolution that meets none of them (`SegmentEnd::Closed`, the line joining up). `horizonSeconds` survives as a hard cap for the one genuinely unbounded case and as the maneuver planner's way of asking for a plan that stops at the node. The scan step follows from the same idea — a fraction of the orbit's OWN period rather than of the horizon — so a 90-minute parking orbit and a two-year transfer are scanned at the same angular resolution. It is also four times cheaper than the old scan: 1.5 ms per refresh, measured.

**...and it is a LINE.** Dots hide the one thing a map is for: you cannot tell a plan that ends from a plan whose dots have spread out, and at map zoom they always spread out. Each sample pair is now a stretched box — same draw item, batched by mesh, and the end of the line means something.

That change immediately exposed a second fault, which a probe measured rather than a review guessing at: sampled evenly IN TIME, a Terra-to-Luna transfer (e = 0.965) puts one 107° chord across the periapsis, and the drawn line passes **802 km below Terra's surface**. Nine of its ten days are spent near apoapsis. `kepler::timeAtArcFraction` parameterises the arc by eccentric (or hyperbolic) anomaly instead, which is very nearly constant in arc length: the same 320 samples then keep the line 6 766 km from the centre against a true periapsis of 6 771 km, and the worst chord spans 8.5° instead of 107°. `PhysicsTests` measures both and holds the ratio.

**Warp goes to x1 000 000 and x10 000 000.** At x100 000 a Mars transfer is still three real hours. The two new rungs are gated on altitude like every other — a million beyond 100 000 km, ten million beyond Terra's sphere of influence — because a million times real time moves a craft 30 000 km per rendered frame, which near a planet is a jump straight through it. What makes the rungs safe is that above physics warp nothing is integrated: every orbit is analytic and the rate-based lanes bulk-consume whatever interval they are handed. A test proved that and caught the bug that would have made both rungs do nothing: `Simulation::setTimeScale` clamped at 100 000, so the button would have worked and nothing would have happened.

### F5b — a line one pixel wide, and space bar means jump (done)
**The trajectory line was 274 pixels thick.** Not everywhere — only where it mattered. A line is drawn as a box stretched between two samples, and a box has ONE width; sized from the distance to its midpoint that is correct until a single chord spans a huge depth range. One chord of Terra's own orbit is four million kilometres long, and the camera sits ON that orbit, because the camera is at Terra. The box was sized for a midpoint two billion kilometres away — 3 618 km wide — and it passed 12 000 km from the eye. That is the grey band across the planet in the screenshot.

A piece is now SPLIT until its near and far ends are within a factor of two of each other, and each piece is sized from its own closest approach. Measured across every orbit scale in the game and every zoom level the map allows: **1.73 pixels, everywhere**, for ten extra draw items on the one orbit that needs them. Almost every chord passes the test first try; only the handful genuinely near the camera subdivide.

**Space bar is the action key.** It was pausing the simulation — the one thing a pilot's thumb should never do by accident, on the key every game in the genre uses to stage. It now fires the next decoupler in flight and JUMPS on foot; `Z` stays as the second name for staging. Pausing moved where it belongs: x0 is a warp rate, so `,` at x1 stops time and `.` starts it again. The HUD reads `WARP X0`.

The jump is `phys::surfaceJumpVelocity`, in the engine, because it is THE CARRIER-VELOCITY RULE in three lines: a suit standing on Terra is already doing 30 km/s around the Sun and 465 m/s with the spin, and a jump changes exactly one component of that — the radial one, measured against the ground underneath. It SETS that component rather than adding to it, so a jump taken while already rising cannot be stacked. Four and a half metres per second is a one-metre hop on Terra and a six-metre float on Luna, from the same suit and the same key, because the height is the planet's business.

### F5c — the maneuver step is a ladder (done)
A node spans five orders of magnitude. Trimming a rendezvous is a tenth of a metre per second; leaving Terra for Mars is three and a half kilometres of it. With one step size and a x10 modifier, the second job was four hundred taps.

`space::maneuverStep` is the whole change: **ctrl+shift x1000, alt x100, shift x10, nothing x1, ctrl x0.1**, and the node's TIME moves by the same factor — ten seconds at the base rung, because a burn a hundred times bigger is one you are planning a hundred times further out. One tap of the coarsest rung is a real transfer burn; four of them clear Terra escape from low orbit.

It lives in the engine, and is tested, for one reason: **Control means two different things depending on whether Shift is with it.** Alone it is the FINE step; with Shift it is the coarsest one. Read the flags in the obvious order and a player reaching for a kilometre per second gets a tenth of one — so the combination is tested before either key on its own, and `SpaceTests` checks precisely that inversion rather than just the five rungs.

The armed step is on the HUD, under the node's vector, and changes under the player's thumb as they hold the modifier. A ladder you cannot see is a ladder you have to memorise.

### F5d — dragging the node with the mouse (done)
The keys move the node in fixed steps. The mouse moves it where you point: grab the violet marker with the left button and the node's TIME follows the pixel under the cursor, the burn sliding round the orbit while the planned trajectory redraws under your hand. It is the only way to answer "where on this orbit should I burn?" by looking rather than by counting taps.

**The pre-burn plan is drawn whole now.** It used to stop AT the node, on the reasoning that the post-burn path took over there — but a line that ends at the thing you are dragging gives you nowhere to drag it to. Drawing the current orbit entire and branching the planned one off the marker is also what KSP does, and for the same reason. It has a second virtue: the plan no longer depends on the node's time, so the drag cannot chase its own tail.

`space::timeNearestScreenPoint` is the pick, in the engine and tested. It samples exactly as the map draws — spaced by anomaly, not by time — so the moment it returns belongs to the pixel the player is pointing at. **It skips everything with w ≤ 0.** That is the whole reason it is a tested function rather than four lines in the game: a perspective divide behind the camera does not fail, it mirrors the point through the origin, so the far half of the orbit lands on screen looking perfectly plausible and the burn jumps to the wrong side of the planet. `SpaceTests` puts the camera inside the orbit, where half the plan is behind it, and checks the pick stays on the visible half.

Two rules make it feel like a grab rather than a teleport: the pick is refused if the nearest point is more than 0.30 NDC from the cursor — the node is stuck to its LINE, not to the pointer — and a HUD button under the cursor keeps first claim on the click. Measured over a full drag round a parking orbit: worst error 5.8 s against a drawn sample spacing of 17.3 s, and not one backwards jump.

### F5e — a target, and where it will be (done)
Click a body on the map and it is the target. The plan then answers the question a transfer is actually about: **how close do I pass, when, and where will the thing have moved to by then.**

The last part is the one that makes a transfer flyable, and it is the part a naive implementation gets wrong twice over. First in the physics: the body is not where the map shows it now, so the separation is a minimisation of two analytic functions of time, not a distance to a fixed point. Second in the DRAWING: a rendezvous three days out happens 7.8 million kilometres along Terra's own orbit, so a marker at the target's true world position sits in empty space far off the side of a map centred on Terra now. `ClosestApproach` therefore returns positions RELATIVE TO A PRIMARY, exactly as the drawn orbits are, and the map adds that primary's current position — the marker lands on the ring the player is looking at.

`space::closestApproachToBody` scans the plan for the smallest separation and then refines it by golden section. The coarse scan's only job is to find the right valley: measured across encounter phases from a 0.5 km grazing pass to a 130 000 km miss, **256 samples and 4096 return the same answer to the metre**, for sixteen times less work — 175 µs on a parking orbit against 6.4 ms. That measurement is the reason the default is 256 and not a comfortable-looking big number.

It reads the patched plan correctly for free: on an intercept the plan splits at the sphere of influence and the inner patch's primary IS the target, so the minimum comes back measured from the body itself — which is what lets `IMPACT` be printed as `IMPACT` instead of as a distance to a centre. With a node up, a second approach is computed for the post-burn trajectory, so the player dials the burn while watching the number they are dialling it for.

Targets are celestial bodies. Vessel-to-vessel rendezvous needs the other craft's own prediction and is not wired yet.

### F5f — flying the burn (done)
Three things the maneuver node needed once it could be planned properly, and one of them was a bug worth the measurement.

**The remaining dv never came down.** The readout sat at the full planned burn from the first second of the engine to the last, which is the one thing a burn readout must not do — it exists to tell you when to stop. The cause is the obvious formula: the target was recomputed every refresh as "my velocity at the node, on my CURRENT trajectory, plus the planned dv", and burning changes the current trajectory, so the target moved with the ship and the difference stayed pinned. A probe put numbers on it: **100.0 m/s for the whole burn, and 535 m/s while merely coasting toward the node.**

Near the node the plan is now FROZEN — the pre-burn trajectory and the dv vector, captured once — and what remains is the plan minus what has actually been applied. "Applied" is measured against the COASTING velocity from that frozen plan, which is what subtracts gravity's own contribution: a two-minute burn in low orbit picks up a kilometre per second of it, and counting that as thrust would have the readout reach zero with the burn half done. Same probe, same burn: **100 → 3.9 m/s, hitting zero exactly as the last metre per second goes in.** `space::remainingBurn` is the engine function, and `SpaceTests` flies a real 100 m/s burn through a real integrator — computing the naive formula alongside and asserting it stays pinned, so nobody can quietly simplify back to it.

One vector now feeds the readout, the navball marker and the autopilot, so the three can never disagree.

**A NODE button on the SAS row.** A burn is almost never prograde — a plane change is normal, a circularisation is prograde only by accident — so flying one by eye means chasing a marker across the navball with the throttle already open. The mode points at the remaining burn, which shrinks as it is flown, and greys out when there is no node.

**A WARP TO NODE button**, in the map and in the cockpit, stopping one minute short. The rung is chosen so a real second never advances more than half the time left, so the approach decelerates by itself and lands at x1 — the overshoot a fixed ladder plus human reaction time cannot avoid.

### F6 — real aerodynamics (done)
The old model was one number per vessel: `ballisticFactor`, the sum of a hand-typed `Cd*A` over the parts, divided by mass. It could not distinguish a rocket flying nose-first from the same rocket flying sideways, it produced **no torque**, so no fin ever stabilised anything, and `PartDefinition::liftCoefficient` had carried the comment "used by the future aero pass" since Milestone 16. This is that pass.

**The split is the design.** The expensive work happens OFFLINE, once per part, and the game only reads a table. `Tools/AeroForge` solves each `.swpart` over 342 wind directions and writes a `.aero.json` beside it — force and moment per direction, divided by the dynamic pressure, so the entries are an area (m²) and a volume (m³) and one table is valid at every speed and altitude. **Force and moment, never acceleration:** an acceleration depends on the whole vehicle's mass, a force does not, so the same fin produces the same newtons on a probe and on a booster and the table belongs to the PART.

**The solver is a rasteriser, and that is why self-occlusion is free.** Looking along the wind, the elements that receive air are exactly the ones a depth buffer keeps — no ray casting, no visibility pass. Two buffers per direction (frontmost and rearmost), and three terms integrated per element:

* **impact pressure**, `Cp = Cp_max·cos²θ`. Dividing the true area by the same cosine leaves ONE factor of it, so the term never blows up on a surface lying along the flow. This is what makes a cone cheap and a flat plate expensive, out of geometry alone.
* **base suction**, the same law with a negative coefficient on the rearmost elements. A blunt tail collects all of it, a boat-tail almost none.
* **skin friction**, flat-plate shear along the local tangent over the wetted area.

Validated in `AeroTests` against published figures: flat plate **1.20** (measured 1.17), sphere **0.60** (0.47 subcritical / 0.92 hypersonic), 14° cone **0.27** (0.25 with base drag), and a body twice as long is not twice as draggy nose-on.

**A probe found the theory's boundary before a player could.** Solved with impact pressure alone, a fin at 10° produced **a thirtieth** of the force it should, and a full set of tail fins still let the rocket flip: measured `+0.80 rad/s²` DIVERGENT with fins at the tail. The reason is not a bug — Newtonian theory counts only the momentum surrendered normal to the surface, which is right behind a detached shock and wrong for a thin surface at a shallow angle, where nearly all the force is circulation, and impact theory has no far side.

So the forge carries the **linear (potential-flow) term** as well: `Cp = k·sin(d)`, compression on the windward face and suction on the leeward one, both pushing the surface the same way, and its area factor cancels the cosine COMPLETELY — which is why it survives to shallow angles where the impact term has already vanished. A plate collects it on both faces and lifts like a wing.

**Which parts get it is a per-part decision, made from proportions.** Applying the linear term everywhere would treble every rocket's drag: a nose cone's flank is inclined 15° to the flow and is not a lifting surface. So the forge measures the part's bounding extents and calls it a WING when the thinnest is under a fifth of the longest, a BODY otherwise — the classic component build-up split, printed by `--report` so it is inspectable. Of the shipped catalogue exactly two are wings: the AV-F1 fin and the SP-2 solar panel. The taper out of the linear term at high incidence **is the stall**, and `AeroTests` asserts the shape of it: near-linear growth to ~12°, then a fall.

**At runtime it is addition.** `VesselAerodynamicsSystem` (Physics lane, after thrust, before the ground) finds the atmosphere, the altitude, the density, the co-rotating air and its wind; makes one dynamic pressure corrected by a transonic Mach curve; then per part reads the table in the part's own frame, scales it by **exposure** — nine rays against the other parts' boxes — rotates force and moment into the vessel frame, shifts the moment onto the centre of mass, and sums. Measured: five tanks nose to tail cost **1.20×** one tank, not 5×; **19 µs per tick** for a seven-part vessel.

**Nothing in the engine knows what a fin is for.** Same vehicle, fins moved: tail **−1.87 rad/s²** into the wind, nose **+4.05 rad/s²** away from it, and an 8° disturbance decays from a 16.8° overshoot instead of ringing — because `aero::dampingMoment` uses the SAME lever arm that produced the restoring moment, so a stable vehicle is damped and an unstable one is neither.

Three supporting changes, each of which was load-bearing:

* **Angular velocity moved from the game's `ShipComponent` down to `phys::DynamicBodyComponent`.** It is rigid-body state, and the thing that most wants to write it now lives in the engine. The RCS rate clamp had to change with it: it caps what the PILOT commands against the rate the vehicle ALREADY had, because the air is under no obligation to respect an RCS rating — clamping the total would have made a rocket quietly stop flipping at 0.8 rad/s.
* **`VesselComponent` gained centre of mass, diagonal inertia and hull extents**, recomputed every tick from the parts AND their fuel. A rocket therefore grows more stable as its tanks drain, which is not expressible with a mass and a drag number.
* **A vehicle turns about its balance point.** `ThrustSystem` shifts the position by exactly as much as the rotation moved the centre of mass. On a 20 m rocket, turning about the root part instead swings the tail ten metres.

The centre of pressure is averaged over the force ACROSS the wind, not the total — weighting by everything puts it on the nose of every vehicle ever built, and a measurably stable rocket then reads as though it should tumble. The HUD shows Q / Mach / AoA and the **stability margin in calibres**, because that is the number a player can fix. A decoupled stage inherits its own `AeroStateComponent` and tumbles away under its own aerodynamics. Parts with no table fall back to the isotropic model — the right answer for a part nobody has run the forge on.

### F6b — the autopilot, after the air (done)
Two faults the aerodynamics made impossible to live with.

**`SAS` was a fourth spelling of OFF.** Button 0 set `m_sasMode = 0`, which is `kOff`; the only thing that ever countered rotation was the `X` key, held. That is fine when nothing is trying to turn you and useless the moment something is — an atmosphere does not get bored. `SasComponent::kStability` is now a real mode: it takes the spin vector down by `angularAccel * dt` per tick and holds at zero. The bound is the point. Measured: a 0.42 rad/s tumble stops in **0.84 s** against a theoretical 0.83 s at 0.50 rad/s²; against a 1.20 rad/s² aerodynamic moment the rate still grows, at **0.70 rad/s²** rather than 1.20. A mode that simply wrote zero would have made the whole aerodynamics pass stop mattering the moment the player pressed a button.

Every autopilot button toggles now, so there is no button whose only job is to mean "none of the others", and `T` cycles OFF → SAS → PGD → RTG → NODE.

**`PGD`/`RTG` ignored the reference frame.** `V` already switched the speed readout and the navball's own prograde markers between orbital and surface-relative; `SasSystem` only ever subtracted the primary's TRANSLATION, never its spin. Measured on a descent at 5 km with 20 m/s over the ground: the orbital and surface retrogrades are **100.8° apart**. Pressing RTG on final approach therefore held the nose a hundred degrees off the marker the pilot was flying to — which is the whole of "landings are practically impossible", and not a tuning problem.

`SasComponent::surfaceRelative` is written by the game each frame from the same flag, alongside `targetDirection`, so one toggle drives the readout, the markers and the autopilot and they cannot disagree. The active frame is printed above the button row, because two buttons that mean different things depending on a toggle elsewhere on the screen need to say which.

### F6c — the ground, and rotation (done)

`SurfaceInteractionSystem` had always been honest about one half of contact and silent about the other. It clamped the lowest point of the hull to the terrain, absorbed the radial impact, and rubbed the tangential slide off under `groundFrictionPerSecond` — and it never once mentioned `angularVelocity`. Before F6 nothing much wrote that field, so the omission was invisible. Once the atmosphere could spin a vehicle, it was the first thing anyone would notice: a rocket lying across a hillside at forty degrees, propped on the corner of its own bounding box, still turning at whatever rate it last had.

**`phys::topplingAcceleration` is the missing statics**, and it is a pure header function so the numbers can be checked without a world. It takes the eight corners of the `GroundHullComponent`, keeps the ones within a size-scaled tolerance of the lowest — those are the support — and asks how far the centre of mass leans past that support's reach in the direction it is leaning. Inside, the answer is EXACTLY zero: the ground holds the body up and there is no torque to explain. Outside, the edge is the pivot and the overhang is the lever, so it topples and accelerates as it goes.

Everything is computed in MODEL space, because a torque is frame-agnostic and the corners are already there. The mass distribution comes from `aero::AeroStateComponent` when the aerodynamics pass has computed a real tensor from the parts and their fuel, and from `aero::boxInertia` of the hull otherwise — so a crate and a rocket both get an answer, and the rocket's is the better one.

`Config::groundAngularFrictionPerSecond` (3/s) is the twin of the linear friction. It is chosen so a vehicle settles in under a second while still leaving RCS enough authority to swing a landed craft slowly: 0.5 rad/s² against a 3/s decay holds about 9°/s.

Measured on a 12 m rocket 2.4 m across, balanced at its middle — statics says it tips once `6 sin(t) > 1.2`, past 11.5°:

| start tilt | after 60 s |
|---|---|
| 0°, 5°, 10°, 11° | unchanged — standing |
| 12° | 91.8° — over |
| 25° | 90.3° — over |
| 45° | 88.1° — over |

A 1 rad/s spin is under 0.1 rad/s in a second and exactly zero after three. An upright rocket left for thirty seconds sits at 6.000 m doing 0.000 m/s and 0.000 rad/s — the regression this pass most had to avoid, since contact now writes to an attitude.

All four results are in `PhysicsTests`, including the pure-function one that asserts a balanced body gets **exactly** zero rather than merely a small number: a rocket that creeps while balanced is a rocket that cannot be left on a pad.

### F6d — the drawn ground and the collider, reconciled (done)
Reported as "in some places the rocket or the player goes through the ground", with a screenshot of a rocket half-submerged in a hillside. Physics was not at fault: `SurfaceInteractionSystem` samples `planet::terrainElevation` exactly, and it samples the footprint's extremes and takes the MAX so an object may rest above the ground but never inside it. The **mesh** was at fault, and in a way that is worth writing down because it is a whole class of bug.

The terrain patch draws a grid of samples with flat triangles stretched between them. The analytic field is a RIDGED fractal — it has creases — so the chord error across a cell falls roughly with the cell WIDTH, not its square. The drawn surface therefore tracks the collider only as well as the grid is fine, and the grid's fineness was being chosen from **sea-level altitude**. Standing on the launch site's 1,100 m plateau, the LOD sized the patch for somebody flying at 1,100 m: a 6.6 km square in 137 m cells. Terra's terrain reaches 9 km, so this was the normal case rather than an edge one.

Measured over three sites and ~250,000 interior samples per configuration, gap between the analytic surface and the mesh's own bilinear interpolation:

| configuration | cell | worst sink | worst hover | rms |
|---|---|---|---|---|
| sea altitude 1,100 m → extent 6,600 m, 96 cells | 137.5 m | **8.77 m** | 17.72 m | 2.30 m |
| ground altitude 0 m → extent 1,500 m, 96 cells | 31.25 m | 1.25 m | 3.85 m | 0.38 m |
| ground altitude 0 m → extent 1,500 m, 192 cells | 15.62 m | **0.50 m** | 2.30 m | 0.14 m |

The same probe falsified the obvious first guess. The patch builder already band-limits its octaves to the cell size, so the suspicion was that it was keeping detail it could not represent — but capping the stack at 14 or 13 octaves moved the worst sink from 1.19 m to 1.14 m and 1.19 m. The error does not live in the finest octaves; it lives in the creases of the mid-band ones, and only a finer cell touches it (31.25 m → 1.19, 23.4 → 0.79, 15.6 → 0.46, 11.7 → 0.36).

So: the LOD reads `distance - (bodyRadius + terrainElevation(centreDir))`, one extra heightfield sample per frame on a direction already computed; and `kCells` stopped being a constant — 192 at landing extent, 128 in the middle, 96 for the patches measured in hundreds of kilometres, where nothing is touching anything. The landing patch costs ~37,000 heightfield evaluations per rebuild instead of ~9,400, on a worker thread, at most once a second.

`TerrainTests` pins the contract rather than the numbers: a 15.6 m grid follows the collider to under 0.6 m, a 137 m grid does not, and halving the cell may never make the agreement worse. That last clause is the one that would catch a future LOD change quietly going the wrong way.

### F6e — ground you can read (done)
Three additions to the terrain patch, all baked, all on the worker thread that already rebuilds it.

**Relief shading.** The patch's normals were honest and its lambert was correct, and rolling ground still looked like a painted sheet: nothing cast, nothing pooled, and a dip and a rise of the same slope were the same colour. Two terms fix it, both computed on the height grid already in hand — no extra heightfield evaluations, so the shading cannot disagree with the surface it shades. A CAST SHADOW marched toward the sun a cell at a time, softened by the distance to the blocker so a far ridge throws a vaguer shadow than a near one; and a SKY OCCLUSION term, the same march over six azimuths with no sun in it. Baked into the albedo so they multiply the shader's lambert rather than replacing it. The sun direction is captured in the body frame when the job is submitted: Terra turns 0.004° between rebuilds, so a one-rebuild-old shadow map is a correct one.

Measured: 21 ms for a 192-cell grid; the term runs 0.40–1.00 on Terra's roughest ground (found by scanning 4,000 directions for local roughness) and is **flat at 1.00 on the launch plain**. That last number is the useful one — it says the launch site genuinely has under a metre of relief per fifteen, and that no amount of correct shading will make it read as textured. Which is what the plants are for.

**Plants.** ~5,600 tufts within 120 m, baked into the patch: one mesh, one draw call, no collision, rebuilt with the ground. Three tapered blades per tuft, both windings so a blade is never invisible from the side the culler happens to be on.

The scatter is anchored to the PLANET, not the patch. Each tuft's lattice cell is `floor((anchorU + u) / spacing)` where `anchorU`/`anchorV` are the patch centre's absolute plate-carrée coordinates in metres — so re-centring the patch under a walking player leaves the grass exactly where it was growing. A field that reshuffles on every terrain rebuild is worse than no field.

Where they grow is read off the ground rather than declared: the palette's own greenness (`g - (r+b)/2`) sets the density, the interpolated normal rejects anything steeper than about 28°, and anything at or below a metre of elevation is surf. Nothing has to be kept in step with the biome table because nothing duplicates it. Density thins with distance — cover within 25 m, texture beyond — and the tufts inherit the ground's baked shadow along with its colour.

**A rim skirt.** The globe is a second ground surface under the patch (133 km between vertices at LOD 0, so a few kilometres of it is the interior of one triangle). It stays, because past the 1.5 km rim it IS the horizon — but at the rim the sheet simply stopped and the eye followed the cut down onto it. One ring of quads dropped from the border vertices, darkened like a cut bank: 4 × kCells triangles, half a per cent of the patch. Nothing else about the hidden surface needed changing, because the front-to-back batch sort plus the early depth test declared in `Mesh.frag` already reject its covered fragments before a single noise octave is evaluated.

Patch cost at landing extent: 45 ms heightfield + 21 ms shading + ~2 ms scatter, ~141,000 triangles, on a pool thread at most once a second.

### F6f — the aerodynamics pass, measured and cut by 4x (done)
Written while producing `docs/Performance.md`, which is the point: the number only turned up because somebody timed it.

`VesselAerodynamicsSystem` cost **283 us a tick on a 31-part vehicle** — 1.4 % of a 20 ms budget, harmless in itself, but quadratic in the part count and therefore a wall at KSP scale. The cost is one loop: every part's nine occlusion rays against every other part's boxes.

The first rejection tried was the obvious one — is the box far from the ray's LINE — and it bought **11 %**. That is the useful measurement, because it says why: on a rocket every box hugs the same axis the airflow runs along, so almost nothing is off-line and the test almost never fires. The rejection that pays is **along the flow**: a box the ray reaches only after it has already struck the part it would have to shade cannot shade it, and flying nose-first that is half the vehicle. Together with moving two per-vessel scratch vectors off the stack and onto the system:

| parts | before | after | |
|---|---:|---:|---|
| 7 | 13.3 us | 5.5 us | 2.4x |
| 15 | 64.5 us | 21.4 us | 3.0x |
| 31 | 282.9 us | 67.5 us | 4.2x |

Extrapolated, 120 parts is ~1 ms (5 % of a tick) and 250 parts ~4.3 ms (22 %). The next lever, if it is ever needed, is to recompute exposure only when the flow direction has moved more than a degree or two — it changes slowly — which would take the dominant term down to a fraction of the ticks. Not done, because nothing asks for it yet.

`docs/Performance.md` carries the full per-system table, the terrain patch breakdown, the geometry counts and where the ceilings are.

### F6g — the crash the grass was blamed for (done)
Reported as "the game crashes as soon as the grass is meant to appear". The grass was innocent; it was simply the next thing the same build produced.

The rim skirt held REFERENCES into `mesh.vertices` — the two rim vertices it copies down — and then pushed onto that same vector. The vector had been `resize`d to exactly the ground grid, so capacity equalled size and **the very first push reallocated**, freeing the block those two references pointed into. Every read after it was a use-after-free.

Why it presented as a grass bug rather than a permanent one: at 192 cells the freed block is 37,249 x 48 = **1,787,952 bytes**, well past glibc's mmap threshold, so `free` hands the pages back to the OS and the next read touches an unmapped address — a hard segfault, every time. The 192-cell grid is exactly the near-ground patch, which is exactly where the grass grows. Reproduced standalone under AddressSanitizer:

```
ERROR: AddressSanitizer: heap-use-after-free ... READ of size 48
0x... is located 48 bytes inside of 1787952-byte region
freed by thread T0 here: ... std::vector<Vertex>::push_back
```

The same reproduction with the two vertices taken BY VALUE is clean. That is the fix, plus a `reserve` up front so the appends never move what is already there, plus a stated fallback for the degenerate-edge normal that would otherwise have normalised a zero vector into NaN.

Two guards went in with it, because this class of fault is invisible to the compiler and silent until a driver chokes:

* the builder walks the finished mesh once for non-finite positions/normals and out-of-range indices, and REFUSES to hand over a mesh that has any — 0.3 ms against a 60 ms build;
* the main thread's landing pad checks that the pending mesh is non-empty before creating a GPU buffer from it, because a zero-vertex buffer is its own kind of crash. A refused build simply leaves the patch already on screen where it is.

The lesson is narrow and worth keeping: **nothing may hold a reference into a container it is about to append to.** `mesh.vertices[i]` re-subscripted per read is fine; a `const Vertex&` carried across a `push_back` is not.

### F6h — the field stopped teleporting (done)
Reported as "the grass appears all at once and disappears all at once when the player goes far enough". It was not a fade problem; it was a **re-centring** problem, and the two numbers involved were three orders of magnitude apart.

The grass only exists within a disc around the PATCH CENTRE, and that centre only moved when the player had travelled 30 % of the patch extent — **450 m** at landing scale. The field's radius was **120 m**. So walking in a straight line: you leave the field, spend a few hundred metres on bare ground, and then the patch re-centres and an entire field arrives in one frame while the old one leaves in the same one.

Thirty per cent of the extent is the right threshold for the GROUND: its vertices are computed from absolute body-frame directions, so the terrain surface is bit-identical before and after a re-centre and the rebuild is literally invisible. It is wrong by an order of magnitude for anything that only exists near the centre.

Two changes, sized against each other:

* **A patch that carries plants follows the player at the scale of the field** — `min(extent * 0.30, 40 m)`. The ground pays a 60 ms rebuild every 40 m instead of every 450 m: on a worker thread, against a patch already on screen, capped at one per second. Walking that is one rebuild every ten seconds (0.6 % of a core); at 40 m/s it saturates at one a second (6 %).
* **The field reaches further than the patch moves.** Radius 120 m -> **260 m**, with the density thinned as an inverse power (`(30/r)^1.5`) instead of a ramp, and the height fade moved out to the last 18 %. Measured, the count barely changes — **6,082 tufts against 5,600** — for a field 4.7x wider, and the triangles go 140,928 -> 146,712. The scatter costs 2.5 ms instead of 0.2 (94,388 lattice cells walked instead of 20,107), which is nothing against a 60 ms build.

The point of the sizing: a re-centre shifts every tuft's distance from the centre by at most 40 m, so the only tufts whose size can change in that one frame are those between 173 m and 260 m out. Measured there: a 0.6 m blade subtends **0.13-0.19 degrees — four to six pixels** at a 60-degree FOV on a 1920-wide screen, and the field has thinned to one tuft per 56-104 m^2. The pop is not eliminated. It is moved to where it cannot be seen, and the numbers say by how much.

If the rebuild rate ever bites, the next step is to give the grass its own mesh and its own job reading the patch's cached height grid — re-centring the field would then cost ~3 ms instead of 60, independently of the terrain. Not done, because 60 ms every ten seconds on a pool thread is not a problem.

### F6i — the upload spike, and why the field moved off the patch (done)
Reported as a lag spike every time the grass is placed. It was mine, and it was a rate regression meeting a synchronous uploader.

`VulkanMemory::uploadToBuffer` creates a staging buffer, records a one-shot copy, **submits it to the GRAPHICS queue and then blocks on a fence**. The wait therefore drains whatever that queue is already holding — up to a whole frame of GPU work — and it happens on the main thread, twice per mesh (vertices, then indices). That has always been true; it was invisible while the terrain patch re-centred every 450 m. Making the patch follow the grass at 40 m multiplied it by eleven, on a mesh that had also grown to 107,521 vertices, **5.8 MB**.

The fix is structural rather than a tuning of the threshold: **the grass is its own geometry now**, built from the ground grid the patch already computed and kept for exactly this purpose (1.8 MB of vertices, copied into the job so a terrain rebuild landing mid-seed cannot pull the ground out from under it). That buys three things at once:

* the ground goes back to re-centring at 450 m, so its 5.8 MB upload is rare again;
* the field re-centres at 40 m, and a re-centre now uploads only the grass;
* the grass is cut into **six chunks uploaded ONE PER FRAME** — 603 kB each against 5.8 MB, and the stall is divided by six and spread over six frames instead of landing on one.

Chunks are assigned by hash, so the six come out within 7 % of each other (858 / 830 / 809 / 829 / 833 / 801 tufts) — the split is about balancing six uploads, not about what appears first, because the field is only ever shown complete: the new set fills the spare slot bank while the old one keeps drawing, and they swap when the last chunk lands. No gap, no partial field, no flicker.

Building it from the patch's grid rather than resampling the heightfield is not an optimisation, it is the correctness condition: a second sampling would put the blades up to half a metre off the surface that is drawn, which on a 0.6 m blade is half the blade.

Two latent faults went out with it: the slot sentinel was `0`, which is a legal mesh index, and an empty chunk cleared its slot handle rather than its validity flag — which would have grown the mesh table by one entry on every rebuild over bare rock.

### F7 — multiplayer, from the wire up (done)

No design was handed down for this one, so the decisions are stated here with the reasons, because every one of them closes doors.

**Authoritative host, not deterministic lockstep.** Lockstep sends almost nothing and would have been tempting. It cannot work here, for three independent reasons: this engine cannot promise bit-exact floating point across machines (the physics lane sums forces over parts in archetype order, the aero tables are interpolated, and the compiler may contract a multiply-add — one bit of divergence compounds into two different worlds within minutes, invisibly); nobody could join a game already running, because there would be no state to hand them; and time warp would have to be unanimous. So the host simulates, clients mirror, and what a client sends back is INTENT — "pitch up, throttle 60 %" — never state. That is also the whole anti-cheat story: a client that lies about its inputs flies badly, and that is all.

**The mirror carries the host's entity indices exactly.** The obvious alternative — a network id per entity, mapped to a locally allocated one — dies on the first component that stores an entity handle, and this game is full of them: a conveyor names its body and its link, a cable names its two poles, a cloud deck names its planet, a part names its vessel. Nothing declares which fields are handles, so nothing can rewrite them. The save file reached the same conclusion years ago and restores indices exactly for exactly this reason; replication just does it incrementally, through one new ECS primitive (`World::mirrorEntity`) and type-erased component access, because a mirror learns its component types at runtime from a table the host sends.

**A delta is diffed against the last snapshot the client ACKNOWLEDGED, never the last one sent.** On a lossy link those differ, and diffing against something the client never received produces a world that is wrong and stays wrong. Diffing against the last confirmed state means a lost snapshot costs bandwidth (the next one carries more) and nothing else — there is no repair path because there is nothing to repair. A snapshot naming a baseline the client does not hold is refused whole, without touching the world.

**Change detection is a memcmp.** Components are trivially copyable by ECS rule, so "did this change" is a byte comparison: no per-component code, no dirty flags, and no chance of a system forgetting to raise one. The one place bytes are not enough is a RECYCLED entity index — same index, new generation, and possibly identical bytes. That entity is forced to resend everything, because the mirror has to throw the old occupant away wholesale; a plain byte diff would have left an empty entity carrying somebody else's links. There is a test that fails without it.

**Two delivery services over one UDP socket.** State deltas are unreliable and sequenced: a lost one is worthless because a newer one exists, and resending it would deliver stale truth behind fresh truth. Everything else — handshake, world transfer, pilot input, disconnect — is reliable ordered, with a 32-datagram window (not a tuning knob: the acknowledgement is one sequence plus a 32-bit field covering the 32 before it, so anything further back would fall out of every future ack and resend forever). TCP was not an option precisely because it forces ordering on the half that must not have it.

**Which channel a datagram rode is on the wire, not inferred from its type.** This was a bug before it was a decision. A Snapshot is normally unreliable, but one too large for a single datagram is promoted to the reliable channel so it can be fragmented — and the receiver, deciding by message type alone, handed each fragment upward as a whole snapshot. One flag bit fixed it. A fragmented message takes CONSECUTIVE reliable sequences, so reassembly is just walking the channel in order: no reassembly table, no message-id bookkeeping, and no way to hand up half a message.

Two smaller ones, both found by writing the tests: a peer that has received nothing must not appear to acknowledge datagram zero (zero is a legal sequence number, so the ack fields need a validity bit), and the snapshot beat is PER PEER — a global timer fired again milliseconds after a client was accepted mid-beat and sent it a second full snapshot, 45 kB the client correctly refused.

**Measured** (`Tools/NetProbe`, real UDP sockets, real component layouts): a 100-entity co-op world costs **606 B per delta, 14.7 kB/s downstream, 1.6 kB/s up**, and the delta fits in one datagram, so state really does ride the unreliable channel. A 500-entity world with twenty craft under thrust costs 2,316 B and 56.6 kB/s. The encoder costs 5 µs per snapshot at 100 entities, 26 µs at 500, 112 µs at 2,000 — at twenty snapshots a second that is 0.2 % of one core for a busy session. A joining client has the whole world in its mirror **0.9 ms** after connecting on loopback (45 kB transferred reliably and fragmented). The whole stack is tested against a simulated wire with seeded loss and jitter — 20 % loss with reordering, four seconds, and every mirrored value still agrees with the instant the client believes it is looking at, to 1e-9.

**What is deliberately not here.** No interest management: every client is sent every replicated entity, which is why the 500-entity delta needs three datagrams instead of one. That is the next thing to build and the game design already argues for it — a player on Terra does not need Luna's base at twenty hertz. Nothing is wired into the game layer yet either: no player avatars, no command vocabulary, no host/join UI. This milestone is the wire, proved.

### F8 — a menu for the session, and a clock each (done)

The question this milestone answers is the one that decides whether time warp and multiplayer can exist in the same game: **whose clock is it?**

**Everybody's own.** A player warps because *they* are waiting for an apoapsis; the player landing a rocket elsewhere is not. Dragging the session forward would make warp a thing you negotiate, and the alternative — forbidding it — is worse. So nobody is dragged, and two players can legitimately sit hours apart.

**What crosses the gap is a STAMPED action.** Not "this happened", but "this happened AT INSTANT T". A receiver whose clock has not reached T holds it, unopened, and it lands for them at exactly the moment it landed for its author. No world is rewritten and no world runs ahead of itself. `Network/Timeline.hpp` is thirty lines of queue and the whole model. The case it cannot cover is an event stamped in the local past — a player *behind* you acting, or a very late packet — where nothing short of rewinding the simulation would place it correctly; it lands at once and is **counted**, and that count is the honest diagnostic for "these two should have synced first".

**Time warp stopped being free.** Past ×5 the integrator is off and the world is analytic, which is exact when the motion already IS a conic clear of the air and fiction otherwise. The old gate was ALTITUDE, and altitude answers a different question: it happily permitted ×10,000 on a trajectory whose next event was the ground. `phys::warpPermitted(grounded, closedOrbit, periapsisAltitude, atmosphereTop)` is the rule, in the engine so it can be tested without a window, and the flight state it reads is computed **once per frame before anything consumes it** — the warp control runs at the top of the frame and the HUD at the bottom, and two independent computations of "am I in a stable orbit" would eventually disagree.

**The catch-up warp bypasses the ladder and not the rule.** Closing a three-hour gap from a 200 km orbit at the altitude-capped ×100 takes a real hour, so nobody would press it; SYNC goes to ×10,000,000. It still requires an orbit or the ground, because that requirement is about whether the world can be faked at all, not about how fast. Measured on the real servo (largest rung under half the remaining time, re-chosen every frame): **3 hours closes in 45.7 s, one day in 61.6 s, one year in 109.9 s**, overshoot under 0.03 s, and the numbers are the same at 30 fps as at 60 — the servo reads the time remaining, not a key count.

**The engine had no text input.** No `glfwSetCharCallback` anywhere, no character path, no buffer. Typed characters now arrive as CHARACTERS — through the keyboard layout, the dead keys and the shift state, which is not what a key code is on any layout but a US one — latched per frame exactly like the key-press events and cleared in `newFrame()`. While a field has focus, every gameplay key in `onUpdate` asks through one predicate rather than each guarding itself, so adding the next field cannot silently leave a key live underneath it. The panel toggles on **F3**: `N` was already maneuver-node creation, and every letter within reach is a flight control.

**Found in a picture, before the code ran.** The panel's height comes from the roster length; a mock of the layout — the same NDC arithmetic and the same glyph advance, rendered to an image and looked at — put the footer verdict *outside* the panel. Five fixed rows, not four. That is a screenshot's worth of debugging paid for with a python script.

**What is deliberately not here.** The client's mirror world is kept separate from the live world rather than merged: this build does not yet draw other players' craft, and decoding a remote world into the local one would fight the local simulation for every entity. The one event kind on the wire is a stamped position beacon, which exercises the timeline on real traffic; staging, construction and the rest of the vocabulary join it at the same seam. And the roster is sampled at 4 Hz, so a SYNC lands a fraction of a second short of a player who is still moving — the button simply reappears.

### F9 — you start on foot, and a rocket has to be paid for (done)

The starting rocket, the asteroid and the eight orbital cubes are deleted. What survives is the outpost on Terra, which was always the interesting half and was always decorative next to a vehicle the player was simply given.

**The rule.** A vessel exists in this world only because an assembly hall was handed a design and the metal to build it. The hangar's `BUILD` — one click, a finished vehicle out of nothing, no cost, no pad, no queue — was a door in the side of the factory, and everything the mine, the smelter, the belts and the grid produce could be had for free by pressing it. It is now `ORDER`: it writes and registers the design, then queues it at the nearest hall, which pays in iron and copper at its own rate out of its own bin and ships the hull down the belt to a pad. The in-place "rebuild this vessel" mode went with it; the hangar creates nothing at all now, which is the only version of the rule with no seam in it.

Measured (`AssemblySystem` driven directly, shipped catalogue, the outpost's own seeded stock): **STARLING costs 2,950 kg iron + 450 kg copper and takes 85 s at full power**; the VAB is seeded with 3,000 and 500, i.e. **exactly one rocket**, and with the copper removed the order stalls at **87 %** with the surplus iron left in the bin where a smelter can route it elsewhere. There is no standing order at start-up either — seeding one would have re-created the rocket that had just been deleted a hundred lines above.

**On foot became the normal state**, and that is a bigger change than it sounds. `controlledEntity()` used to fall back to the ship; it now falls back to the SUIT, which is created with the world and never destroyed. Everything downstream — the HUD, the chase camera, the terrain patch focus, the simulation bubble, the trajectory prediction — dereferences that without checking, and there is now no ship for it to find on frame one.

The crashes that had to be closed, all of them latent the moment a rocket stopped being guaranteed: the ship's control block and the throttle readout dereferenced `m_shipEntity` unconditionally, once per frame each; `toggleEva` spawned the suit *relative to the ship*, so there was no way to be on foot without first being in a cockpit; `cyclePilotedVessel` refused to act below two vessels, which meant the first rocket ever built could never be boarded; and the hangar key required `!m_evaMode`, putting the design tool behind a vessel obtainable only through the design tool.

**The suit spawns at 464.62 m/s.** Fourteen metres north of the hub, two metres above its own ground, carrying Terra's orbital velocity plus its spin at that point — the site is on the equator, so the spin term is the full 465 m/s. Spawned at rest in the world frame it would watch the ground leave at half a kilometre a second, which is the same carrier-velocity rule that governs every landed craft in this engine. Measured alongside it: the ground steps 0.09 m across those fourteen metres and at worst 0.35 m within twenty, so the suit lands on flat ground outside the hub's hull rather than inside its floor.

`P` boards the nearest vessel rather than cycling a list — cycling is the right verb when you are already flying and want the other rocket, and the wrong one when you are standing next to exactly one. `G` steps back out beside the vessel and co-moving with it. Staging on foot is refused: it would fire a decoupler on a vehicle the player is not aboard and may not be able to see.

### F9b — the hangar draws, the VAB builds (done)

F9 made the VAB the only door a vessel can come through and then left a second verb on the hangar's own action row. Two rooms now have one verb each.

**The hangar saves a design, on the press, and does nothing else.** `SAVE` writes the `.swship` and registers it; there is no autosave, because a drawing office that wrote a file on every part placed would fill the catalogue with forty near-identical rockets. The manufacturing button is gone, and so is the "rebuild this vessel in place" path that sat behind the same call — that one also spawned parts for free, just more quietly.

**The VAB's panel became a catalogue.** Designs down the left; on the right the selected one, drawn in 3D and turning, with part count, dry mass, each metal's cost against what is actually in the bin, the build time at full power, and a `PRODUCE` button. A row click SELECTS. Ordering is one deliberate press beside the price and the picture — the old behaviour ordered a rocket as a side effect of reading the list.

**Solid geometry inside a HUD panel needed a second pipeline.** The screen-space pass has no depth buffer and no culling, which is right for flat panels and hopeless for a solid. `DrawItem::hudSolid` routes to a pipeline identical to the HUD's but for one flag — back faces culled — which is a complete hidden-surface solution for a convex part, and the caller sorts the parts back-to-front among themselves. The two pipelines interleave inside the HUD batch list in painter's order, so the model draws on its panel and under the text; the bind happens where the flag changes, twice a frame for one preview.

**The handedness is the trap, and it is not a matter of taste.** The camera negates Y once, at the source (`Camera.cpp`: `m_projection[1][1] *= -1`), and the counter-clockwise front-face convention was settled empirically against that. A preview transform with no flip — or with two — presents the opposite winding and culls exactly the faces that should be kept: the rocket renders inside out. So the transform is a proper rotation times a scale with a NEGATIVE Y, and that is written into the `hudSolid` comment as a requirement rather than left to be rediscovered.

Verified numerically, over a full turn, because it cannot be verified by eye without a window: determinant **negative**; worst overflow past the preview box **−0.014 NDC**, i.e. always inside; one metre of design spans **14.98 px across and 14.98 px down**, so nothing is stretched; the depth sort is monotone. Framing fits the shape and not its bounding sphere — a rocket is long and thin, and the sphere fit gave up a third of the available height (152 px of a 205 px box) for a width nothing occupied; the axis-aware fit gets 188.

### F9c — on foot properly, and a check on the rocket that would not fall (done)

**The jump was a one-frame edge feeding a fixed-rate consumer.** `ShipControlsComponent` is cleared and rewritten once per rendered frame; `CapsuleMovementSystem` reads it on the physics lane at 50 Hz. Above 60 fps most frames tick that lane zero times, so roughly one press in three was overwritten before anything could act on it. It is a latch now, cleared only once `advance()` has actually run a physics tick. Double-firing is prevented by the walker itself, which clears `isGrounded` as it jumps.

That failure mode is general and worth naming: **any edge written by the frame loop into a component consumed by a fixed-step lane must be a latch, not a pulse.** Staging, docking and every future one-shot input has the same shape.

**`E` is one ray with two answers.** It used to scan only buildings; it now scans building hulls and part hulls in the same cast and returns the nearest hit, and the caller decides — a machine opens its panel, a vessel gets boarded. Casting twice and comparing afterwards would be two answers to one question.

**Walk speed 4 -> 8 m/s.** The outpost is 200 m across.

**The rocket that leaned and would not fall — measured, not argued.** A probe stood the shipped design on the real launch site, with the real terrain, the real `GroundHullComponent` and the game's own systems (including `ThrustSystem`, which turns out to be the only thing in the build that integrates `DynamicBodyComponent::angularVelocity` into a rotation, gated on `ShipComponent`):

* 0 deg -> 0.25 deg, 8 deg -> 8.00 deg, **15 deg -> 14.99 deg**, **30 deg -> 88.43 deg** after 60 s. The statics put this hull's threshold at **15.1 deg**, and the measurements straddle it exactly.
* The warp path: frozen at 67.8 deg mid-topple by the bubble's surface anchor, held there for the whole warp, back to 91.1 deg sixty seconds after release.

So nothing is broken. What is wrong is that both correct behaviours are illegible — a rocket leaning fourteen degrees on its own base and a rocket frozen by warp look identical to a rocket the physics has forgotten. The HUD now prints `LANDED  LEAN n DEG  RESTING|TIPPING`, computed from `phys::topplingAcceleration` — the very function the ground contact uses — so the readout cannot drift from the behaviour it describes.

**And a lesson about the probe.** Its first run showed the rocket climbing 40 m and never settling, which looked exactly like the reported fault and would have been reported as one. The harness never advanced Terra's spin angle, so the terrain was sampled in a frozen body frame while the craft co-rotated at 465 m/s — nine kilometres of stationary landscape dragged under it every minute. A measurement rig that does not reproduce the systems around the thing under test measures the rig.

### F10 — shipping it, and reaching it across the room (done)

Two failures with the same shape: the code was right and the machine refused to run it, and in both cases the refusal happened *before* anything of ours executed, so there was nothing to read.

**`build\windows\bin\Debug\StarWorks.exe` runs on the machine that built it and on no other.** A Debug build links the debug C++ runtime — `vcruntime140d.dll`, `msvcp140d.dll`, `ucrtbased.dll` — and Microsoft does not redistribute those; they ship with Visual Studio and with nothing else. Windows refuses to create the process, so there is no log, no message box and no clue. `SW_STATIC_RUNTIME` links the runtime statically and is set **before** `include(Dependencies)`, because glfw compiles with whatever `CMAKE_MSVC_RUNTIME_LIBRARY` says at declaration time and a mismatch is a link error rather than a warning. `package.ps1` turns it on, builds RelWithDebInfo, and stages the exe with the shaders and assets taken *from beside the built binary* rather than from the source tree — so the folder contains exactly what was compiled, including the `.spv`, and never a shader edited since.

**A LAN game needs nothing from the router and everything from the host's firewall.** The asymmetry is the whole story: the joining side sends first, so its reply arrives through the flow its own packet opened and no rule is needed. The host only ever waits to be spoken to, and Windows Defender drops the unsolicited `ConnectRequest` that would start the conversation. The client then retries until it gives up. Neither end has anything to say about it — the host's process never saw a byte.

So the game now says which of the two timeouts it is. `NO REPLY - CHECK FIREWALL` when `datagramsReceived` is still zero: nothing at that address ever answered, and the cure is a rule, an address or a host. `TIMED OUT - LINK LOST` when datagrams did arrive and then stopped: a real network fault, and the firewall rule is useless against it. One counter separates two problems whose fixes have nothing in common.

**The game asks for the rule itself, and is never the thing that gets elevated.** Pressing `HOST` is the only moment in the game that needs an unsolicited inbound datagram and the only moment at which asking for administrator rights is explicable, so that is where the prompt lives. Two constraints shape it:

* *Ask only when there is something to ask for.* Reading the firewall needs no privileges, so `INetFwPolicy2` is queried in-process first and the prompt only appears when no enabled inbound Allow rule covers this executable. A UAC prompt on every press of HOST would teach the player to click through it, which is how a UAC prompt stops being a security feature.
* *Elevate a helper, not the game.* `ShellExecuteEx` with the `runas` verb runs one hidden `netsh` that adds the rule and exits. A game running as administrator writes every save, log and crash dump as administrator and refuses drag-and-drop from Explorer — a permanent cost for a one-time configuration change. `netsh` rather than PowerShell: present on every edition including Home, subject to no execution policy, and no script host flashing a console.

`ERROR_CANCELLED` is reported as `Declined`, not as a failure — dismissing a UAC dialog is a decision, and dressing it up as an error invites a retry loop. Hosting continues either way, and the panel says so. The exit code is not trusted on its own either: the firewall is re-read afterwards and the result reports what is actually there. The rule is matched and created **by executable path rather than by port**, so it survives a port change, opens nothing else on the machine, and recognises a rule Windows created through its own prompt as already sufficient — `firewall.ps1` creates the identical rule, so using the script means the game never asks.

The panel's firewall verdict is a line of its own rather than a suffix on the status. Measured before writing it: `HOSTING ON 192.168.1.61:7777 - FIREWALL REFUSED` is **0.725 NDC** wide at the status size against **0.494** of usable panel width. That is the third HUD overflow this project has caught by arithmetic instead of by looking at it.

**`net::localAddress(port)` asks the routing table rather than listing adapters.** A UDP socket is connected to 8.8.8.8:53 and immediately asked, via `getsockname`, which source it *would* use — nothing is transmitted, because `connect()` on a datagram socket only fixes a destination. A developer machine with Docker, WSL, a VPN and a Bluetooth PAN has half a dozen addresses and exactly one of them is the answer; enumerating them and guessing is how you end up telling the other player to type a bridge address. The host panel prints the result, so nobody reads it out of `ipconfig`. `firewall.ps1` adds the inbound rule on Private and Domain only, deliberately not Public, and its `-Check` mode reports the address, the rule and the network profile without changing anything — Public blocks inbound whatever the rule says, and Windows picks Public silently for any network you never answered the discoverability prompt for.

### F10b — the silence, told apart (done)

A firewall rule was added and the client still said `NO REPLY`. Ping between the two machines returned nothing either. Neither observation means what it looks like it means, and that is the whole lesson of this entry.

**Ping is not evidence.** Windows Defender ships the inbound ICMP echo rule *disabled*, so two PCs on the same switch refuse to ping each other by default. A failed ping is the expected reading of a working network. `netcheck.ps1 -AllowPing` adds the rule when you want ICMP as a diagnostic, and the script says out loud that the game does not need it.

**Three causes wore the same face.** From the client, a firewall eating the packet, a wrong address, and a host that received the packet and threw it away are one symptom. Only the host can separate them, and until now it dropped strangers with a bare `continue`. `Host::Reception` counts what reached the socket before any of it was believed: `arrived`, `fromStrangers`, `refused`, `wrongVersion` (with the offending version), `notOurs`. The `F3` panel shows `RX n REFUSED n` and a verdict; `RX` at zero while someone is trying is a completely different problem from `RX` climbing alongside `REFUSED`.

**The version mismatch deserved its own counter.** `readHeader` rejects a datagram whose version is not ours, silently, before anything else — so two machines running two different builds produce exactly the symptom of a blocked port. The header's first six bytes are read by hand at the drop site precisely because the point is to see the values `readHeader` refused. It is logged once per distinct version rather than per datagram.

**A rule can be present and inert.** The rule covers Private and Domain; a Public network profile blocks inbound however many rules exist, and Windows assigns Public in silence to any network whose discoverability prompt was never answered. `onPublicNetwork()` reads `INetFwPolicy2::get_CurrentProfileTypes` — a mask, not a value, because a machine can be on several networks at once and the one the other player is reaching may be the Public one. The panel says `NETWORK IS PUBLIC - RULE INACTIVE`, which is the most confusing state this feature can be in and the one no tool that lists rules will reveal.

**And an instrument that is not one of the suspects.** `netcheck.ps1` sends and receives plain UDP with nothing of ours in it, in pairs across the two machines, and the listener answers so the sender learns whether the path works *in both directions* — a firewall is one-way, and the return trip is the half that usually works. It prints each address with its prefix length and consults ARP, because two machines on different subnets, or one behind a router with client isolation, will never reach each other whatever is written into either firewall. If that script works and the game does not, the game is at fault; if it fails too, the game never had a chance. Its socket logic was run end to end before shipping, one process to another.

### F10c — the project does not know where it lives (done)

The whole tree moved from one drive letter to another and `launch.ps1` broke on its **first line**, which was `cd F:\StarWorks`. Nothing else in the project broke, which is the interesting part: the C++ already resolved every asset, shader and save from `FileSystem::executableDirectory()`, and CMake from `${CMAKE_SOURCE_DIR}`. One convenience line, written once, was the entire failure.

The rule now: **nothing assumes where the project lives.** Scripts resolve from `$PSScriptRoot`, CMake from its own source dir, the game from its own executable, and anything that genuinely needs the source root asks `FileSystem::projectRoot()`.

**`projectRoot()` looks for a marker, not for a level count.** It walks up from the executable for a directory carrying `CMakeLists.txt` **and** `Assets/` together. The pair matters: Part Studio's old write-back climbed a fixed five levels looking for any folder named `Assets/Parts`, and from a project at `G:\StarWorks` five levels reaches the drive root — so a stray `G:\Assets\Parts` would have been silently written into. The walk is bounded by the filesystem instead, stopping when `parent_path()` stops moving, which is what a root does on both Windows and POSIX. It returns **empty** for a packaged build, and that is the correct answer rather than an error: `dist\StarWorks\` deliberately ships no source tree, and a caller treating "no root" as a failure would break the shipped game to serve the developer's convenience.

**A rule nobody checks lasts about a week**, so `PortabilityTests.cpp` walks the source tree and fails on any drive-letter path in a `.ps1`, `.cpp`, `.hpp`, `.cmake`, `.json` or `.txt` file. Whole-line comments are skipped, because the files that *explain* the rule have to be able to name the paths they ban; a path in a comment trailing real code is still flagged, since deciding where a comment starts on a mixed line means parsing string literals, and a matcher needing a parser is a matcher nobody trusts. Markdown is out of scope — `-Exe <path>\StarWorks.exe` in a document illustrates a path the *user* supplies, which is what documentation is for. The guard was verified in both directions: it caught the seven explanatory comments on its first run, and after the comment rule it was proven again by planting `$broken = "G:\StarWorks\build"` in `launch.ps1`, watching it fail by name and line, and removing it.

One absolute path could not be removed, only handled: **the one CMake writes itself.** `CMakeCache.txt` records the source directory and its own directory, absolutely, and so do the generated project files, the dependency rules and every `.vcxproj`. Moving the tree from one drive to another therefore fails at configure time with a message about directories not matching — through no fault of the person who moved it, and with no fix short of throwing the tree away. `Build-Common.ps1` compares the cache's `CMAKE_HOME_DIRECTORY` and `CMAKE_CACHEFILE_DIR` against where the project actually is, purges when either has drifted, and says which one moved; `launch.ps1` and `package.ps1` both call it before configuring. It does nothing when the cache is consistent, because purging on a hunch costs a full rebuild. Path comparison normalises separators, trailing slashes and case, since CMake writes forward slashes, PowerShell writes backslashes and Windows does not care about either. Verified by reproducing the failure exactly — configure a project, move the directory, watch CMake refuse — then running the helper and reconfiguring cleanly.

The last absolute path in the tree was not in any file the build reads: a `StarWorks - Raccourci.lnk` sitting in the root, still pointing at `F:\StarWorks\build\windows\bin\Debug\StarWorks.exe`. A Windows shortcut stores an absolute target and cannot be made relative, so it is replaced by `StarWorks.cmd`, where `%~dp0` is the batch equivalent of `$PSScriptRoot`: it finds the executable itself, preferring the packaged folder over the development builds, and runs it from its own directory so `Shaders\` and `Assets\` resolve. The guard scans `.cmd` and `.bat` too, and its comment rule learned `rem` — with the trailing space required, because `remove` is not a comment and batch agrees.

`launch.ps1` was rewritten around that: `$PSScriptRoot`, `-S`/`-B` rather than `--preset` so it cannot silently depend on the caller's working directory, configure only when there is no cache, and `-Clean`/`-Release`/`-Test`/`-NoRun`/`-GameArgs`. Its parameter is `-GameArgs` and not `-Args` because `$Args` is a PowerShell automatic variable and declaring it in `param()` is a syntax error rather than a warning.

### F11 — a jump is not a flight, and the grass was waist-high (done)

Reported as one thing — "warp locked should only be about syncing; right now jumping is enough to stop you moving in EVA" — and it was three, with only one of them where it looked.

**The physics was innocent, again.** A probe stood the suit on the real launch site with the real terrain and the game's own walker: the jump peaks at **1.77 m**, ground contact returns after **0.88 s**, and walking after landing covers exactly the same distance as before. So nothing about the jump breaks movement.

**What actually took the controls away was the warp rate.** `updateShipControls` returns early above ×5 — correct for engines, since the world is analytic there — but that early return also drops `W`, `A`, `S`, `D` and `Space`. On foot, rails warp additionally turns the suit into a surface anchor. The player keeps the camera, loses the legs, and nothing on screen says why. Rails warp is now capped to physics warp whenever the player is on foot: a person standing on a planet has nothing to fast-forward through, and the one case that genuinely needs to skip hours on foot — catching another player's temporality — sets `bypassAltitudeCap` and is deliberately not caught.

**`WARP LOCKED` was furniture, not a warning.** It was drawn from `!warpAllowed()`, a *standing* condition: true throughout every ascent, every reentry, and — because `isGrounded` drops the instant the feet leave the dirt — every jump. A message that is on almost always tells nobody anything, and the player reasonably read it as the thing that had disabled them. It now appears for four seconds *after the gate actually refuses something*, which is the one moment the reason is worth screen space. In the multiplayer panel the footer answers the multiplayer question instead: a running sync counts down, a reachable one says whether it can start, and with nobody ahead of you the row is simply absent.

**And the gate stopped mistaking a hop for a departure.** The footing is remembered for as long as a jump can physically last. That window is computed, not written down, because a constant is wrong nearly everywhere: the same legs and the same key give 0.92 s of air on Terra, 2.4 s on Mars and 5.6 s on Luna, so a value tuned on Terra would leave a Luna walker "airborne" for most of every hop. `phys::jumpHangSeconds` lives in the engine beside `warpPermitted` so the rule is testable without a window, and the test asserts the one inequality that matters — window > hop — on all three bodies, plus that it is not so generous that someone who really did step off a cliff keeps a free pass.

**The grass.** Four knobs, named together because they interact: spacing sets how many cells exist, the density pair how many of them grow, the height pair how big what grows is — and height is multiplied by density, so thinning the field also shortens it. Measured over the planted disc with the ground held fully green, the densest case there is: **8 834 tufts / 26 502 blades → 3 717 / 11 151**, 0.042 down to 0.018 per square metre; mean blade **0.434 m → 0.224 m**, tallest **1.38 m → 0.61 m**. The 1.75 m suit now stands nearly eight times the mean blade rather than four, and nothing reaches its waist. The first draft of that comment quoted numbers written before the probe was run and every one of them was wrong — measure, then describe.

### F11b — three things a review found, and one thing the review got right that I got wrong (done)

**Saving was impossible, not broken.** `AssemblyComponent` and `VehicleQueueComponent` are added to every assembly hall — so to the starting outpost, on frame one — and neither was registered in the save schema. `saveWorld` throws on the first column it cannot name and writes *nothing*: not a corrupt file, no file, from the very first minute of a new game, with a single "Save failed" line to show for it. Both are registered now, but the interesting part is the second half: `save::unsaveableComponents(world, schema)` asks the same question of a world that has just been built, and the game asks it at startup, right after `buildScene`. A component added to a live entity and never registered is a broken save, always, and that is now discovered on frame one instead of hours later. The test pins the three properties that make the check worth having: silent on a clean world, naming exactly one component and not the registered ones beside it, and agreeing with `saveWorld` — an early check that can disagree with the thing it predicts is worthless. Empty archetypes are deliberately not reported, because `saveWorld` skips them and a false alarm teaches people to ignore the real one.

**The SL-1's power node floated, and I mis-measured it twice before believing the reviewer.** The node sits at y = 1.75; its pedestal is a box at y = 1.0 with `size` 0.5. I read `size` as a full extent, made the top 1.25, called the gap 0.50 m, and "fixed" it — and the probe still failed, now hitting at 2.133. `Parts.hpp` says it plainly: `Box = 0, // size = half extents`. The pedestal spans 0.5 to 1.5, the gap was **0.25 m**, and the reviewer's numbers were right from the start. The pedestal now spans 0.5 to 1.75 — grown upward, base untouched — so the node is exactly on the surface rather than moved, because moving an attach point moves it for every design that already references it. Lesson, and not a new one in this project: when a measurement disagrees with a human who says they probed it, check the units before checking their arithmetic.

Two further things this exposed. The shipped asset in the repo had `"nodes": []` while the one on disk had the node — Part Studio writes back to the source tree, so a catalogue file can drift from the one under test; the fix had to be made against the *real* file, staged from the machine. And `Assets/` is mirrored next to the test binary by a POST_BUILD step, which does not run when only an asset changed — a green test after editing a `.swpart` may be green about the old file.

**Undefined behaviour in the star field.** `hash01(seed++) + hash01(seed++)` leaves the two increments unsequenced relative to each other: the standard does not say which call gets which seed, so MSVC and GCC may generate two different skies from the same save. Split into two statements. The sum of two uniforms is genuinely wanted — it is what makes the band's cross-section gaussian-ish — but being unable to say which two is not.

### F12 — the review backlog, fixed by fan-out and then attacked (done)

Sixteen agents over two waves, partitioned **by file rather than by issue** so no two could write the same thing, each with its own build tree; every territory then re-read by an agent whose only job was to refute it. Verified centrally: 182 file hashes before and after (no agent ever wrote outside its allow-list, no file created or deleted) and a from-scratch build with its own dependencies. **191 -> 220 tests, 0 failures.**

**Wave 1 — the backlog.** Network: every wire-driven length bounded against what the datagram can hold, `catch(std::exception)` at the parse boundary (`bad_alloc` was never an `sw::Exception`), a stateless cookie challenge before any allocation for an unproven address, and fragment reassembly that finally reads the `messageId` it had been storing and ignoring. Measured: one 39-byte spoofed request drew **5.9 MB back over ~5,550 datagrams — 151,962x amplification**; after, 24 bytes, 0.62x. And the bound that mattered most was not the one anyone expected — `reserve()` of 20,000,000 roster entries and `resize(0xFFFFFFFF)` of an event payload both **succeed**; nothing throws, the memory is simply taken. Factory: the assembly hall spent its whole tick budget instead of one hull (a 8 h tick finished 1 rocket and discarded 1,140,000 kg; now 96, identical across 1x28800 s / 480x60 s / 28800x1 s), and `solarFactor` is integrated across the tick rather than sampled at an instant — sampling gave **+212 % at noon and -100 % at midnight**.

**What the adversarial pass caught, and why it paid for itself.** Six tests that passed with their own fix removed. Three numbers written in comments that the code does not produce. And one regression introduced by a fix: splitting rotation integration out of `ThrustSystem` moved it *after* thrust, so the engine fired along the previous tick's attitude — found by recovering the pre-change DWARF line table from a stale object file and comparing statement order. The rule this earns: **a fix is not done until its test has been shown to fail without it.** Wave 2 made mutation-testing mandatory and per-test.

**Wave 2 — what wave 1 broke or under-proved.** `ReplicationDecoder::apply` is now all-or-nothing: the snapshot is parsed into staged lists and fully validated before the world is touched. It had been half-applying and then throwing — and wave 1's new `catch` had quietly turned that crash into *silent, permanent mirror divergence*, which is strictly worse. Reconnection, broken by wave 1 restarting the reliable channel at sequence zero, works again through a session nonce that lets the host tell a retransmission from a genuine rejoin. The contact tolerance was re-derived: the 0.5 m cap bound on essentially every vehicle larger than the tuning body and produced a **non-zero restoring torque inside the support polygon**, where the answer must be exactly zero.

**Wave 3 — closing what wave 2 left unproved,** by hand rather than by fan-out, because three items is not a fan-out. Reconnecting now forgets the mirror: `connect()` cannot clear a world it does not own, so it records the intent and the next `update()` wipes it before reading a single datagram — otherwise the re-accepted peer's full snapshot (all spawns, **no removals**) leaves anything the host destroyed alive for ever. And the two guards nothing was proving now have tests, each mutation-checked individually.

The spoof test is the one worth remembering, because writing it corrected the story. The claim was that a forged rejoin tears a playing client down; measured, it does not — the victim's own reliable channel re-establishes and it is still Connected 34 s later. What the attack really does is make the host announce a departure for a client that never left, and gameplay code holding that id drops what it was tracking. The first version of the test asserted connection state and passed happily with the guard removed; the discriminating assertion is that **no departure is ever announced**, watched on every update because `departed()` is cleared at the top of each one. A test that asserts the wrong consequence is indistinguishable from no test.

The `aliveAfterwards` case needed no such care: with the predicate mutated to `return true`, the exact snapshot the test builds **segfaults**. `SW_ASSERT` does not cover it — it compiles out of RelWithDebInfo and Release, which are the builds anyone plays. An assertion is not a guard.

**Still open.** Interest management, the real answer to head-of-line blocking, remains a separate milestone.

### F13 — a frame around the game (done)

The game had no outside. It opened straight into a world already built during the constructor — a second or two of a black window in which a slow disk and a hang look identical — and `Esc` called `requestClose()` on the spot, so the reflex that closes a dialog everywhere else threw the session away with no chance to save.

**The shell is a state machine, not a set of flags**: `Booting → Menu → Playing`. Flags would let the game be two of those at once, which is exactly the bug class this replaces.

**The bar measures something.** All the start-up work — four catalogues, the scene, the meshes, the save schema — moved out of the constructor into a *plan* of seven named steps run one per frame, so the first frame appears immediately and the bar's width is steps-done over steps-total. A bar driven by a timer while the main thread blocks is a decoration; worse, one that reaches 90 % and stalls turns "this is slow" into "this is broken" at the moment the player decides whether to kill the process. The label names the step *about* to run, because a step that has finished is not what anyone is waiting for. The one thing still built in the constructor is the glyph meshes — the bar has to draw text on the very first frame and cannot wait for a step that has not run.

**Saves got names.** `saveGame`/`loadGame` became thin wrappers over path-taking versions; the quicksave stays exactly where it was (`starworks.sav`, beside the executable) so existing ones keep working, and long saves go to `Saves/<name>.sav`. A name typed by a human goes into a filename, so it is sanitised rather than trusted — a slash would silently write somewhere else and a colon is not a filename on Windows. The list is newest-first with ages rather than dates, because `file_time_type` has no portable calendar conversion and "3 H AGO" is what someone picking a save actually wants.

**Two things worth keeping.** A failed load leaves the world it was loading *into* in an unknown state, so the shell stays in the menu and says so rather than dropping the player into a half-world. And the hangar's early return in `onRender` would have made a menu opened from in there invisible *and* frozen — a soft lock; nothing opens one today, which is precisely why the two-line guard is there instead of a comment claiming it cannot happen.

**And the deferral had a bill.** Moving the scene out of the constructor left everything after it still expecting a world. Two dependencies, only one of which announced itself: the free camera's parking spot reads Terra's transform, which became `getComponent` on a null entity — a loud assert on the very first run. The other was silent. `PowerGridSystem` is *constructed* with the Sol entity, so wiring the lanes before the scene handed the entire power grid a null star: every solar panel on every base would have produced nothing, for ever, with no error anywhere. Both now live in a `STARTING THE SIMULATION` boot step that runs after the scene. The assert is what made me look; the second one is what mattered — and the general shape is worth naming: **when work moves later, everything that silently read its results moves with it, and the loud failure is the lucky one.**

**And the startup check paid for itself, then showed its own weakness.** Deferring the scene did not create it, but `phys::HullComponent` — carried by every building straight from its `.swpart` hitboxes — and `phys::HullMoverComponent` on the EVA suit had never been registered, so saving was impossible exactly as `AssemblyComponent` had made it impossible before. The F11b check reported it on frame one as designed. What it reported was `runtime id 13, size 200`, which is true and nearly useless: a component id is assigned by order of first use and means nothing to a human, and identifying it took a generated `sizeof` probe over all 43 component types. The message now names the culprit's NEIGHBOURS — `on an entity that also has [sw.Transform, phys.GroundHull]` — because those are registered, and a 200-byte stranger sitting on a building is then obvious. A diagnostic that is correct but unreadable is only half a diagnostic.

Both screens were laid out as SVG mocks and looked at before the code ran, the technique that has now caught four HUD overflows in this project; the widest row, `NEW GAME (ABANDONS THIS ONE)`, clears its box, and the nine-row load list ends at y = 0.416 against a limit of 1.0.

### F13b — the game file splits, and Debug learns to refuse what Release refuses (done)

**`StarWorksGame.cpp` had reached 12,407 lines — a quarter of the project in one
translation unit.** Every edit anywhere in the game layer recompiled all of it, and
every theme's helpers hid in one 750-line anonymous namespace at the top. It is now
thirteen files, one theme each, same class: the core file keeps the constructor and
the frame loop (~780 lines), the largest theme (`GameFactory.cpp`) is ~1,700, and the
old anonymous namespace became `GameInternal.hpp` — `inline` functions and
`inline constexpr` constants, so every Game TU can include it without ODR trouble.
Helpers used by a single theme (the hangar's quaternion arcs, the factory's
`resourceMined`) stayed in that file's own anonymous namespace, which is the rule
going forward: shared → `GameInternal.hpp`, private → your own file.
`Game/Source/README.md` is the index, and `Game/CMakeLists.txt` says the same thing
at the top.

The move was mechanical and checked like one: the splitter refuses to finish while
any line of the original is unclaimed or claimed twice, and afterwards the
line-multiset of the thirteen files against the original differed only by the
intended transforms (`constexpr` → `inline constexpr`, helper functions gaining
`inline`, one anonymous-namespace wrapper). Then the full suite in **both**
configurations, which is how the second finding surfaced.

**The Debug test suite could not run to completion — Release refused what Debug
trapped on.** F12's adversarial test feeds `mirrorEntity` the index 4,294,967,295 and
expects the gigabytes-refusal throw; but that index is also the null-handle sentinel,
and an `SW_ASSERT(!entity.isNull())` sat above the refusal. Release compiles the
assert out and throws as intended; Debug hit `__builtin_trap` mid-suite — 223 green
in one build, SIGILL in the other, from the same test. The assert is gone: the null
index is above `kMaxMirrorIndex`, so the existing check already refuses it, and the
function's own comment states the principle the assert violated — a value that comes
off a wire anyone can write to is hostile input to be refused, not a bug of ours to
be trapped. The rule this earns: **an adversarial test is part of the suite in every
build type, so a guard it exercises must behave identically in every build type** —
`SW_ASSERT` belongs on our own invariants, never on the wire's.

### F13c — a title screen worth the planet behind it (done)

**The background image is not an image — it is the game.** The engine has no
texture pipeline (everything is vertex colour plus procedural shading), so a
splash PNG was never on the table. It was also never needed: F13 already builds
the scene *before* the menu, so the title screen simply renders the world from a
camera of its own — a slow orbit of Terra at ~6,700 km, parked ~54° off the
subsolar point where the lit limb is at its most photogenic, drifting a full lap
every thirteen minutes on WALL time, because the simulation stays paused and the
motion is presentation, not state. Everything downstream of `activeCamera` —
sun, shadow spheres, aerial perspective — needed no change; the render path
cannot tell a menu camera from a chase camera. A pause menu deliberately keeps
the player's own frozen view instead: that is *their* game behind the wash.

**The title is three passes of the same glyphs.** A faint blue glow as four
same-size offset copies, a hard drop shadow, then the face — and the glow must
NOT be a scaled-up copy, because a bigger glyph has a bigger advance, so its
letters drift out of register as the string runs on (measured on the first
attempt: visibly ghosting by the K). Same-size offsets share the advance and
stay aligned under every letter.

**The scrim is a gradient, because the wash was hiding the point.** The old
menu covered the frame in a 94 %-opaque quad; over the new backdrop, a
four-band vertex-alpha gradient (0.72 over the title's open space, 0.24 across
the planet) keeps the text readable without burying the planet the screen is
built around. The flight HUD is no longer merely *hidden* behind the menu — it
is not collected at all, since a throttle readout glowing through a light scrim
reads as a bug.

And two rows of housekeeping the layout pass surfaced: menu labels now
**shrink-to-fit their row** — measured with the same glyph advance the renderer
uses, the arithmetic that has now caught its fifth HUD overflow
(`NEW GAME (ABANDONS THIS ONE)` spilled both edges of the pause menu's centred
column) — and that label lost its parentheses, because the 5x7 charset never
had `(` and had been silently rendering spaces in their place since F9.

### F14 — the whole solar system, and the ring that was invisible (done)

**Eighteen new bodies from one table and one loop.** Mercury, Venus, Jupiter with
the four Galilean moons, Saturn with Enceladus, Rhea and Titan, Uranus with Titania
and Oberon, Neptune with Triton, and Mars's two captured rocks — real radii, real GM,
real semi-major axes, eccentricities and inclinations, real SOI radii computed as
`a·(μ/μp)^(2/5)`. The scene builder gained no eighteen copy-pasted blocks: a
`PlanetDef`/`MoonDef` table and one loop, because eighteen hand-built bodies is
eighteen chances to typo a mantissa. Two rules in the table are worth naming. **A
moon's spin is computed, not typed**: every one is tidally locked, so its rate is
`sqrt(μp/a³)` from its own orbit and cannot disagree with it. And **Phobos and
Deimos are scenery on rails**: their SOI radii come out at 7 km and 3 km — INSIDE
their own rock — so they get no gravity, no ground, and no pretense; a docking-scale
visit is a milestone of its own. Uranus rolls on its side (97.8°), Venus spins
backwards (axis flipped), Triton orbits backwards (i = 157°) — the one moon in the
sky that rises where the others set.

**Eleven new landable worlds through one parameterised preset.** The terrain system
keys its presets by style id in two places that must not drift — `Planet/Terrain.hpp`
and its GLSL mirror — so eleven new worlds did not get twenty-two hand-copied
parameter blocks: one `presetSmallWorld(seed, amplitude, frequency, ...)` helper in
each language, eleven one-line calls, and the parity harness extended from three
worlds to fourteen. `PARITY OK`, elevation delta 0.000000 m on every one. The
palettes are families: young ice (Europa's rust lineae, Enceladus's south-polar
stripes, Triton's dark streaks — the family trait is cracks, thin lanes where
|fbm−0.5| pinches to zero), old ice-rock (Ganymede's grooves, Callisto darkest,
Oberon reddened), Io's sulfur with SO₂ frost and eruption floors, Titan's dune belts,
Mercury's baked lunar bones. Deposits follow the geology: the ice moons are
propellant country, Io is metal with not a gram of water, Mercury hides its ice in
the polar craters at bias 0.98.

**The gas worlds are a second shading family, not a terrain.** Styles ≥ 20 skip the
heightfield entirely: latitude bands warped by low-frequency turbulence, per-planet
ramps, the Great Red Spot fixed in Jupiter's body frame (longitude distance via
`cos(lon−lon₀)` — no `mod()`, whose negative behaviour differs between GLSL and C),
Neptune's methane streaks, Venus's featureless chevrons — a rocky planet whose
visible surface is nonetheless a cloud deck nobody sees through, and whose ground is
deliberately a later milestone. The same function exists twice, C++ and GLSL,
because beyond close orbit the vertex-coloured LODs are all anyone sees.

**Saturn's rings were invisible on the first run, and the reason is a material.**
The annulus (1.24 R to 2.27 R, radial-noise bands, an alpha gap for the Cassini
division and a thin one for Encke, tilt BAKED into the vertices) rendered as
nothing — because every transparent `MeshComponent` was routed to the atmosphere
shell material, and a fresnel limb evaluated on a flat annulus is a ring-shaped
nothing. `MeshComponent::kLitTransparent` now says what the cloud-deck flag already
said: the material is part of the mesh's identity, not a guess made at draw time.

**And the map taught the far-plane lesson.** Framing Neptune needs a zoom ceiling of
1.3e13 m; the map camera's far plane was 2e12; past it the entire universe clipped
to an empty screen with one beacon label in it. A zoom ceiling and a far plane are
the same number written in two places — they now agree (far 3e13), and the whole
system fits on one screen, markers, rings and all. Boot pays for the new worlds:
BUILDING THE SYSTEM colours five LODs for fourteen relief worlds (~40 s single-core
under llvmpipe, far less on real hardware); parallelising the LOD builds across the
thread pool is the obvious next cut and is listed below.

### F14b — creative mode: the fuel stays in the tanks (done)

**One flag, chosen where a mode belongs and applied where the fuel burns.** The
title screen's root page gained a row under NEW GAME — `MODE - SURVIVAL` /
`MODE - CREATIVE - NO FUEL BURN` — that decides what the NEXT new game is.
Mid-session the row only reports: the mode is part of the save, and flipping a
survival world to creative from the pause menu is the kind of decision a save file
should not have made behind it.

The mechanics are three small pieces with one idea each. `ThrustSystem` carries
`setInfiniteFuel` — a flag on the SYSTEM, not per-entity state, because the mode
belongs to the session, and the host owns it in multiplayer for the same reason it
owns everything else. The skip sits exactly at the burn (`fuelNeeded > 0 &&
!m_infiniteFuel`): full thrust, nothing leaves the tanks — deliberately not a
"refill afterwards", which would jiggle the vessel's mass through the assembly pass
every tick. And the game keeps a pointer to the system it registered
(`m_thrustSystem`, the `m_aerodynamics` pattern) so `applyCreativeMode()` can reach
it at wiring time, at NEW GAME, and after a load.

**The save carries it: version 10.** One byte appended to the session block; v9
saves load as survival by definition (they predate the mode), and the loader accepts
both. The HUD stops pretending: `FUEL INF - CREATIVE` instead of a kilogram count
that cannot fall — and the low-fuel warning cannot go red over a number that means
nothing. Verified end to end on the real binary: toggle, `New game (CREATIVE)` in
the log, the HUD line, F5 (v10 on disk, flag byte set), F9, still creative.

### F14c — the map camera learns to visit (done)

**Tab, in the map, cycles the body the camera orbits.** The default is unchanged
and stays the natural one — AUTO follows the controlled craft's SOI primary, the
smallest body whose gravity owns you — but planning an arrival is done from the
OTHER end: drag a node for a Jupiter transfer and the encounter markers land on a
pixel three screens from home. Tab now walks AUTO → Sol → Terra → Luna → … →
Triton → AUTO (Shift+Tab backwards), and the map camera rides the chosen body
wherever the craft is. This is a camera, not a frame of physics: nothing about the
craft, the prediction or the node changes — only where the eye stands.

**And the zoom follows the body, which is what makes the cycle real.** The first
build cycled correctly and LOOKED broken: a height tuned for Terra (6e7 m) is
inside Sol (radius 7e8 m) and inside Jupiter, so Tab appeared to skip the giants —
the camera was standing in their interiors. Arriving on a body now reframes to at
least four of its radii, and the zoom floor is per-body (1.5 radii of whatever the
camera orbits) instead of one global constant that three bodies in the system are
bigger than.

Three small rules keep it honest. The focus is VIEW state, not save state: a new
game and a load both reset it to AUTO, because an index into a rebuilt celestial
table would point at whatever now lives there. In flight Tab still toggles chase
and free camera — the same key, owned by whichever view is up, never both. And the
map's mode line says what the camera is doing (`MAP  FOCUS AUTO TERRA - TAB
CYCLES`, or the picked body's name), because a control that exists in one view
only is a control nobody finds.

### F14d — planets stop shouting from orbit (done)

**The mid-range band looked crumpled into ten-kilometre mountains, and the number
says why.** Three relief pipelines light the ground, by distance: the terrain patch
up close, the per-fragment procedural path under 1.6 R (normal exaggeration
**x5.5**, tilt capped at 45°), and the baked vertex normals of the LOD spheres
everywhere else — still carrying M22's **x220**, tuned when they were the only
relief in the game, with no cap at all. Forty times louder than the path it hands
over to: from orbit the planet read as foil, then ironed itself smooth on approach
exactly at the 1.6 R line.

The vertex exaggeration is now 22 on the closest LOD (its gradients are sampled
over a ~25 km arc, already smoothed, so a little more than 5.5 compensates), 9 on
the next, and the tilt is capped at 45° like the shader's `kMaxNormalTilt` — the
saturated-everywhere look WAS the missing cap. Albedo relief (snow lines, rock,
altitude tints) is untouched: from space a mountain range should read as colour
and, at the terminator, a whisper of shading — not as geometry the size of the
atmosphere. One constant per LOD in `makeSphereLodSet` if taste says otherwise.

### F15 — the Endurance, and learning to fly it (done)

**The ring ship from Interstellar, and the rule it proves: a famous silhouette is
just a blueprint this engine can already express.** Ten `.swpart` files — EN-1
habitat, EN-2 propulsion, EN-3 command, EN-4 connecting tunnel, EN-5 Ranger, EN-6
Lander, EN-7 cargo pod, EN-8 cryogenic bay, EN-9 core docking hub, EN-10 core
spoke, stable ids 200–209 — and not one of them knows it is part of a ring. Each
is authored like every part before it: composed primitives, nodes on the collider
surface, authored hitboxes, honest masses. The SHIP is `buildEndurance()` in
GameScene: a 35-part programmatic blueprint pushed through the very same
`instantiateBlueprint` the hangar uses, so every module arrives as a real part
entity with a real `HullComponent` — you can walk into the Endurance, not through
it — joined by real breakable joints.

**The first pass was built from memory and it was wrong in the way memory usually
is: confidently, in the shape of a different ship.** Reading the design notes
instead of recalling them gave the real parts list: twelve modules of five kinds
(four PROPULSION, three plasma engines apiece for twelve nozzles in all; four
detachable CARGO pods, which are meant to come off and be the ground base; two
HABITATS wearing photovoltaic arrays; one CRYO bay; one COMMAND module with the
cupola), strung on twelve short TUNNELS, because the modules do not touch. Inside
sits the core docking hub — six berths, the "mounts six support craft at one
time" the design claims — carried on six spokes. The ring is what you recognise;
the core is what you dock with.

The trigonometry is one paragraph and it now checks itself twice. Module centres
sit on the apothem a = 29.4 m of a regular dodecagon, so the outer faces land at
32.0 m and the ring measures the film's 64 m across; each polygon vertex is
exactly a·tan(15°) = 7.88 m along the tangent from its module's centre, leaving
1.68 m of half-tunnel; and the hub's 3.4 m skin plus a whole 23.6 m spoke lands on
the inner faces at 27.0 m to the centimetre. `buildEndurance` compares the first
against the tunnel actually shipped, and `EnduranceCatalogueDescribesAFlyableRing`
asserts all of it against the shipped catalogue — widen a module in Part Studio
and a test says so instead of twelve joints quietly opening. **Nodes are resolved
BY NAME**, never by index: a blueprint that says "node 2" re-plumbs itself the day
somebody inserts a port, and one that says "dock" fails loudly.

**VESSEL FRAME: the ring axis is Z and the nose is −Z**, because −Z is where this
engine's thrust points and the Endurance is meant to be flown. That single choice
is why the twelve nozzles, authored on each propulsion module's own aft face, all
fire out the ship's tail without a special case.

**And it is a ship, from the first frame of a new world.** The root carries
`ShipComponent`, controls, SAS and an `AeroStateComponent`, so `P` boards it like
any other vessel and the bubble converts it to a real dynamic body the moment it
becomes the focus. It flies on what the catalogue says: four propulsion modules of
22 kN at 4,200 s — plasma, not chemical — for 88 kN over roughly 500 t, and 60 t
of propellant in those same modules, which is a little under 4 km/s of budget.
Power is the habitats: 400 MJ of batteries apiece and 120 kW of photovoltaics
between them, `Battery`-typed because `SolarChargeSystem` only pays a vessel that
carries one. **AeroForge solved a table for all ten parts** — the boxes come out
at Cd ≈ 1.20 head-on and the tunnel and hub at 0.80 side-on, which are the
textbook numbers for a box and a cylinder, and the Ranger is classified WING with
Cl 1.73 at 15°, which is what a lifting body should be.

Fuelling is explicit, and that is a rule showing its work: `instantiateBlueprint`
fills only parts whose TYPE is `FuelTank`, which is what makes a tank a tank. The
Endurance has none — its propellant lives in its engines and its joules in its
habitats — so it is fuelled after assembly, exactly the way the starting outpost's
battery bank is charged.

**It spins at the film's 5.6 rpm, and stops when you take the controls.**
`RailsSpinSystem` gives anything carrying both a `SpinComponent` and an
`OnRailsComponent` an ANALYTIC rotation, angle = rate × present, for the reason
the day cycle is analytic: under warp the Physics lane drops the backlog it cannot
afford, and an integrated spin falls behind the orbit it is riding, then reloads at
an angle it never passed through. The axis is read as *where the craft's own +Z is
held in the world*, with the base derived by the shortest arc — a closed form has
no attitude to build on, and deriving it means a ring can be parked in any plane
without a component to store it. Rate zero means "not spinning", never "snap
back". Because 5.6 rpm is also 0.59 rad/s of roll — a ship nobody can fly and a
camera nobody can watch — boarding de-spins the ring, the way its crew does before
every manoeuvre in the film, and it stays stopped: restarting it is a crew action
this game does not have yet, and pretending otherwise would mean a ship that
silently starts rolling the moment you look away.

Parked, it rides an `OnRailsComponent` around Saturn: a = 4.0e8 m — 6.9 Saturn
radii, clear of the rings, inside Rhea — with `dynamicMass` summed from the
catalogue so the hand-off wakes it at the mass `VesselAssemblySystem` immediately
recomputes from its parts. `Tools/endurance_preview/render_endurance.py` is the
CPU twin of the blueprint math — same constants, same two joint checks — for a
GPU-less look at the assembly. **224/224 tests, both build types.**

### F16 — physics warp to x100, because ion engines are slow (done)

**A plasma engine pushing five hundred tonnes at 88 kN is 0.18 m/s².** That is
four real hours of burning for a 2.5 km/s plane change, and rails warp cannot
help with any of it: above physics warp the integrator is switched off and the
world goes analytic, and an analytic orbit has no engines. The Endurance arrived
flyable and immediately proved that the interesting half of flying it was
unreachable.

**The x5 ceiling was never a stability limit, and reading the lane is what showed
it.** `SimulationLane` integrates a FIXED step — 1/50 s — at every time scale;
`timeScale` changes how many steps run per rendered frame, never how long one
step is. So the integrator at x100 does precisely the arithmetic it does at x1, a
hundred times over: same truncation error per step, same contact solver, same
everything. What x5 actually encoded was `maxTicksPerFrame = 16`, and sixteen
50 Hz ticks at 60 FPS is x19 and no more. The old comment said "integration at
50 Hz stays stable to x5" and had quietly turned a budget into a law.

So the budget became a knob. `setMaxTicksPerFrame` is raised to what the selected
rung needs (x100 at 60 FPS wants 84; the cap is 128 with slack) and dropped back
to sixteen the moment the rung does — a big budget is only wanted while somebody
is spending it, because at x1 it is also the bound on how much simulation a
single hitch frame may burn. Nothing here can desynchronise: strict catch-up
already holds the master clock to what the lane consumes, so a budget the machine
cannot afford makes the world run *slower*, never wrong.

**The altitude ladder was answering a question nobody had asked it.** Its whole
subject is how far an ANALYTIC jump may throw a craft between two rendered frames
— which is why a million times real time is refused near a planet and allowed in
the deep. None of that applies to physics warp, which takes its usual 1/50 s
steps and therefore cannot skip an SOI boundary, miss a contact or outrun the
terrain. The ladder now caps rails warp only and never pulls the ceiling below
`kMaxPhysicsWarp`; that single line is what unstuck a burn in low orbit from x10.
The exception is air: `kMaxAtmosphericWarp` holds x10 inside an atmosphere, and
that limit is human rather than numerical — a winged vehicle at x100 covers a
kilometre between two of your heartbeats and no reflex closes that loop.

**And the HUD now prints what the hardware paid.** `achievedTimeScale()` is the
clamped master-clock advance over the frame, smoothed across about a second; the
warp line shows it beside the request whenever it falls more than 10% short, so
`WARP X100-X47` says "this box gives you 47 of the 100 you asked for" where
silence used to say "warp is broken". Three tests pin the lot: that every dt
handed to a system is the lane's own step at x1, x5 and x100; that raising the
budget on an unchanged lane takes it from 4 ticks a frame to the 50 that x100
actually needs; and that the achieved figure converges on x8 when the budget is
short and on x100 when it is not.

For the Endurance that is a full tank — 60 t of propellant, ~5 km/s — burned in
under five real minutes, with every newton of it integrated.

### F17 — the gas giants stop looking like paint, and warp stops locking (done)

**A frame from ten thousand kilometres over Saturn showed a flat grey-brown ball
covered in a regular lattice of dark ellipses.** Both halves of that were the same
bug, and the fix was to stop asking a noise lookup to be a band.

`gasGiantAlbedo` built its banding by reading a 3-D value noise along a line —
`fbm3(vec3(0.37, latitude * bands, 0.71))`. From orbit that *is* banding. From ten
thousand kilometres you are inside a single lattice cell, so the planet renders as
one flat colour with the cell lattice showing faintly through it: exactly the
reported frame. The band profile is now PERIODIC — a zone, a belt, a zone, from a
`sin()` — because a sine has the same amplitude whether you can see a hemisphere
or a hundred kilometres of it. Its phase is bent by turbulence sampled **six times
finer in latitude than in longitude**, which is the whole trick: zonal winds
stretch eddies along the flow, and anisotropic noise turns round blobs into
ribbons. Two further octave bands of churn are added and each is faded out by the
fragment FOOTPRINT once a pixel covers more of the planet than the detail is wide
— the same discipline the ground path uses, so nothing here can alias from orbit,
and the vertex-coloured LODs get a footprint of 0.03 rad (the angle between two
LOD0 vertices) which switches every churn octave off.

The palettes were repainted at the same time, and measured rather than eyeballed:
rendering the CPU twin at the reported altitude and taking percentiles of the
result, Jupiter's luminance spread across the frame went from **28 levels out of
255 to 60**, Saturn's from **16 to 39**, on a tone curve whose white point at 4.0
compresses a 2:1 albedo ratio into about 13% of output range. That curve is why
the old numbers looked like mud: the contrast has to be in the material because
the grade will not give it back.

**Then a frame of the WHOLE disc showed the rest of it: a beach ball.** Evenly
spaced stripes of identical width and identical darkness, ruled edge to edge — and
a single sine wave cannot draw anything else. The band profile is now a FIELD:
five octaves of noise in latitude alone, so its cells *are* bands at every scale,
from the six major belts down to the sixteenth of a belt the fifth octave draws.
No two are the same width. A second, slower field decides how deep each belt runs,
which is the difference between banded and striped; a `smoothstep` with a
per-planet edge width turns the field into plateaus, because a raw fbm is a
gradient and reads as haze; and every one of these planets gets its bright
EQUATORIAL ZONE, which is what makes the banding read as a system with an axis
rather than as wallpaper. Measured over two thousand latitudes, Jupiter's albedo
now spans 0.32–1.01 against a belt colour of 0.27 — the belts reach their paint.

**And the disc got LIMB DARKENING, which is the difference between a sphere and a
painted ball.** On a rock the disc really is nearly uniform: you see the same dirt
at the centre and at the edge. On a gas giant you do not — near the limb the line
of sight enters the cloud deck at a grazing angle and never reaches as deep, so it
gathers less scattered light. Every photograph of Jupiter has that dark rim, and
without it the banding sat on the disc like a decal. A power law on the facing
cosine in `Mesh.frag` (the cheap stand-in for the Minnaert law, indistinguishable
from it at the angles a planet is ever seen at), mirrored into the preview tool's
harness so the tool keeps telling the truth about what the game draws.

**And a second bug fell out of reading the atmosphere code: `atmospherePreset` had
two branches, Mars and Terra.** Every gas giant in the game was wearing Earth's
blue Rayleigh sky. Jupiter and Saturn now get hydrogen — a third of air's
scattering cross-section, far less blue-weighted — under a warm ammonia Mie haze
that gilds the limb; Uranus and Neptune get the blue that is really methane
ABSORPTION eating the red; Venus gets an all-Mie ivory deck. The layer over a gas
giant is deliberately thin (120 km): what the renderer is drawing IS the cloud
deck, so a deep column would only extinguish the thing it is meant to sit on.
Tried at 320 km with Saturn's real 59 km scale height first — it dimmed the disc
by a fifth and bought nothing. The two twins of `gasGiantAlbedo` were checked
mechanically afterwards: **118 numeric constants, same values, same order.**

**The warp lock was the other half of the report, and it was asking the wrong
question.** `warpPermitted` refused anything that was not standing on the ground
or in a closed orbit clearing the atmosphere. Sound intent — rails cannot express
decay — but as a standing condition it fires on transients: nudge a burn and the
orbit is briefly open, the ladder slams to the physics ceiling and STAYS there,
which is how a craft 10 357 km over Saturn with a periapsis 6 000 km clear of the
air ended up pinned at x5 with nothing on screen to explain it.

The ordinary ceiling is now a question about WHERE YOU ARE and nothing else, and
it is self-enforcing in a way the trajectory test never was: re-evaluated every
frame, it walks a falling craft down the rungs as it descends — through the
physics band and into the atmosphere's **x5** — instead of refusing it once, from
high up, on a prediction. What the old gate protected against is what descending
now does by itself. The step down sits exactly at the top of the atmosphere,
because entering the air is the one boundary a pilot can feel; and above it the
ladder is written in **body radii**, because the same number has to mean the same
thing at Terra and at Saturn, which is nine times wider — in kilometres, "deep
space" would have begun while you were still skimming Saturn's cloud tops. Every
gravity source votes and the nearest one wins, judged in its own terms: its air,
its radius. The reported frame is worth x1000 now.

`warpPermitted` survives, scoped to the one warp that still deserves it: the
multiplayer SYNC, which skips whatever hours separate two clocks and is the only
warp that bypasses the altitude ladder. **228/228 tests, both build types,
`PARITY OK`** — including a new case that pins the ceiling at the atmosphere on
two worlds whose atmospheres differ thirtyfold, reproduces the reported frame, and
checks the ladder is monotonic in altitude, which is the property the whole
self-enforcing argument rests on.

### F18 — continents, measured against the only planet we have a photograph of (done)

**The land mask was a single call to fBm thresholded at sea level, and a plain
fBm carries the same structure at every scale.** What that produced was not
confetti, which is what I first said: it was the opposite. **53.4% of the globe
was land** — one sprawling continent with inland seas, a world with lakes rather
than a world with an ocean.

The mask is now two fields. A LOW-frequency one of three octaves pushed toward
plateaus and basins by a smoothstep — the plate, whose gradient at the shoreline
is STEEP, which is what makes the second field behave: finer noise then displaces
the coast by tens of kilometres instead of punching holes through the middle of a
continent.

**Every constant in it was found by search, not by eye.** `pip install
global-land-mask` bundles Earth's real land/sea raster; `Tools/earth_reference/
continent_stats.py` measures it on an equal-area grid and measures the procedural
field the same way, giving three numbers to aim at: 28.9% land, largest landmass
54.3% of it, top five 95.8%. 360 parameter combinations were scored against them.
The shipped field measures **26.5% land with its largest continent at 54.1%**,
in four masses. What it still lacks is Earth's fringe of small islands, which
hold 4% of the land — the reason our top-five figure is 100% and Earth's is 96%.

**And the diagnosis that cost two sessions.** The pale patches all over the ocean
that started this were never land: they are the CLOUD DECK. The preview tool's
header said "no clouds" and had said so since M28 added them, so three successive
hypotheses were tested against an instrument that was lying about what it drew.
What finally settled it was rendering the land mask ALONE as a flat map, outside
the shader — it was clean. The comment is fixed, and it now says what it cost.

**Three tests rebased, none of them softened.** Determinism went from "more than
150 of 200 samples differ" to `differences == landInEither`: 101 samples fall on
land in one world or the other and all 101 differ, while the other 99 agree
because both read exactly 0 — they are at sea, and a quarter of a planet being
ocean is not a failure of decorrelation. `TerrainV2PreservesEveryCoastline`
pinned a contract F18 breaks on purpose and is retired, replaced by
`TerrainIsShapedLikeAPlanetAndNotLikeConfetti`, which walks twelve great circles
and counts shoreline crossings: 8 at worst, where a shredded coast gives dozens
whatever its land fraction. And the mesh-versus-collider probe was re-surveyed —
its old site is open ocean now, and the first replacement went to the other
extreme (143 m of relief in 400 m, which no mesh follows); the final one is
ordinary hill country, 817 m up with 34 m of relief, which is the case the
resolution contract is actually about.

### F19 — the moons get craters, the air gets a twilight, the rings get a shadow (done)

**THE MOONS.** Twelve worlds rendered as grey speckled balls; they are now
dominated by impact craters across six size classes, from 140 km walled plains
down to 700 m pits, each with a darker floor, a bright rim and an ejecta blanket,
plus ray systems from the youngest impacts. Two sites per lattice cell rather
than one, because one site of mean radius covers only 6% of the ground and five
classes of that is dots on a smooth ball; two takes a class to 19% and six
classes to saturation, which is what a highland is. The field's mean was measured
at +0.0248 over four million samples and is compensated, so a body does not
change brightness as classes fade out with distance. Per-body character followed:
Callisto saturated wall to wall, Europa smooth young ice with almost none, Io
with **no craters at all** because it resurfaces itself, Enceladus's tiger
stripes, Ganymede's grooves.

**THE AIR.** Five things were keeping the limb flat, four of them physics the
model had left out. The TANGENT-HEIGHT sun path: past the terminator the ray to
the sun does not climb out of the air, it dives through it and comes back up on
the far side, so it is filtered at the tangent height and crosses that air twice
— the old code clamped the airmass at its horizon value, so light stopped
reddening exactly where a real twilight starts. OZONE, which absorbs and does not
scatter: invisible looking up (0.028 of an optical depth), worth 1.0 along a
horizontal path at 25 km, and the reason deep twilight is violet rather than
brown. A bounded MULTIPLE-SCATTERING gain, 3.3x on the limb and 1.3x at nadir,
which is the shape the real thing has. NIGHTGLOW, integrated analytically because
a 20 km shell falls between two samples of any march that can afford to exist.
And an ENERGY-CONSERVING segment integral: a rectangle rule cannot know the slab
it integrates is opaque, and a limb ray is eighteen optical depths of opaque —
the old march came out 30% bright on the rim's brightest pixel, and eight steps
now land within one grey level of a hundred and twenty-eight.

The shell's blend was wrong in a way only the nightglow exposed: it took its
alpha from `1 - transmittance`, which counts the same factor twice and hands
alpha near zero to a band that EMITS and has no extinction of its own. Above
about 72 km the airglow was being discarded outright. Coverage now comes from the
graded colour and the colour is pre-divided by it.

**THE RINGS.** `ringOpacity(r)` is now the ring — edges, Cassini, Encke, ringlets
— written once and mirrored exactly (worst delta over 200 001 radii: 0.000e+00),
called BOTH by the mesh that draws the annulus and by the planet's shading to
work out how much sunlight a ring radius takes from the cloud tops below it. A
shadow cast by different gaps than the ones you can see is worse than no shadow.
The band of ring shadow lying across Saturn is in every photograph of the planet
and in almost no game; it costs one ray-plane intersection in the body frame,
where the equator IS the ring plane. Grazing light crosses more ring material
exactly as it crosses more air, so the shadow is darkest when the sun is lowest
over the plane. The annulus went from 40 radial steps to 160 — radial is where
all of a ring system's structure lives — and the C, B and A rings no longer share
one colour.

That shadow shipped WRONG and the bug is worth keeping written down, because it
is a units mistake wearing the costume of a physics one. Grazing light really
does cross more ring, so the first version multiplied the ring's opacity by the
slant factor `1 / |sun.y|` and subtracted the result from one. But an opacity is
already `1 - exp(-tau)` — it is a fraction, it saturates at one, and it has no
business being multiplied by anything. At a slant of twelve an opacity of 0.9
becomes 10.8, and `1 - 10.8` clamps to the floor: half of Saturn's disc went flat
black behind a perfectly straight edge, visible from the Endurance's orbit as a
bite taken out of the planet. The fix is to go back to the quantity that IS
linear in path length — the optical depth `tau = -log(1 - opacity)` — scale THAT,
and come back out through `exp(-tau * slant)`. The slant is capped at 3 and the
result floored at 0.18: the sun is never in the ring plane for long, and the
alternative to a cap is a silhouette. Anything that scales with distance
travelled scales in log space; opacity, alpha and coverage never do.

**THE AURORA (F20).** It was blocked on a mesh, not on optics: the atmosphere
shell was drawn at 83 km, and a layer above the mesh has no fragment to be
drawn into. Raising it to 350 km costs 8% more sky at the same distance and
buys both the nightglow at its real altitude and the curtains. What is drawn is
an OVAL, not a cap, tilted 4.5 degrees toward midnight off a magnetic axis
tilted 11 degrees off the spin axis — one displacement, and the ring sits at
magnetic latitude 67 at midnight and 76 at noon, keeps facing the night with
nothing telling it where the night is, and needs no time input. The curtain
pattern is a function of DIRECTION ONLY, which is the whole trick: feed the
same value to samples at 100 km and at 300 km and a twelve-step march draws a
vertical sheet for free. Green (557.7 nm) is quenched below 90 km and out of
oxygen above 185; red (630.0 nm) needs 110 seconds to radiate, so it is
hopeless below 200 km and easy above it — which is why a strong display is
green with a red crown and never the other way round.

The radiance is NOT physical and the first version's was. An IBC III display
puts the green thread at a linear 0.06 seen from orbit; that measured out at
sRGB 93 green over a night side at 58, a grey-green fog twenty levels above the
ground it was lighting. Nothing was wrong with the number — the fault is
downstream and unfixable from the shader. The grade lifts blacks to sRGB 37 so
shadows breathe and desaturates by a fifth for film; both are right for a lit
frame and both are poison for a faint one, because an additive lift eats chroma
in proportion to how dark the subject is. So the amplitude is set from the
OUTPUT: 4.5x, landing the brightest thread near linear 1.0 and roughly
(135, 250, 153) after the grade. That is a long exposure rather than an eye,
which is the only aurora anyone has actually seen. Twelve march steps and
forty-eight produce the SAME image to the byte — the banding on an oblique pass
is the ring seen edge-on, not the quadrature, and that was checked rather than
assumed.

## Milestone 33 — the night the renderer got a mirror

**THE TOOL FIRST, AND IT PAID FOR ITSELF IN AN HOUR.** `--capture <path>` writes
the last frame of a `--frames N` run to a PNG, straight off the swapchain, with
a forty-line deflate-stored PNG writer that needs no zlib. Together with
`--cpu` (llvmpipe), `xvfb-run` and the `SW_SHOT=BODY@RADII` camera, a build can
now be POINTED AT A PLANET AND PHOTOGRAPHED, headless, from a shell script.

That matters because the CPU preview — the instrument this project had been
judging planets with for four milestones — ray-casts the FRAGMENT path and
nothing else. It cannot see which pipeline an object took, whether it was
shaded per-vertex or per-fragment, how a blend resolved, what a mesh's
silhouette does, or what the tessellation leaks into the shading. Every bug
below was invisible to it, and every one of them was in the shipped frame. The
preview was right and the game was wrong, for months, and nothing in the loop
could tell the difference.

**THE BUILD WAS LYING.** The shader mirror step was a POST_BUILD on the
executable, so it only ran when the executable was relinked. Edit a shader and
nothing else: glslang recompiles the SPIR-V into the build directory, the link
step has nothing to do, the copy never runs, and the game loads the PREVIOUS
shader out of `bin/Shaders`. The build succeeds. Nothing warns. The obvious
conclusion — "my change did nothing, so the theory must be wrong" — is exactly
backwards, and it cost a diagnostic that read as a disproof. The copy hangs off
the shader target now, which is always out of date, so it always runs.

**EVERY OUTER PLANET WAS UNLIT.** `vSunDirBody` was computed as
`transpose(rotation * radius) * (sunPosition - worldPosition)` and normalized in
the fragment shader, on the reasoning that the length is irrelevant. The
mathematics agrees; f32 does not. Saturn is 1.43e12 m from the sun and 5.82e7 m
in radius, so the product is 8e19 — and `normalize` SQUARES its argument first.
6.9e39 is past the f32 ceiling of 3.4e38, so the length was infinity, the
direction NaN, and every lambert term on the planet zero. Jupiter, Saturn,
Uranus and Neptune were drawn with nothing but the 2% cold ambient. That is why
they looked like grey balls no matter what the surface shader did, and it is
why the player kept reporting it. Terra survived on luck: 6.37e6 x 1.5e11
squares to 9e35, one part in four hundred under the ceiling. Normalize before
the matrix and every intermediate stays near 1.

**THE EXPENSIVE PATH WAS OFF WHERE IT WAS CHEAP.** The per-fragment planet
shading was gated on `distance < 1.6 radii` — 93 000 km on Saturn — with the
reasoning that it should only run where its sharpness shows. Backwards: a
fragment shader costs per PIXEL, and a planet's pixel count falls as the square
of the distance. At two radii it runs on a third of the screen, at seven on two
per cent of it. The gate was turning the good path off exactly where it was
free, and handing the body to the vertex path, whose cost is fixed and whose
quality is a LOD sphere's worth of quads. There was nothing left to trade.

**THE MESH WAS LEAKING INTO THE SHADING, THREE WAYS.** A latitude-longitude
grid, four grey levels deep, lay over every gas giant. It was blamed on the
mesh, then the derivative, then the noise's interpolant, then the dither, then
the churn octaves — six hypotheses, each measured and each wrong — before a
high-pass of a headless capture and a term-by-term bisection found the three
real culprits:

- `fwidth(normalize(modelPosition))` as the LOD footprint. It asks the
  RASTERISER how fast the body direction changes, and the rasteriser answers
  with a number that has the tessellation in it. Replaced with the analytic
  footprint — pixel angle times distance over radius times cos incidence — the
  same formula the CPU preview has always used, which is precisely why the
  preview never showed the bug;
- the shading DIRECTION itself, taken from the interpolated mesh position,
  which lies on the flat facet rather than the sphere. Replaced with a ray-
  sphere intersection in body space from a `flat` camera position: one
  quadratic, no derivatives, and the mesh goes back to being a silhouette and a
  depth value;
- and the one that was actually most of it: the AERIAL-PERSPECTIVE MARCH IS
  CLIPPED BY THE DISTANCE TO THE FRAGMENT, and that distance was the
  interpolated one. The facet sits up to a sagitta inside the true surface —
  eleven kilometres on Saturn — and steps from facet to facet. A six-step march
  is sensitive to where it is cut off at the percent level. Three quarters of
  the whole artefact was that one number.

A gas giant's visible surface IS the top of its own atmosphere, so the march
does not belong there at all now; rocky worlds keep it, where the column
between the ground and orbit is real and is the blue.

**THE SKY IS TRANSLUCENT.** The star dome was drawn OPAQUE, with brightness in
its RGB and alpha pinned at 1. The grade lifts blacks to sRGB 37 against a sky
of 13, so the dimmest thing an opaque dome can draw is a light grey — there is
no FAINT in it — and every quad is therefore a step. Twenty-four thousand grey
squares is not a galaxy, it is static, and that is what the player saw. Blended,
with brightness on the alpha and hue alone in the RGB, a mote can be worth one
grey level. The blobs are soft (a centre vertex at full opacity, a rim at zero,
four triangles) because a quad has four corners and one colour and cannot fade
out at its own edge. The day fade falls out of the same arithmetic for free: the
instance tint multiplies the whole alpha and opacity is `alpha - 1`, so a
brightening sky erases the faint stars first and Sirius last.

Two things had to move for that to work. A dome the camera sits INSIDE has its
bounding centre at the camera, so the back-to-front key sorted it nearest and it
painted over the atmosphere; it now declares its radius as its sort distance.
And the emissive convention uses the INTERPOLATED alpha as its material flag,
which a soft star's rim has to cross — so every blob came out as its own
wireframe outline, the rim ring taking the lit path while the middle took the
emissive one. The dome has its own route off the `flat` instance tint now. A
flag and a value should not share a channel.

**THE GRADE'S FLOOR WAS PAINTING A HALO.** The atmosphere shell takes its alpha
FROM its own graded colour, and every term of the grade has a floor: the black
lift adds 0.016 and the contrast pivot another 0.008. `gradeCinematic(0)` is
therefore 0.024, twelve times the 0.002 the vacuum test compared against — so a
ray through empty space painted a pale grey at 1% alpha. With the shell at 83 km
that was a thin rim nobody questioned; F20 raised it to 350 km for the aurorae
and it became a hard-edged grey disc half again Terra's radius, in every
orbital screenshot. Vacuum is tested on the RADIANCE now, before the grade, and
anything painting over an already-graded frame uses `gradeCinematicCore`, which
has no lift: an image-wide decision belongs to the image, not to each surface
in it.

**AND THE SHADOWS WERE PICKED BY ARRAY ORDER.** The eclipse test takes eight
occluder spheres and the game filled them with the first eight celestial bodies
in the index — Sol, Terra, Luna, Phobos, Deimos, Mars, Mercury, Venus. The cut
fell before Saturn, every frame, so the one body in the game with a ring system
was the one body that could not cast a shadow: its rings ran straight through
its own shadow cone as if the planet were not there. They are ranked by ANGULAR
SIZE now — (distance / radius) squared, smallest first — because a shadow
matters when the thing casting it is big in the sky, and Luna at 400 000 km
beats Jupiter at five astronomical units on both counts.

Putting Saturn in the list immediately drew a hard vertical cut down the middle
of its disc, and that was correct behaviour from the wrong premise: a body was
eclipsing ITSELF. Its own night side belongs to the lambert term, which has a
deliberately soft terminator standing in for scattering and bounce; the eclipse
test has no such thing and goes from lit to black across a penumbra measured in
miss distance. Anything within four thousandths of a radius of a sphere's
surface is ON it — 25 km on Terra, above the highest ground and below any orbit
— and skips that sphere.

**THE RINGS ARE A SLAB, NOT A SURFACE.** They were lit with a lambert on a
vertex normal, which gets wrong the one thing everybody knows about Saturn's
rings: that they look completely different from the two sides. Lit from the
front the dense B ring is the brightest thing in the system; lit from behind it
is the DARKEST, because dense is exactly what light cannot get through. So the
material is now single scattering through a plane-parallel slab of optical
depth tau — Chandrasekhar's case, four exponentials — with an opposition surge
at zero phase (every particle hides its own shadow; worth 45% over a couple of
degrees) and a forward-scattering peak on the backlit side, which is the
sunbeam through the C ring. The radius comes from the model position, so every
ringlet `ringOpacity` knows about is drawn at PIXEL resolution instead of at the
mesh's 160 radial steps, and the mesh goes back to being geometry.

Coverage is `1 - exp(-tau / mu)`, not the opacity: a slab seen at a grazing
angle is thicker along the line of sight, which is why a ring system closes up
into a solid band as it turns edge-on. And the planet's shadow falls across the
annulus from the same occluder spheres the surfaces use, with Saturnshine — a
warm inverse-square fill off the lit disc a radius away — keeping the shadowed
part from being a hole cut in the frame.

**THE SUN WAS AUTHORED AS A COLOUR WHEN IT IS A BRIGHTNESS.** It measured
sRGB 190 against a background star at 140 — thirty-six per cent brighter than a
speck — and the reason is one line of arithmetic: the grade maps a radiance of
1.0 to 0.43 and only reaches white at 4.0, so a disc authored at (1, 0.99, 0.94)
could not come out brighter than 176 no matter what was done to it. The
photosphere now carries a radiance of EIGHTEEN and the glare's core THIRTY-FOUR,
four and eight times the white point, so both clip — and the margin is the
point, because it keeps the white region's edge in the same place as the sun's
apparent size changes from Mercury to Neptune instead of the whole disc fading
as it shrinks.

The glare is three layers with a power-law profile in twenty radial rings, not
one linear ramp between a centre vertex and a rim: light spread by an optic
falls off steeply and then trails, and a straight line does neither. Three
things had to be got right and each was wrong first:

- ONE COLOUR PER LAYER AND THE FALLOFF IN THE ALPHA. Both were in the colour to
  begin with, and the sun came out as a flat cream BALL with a hard edge — a
  src-alpha blend is not an add, so a layer with high alpha REPLACES what is
  behind it, and a faint outer glare at high alpha is just an opaque disc that
  happens to be dim;
- the layers are ROUTED TO THE SOFT-EMISSIVE BRANCH, the one the star dome uses,
  because the plain emissive convention flags itself with the interpolated
  alpha and a glare must reach zero opacity at its rim — which is alpha exactly
  1.0, the wrong side of the test. Everything below it fell through to the LIT
  path and was shaded as an ordinary grey surface, so each layer drew a solid
  RING at its own edge. The sun had two of them around it, and the same bug had
  already turned every star into a wireframe quad a milestone earlier;
- and a FLOOR ON THE ANGULAR SIZE. From Saturn the sun is a tenth of a degree
  across, the core disc lands inside one pixel, and the rasteriser covers that
  pixel only partially — so the brightest object in the solar system came out
  DIMMER from the outer planets than from Terra, which is backwards twice over
  at magnitude -23. Every optic has a point spread function with a width of its
  own; below it a source stops shrinking and only fades, which is why a star is
  a disc in every photograph ever taken.

The last of the missing brightness was the sun's own LENS FLARE. Ghosts are
off-axis reflections between lens elements: the further the source sits from the
optical centre, the further they walk across the frame, and at zero field angle
they land ON it. Here that meant a 16%-alpha amber disc painted over the sun,
pulling its clipped white core from 255 down to 243 — the star was dimming
itself with its own glare. The chain now fades out as the source approaches the
axis, which is both what optics does and what looks right.

**IS IT THE RIGHT SIZE? TWO CHECKS, BECAUSE IT IS TWO QUESTIONS.** An apparent
diameter is a radius over a distance, and either can be wrong without the other
noticing. `Tools/solar_scale/check_scale.py` parses the radii and semi-major
axes OUT OF THE SHIPPED SOURCE — `GameInternal.hpp` for the planets, the
`MoonDef` rows in `GameScene.cpp` for the moons — and diffs them against NASA's
fact sheets; a checker carrying its own copy of the data checks nothing but the
person who typed it twice. Twenty-two bodies, two discrepancies: Phobos was
11.1 km against a measured 11.267 (1.5% small) and the Sun carried the older
6.9634e8 rather than the IAU 2015 nominal 6.957e8 (0.09% large). Both corrected.
Everything else is exact: the Sun is 0.5334 degrees from Terra against a real
0.5329, Luna 0.5267 against 0.5267, Io 0.5934 from Jupiter's cloud tops.

`check_render_size.py` asks the other question — does the ENGINE draw them that
size — by parking the camera a known number of body radii out, photographing
the real frame and measuring the disc. Ten shots, all inside tolerance: the Sun
0.12%, Luna 0.16%, Jupiter 0.12%, Mercury 0.19%. The prediction is
`tan(asin(R/d)) / tan(fov/2) * height/2`, and the `asin` matters: the silhouette
is the TANGENT cone, not the cone through the equator, and the two differ by a
fifth from low orbit.

Two things had to be got right for that measurement to mean anything. A body
with air does not end at its ground, so the test is a BRACKET — at least the
solid body, at most the envelope its air is drawn in — which turned Mars's
apparent 2.6% oversize and Terra's 2.9% into two correct results, both being
the thickness of their own atmospheres. And the search window is the middle
forty rows of the frame, because the first version searched half the height and
found the NAVBALL: two measurements came back as exactly 202 pixels because
that is how wide the navball is, and one of them PASSED, Terra at eight radii
being 196 pixels across. A measurement that can accidentally be right is not a
measurement.

The Sun is measured with `SW_NO_GLARE=1`. Its glare is a deliberate optical
overlay three body radii wide with a floor on its angular size, so with it on
there is nothing to measure but the overlay — and the floor itself came down
from 0.0055 rad to 0.0022 (under two pixels of radius) once this check existed
to say what it was costing: at the old value the Sun's glare from Saturn was
eleven times the true disc, which is the kind of lie somebody measures.

**"MET TOI AU NIVEAU DE SATURNE ET ESSAYE DE TROUVER LE SOLEIL."** He was right,
and the floor above had only made the peak pixel correct. `SW_SHOT=SOL@2040`
parks the camera at Saturn's distance and photographs the frame; measured
against the brightest background star in the SAME frame, radius by radius:

| px from centre | 0 | 4 | 6 | 8 | 15 | 26 | 40 |
|---|---|---|---|---|---|---|---|
| sun, before | 255 | 145 | 83 | 62 | 41 | 18 | 18 |
| a background star | 170 | 137 | 115 | 82 | 59 | 43 | 18 |

The centre pixel was already clipped to white and the sun was still the SMALLER
and FAINTER object from two pixels out — it died at twenty-five pixels while an
ordinary star was still going at forty. That is why it read as a pinprick: what
makes a source look bright is not its peak, which an 8-bit frame caps for
everything, but how far out it stays above its neighbours. Three findings:

- THE EXPONENT BELONGED TO THE FAR WING. The glare's world radius grows as
  `d^(1-2/n)` for a point spread function going as `theta^-n`, and the first
  pass used n = 3.45 — a lens's CORE PSF. What you see around a source eight
  orders of magnitude past full scale is its VEILING GLARE, the scatter off
  every surface in the barrel, and that wing falls closer to `theta^-10`. At
  n = 10 the exponent is 0.80 and the glare goes 133 pixels at Terra, 85 at
  Saturn, 66 at Neptune instead of collapsing to 25. Distance still reads, and
  it reads through the disc underneath, which shrinks honestly as 1/d.
- THE OUTER LAYER NEEDED A TAIL, NOT A BUMP. The skirt term was hard-coded at
  `0.045/(1+30t^2)`, which is a thousandth of its peak a third of the way out.
  It is now per-layer, and the aureole carries a near-`1/t^2` wing that holds
  the glow up to its own rim.
- AND THE BLEND HAPPENS IN LINEAR LIGHT. The swapchain is `B8G8R8A8_SRGB`, so
  the hardware un-encodes both sides before mixing: an alpha of 0.25 puts a
  quarter of the layer's LIGHT on screen, not a quarter of its sRGB value,
  which is a much larger number once re-encoded. Modelled the naive way these
  profiles predicted 116 where the renderer measured 170 — so the first tail I
  tuned was fitted to a curve already half again too bright, and the sun came
  out as a smooth cream BALL with an edge on it. Forty percent, invisible from
  inside the shader, and the fix was to put the model through
  srgb->linear, mix, linear->srgb before believing it.

Measured after: 255 / 255 / 229 / 205 / 173 / 102 / 79, dying at seventy-eight.
Two to three times the star at every radius and twice as wide, which is what
magnitude -23 against -1.5 should look like when the exposure is fixed. The
ladder across the system is monotonic — Mercury reaches past 140 pixels, Terra
105, Jupiter 80, Saturn 78, Neptune 62 — and `check_render_size.py` still
reports ten shots, zero discrepancies, because it photographs the photosphere
with `SW_NO_GLARE=1` and none of this touches the body.

## Milestone 34 — twelve light-years, and the origin that moves with you

**"AJOUTE LES ETOILES PROCHES AINSI QUE LEURS PLANETES (TAILLE ET DISTANCES
REELLES) ET AJOUTE WARP X100M ET X1B QUI NE PEUVENT S'ACTIVER QUE QUAND ON SORT
DE LA SOI DU SOLEIL. LES SYSTEMES SOLAIRES NE SE DEPLACENT PAS."**

Twenty-four systems, thirty-six stars, twenty-five confirmed planets, and two
new rungs on the warp ladder. Four pieces, and only the first is data.

**THE CATALOGUE** (`Engine/Source/Space/StarCatalogue.{hpp,cpp}`) is every
system inside twelve light-years, with J2000 right ascension and declination,
modern parallaxes, interferometric radii where they exist, and the planets that
are actually confirmed as of 2026. It lives in the ENGINE rather than the game
because the part that can be wrong is not the numbers, it is the rule that
turns them into a frame: right ascension is an angle on the equator of a planet
this simulation does not privilege, and the world runs on the ecliptic with +Y
north. `equatorialToGame` is a rotation by the obliquity and a relabelling of
axes, and a sign error anywhere in it produces a sky that is perfectly
convincing and rotated 47 degrees, or mirrored. So it is tested against
arithmetic: the vernal equinox must come out as exactly +X, the ecliptic pole
at RA 18h / Dec +66.56 as exactly +Y, the solstice point with exactly zero Y,
and the cross product of the first two must be the NORTH pole and not the
south. Then the data is tested for self-consistency — every catalogued distance
against the length of its own vector, every planet outside its own star, every
hard-coded host index against the star name it was written for.

The check that turned out to matter most is that **Proxima and Alpha Centauri
land a fifth of a light-year apart**. Their catalogue entries differ by 0.098 ly
in distance and 2.2 degrees on the sky, and only a correct three-dimensional
placement turns those two numbers into the 0.21 ly the pair really is. A
mirrored or mis-rotated sky passes every distance test — distances from the
origin are rotation-invariant — and fails that one.

Two more rules are derived rather than tabulated, and both are labelled guesses
in the source. Nobody has photographed any of these worlds; what is known is a
mass, a period and a star. `equilibriumTemperature` gives 255 K for Terra by
construction, and `styleForWorld` maps mass and temperature onto the surface
presets the solar system already has, with the BANDS ANCHORED TO THE SOLAR
SYSTEM'S OWN BODIES — Mercury 440 K, Mars 210, Callisto 134, Europa 102, Triton
38 — so that fed the solar system's numbers the rule hands back the solar
system. That is tested too. The result is a neighbourhood with the variety it
really has: baked rock at Barnard, rust at Proxima b and GJ 887 d, cracked ice
further out, ice giants at Lalande 21185 c and GJ 15 A c.

**THE FLOATING ORIGIN** is not a convenience, it is the only way the numbers
fit. A double carries about sixteen digits, so at Proxima's 4.0e16 metres the
gap between two representable positions is EIGHT METRES: a craft in orbit there
would jitter by its own length, a landing would be a coin flip, and the terrain
— which subtracts two nearby world positions to find a patch origin — would
dissolve. The world's origin is therefore a star, and it moves; `rebaseOrigin`
shifts every absolute position in the game by the difference, and the same
craft is then at 1e7 metres where the gap is a NANOMETRE.

What makes it cheap is that the renderer was already camera-relative:
everything reaching the GPU is a difference, so a shift the camera shares is
invisible. What makes it dangerous is the handful of caches that hold an
absolute position ACROSS frames, and finding all of them was the work. Four
places: every `TransformComponent` and — easy to forget — every
`PreviousTransformComponent`, because missing the second does not look like a
missing shift, it looks like one frame of motion blur four light-years long;
all four cameras; the free camera's own private copy of the eye, which would
silently undo the shift the next time a movement key was pressed; and the
reentry-plasma particles, the one non-ECS array in the game integrated in
absolute space. The origin hands over at the MIDPOINT between two stars, not at
a sphere of influence, because between two systems there is no containing
system for light-years at a stretch and holding the origin at the star you left
would let a craft reach the eight-metre regime during the cruise.

**SYSTEM STREAMING.** All thirty-six stars exist for the whole session — you
have to be able to see where you are going, and a star four light-years away
costs one transform and one distance computation a tick. Their PLANETS are
built the frame the craft enters the star's sphere of influence, and are never
destroyed: the mesh registry is append-only, so a system that loaded, unloaded
and reloaded would leak eight GPU meshes a visit, and four entities in the
per-tick loops is much the cheaper of the two. Sol is never streamed at all —
the colony is on it.

**THE WARP RUNGS.** Two new ones, x100M and x1B, and three things stood in the
way. `Simulation::setTimeScale` hard-clamped to 1e7 (the comment above the
clamp warned about exactly this bug class — a button that works and does
nothing — which is how it was found). `warpText` had no branch for billions and
would have printed "X1000M", unreadable next to the rung below it. And the
gate: the altitude ladder tops out at 1e7 and knows nothing about stars, so
outside them it has to be OVERRULED rather than consulted, which is what
`phys::maxWarpForSpace` does — inside any star's sphere of influence it can
only lower the ceiling, outside every one of them it can only raise it. The
asymmetry is the whole rule. `containingSystem` answers the question, and it
asks about the ABSOLUTE position, because the origin travels with the ship.

Why the rungs must exist: Proxima is 4.0e16 metres away, and at ten million a
crossing is a fortnight of real sitting. At a billion it is seven minutes. Why
they must be gated: above physics warp a rung says how far an ANALYTIC jump may
throw a craft between two rendered frames, and at x1e9 a frame is half a year
of orbit — a craft would cross Neptune's entire sphere of influence inside one
of them and arrive with a conic that never noticed. Between the stars there is
nothing to notice.

**AND THEN THREE THINGS BROKE, EACH FOUND BY MEASUREMENT.**

The far plane was 1.0e13 metres — Neptune's orbit and a bit — so every star in
the catalogue was BEHIND IT and clipped away. Flying to Sirius meant flying
toward an empty patch of sky that stayed empty until the moment of arrival.
Reverse-Z is what makes 1.5e17 free: depth is stored with the near plane at 1.0
and the far at 0.0 in an f32 buffer, so precision is set by the near plane and
moving the far one out by four orders of magnitude costs a fraction of a bit.

"Is this the sun" was `entity == m_solEntity` in three places, and there are
thirty-six suns now. The light direction, the photosphere radiance and the
glare layers were all written against that one comparison. The photosphere is
now per star: eighteen is Sol's surface radiance and a photosphere goes as the
FOURTH POWER of its temperature, so Sirius B's 25 000 K is fifty-six times
Sol's per square metre and Proxima's 2992 K is a fourteenth of it — which is
also why the component carries surface brightness and not luminosity. Sirius B
is a thirty-thousandth of Sol's OUTPUT and fifty-six times brighter per unit
area, because the whole star is the size of Terra; drawn by luminosity it would
be a dim ember instead of the blue-white pinprick it is.

The third one was the good one. Proxima b came out as a salt-and-pepper mess,
and measured against Mars at the same distance and the same surface style it
carried **fourteen times the high-frequency noise** — 88 against 6.3 on a
Laplacian. Two hypotheses died first: the atmosphere (removing it made the
noise WORSE, 122) and the new far plane (reverting it changed nothing at all,
121.72 both ways, to the hundredth). What broke it was that the pixels were
BINARY — every one of them either (127,103,86) or (43,44,47), the planet's own
day side and its own night side interleaved. A per-pixel binary lit/unlit test
is a shadow test, and the shadow occluder list excluded `m_solEntity` by
handle. Proxima was not Sol, so Proxima was casting a shadow on Proxima b: the
cone's apex sits inside the source that is also the light, and the test comes
out lit or unlit essentially at random. Excluding everything with a
`StarVisualComponent` took it to 4.8, cleaner than Mars.

**WHAT IS NOT DONE.** Multiplayer beacons still put an absolute position on the
wire with no origin tag, so two players on different origins would read each
other light-years out; the replication encoder's sixteen-snapshot delta history
is not invalidated on a shift either. Both are noted rather than fixed —
interstellar multiplayer is a milestone of its own. The exoplanet air columns
are a first-order guess (a twelfth of the column for the scale height, Terra's
sea-level density scaled by surface gravity) because nobody has measured one.
And the surface styles are, as the source says, a rule applied to a mass and a
temperature — not a photograph of anything.

### F28b — the three things that were wrong about leaving

**"LA VITESSE X100M DOIT ETRE AUTORISE A PLUS DE 5 MILLIARDS DE KILOMETRES.
QUAND JE PLACE UNE NODE QUI SORT DE LA SOI DU SOLEIL IL Y A UNE QUANTITE
SUBSTANTIELLE DE LAG. RAJOUTER UN NOUVEL OUTIL POUR LES VOYAGES ENTRE
SYSTEME."**

**THE MIDDLE RUNG WAS ON THE WRONG BOUNDARY, by four orders of magnitude.**
Gating x100M on leaving the star's sphere of influence sounded right and was
not: Sol's SOI is two light-years, and a craft is past Neptune with nothing
left to encounter after five billion kilometres. That is where an analytic jump
between two rendered frames stops being able to skip a planet, so that is where
the rung belongs. `maxWarpForSpace` now takes the distance to the star as well
as the SOI test, and the asymmetry it already had grew a middle term: under 5e12
metres the in-system ceiling stands, past it x100M opens, outside the star
entirely x1B. The altitude ladder still wins wherever it binds — a craft landed
on a Kuiper object six billion kilometres out is in the ladder's territory and
must not be handed x100M because of where the SUN is.

**THE LAG WAS 341 MILLISECONDS PER PREDICTION, four times a second.** He was
right that it was the trajectory. Measured on an escape from Terra's orbit with
eight planets in the index:

| | ms per call |
|---|---|
| before | 341 |
| range cap alone | 138 |
| range cap + anomaly walk | **8.4** |

Two separate mistakes, and the first fix only found the second. A hyperbolic
escape from the outermost body has no sphere of influence to leave and no
revolution to close, so the scan ran to the twenty-year horizon: a line reaching
a hundred billion kilometres, then split into thousands of screen-space boxes by
the map, because every chord of it spans four orders of magnitude of camera
distance. Capping the plan at ten billion kilometres — past every planet, two
orders of magnitude short of the nearest star — stopped it at the right PLACE
and took less than a third off the cost, because the scan still had to WALK
there one step at a time.

The step is the second mistake. It is a fraction of the orbit's own time scale,
which is the right idea for a closed orbit and meaningless for an open one: for
a solar escape it is about an hour, against an arc eight years long. Seventy
thousand samples, eight child bodies probed at each. Spacing the samples evenly
in HYPERBOLIC ANOMALY instead puts them where the geometry changes — dense at
periapsis, spreading out as the arc straightens — so four thousand of them cover
the whole escape and still cannot step over a planet's sphere of influence. The
bound is a closed form: r = a(1 - e cosh H) inverts, and M = e sinh H - H is
linear in time. It applies to every hyperbolic segment, so a Terra flyby got
seventeen times cheaper too, and `SpacingSamplesByAnomalyStillCatchesTheSoiExit`
pins the exit radius to a part in ten thousand so the saving cannot quietly
become a truncation.

**AND THE NEW INSTRUMENT.** Between the stars there is no orbit to read. A conic
drawn around a star you have already escaped is a straight line to within the
width of the screen, and the map's whole vocabulary — periapsis, encounter,
closest approach — has nothing to say about a four-light-year crossing. Select
a star, escape the one you are at, and the HUD shows the heading error instead:

    INTERSTELLAR PROXIMA CENTAURI  4.246 LY
    DEV X +46.59  Y -24.12  Z +0.00 DEG   (TOT 52.47)
    CLOSING 73109 M/S  ETA 17.4 YR

The three numbers are ONE ROTATION written down three ways, not three separate
angles. Asking "how far off am I about X" three times over gives numbers that do
not compose — correct each in turn and you arrive somewhere else, because
rotations do not commute. The rotation that takes the current heading onto the
required one has a single axis and a single angle, and its components about the
three axes vanish together and each carries the direction to turn. Green inside
a degree, amber inside ten, red beyond: a crossing is four light-years long, so
one degree at departure is seven hundred astronomical units of miss at arrival,
and the colour is most of the instrument.

The trigger is the two-body ENERGY against the local star, not the shape of the
drawn plan — the plan now stops at ten billion kilometres, so a bound orbit with
an apoapsis past that looks exactly like an escape on screen. Energy cannot be
fooled that way: it is positive if and only if the craft never comes back.

## Milestone 35 — parts that move, and a way to make them move

**"IL FAUT QUE CERTAINES PIECES PUISSENT ETRE ACTIVEES PENDANT LE VOL OU AIENT
DES ANIMATIONS... POUR CELA IL FAUT AJOUTER AU LOGICIEL DE DEVELOPPEMENT DES
PIECES UN MOYEN DE DEVELOPPER DES ANIMATIONS."**

**THE BLOCKER WAS THE MESH.** A part was ONE welded mesh: every shape's pose
baked into its vertices, uploaded once at boot, shared by every instance of that
part id. There were no sub-parts, no named groups, no per-shape draw calls — and
`DrawItem` carries one mesh, one matrix, one tint, with nowhere to put a bone.
So the feature is a SPLITTING and not a skinning pass: shapes belong to an
animation group, each group gets its own mesh, and a group is drawn with one
extra matrix in front of the part's own. Nothing about the vertex format, the
draw item or the shader had to learn what an animation is, and a part that
animates nothing gets exactly what it always got.

**TWO POSES, NOT A TIMELINE.** A shape carries its rest pose in the fields it
always had and its deployed pose in three new ones. That is the whole format,
and it is what makes the authoring possible: an animation can be posed by MOVING
THE THING, which is the entirety of Part Studio's existing vocabulary. Push the
phase slider to 1 and the same G and R that have always moved a shape now write
the DEPLOYED pose instead, with the panel swinging in the viewport as it is
dragged. Keyframes would have bought sequences nobody asked for at the price of
a tool nobody would enjoy using.

**BUT TWO POSES INTERPOLATED THE OBVIOUS WAY IS A TELESCOPE, NOT A HINGE.**
Slerp the rotation and lerp the position and both ENDS are right while the
middle cuts the corner: a panel swinging ninety degrees about a mount at its
root shrinks toward the hub at mid-travel and springs back out. For a 2 m panel
that is 0.6 m of travel in the wrong direction, and it is the first thing anyone
would notice.

Chasles' theorem says every rigid motion is a rotation about some axis through
some point plus a slide along that axis, so the hinge is RECOVERED from the two
poses rather than authored: the axis and angle come out of the quaternion
difference, and the pivot solves `(I - R) c = t` in the plane perpendicular to
the axis, where `(I - R)` is invertible for any angle short of a full turn. The
pivot is only determined perpendicular to the axis — sliding it along the hinge
changes nobody's path — so the minimum-norm solution is taken, which is the
point a person would put their finger on if asked where the hinge was. A motion
with no rotation at all (a landing gear sliding out) has no pivot and no plane
to solve in, and takes a separate branch that is a straight line.

`AHingeIsRecoveredFromThePosesItProduced` sets up a hinge whose answer is known
by construction and checks the property that matters: the RADIUS is constant
along the whole travel. A lerp gives 1.414 at the midpoint against a true 2.0.

**THE PILOT'S MENU.** Right-click a part in flight and a small panel opens over
it with one row per thing that part can be told to do — OPEN / CLOSE on a solar
array, TURN ON / TURN OFF on an engine. The label is the verb for where it is
GOING, not for where it is: "CLOSE" on an open panel is the button you press to
close it, and a label reading "OPEN" while the panel is open would be a status
light wearing a button's clothes.

The right button already turns the camera and had to keep doing that, so the
menu opens on RELEASE and only when the mouse barely moved in between. Anything
else makes looking around open menus.

**WHAT A STOWED PART DOES NOT DO** is the line between an animation and a
decoration. Each animation declares what it GATES — power, thrust or nothing —
and a folded wing makes no power while a shut-down engine makes no thrust. Half
open is half of it, deliberately: a panel caught mid-travel really is presenting
half its area, and a rule that waited for the phase to reach exactly 1 would put
the whole effect in the last instant of a four-second deployment. The default
when nothing gates is 1 and not 0 — getting that backwards would have switched
off every engine in the game the day animations were added, which is what the
test on it exists to say.

**ONE PLACE WHERE THE HOUSE RULE IS BROKEN, ON PURPOSE.** Everything else in
this codebase that moves with time — spin, conveyors, orbits — is a closed form
of the clock, so that warp and save/load are exact. An animation cannot be: its
start time is whenever the pilot clicked, which is not a quantity the world
knows. So the phase is INTEGRATED, and under warp it simply snaps — which is
also what a pilot would see, since three seconds at x1000 is three milliseconds.
Both the phase and its target are stored and saved, so a panel caught halfway
through opening reloads halfway through opening and closes again from where it
is rather than snapping shut.

**AND THE SHIPPED PARTS USE IT.** SP-2 Solar Wing folds alongside the hull and
swings out over four seconds, gating its 4 kW. V-400 Vector gained two: an
ON/OFF toggle gating thrust, and a throttle-driven one that takes the nozzle's
glow cone from 0.3 emissive to 1.0 — the one animation that does not move at
all. That glow cone had been sitting in the asset since the part was drawn,
glowing identically whether the engine was firing or not.

`SW_SHOT=SHIP@<metres>` and Part Studio's `--part / --phase / --capture` were
added for the same reason `--capture` and `SW_SHOT` were: a rendering path
nobody can photograph is a rendering path nobody has checked. The first three
attempts at a ship capture came back as the inside of a terrain patch, because
the shot camera's offset is computed from the SUN's direction and on a launch
pad at dawn that points along the ground.

### F29b — the ladder measured in kilometres, not in spheres of influence

**"CHANGER LE WARP X1B POUR QU'IL SOIT A 50 MILLIARDS DE KILOMETRES ET AJOUTER
UN X10B POUR 1000 MILLIARDS (LA SOI EST BIEN TROP GRANDE POUR ETRE UTILISEE
COMME LIMITE)."**

He is right, and by more than it looks. Sol's sphere of influence is a shade
under two light-years — 1.9e16 metres — because what bounds a star's
gravitational reach is the Galaxy on one side and its nearest neighbour on the
other. Gating the fastest rung on leaving it meant the rung unlocked forty per
cent of the way to Proxima: after the part of the journey that needed it.

The gate is now distance from the star and nothing else, in three bands, each
one an order of magnitude of warp against roughly an order of magnitude of
distance:

| from the star | rung | what is out there |
|---|---|---|
| 5 billion km | x100M | past Neptune's 4.5, in the Kuiper belt |
| 50 billion km | x1B | ten times the outermost orbit |
| 1000 billion km | x10B | a tenth of a light-year, nothing at all |

The property that makes each one safe is the same one every time: how far an
ANALYTIC jump may throw a craft between two rendered frames, against what there
is out there to hit or be captured by. And the ladder **walks itself back down
on arrival** — a craft falling toward Proxima crosses the same three radii in
reverse and loses a rung at each, so it cannot arrive at x10B and cross a whole
planetary system inside one frame. That is not a safety check bolted on, it is
the same rule read from the other end.

Two details worth keeping. The altitude ladder still wins wherever it binds, so
a craft landed on a Kuiper object six billion kilometres out is not handed
x100M because of where the SUN is. And the refusal message asks
`warpRadiusForRate` what the rung it just took away actually needed, rather
than naming a distance in a string — there are three bands now and there will
be no fourth whose message quietly says the old number.

The sphere of influence has not gone away; it just does a different job. It is
what decides which system's planets are loaded, and the test that every pair of
systems has clear space between them now says so.

Photographing this needed two more debug hooks, for the reason `--capture` and
`SW_SHOT` existed in the first place: `SW_WARP=<rung>` asks for a rung and lets
the ladder refuse it, because the clamp only runs when something has been
requested and a capture has no keyboard. The first three attempts came back
reading "WARP X100" at every distance and looked like a broken gate — the game
starts on foot, and on foot the ladder is capped at x100 whatever the star
says, so it was the pilot who was standing outside rather than the rule that
was wrong.

### F29c — the quick click that never fired

**"LE CLIQUE DROIT POUR INTERAGIR AVEC LES PIECES NE MARCHE PAS. IL FAUT QU'UN
CLIQUE RAPIDE SOIT L'INTERACTION ET RESTER APPUYER POUR UNE ROTATION DE LA
CAMERA COMME ACTUELLEMENT."**

The rule was **six pixels of travel and no time bound at all**, written inline
in the HUD, and it never fired once. Six pixels is inside the slop of an
ordinary click on an ordinary mouse: every tap was read as a drag, so a feature
that was entirely written looked as though it had not been.

The test that separates the two jobs is now `Input::isQuickClick(held, travel)`,
in the engine, and it takes **both** numbers because neither alone is enough — a
slow careful drag covers almost no distance, and a fast flick covers a lot in
very little time. A quarter of a second and forty pixels: longer than any tap,
shorter than the briefest deliberate look-around; a shaky hand rather than a
drag. Holding the button turns the camera exactly as before.

Two things came out of chasing this that are worth more than the fix.

**A MISS AND A DEAD FEATURE LOOKED IDENTICAL.** A click that found nothing left
the screen exactly as it was, which is indistinguishable from a click that was
never registered — so the first report could have been either. The menu now says
`NOTHING TO OPERATE THERE` for a second and a half, which turns silence into
information.

**AND THE PICK WAS TOO EXACT TO USE.** A ray against the collider boxes is
correct and, for a solar wing seen edge-on from twenty metres, correct means a
target two pixels wide. There is now a second pass: a part whose CENTRE falls
within its own on-screen radius of the cursor counts as clicked, scored beyond
every true hit so a real hit always wins and a near miss on the only operable
thing in view still does something.

What was checked and what was not: `isQuickClick` is pinned by a test that
includes the exact case the old rule rejected (a ten-pixel wobble on a tenth of
a second). The in-flight menu itself is still unphotographed — the ship-framing
capture camera lands inside the terrain patch at the launch site, which is its
own bug and not this one.

### F29d — one ray, and whatever is in its way

**"LE CLICK EST POSSIBLE MAIS EXTREMEMENT DIFFICILE. CE QU'IL FAUT FAIRE C'EST
UN RAYTRACING DE LA CAMERA VERS LE CLICK ET SI UNE PARTIE DE VAISSEAU CLIQUABLE
EST ENTRE ALORS ELLE S'ACTIVE."**

That is the right design and it is what the code does now. What it did before
was an exact ray against the part's COLLIDER boxes, and that is wrong twice
over.

**Wrong once because the colliders are at the REST pose.** A solar wing drawn
swung out has its collider still folded against the hull, so clicking the panel
you can see misses it and clicking empty space beside the tank hits it. Half of
the animation — the half the pilot actually spends looking at the deployed
panel — was the half that could not be clicked.

**Wrong twice because exact is the wrong ambition.** A wing seen edge-on from
twenty metres is two pixels of collider. A switch that demands two pixels is a
switch nobody can work, however correct the arithmetic behind it.

It is now one ray against the part's BOUNDING SPHERE, which already had to
cover the deployed pose for culling, and the nearest entry along the ray wins —
so a part in front of another shadows it exactly as it looks like it should.
At twenty metres a 1.5 m part is four degrees of tolerance instead of a
twentieth of one. `rayEntersSphere` is in the engine with the reasoning next to
it, and the test pins the tolerance IN DEGREES, because degrees are what a hand
has to hit: three degrees off must still hit, five must miss, a part astern
must never hit, and a part the eye is inside is at zero rather than behind.

The previous attempt at forgiveness — an exact test first, a screen-space near
miss as a fallback — was worse than either half alone: it scored every near
miss beyond every exact hit, so an exact hit on a part you were NOT pointing at
always beat the near miss on the one you were.

### F29e — the ship the pilot is actually in

**"CELA NE MARCHE TOUJOURS PAS NI POUR LE MOTEUR NI POUR LE PANNEAU SOLAIRE."**

Three rounds of fixing the click had produced three defensible corrections and
no working menu, so the fourth round started with a measurement instead of an
argument. `SW_PICKPROBE=1` writes what the pick can see straight to a file —
straight to a file, because a capture run aborts at teardown and loses whatever
is still sitting in the log's buffer. It came back with one line:

    35 part entities, 0 with animations, 0 with live state

Nothing was broken. The animations had been authored on the CATALOGUE parts —
the SP-2 wing and the V-400 engine, the ones a player meets in the hangar — and
the Endurance the game starts you in is built from none of them. Its habitat and
propulsion modules are their own definitions, and both had an empty `animations`
list. The pick correctly refuses parts with nothing to operate, so every part
within reach of the cursor was correctly ignored, and a feature that worked
perfectly could not be reached from inside the game.

So the modules got the hardware the pilot was reaching for. The EN-1 habitat
carries two folding arrays, four and a half metres by ten, stowed flat on its
roof and floor and hinged at the outer edge so they stand off radially when
deployed; the animation gates the module's own sixty kilowatts, so a stowed
array really does stop charging. The EN-2 propulsion module gets a hand switch
that gates its thrust and a second, throttle-driven animation on the three
nozzle discs — which were authored at 0.9 emissive, permanently lit, an engine
that looked like it was burning while it was shut down. They now sit at 0.06 and
run to 1.0 with the throttle, and the throttle animation is gated by the hand
switch as well, so a shut-down engine's nozzles stay dark however far the
throttle is pushed.

**AND THEN THE SAME MEASUREMENT FOUND THE SECOND BUG.** With animated parts
finally on the ring, the probe reported `aimedPick 'nothing'` — a ray aimed
straight down the line to a part, missing that part. The pick tested the
SIMULATED transform. Physics runs at 50 Hz and the screen does not, so every
mesh is drawn at the lane's alpha mix of the last two ticks, and the chase
camera has always used that mix too. In Saturn orbit at nine and a half
kilometres a second one physics step is a hundred and ninety metres: the sphere
the click tested against sat up to a hundred and ninety metres away from the
pixels the pilot was aiming at, drifting in and out of alignment sixty times a
second. A click that landed was a coincidence — which is exactly what "possible
but extremely difficult" describes, and it means the earlier report had TWO
causes and the first three fixes had only ever addressed one of them.

The menu's anchor had the same fault with a louder symptom: it could compute the
part as being behind a camera that was looking straight at it, and a part behind
the camera closes the menu. That is why a capture of the panel came back with
the menu missing altogether on one frame and pinned to the screen edge on the
next.

`renderPosition` is now the single answer to "where is this drawn", and the
pick, the menu anchor and the capture camera all ask it. Anything that has to
agree with what the pilot can SEE must ask it too.

What is checked: `TheEnduranceModulesArePartsThePilotCanOperate` loads the
shipped catalogue and asserts the property whose absence caused all of this —
that the ship the game starts you in has parts the menu can open, with a gate on
one side and moving geometry on the other. It also pins that no nozzle is
authored bright at rest and that every animated shape's deployed pose is inside
the bounding sphere the click tests. The gate arithmetic itself was already
pinned; the phases were then measured live, with the engine on and off against a
held throttle, and read 1.00/1.00, 0.00/0.00 and 1.00/0.00 exactly as the design
says they should.

What was NOT done: the deployed arrays have no collider, so a walker can pass
through one. That is the same rule the SP-2 wing has followed since animations
were added — colliders are authored at the rest pose — and it is the honest
remaining edge of the whole feature.

### F29f — the button that armed building number 500

**"LE MENU S'OUVRE BIEN MAIS CLIQUER SUR LES BOUTONS NE CHANGE RIEN."**

Every clickable rectangle on the HUD carries a NUMBER, and the number says what
pressing it means. The ranges are open-ended upward — "400 and above arms a
building", "100 and above takes a part in hand" — because each panel wants room
to grow, and a chain of `if (id >= N)` tests is then only correct if it is
written in descending order of N.

It was not. The part menu's rows are 900 and up, and its branch had been
appended at the BOTTOM of the chain, below `if (button.id >= 400)`. Nine hundred
is above four hundred, so every press of OPEN or TURN OFF was read as "arm
building number 500", set `m_heldBuilding` to a definition that does not exist,
and returned. The menu appeared, the rows highlighted under the cursor, the
click was consumed — and the panel never moved. A dead button that looks alive.

The dispatcher's own comments warn about this hazard twice, in the code that
then got it wrong: *"tested before everything else because the ranges below are
open-ended upward and would otherwise swallow these"*, and *"610+id would
happily swallow 900"*. Being warned twice in the same function and doing it
anyway is what decided the shape of the fix.

The routing is no longer expressed as the order of a hundred and forty lines of
side effects. `sw::ui::routeHudClick` is a pure function that maps an id, plus
the two pieces of context two panels genuinely share a range over, to a
`HudAction`; the dispatcher switches on what it returns and owns only the side
effects. It sits beside `HudOrder.hpp`, which exists for the same reason and
about the same surface: a HUD rule that is invisible in review and glaring on
screen belongs in a pure function a test can hold.

Three things fell out of writing it down. **900+ is shared** between the
assembly catalogue's rows and the part menu, and the separator is whether the
machine configuration panel is open — which was already true and already
load-bearing, and had never been stated. **The part menu's range is now
BOUNDED** at the four animations a part can carry, so the ids between it and the
multiplayer panel keep falling through exactly where they did rather than being
eaten by a range that grew to fill the gap. And **the map's veto moved into the
table**: the map owns only its own buttons, so a hangar id pressed there must do
nothing rather than the nearest thing below it.

`HudRoutingIsOrderedByDescendingRange` walks every range's lower bound and
asserts it routes to its own owner; every case in it fails against the old
ordering. `SW_PARTMENU` now presses the id the menu actually registers instead
of calling `togglePartAnimation` directly — the capture that "proved" the
animation worked had been stepping over the exact line that was broken, which
is the more useful lesson of the two.

### F29g — the button that had already been thrown away

**"LES BOUTONS NE MARCHENT TOUJOURS PAS."**

The routing was fixed and provably correct, and pressing id 900 by hand drove
the animation. The button still did nothing, because by the time the click was
handled there was no button.

`m_hudButtons` was emptied by whichever collector happened to run FIRST on each
screen, and everything after it appended. On the flight HUD that collector is
`collectSasButtons` — and the part menu is collected three lines ABOVE it, on
purpose, so that a menu opened over the SAS row takes the click. So every frame
the menu pushed its rows and the SAS collector, running next, cleared them away.
The comment directly above the call spells out the intended precedence: *"BEFORE
the navball and the SAS row, so a menu opened over them takes the click.
m_hudButtons is scanned in order and the first rect under the cursor wins."* The
reasoning is right. It was defeated by a collector three lines later that did not
know it was the one holding the contract.

Opening the table is now its own named act. `hudBeginButtons()` clears, once,
before anything is collected, and asserts if it is called twice in a frame;
`hudSeizeButtons()` is the separate, deliberate case — a modal taking the screen
over, where discarding what is behind it IS the point. Every collector appends.
The two have different names because confusing them is precisely the bug.

**AND THE REAL LESSON IS ABOUT THE TEST, NOT THE BUG.** `SW_PARTMENU` called
`togglePartAnimation` directly. When the routing turned out to be broken it was
changed to synthesise a `HudButton` and route that. Both versions manufactured
their own input, and both therefore stepped over the step that was actually
broken — first the routing, then the table. Three captures in a row "proved" a
feature that could not be used. A hook that fabricates the input it is supposed
to be testing proves only that the code after the fabrication runs.

It now SEARCHES the table the click handler reads, presses the row it finds
there, and writes `BUTTON 900 IS NOT IN THE TABLE: 3 buttons, ids 1 2 3` when it
does not — because "the button is missing" and "the button did nothing" look
identical from outside and have nothing in common. It reads `pressed button 900
of 4`, and the module's animation goes `[0 0.73->0.00]` while its neighbour
holds at `1.00`.

What is checked and what is not: the one-clear rule is held by a debug assert
rather than a unit test, because it is a property of the order in which four
collectors run inside a render pass and there is no seam to test it through. A
direct `m_hudButtons.clear()` added anywhere would still bypass it. Making the
table a type that can only be opened once would close that, and is not done.

### F30 — two suns, and thirty-six stars that look like what they are

**"QUAND NOUS SOMMES DANS UN SYSTEME BINAIRE SEULE 1 ETOILE SUR LES DEUX A LES
EFFETS STYLE SOLEIL. VERIFIE CHAQUE ETOILE: LES NAINES ROUGES PETITES ET ROUGES,
LES ETOILES BLEUES BLEUES."**

Three separate faults, and the catalogue was not one of them. Every radius and
effective temperature in it is interferometric or asteroseismic and every one
checks out; what was wrong was the rendering of them, in three places.

**ONE SUN PER SKY.** The three-layer glare and the photosphere disc were drawn
for `dominantStar` — the brightest from the camera — and everything else went
down the billboard path, whose angular size is capped at `flux^0.2` so a star
four light-years away stays a point. Alpha Centauri B is twenty-three
astronomical units from A, half Sol's output, apparent magnitude about -19 from
a world orbiting A. It got the four-light-year treatment.

The treatment is a function now (`collectStarVisual`) and it runs for every
star that qualifies. **The test is a RATIO, and the ratio is the whole design:**
an absolute brightness cut would POP — fly out of a system and the companion
would cross the threshold and change from a glare to a billboard in one frame —
whereas both members of a pair dim together as you leave, so their ratio is
constant and they keep their glare together all the way out. Measured against
the catalogue: Alpha Cen B from A's habitable zone lands at 6.3e-4, and Alpha
Cen A seen from Proxima b at 1.3e-8, eight orders below the cut. A hundredth of
a percent separates them and nothing sits near it. The lens flare stays on the
dominant star alone: ghosts are an artefact of one optical axis, and a second
chain lands on top of the first.

**THE COLOUR LADDER WAS ONE MISSING CONVERSION.** It held sRGB-ENCODED values
and they were used as linear multipliers. At 2800 K it carried 0.68 in green
where the linear Planckian locus says 0.444 — and 0.68 *is* 0.444 gamma-encoded,
to three decimals, at every rung. So every star in the game was pulled toward
white by exactly the amount sRGB encoding lifts a mid-tone. That is the whole
of "a red dwarf comes out pale peach and Sirius comes out white".

Recomputed from Planck's law through the CIE 1931 observer into linear sRGB, and
**anchored at Sol**: divided by the locus at 5772 K before renormalising.
Normalising each temperature against itself says what a 3000 K body looks like
to nobody in particular; dividing by the Sun first says what it looks like NEXT
TO THE SUN, which is the comparison the words *red dwarf* and *blue giant* were
coined from and the adaptation state of the only eye that will ever read the
screen. It also pins Sol at exactly (1,1,1), so twenty milestones of
solar-system tuning are untouched. Barnard's Star went from (1.00, 0.73, 0.52)
to (1.00, 0.59, 0.25); Sirius B from (0.67, 0.76, 1.00) to (0.31, 0.48, 1.00).
The dome's six class colours are the same function read at each class's
temperature, because a dome corrected while the catalogue was not would put
Sirius in a sky whose neighbours obey different physics.

**AND THE GLARE WAS DRAWING EVERY STAR THROUGH A SUNSET.** With the ladder
fixed the red dwarfs read correctly and Sirius still came out white. The three
glare meshes carried Sol's warm ramp baked into their vertices — (1, 0.98,
0.95), (1, 0.80, 0.52), (1, 0.68, 0.38) — and the star's hue was multiplied on
top. For Sol that is a no-op. For Sirius it is (0.51, 0.66, 1.00) times (1,
0.68, 0.38), which is **orange**.

The meshes are neutral now and the ramp moved into the tint, where it is a
TEMPERATURE FACTOR rather than a colour: each layer is drawn as the blackbody of
a cooler star — 0.97 of its temperature at the core, 0.745 at the halo, 0.627 at
the aureole. Those three numbers are not free, they are what reproduces Sol's
hand-tuned ramp when fed 5772 K (to within four hundredths in blue). So the Sun
is unchanged and every other star reddens outward FROM ITS OWN COLOUR. Measured
in the far wing at seventy pixels: Sirius (0.48, 0.61, 1.00), Procyon (0.55,
0.65, 1.00), Sol (0.86, 0.86, 1.00), Barnard (1.00, 0.76, 0.48), Wolf 359
(1.00, 0.73, 0.44) — monotone in temperature, which it was not before.

**What is checked.** `EveryStarsSizeAndColourAgreeWithItsSpectralClass` walks
all thirty-six against the designation each record already carries: every M
dwarf small AND red, both white dwarfs Earth-sized and hot, every A star
blue-white and larger than the Sun, every brown dwarf planet-sized and barely
glowing — and it counts the families it matched, because a walk that matched
nothing would pass every assertion in it.
`TheColourLadderPutsEveryStarWhereItsNameSaysItIs` pins the ladder monotone in
temperature over the whole range, the brightest channel at exactly 1, Sol
neutral, and nothing green below 1200 K — that last one for WISE 0855, which at
276 K came out (1.00, 0.90, 0.00) before the floor went in, a bright
yellow-green for an object that emits no visible light at all.

**New instruments.** `SW_STARPROBE=1` writes the whole table — radius,
temperature, luminosity, hue, and the word the eye would use — plus which stars
are suns from where the camera is and the irradiance ratio that decided each.
`SW_SHOT=SUNS@<metres>` frames a pair: neither existing mode could photograph a
binary, since a body shot needs a name in the celestial index and a ship shot
points at the craft parked beside one of the two. It stands off the midpoint
along the perpendicular to the line joining them, which is the one direction
that puts both in frame and separated.

**Two things worth knowing.** The probes now run ABOVE the jump latch in
`applyDebugJump`: `SW_JUMP` sets `m_debugJumped` on its first frame and every
later call returns immediately, so a probe waiting thirty frames for the camera
to settle was never reached in any run that also jumped — which is every run
where the question is about another star. And `SW_BOARD` does not combine with
`SW_JUMP`: the Endurance is on rails, so the teleport is undone by
`RailsSystem` on the next tick and the origin follows it back to Sol. Jump
without boarding.

### F31 — the clock that ran out of digits

**"JE ME SUIS POSE SUR PROXIMA CENTAURI B ET LE SOL S'EST MIS A VIBRER, AUSSI
BIEN SUR CETTE PLANETE QUE SUR TERRA."**

Both halves of that sentence are the diagnosis, and the second half is the
one that names the cause. A fault that follows you back to Terra is not a
fault of Proxima b; it is something GLOBAL that the trip changed. Only one
thing does: the clock.

Everything analytic here — every planet's position, every rail, every planet's
rotation — is a function of ABSOLUTE simulated time, evaluated fresh each tick.
A crossing to Proxima Centauri is 4.0e16 m, and at the hundred-odd km/s a ship
makes that is about **3e11 seconds** of simulated time, whatever warp rung you
spend it at — warp changes how long you wait, not how much time passes. A
double at 3e11 has a step of 61 microseconds. Terra covers **1.8 m** in 61
microseconds.

So every tick's time was snapped to a grid 1.8 m wide. Not drift — drift is
invisible, the whole world moves together — but NOISE, a different rounding
every tick, and the walker's own position is integrated rather than slaved to
the planet, so it bobs against ground that is jumping under it. Measured on the
shipped orbit, per-tick position noise:

| simulated time | what it is | Terra | Proxima b |
|---|---|---:|---:|
| 0 | a fresh session | 0.000 m | 0.000 m |
| 1e10 s | three centuries | 0.10 m | 0.16 m |
| 3.3e11 s | **one crossing** | **2.18 m** | **2.90 m** |
| 3.3e12 s | ten crossings | 34.8 m | 54.9 m |

**REDUCING THE MEAN ANOMALY IS NOT THE FIX**, and measuring that first is what
kept the afternoon honest. `M = M0 + n*(t - epoch)` reaches 6.6e4 radians at one
crossing, whose own ulp is worth 2.2 m, so wrapping it modulo the period looks
like the answer. It takes 2.18 m to 1.82 m. The error is in the TIME, not in
the anomaly, and no amount of care downstream can recover a digit the clock
never had.

**So the clock carries exact integer seconds plus a fraction in [0, 1).** The
whole part is exact to 2^53 seconds — two hundred and eighty-five million
years — and the fraction keeps its full seventeen digits however large the
whole part grows. `Simulation` accumulates with a carry instead of `+=`;
`SimulationLane::presentSecondsSplit` subtracts the lane's residue in the
fraction with a borrow; `kepler::evaluateSplit` reduces the elapsed interval
modulo the period using the exact whole part, so what reaches the multiply is a
small number carrying the fraction's full precision. The same treatment for the
two spin systems, where the period is the rotation rather than the orbit.

It reaches the four things that POSITION the world every tick — celestial
motion, rails, celestial spin, rails spin — and `CelestialIndex::stateAtSplit`
carries it down the whole parent chain, because Luna's position is Terra's plus
its own and a moon evaluated exactly on top of a parent evaluated sloppily
inherits the parent's noise. The map, the HUD and the trajectory predictor keep
the plain single-double path: a millimetre is not a picture anybody can see, and
sixty-odd call sites did not need touching.

Measured after: **0.00054 m at one crossing and 0.00055 m at ten** — flat
instead of growing, four orders of magnitude better. In the shipping engine,
with `SW_CLOCK` winding the clock before the scene is built, the primary's
per-frame step jitter reads **0.000000 m** at every clock value tested.

**Two instruments and one lesson about them.** `SW_CLOCK=<seconds>` winds the
simulation clock forward, and it has to do so BEFORE `buildScene` — setting it
from inside the frame loop was tried first and measured twelve kilometres of
wobble that was pure artefact, the planet rotating out from under a walker
still carrying the 464 m/s it had at t=0. `SW_GROUNDPROBE=1` then reports the
per-frame motion of the body under the player. It samples the PLANET and not
the walker, and that is a correction rather than a convenience: even with the
clock set before the build, the scene is still PLACED at spin angle zero, so
the first tick launches the walker on a ballistic hop and what the probe
reports is the hook. The body's own step has no such artefact and is exactly
the quantity the split clock protects.

What is NOT fixed: a walker placed by `SW_CLOCK` still hops, because
`buildScene` positions the outpost at spin angle zero regardless of the clock.
That is a limitation of the test hook, not of the game — a real session
advances the clock with the world attached to it — but it means the hook cannot
be used to photograph a landing at an arbitrary date.

### F32 — the orbit's other four directions

**"AJOUTE DES BOUTONS EN PLUS DE PROGRADE ET RETROGRADE : UN BOUTON RADIAL IN ET
RADIAL OUT ET UN BOUTON NORMAL IN ET NORMAL OUT."**

Four more hold modes, and the reason they are worth four more buttons is that
none of them is reachable by pointing at prograde and waiting. **A plane change
is flown NORMAL** — perpendicular to the orbit — and it is the one burn where
prograde is exactly the wrong answer. **A radial burn rotates the line of
apsides without changing the period**, which is how a periapsis is moved to
where you want it rather than to wherever the last burn left it. Both are
standard vocabulary; both were previously flyable only by hand, against a
navball that did not even draw the marker.

**ONE FUNCTION, TWO CONSUMERS.** `phys::orbitalFrame` returns prograde, normal
and radial-out from a position and a velocity, and both the autopilot and the
navball call it. That is not tidiness: every one of these vectors is a cross
product whose sign is a coin flip until the convention is written down, and a
sign flipped in one of two copies is a plane change that turns the orbit the
wrong way with a marker sitting exactly where the nose already points — a
failure with nothing anywhere that says so. Normal is r x v, radial-out is
prograde x normal, and the function returns FALSE when r x v vanishes (straight
up, straight down, hovering) so the caller holds rather than points at a
normalised zero.

They are built from the **same velocity prograde is**, honouring the `V` frame
toggle, so all six markers move together or none of them do. An autopilot
holding a normal computed in one frame while the navball drew it in the other
would be F6's landing bug again with a different vector.

**Eight buttons, two rows.** The velocity holds keep the row they have always
been on, so a hand that knows where PGD is still finds it; the orbit's other
four go above them, together, because they are one family. Four characters is
the label budget — the glyph advance is 6/7 of the text height, so at 0.036 in
a 0.115-wide button a fifth letter runs off the end — hence `RAD+`/`RAD-` and
`NML+`/`NML-`, and hence the line above the rows that spells the active mode
out in full: `+`/`-` for a direction is a convention, and a convention has to be
written somewhere the player can see it. `T` walks all nine states.

**And the navball got the four markers**, because four buttons without them is
half an instrument: cyan for radial, violet for normal, filled for the `+`
sense and hollow for the `-`, matching prograde-filled / retrograde-hollow.

Two things the capture found that reasoning had not. The mode line was first
placed BELOW the rows at 0.980 and came back with its descenders sheared off by
the screen edge — it lives above them now. And **the 5x7 charset has no
parentheses**, which it never has: `ORB FRAME (V)` has been rendering as `ORB
FRAME  V` with two blanks since the day it was written. Both lines now use a
colon, which exists.

`SW_SAS=<mode>` engages an autopilot mode from a capture. It is what produced
the picture that closes this: holding RADIAL OUT, the cyan diamond sits exactly
on the craft reference at the centre of the ball — the button, the mode, the
controller and the marker all agreeing about one direction.

### F33 — a force has a point of application, and structures bend

**"UNE FUSEE TROP LONGUE SE COURBE SOUS L'ACCELERATION ; UN ATTERRISSAGE DUR
TORD UNE JAMBE DE TRAIN ; UN GRAND PANNEAU SOLAIRE VIBRE. IL FAUT AUSSI UNE
ACCELERATION FOURNIE PAR LES PROPULSEURS A LEUR SORTIE ET PAS AU CENTRE DE
GRAVITE — SI JE DESACTIVE 3 DES 4 MOTEURS DE L'ENDURANCE ET QUE J'ACCELERE JE
SUIS CENSE TOURNER."**

The two halves of that request are one milestone, and the order is not
negotiable: **a beam pushed along its own length is in compression, not in
bending.** What bends a structure is a LATERAL load, and until thrust had a
point of application there was no lateral load for an engine to make.

#### Thrust as a vector at a place

The old model summed every engine into one scalar, `maxThrustNewtons`, and
pushed it through the centre of mass along the hull's nose. Exactly right for a
symmetric rocket; for everything else a number that had forgotten where the
engines were bolted. `VesselAssemblySystem` now collects each engine as a force
VECTOR at its own position and folds them once the balance point is known, so
the vessel carries `thrustForceN` and `thrustTorqueNm` and `ThrustSystem`
applies both. Nothing special-cases a symmetric craft: four arms about a common
centre cancel.

Measured on the Endurance:

| engines armed | thrust | torque (as measured then) | torque (after F37 balanced the ring) |
|---|---:|---:|---:|
| 4 | 88 kN | 10.3 kN·m — read as "its own asymmetry" | **0.04 kN·m** |
| **3** | 66 kN | **651 kN·m** | 647 kN·m |
| 2 | 44 kN | 913 kN·m | 915 kN·m |
| 1 | 22 kN | 645 kN·m | 647 kN·m |

The fourth column is the correction: the 10.3 kN·m in row one was not the
ship's own asymmetry, it was a two-tonne mistake in its manifest. See M37.

**The first measurement found something else entirely.** Summed along the
part's default −Z, the four propulsion modules' forces *cancelled to zero* and
left a pure 2.6 MN·m torque: the ship was a reaction wheel. EN-2 carries its
three plasma nozzles on its −Y face, because twelve modules are strung around a
ring and a ring translates along its axle. So `PartDefinition` gained
`thrustDirection` — default −Z, the part's nose, which is right for every rocket
and was therefore never worth a field until an engine was not one. Before this
milestone that error was invisible: thrust was a scalar pushed along the root's
nose, which for the Endurance happens to be the ring axis.

#### One damped spring per part

There is no finite-element anything here and there does not need to be. A part
hangs off its vessel at one point; the load through that point is its own mass
times the acceleration it is being given AT ITS LOCATION — which on a turning
vessel is `a + α × r`, not the acceleration at the centre of mass — and a beam
under a moment deflects by moment over stiffness. `PartFlexComponent` is three
vectors: an elastic angle, its rate, and a permanent set.

Below `flexYieldNm` it springs back, lightly damped, so a big panel *rings*
after the engines light rather than simply sitting bent. Above it the excess
moves into the permanent set and stays: that is the hard landing. The sum is
applied in `PartAttachmentSystem`, about the vessel's balance point, so the part
both LOOKS bent and — if it carries an engine — PUSHES bent. That feedback is
what makes a long craft wobble instead of settling.

**Zero stiffness means rigid, and rigid is the default.** A game where every
strut suddenly became a spring is a game where every rocket that used to fly no
longer does. Three parts declare flex today and their numbers are computed from
the loads they actually see rather than chosen to look right: an Endurance spoke
carries 3000 kg at 16 m from the balance point, and with three of four engines
the angular acceleration is 3.0e-3 rad/s², so the lateral acceleration there is
0.047 m/s² and the moment 2256 N·m — 6.4e4 N·m/rad puts that at about two
degrees.

Measured, on the ring, under a held throttle:

| engines armed | worst elastic bend | permanent set |
|---|---:|---:|
| 4 | 3.22° | **0.000°** — nothing yields |
| 3 | 5.09° | 1.53° |
| 2 | 5.09° | 7.60° |
| 1 | 5.09° | 10.03° |

The elastic figure caps at 5.09°, which is exactly `flexYieldNm / stiffness` for
the core spoke — the yield doing its job rather than a clamp doing it.

#### Two bugs the measurement caught, both of them the same shape

**`VesselAssemblySystem` zeroes the vessel every tick**, which is exactly right
for every field that is a SUM and catastrophic for the two that are a MEMORY.
The flex pass differences the velocity across a tick to get an acceleration —
that catches thrust, the ground, the air and a collision for free, and cannot
fall out of step with any of them — but with the previous velocity wiped each
pass the difference was the vessel's whole orbital speed over a fiftieth of a
second. Fifty thousand g. Every flexible part on the Endurance yielded to the
clamp on frame one and reported an identical 68.75° whatever the engines were
doing. And **the first difference is not an acceleration** either: a vessel seen
for the first time has no previous velocity, and subtracting zero from nine
kilometres a second does the same thing once.

Both are the same failure — a derivative taken against a value that was never
initialised — and both were invisible until the numbers were put side by side
for four, three and one engines and came back the same.

#### What is not done

**Joints still do not break.** `breakForceN` has sat on `JointComponent` since
joints were written, with a comment promising that an impact stronger than it
"destroys the joint entity and the vessel falls apart along real structural
lines", and nothing anywhere has ever read it. The flex pass now computes the
moment each part transmits, which is precisely the quantity that decision needs,
so the remaining work is a threshold and a call to `splitVessel` — which already
exists and is already tested. That is the next commit, not a design problem.

Also outstanding: a part's collider is still built at its REST pose, so a bent
member collides where it used to be. Same fix as the deployed solar array needs.

## Milestone 36 — the freeze at fifty kilometres a second

> « quand j'accélère beaucoup (+50000 delta v) le jeux se met à freeze puis à
> faire des à-coups pourtant le nombre de fps affiché reste à plus de 200 »

**Two hundred frames a second and a freeze cannot both be true of the same
frame.** That sentence is the whole diagnosis, and it was worth reading twice
before touching anything: a counter that averages over a second cannot see a
single frame that took a sixth of one, so the report is not a contradiction and
not a mistake — it is a precise description of a main-thread hitch between fast
frames.

### The instrument came before the fix, twice

The first hypothesis — the flight plan, recomputed four times a second on the
main thread — was **measured and refuted**. A frame probe (`SW_FRAMEPROBE`,
nine hundred frames of per-phase stopwatch) run six billion kilometres out at
120 km/s reported the plan at **0.07 ms**. Right suspect, wrong place: that
regime is nearly at the range cap, so there is almost no arc left to walk.

What the second instrument changed was the question. `SW_PREDSWEEP` calls the
real `predictTrajectory` against the real thirty-body index from four starting
places — the craft's own state, and low orbit around Sol, Terra and Saturn —
through a ladder of extra delta-v, and prints cost, sample count and every
segment. The answer was immediate and was not monotonic:

| burn from low Terra orbit | before | after |
|---|---:|---:|
| +20 000 m/s | 9.4 ms | 3.4 ms |
| **+50 000 m/s** | **125.1 ms** | **3.6 ms** |
| +100 000 m/s | 9.5 ms | 3.5 ms |

A cost that spikes in the middle of a sweep and comes back down is never a
scaling problem. It is a branch.

### `acosh` has one branch, and the arc has two sides

A hyperbolic escape is scanned by walking its own anomaly rather than by
walking time — that was M32's fix, and it holds. The window it walks is solved
in closed form from `r = a(1 − e·cosh H)`. But `acosh` returns a positive
anomaly whether the craft is falling toward periapsis or climbing away from it;
the sign lives in the mean anomaly, which is odd in `H`. The code read that
positive value as "outbound" and **refused the inbound case entirely**, with a
comment reasoning that an inbound arc reaches the bound in its past. Half true:
it reaches the bound *behind* it in its past, and the bound *ahead* of it after
periapsis.

Refusing it was not a small conservatism. With no closed-form window the
segment fell back on the uniform-in-time walk, whose window is then the
caller's whole twenty-year horizon at the hyperbola's own step of about an hour
and a half: **sixty-six thousand samples with nine planets probed at each.**
And the state that triggers it is not exotic — it is any burn made on the
sunward side of an orbit, which is half of them.

```cpp
if (meanNow < 0.0) { anomalyNow = -anomalyNow; }   // the whole fix
```

The window then runs from a negative anomaly, through periapsis, out to the
bound — which is also where the samples are densest, exactly where they should
be.

### The bug the fix uncovered: a sample sitting on the boundary

With the window now ending *at* the bound, several plans that used to hand off
to Sol stopped after one segment. The scan's test is `radius > soiRadius`, a
strict inequality — and the last sample was computed to land on that radius
exactly. Whether the exit was seen came down to the last bit of a `cosh`, so it
was seen for some burns and not others. This had always been true of the
outbound path; it had simply been getting away with it.

The window now overshoots the bound by one sample's worth of anomaly, so the
crossing is strictly bracketed and bisection refines it as it always did. A
side effect is more honest reporting: arcs that stop because they ran out of
room now consistently say `RangeLimit` instead of sometimes claiming `Horizon`.

### A planet you are nowhere near the orbit of needs no Kepler solve

The residual was still 9–15 ms, all of it in the event scan: at every sample,
every child of the primary is evaluated on its own orbit. But both radii are
measured from the same primary, so a child whose entire annulus — periapsis to
apoapsis, widened by its own sphere of influence — excludes our current radius
cannot possibly contain us. Three flops replace a Newton iteration. An escape
sweeps from one astronomical unit to sixty-seven, and at any single point along
it at most one planet's annulus is in play, where the loop used to solve all
nine. That is the last 3–4×.

In the shipping game, after a +50 km/s burn: worst refresh **16.33 ms → 4.66
ms**, mean **9.74 ms → 3.44 ms**, and the 125 ms case is 3.6 ms.

### What is now measurable that was not

`PredictionStats` (segments and samples) is returned by `predictTrajectory` on
request, because the difference between a plan that costs 8 ms and one that
costs 125 ms was invisible from outside: both returned the same segments,
ending in the same place, for the same reason. The regression test asserts the
**sample count**, not a stopwatch — it is the quantity that actually differs,
and it is the same number on every machine. It fails by sixteen-fold on the
code as it stood.

The frame is also permanently instrumented now: nine `PhaseTimer` accumulators
(simulation, celestial index, streaming, prediction, terrain, grass, reentry,
scene collection, render) reset on the frame boundary, and `SW_FRAMEPROBE`
reports each phase's worst and mean plus the breakdown of the single worst
frame. The next "it stutters" report starts with a table instead of a
hypothesis.

### What is not done

The plan still runs on the main thread. At 200 fps a frame is 5 ms and a
refresh is now about 3.4 ms, so a refresh no longer doubles a frame but it is
still the largest single thing the main thread does outside rendering.
`predictTrajectory` is a pure function of a six-kilobyte index, so handing it
to the thread pool and double-buffering the result is a contained change — and
it is the structural end of this class of bug, rather than another constant
factor off it.

## Milestone 37 — the two tonnes that made a ship turn

> « la propulsion doit être vers l'arrière des moteurs pas vers le coté sinon
> accélérer fait tourner alors que ce ne devrais pas. il faut juste appliquer le
> torque pour que si le moteur n'est pas centré cela produise une rotation en
> plus de l'accélération, mais une rotation dans le bon sens »

Two claims, both worth taking seriously, and the measurement agreed with the
symptom and disagreed with the cause.

### What the probe said

`SW_THRUSTPROBE` now prints one line per engine — position, force, and its own
`arm × F` — because the summed vector cannot tell *"every engine pushes
sideways and they happen to add up"* from *"every engine pushes aft"*, and that
is exactly the distinction the report was about. On the assembled Endurance:

```
engine def 201  at  29.40   0.00  0.00   push  0  0  -22000   arm x F      -1286   644583      0
engine def 201  at  -0.00  29.40  0.00   push  0  0  -22000   arm x F    -648086    -2217      0
engine def 201  at -29.40  -0.00  0.00   push  0  0  -22000   arm x F      -1286  -649017      0
engine def 201  at   0.00 -29.40  0.00   push  0  0  -22000   arm x F     645514    -2217      0
centre of mass 0.101 -0.058 0.000 m
torque -5143.6 -8869.5 -0.0 N m
```

**Every module pushes (0, 0, −22 000): dead along the nose, out of the back of
its own nozzles.** The direction was already right — EN-2 fires from its −Y
face, and the ring places each module with its local +Y along the vessel's −Z,
so the authored aft face *is* the ship's aft face. The four arms are at ±29.4 m
on a perfect square.

The lever was somewhere else entirely: **the balance point sat 11.7 cm off the
ring's axle.** Thrust through a point that is not the balance point is a lever,
and 88 kN on 11.7 cm is 10.3 kN·m. On a 500 t ship that is 2.3e-4 rad/s² — one
and a third degrees per second after a hundred seconds, sixty-six degrees over a
long burn. The pilot was right, and so was the physics.

### Eleven point seven centimetres of nothing

The offset pointed at −29.9°, which is slot 11 of twelve. Slot 11 is the cryo
bay at 22 t and slot 5, opposite it, is the command pod at 20 t. Two tonnes at
29.4 m on 503 t of ship is 0.117 m — the measurement to four digits, which is
how you know there is nothing else in it.

So the ship was never balanced, and every comment in the builder said it was:
four propulsion modules at ninety degrees *"so the thrust passes through the
centre of mass"*, habitats opposite each other, Rangers and Landers likewise,
*"which is what keeps a spinning ring from wobbling about an axis it was not
built to turn on"*. A ring that spins for artificial gravity genuinely has to
balance or it wobbles — so this is a fault in the ship, not a liberty in the
model, and the fix is the one an engineer would sign off: **the command pod is
built to the same 22 t as the module it hangs opposite.** Balance point now
(0.000, 0.000, 0.000); torque with all four lit, **10 253 N·m → 40 N·m**, which
is float rounding.

And because the claim had gone unchecked for a milestone and a half, the builder
now measures it the same way it already measures whether the ring closes, and
warns with the moment it will cost if anyone puts it out again.

### And the flame comes out of the engine now

The other half of the sentence was about what you can SEE. The exhaust plume was
a single jet, nine and a half metres behind the ROOT PART, along the hull's own
axis. On a rocket the root part is the stack and the jet lands in the engine
bell, which is why it survived a year. On the Endurance the root part is the hub
at the middle of the wheel: the exhaust boiled out of the centre of the ring
while the nozzles it was supposed to be leaving through sat thirty metres away.
Framed from behind, that reads exactly as a ship being shoved sideways.

The assembly pass already knows where every engine is and which way it fires —
the torque needs precisely that — so the plume asks it the same question. Each
firing engine gets its own jet, from its own aft face, along its own axis, and
an engine the pilot has shut down makes no flame because it is the same gate
that makes it no thrust. How far back the flame starts is read from the part's
own hull rather than from a constant, so EN-2's nozzle bank at 5.4 m down its −Y
and a V-400's bell at 1.0 m down its +Z both come out right with no number
written anywhere. The particle budget stays the SHIP's rather than each
engine's: twelve nozzles must not cost twelve times the particles.

### The right way round

The second half of the request — *une rotation dans le bon sens* — was true
already, and now it is pinned. Magnitude assertions pass for a cross product
written backwards, and backwards is not subtle: it is a ship that yaws away from
a failed engine instead of into it, which is the opposite of what a pilot has to
correct.

Shutting the module at −Y leaves τ = (−646 800, 0, 0) N·m. The nose's own
motion is dn/dt = ω × n, so it falls **toward the dead engine** — the three
still burning push the far side forward. Two tests state it that way rather than
as two signs, because that is the thing a player sees, and a third separates the
two causes that look identical from the pilot's seat: a craft with symmetric
engines and an off-centre mass still twists, and the nose goes toward the heavy
side.

### Milestone 32+ — candidates (remaining)
Parallelise the LOD sphere builds across the thread pool (fourteen relief worlds
serialised on one core is most of the boot bar). Land on Venus (a ground under the
style-24 deck). The other half of the ring shadow — the
planet's own shadow falling ACROSS the annulus, which is what makes a ring look
like it is orbiting something rather than painted on — plus the forward-
scattering surge that lights a backlit ring from behind. A per-fragment ring
material (they still ride the ordinary lit path on vertex colours). Albedo
contrast on Mars, which is a uniform rust at the moment where the real one has
Syrtis. And the last one per cent of the gas-giant ripple: three quarters of it
was the aerial-perspective clip, and what is left has not been localised.
F4 exploitation UI, part fabrication and real conveyor transport. A rename field for designs (the character path exists now). Fuel and crew as build inputs rather than pad stock. Also: impact-driven joint breakage (fields ready), per-Mach aero tables (the format has room; the runtime call site would not change), reentry heating driven by the same dynamic pressure, control surfaces that deflect, placeholder cleanup. Multiplayer next steps: interest management (per-client relevance by distance and attachment), drawing other players' craft from the mirror world, the rest of the stamped-event vocabulary (staging, construction, resource transfer), and client-side interpolation between snapshots.
