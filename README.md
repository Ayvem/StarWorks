# StarWorks

An industrial space simulation game — mine, automate, build ships and stations, and grow a civilization that spans a star system. Powered by a bespoke C++20 / Vulkan engine written exclusively for this game.

<img width="720" height="720" alt="image" src="https://github.com/user-attachments/assets/b8769305-65d8-4b8e-800f-66696fba902c" />
<img width="1915" height="996" alt="image" src="https://github.com/user-attachments/assets/ab9ed464-d503-4915-b172-38538bee09cc" />
<img width="1912" height="986" alt="image" src="https://github.com/user-attachments/assets/e04eaf62-2bc5-441b-aea4-9a941eabdab8" />
<img width="1917" height="987" alt="image" src="https://github.com/user-attachments/assets/1a2f7ef2-cc68-459a-86ee-86006fd0f714" />
<img width="1915" height="987" alt="image" src="https://github.com/user-attachments/assets/411c6ab4-092e-4daa-a664-a4ab2bd7a26c" />
<img width="1914" height="986" alt="image" src="https://github.com/user-attachments/assets/73f442f4-453d-447d-bfa3-9539f0e5cba3" />
<img width="1588" height="891" alt="image" src="https://github.com/user-attachments/assets/2451b716-d646-4fa3-beee-e75d8c3d7717" />
<img width="1920" height="998" alt="mars-close-orbit" src="https://github.com/user-attachments/assets/817e47e9-c4b9-4838-8d81-0a05d560ab3f" />
<img width="1914" height="993" alt="mars-sun-orbit" src="https://github.com/user-attachments/assets/6bd32046-2c16-44be-b572-778cdbe0f174" />

## Current state — F6: Real aerodynamics, and an autopilot that can fly it

**The air knows which way round your rocket is.** Until now a whole vessel was one number — a ballistic factor, the sum of a Cd·A typed next to each part, divided by mass. It could not tell a rocket flying nose-first from the same rocket flying sideways, it produced no torque, so no fin ever stabilised anything, and every coefficient in it was somebody's guess. All of that is replaced, and the shape of the replacement is the whole idea: **the expensive work happens offline, once per part; the game only reads a table.**

**`Tools/AeroForge` is a wind tunnel.** For each of 342 wind directions it integrates the pressure and the shear over the part's *real triangulated geometry* and records the resulting **force and moment**, divided by the dynamic pressure — so the entries are an area and a volume, valid at every speed and every altitude. The result is one `.aero.json` beside each `.swpart`, about 20 kB, regenerated with one command after any edit in Part Studio. It stores force and moment rather than acceleration on purpose: a fin bolted to a probe and to a three-hundred-tonne booster produces the same newtons for the same airflow, so the table belongs to the **part** and is right on every vessel that part is ever welded to.

The solver is a **rasteriser**, which is what makes the self-shadowing free: looking along the wind, the surface elements that receive air are exactly the ones a depth buffer keeps. Three terms are integrated per visible element — impact pressure (`Cp = Cp_max·cos²θ`, which is what makes a cone cheap and a flat plate expensive, out of the geometry alone), base suction on the rear-facing elements the depth buffer sees last, and flat-plate skin friction. Measured against the textbook: a flat plate lands at **Cd 1.20** (published 1.17), a sphere at **0.60** (0.47 subcritical, 0.92 hypersonic), a 14° cone at **0.27** (0.25 with base drag).

**A probe caught the theory's own boundary before a player could.** Solved with impact pressure alone, a fin at ten degrees produced a *thirtieth* of the force it should, and a rocket with a full set of tail fins still flipped — because Newtonian theory counts only the momentum given up normal to the surface, which is right for a blunt nose behind a detached shock and badly wrong for a thin surface at a shallow angle, where nearly all the force comes from circulation. So the forge carries the **linear (potential-flow) term** as well, and applies it where linear theory is the valid one: on surfaces lying nearly along the flow, front and back alike. Whether a part gets it is decided once, from its proportions — thinnest dimension under a fifth of its longest is a **wing**, everything else is a **body** — and the forge prints which is which. The transition out of the linear term at high incidence *is* the stall: a plate's force peaks near twelve degrees and falls away after.

**In flight it is addition.** Every physics tick, for every part-built vessel in an atmosphere: work out the air (which body, how high, how dense, and the co-rotating wind it is moving through), turn that into one dynamic pressure corrected for the transonic rise, ask each part's table what it does in this flow direction *in its own frame*, scale it by **how much of that part the air can actually see** — a handful of rays against the other parts' boxes, so five tanks nose to tail cost **1.20×** one tank and not 5× — then rotate force and moment into the vessel frame, shift the moment onto the centre of mass, and add. Force over mass is an acceleration; moment over inertia is an angular one. Measured cost: **19 µs per tick** for a seven-part vessel, 0.1 % of a 20 ms budget.

**Nothing in the engine knows what a fin is for.** A rocket weathercocks because the moments of its own parts add up that way. Measured on the same vehicle with the fins moved: tail fins give **−1.9 rad/s²** into the wind, the identical fins at the nose give **+4.1 rad/s²** away from it, and a rocket nudged 8° off its flight path oscillates and settles instead of ringing forever — because the aerodynamic damping uses the same lever arm that produced the restoring moment, so a vehicle that is stable is also damped and one that is not is neither. The centre of mass and the diagonal inertia are recomputed every tick from the parts and their fuel, so a rocket grows *more* stable as its tanks drain, and a vehicle now **turns about its balance point** instead of about whichever part its builder happened to start from.

The HUD gained the two lines that make this playable: **Q / Mach / angle of attack**, and the **stability margin** in calibres — how far the centre of pressure sits behind the centre of mass — because that is the number a player can actually fix, by moving fins or moving mass. A discarded stage inherits its own aerodynamics and tumbles away on its own.

Parts without a table are not broken, they are just old: they fall back to the isotropic model, which is what should happen to a part nobody has run the forge on yet.

```powershell
AeroForge Assets/Parts --report      # re-solve the catalogue, print the coefficients
```

### The autopilot had to grow up with it

Air that can turn a rocket exposes an autopilot that cannot stop one, and both of the faults below were invisible until it could.

**`SAS` was a fourth way of spelling OFF.** The button set the mode to "none of the others"; the only thing that ever countered rotation was the `X` key, held down. That is fine when nothing is trying to turn you and useless the moment something is — an atmosphere does not get bored. It is a real mode now: it drives the rotation rate to zero and keeps it there, spending the RCS's own authority and **not a unit more**. That bound is the whole honesty of it. Measured: a 0.42 rad/s tumble stopped in **0.84 s** against a theoretical 0.83 s at 0.50 rad/s²; against a 1.20 rad/s² aerodynamic moment it **loses**, the rate still growing at 0.70 rad/s² instead of 1.20 — which is exactly what a set of attitude thrusters does. A mode that simply wrote zero would have cancelled the entire aerodynamics pass at the press of a button.

**`PGD`/`RTG` ignored the frame the navball was drawing in.** `V` already switched the speed readout and the prograde markers between orbital and surface-relative; the autopilot did not, and quietly stayed orbital. Measured on a descent at 5 km with 20 m/s over the ground, the two retrogrades are **100.8° apart** — Terra's surface is doing 465 m/s under a craft that has almost stopped — so pressing RTG on final approach held the nose nearly sideways to the marker the pilot was aiming at. That is the whole of "landings are practically impossible", and it was never a tuning problem. One flag now feeds the readout, the markers and the autopilot, so they cannot disagree, and the active frame is printed above the button row.

Every autopilot button toggles now — clicking the lit one switches it off — and `T` cycles OFF → SAS → PGD → RTG → NODE.

## Previously — F5: The factory builds the rockets, and the map flies them

**A rocket is manufactured now.** Draw a design in the hangar, press SAVE, and it becomes a `.swship` file on disk — the same contract as `.swpart` and `.swrecipe`: stable ids, a loader that refuses garbage, a catalogue read at startup. Walk to the **VB-1 Vehicle Assembly Building**, press `E`, and every saved design is listed with what it costs in metal. Order one, feed the hall iron and copper on its eight side conveyors, and it builds the hull, crates it, and ships it down a belt to the **LP-1 launch pad**, where it stands up as a real vessel with the pad's own fuel already in its tanks.

**What it costs comes from what it is made of.** Electrical parts are mostly copper — 55 % for a battery, 60 % for a solar wing, 25 % for an engine's pumps and harness — and structure is steel with a loom run through it. Iron is computed as the REMAINDER, never as an independent number, which is what makes iron + copper equal the part's dry mass to the last gram whatever anyone does to the fractions later. That is also the first reason the copper chain exists at all.

The starting outpost ships with the whole loop standing: the hall, the pad 120 m east where new vessels have always appeared, the belt between them, a second power pole because a 120 m span cannot reach the launch complex in one hop, and enough metal for the first hull. **A pad holds one rocket** — the next crate waits on the belt, and the panel says so, rather than unpacking a second vessel inside the first and letting the collision solver throw one of them off the deck.

**And the map became a flight planner.**

Trajectories run **until something happens to them** — an impact, an encounter, an escape, or a full revolution that meets none of them — instead of stopping after a fixed six days, which on a heliocentric transfer was one and a half degrees of arc. They are drawn as a **continuous line** whose end therefore means something, one pixel wide at every zoom the map allows. Two measurements paid for that: a single chord of Terra's orbit was being drawn **274 pixels thick** where it passed the camera, and sampled evenly in time a Terra-to-Luna transfer put one 107° chord across the periapsis, drawing the line **802 km below the planet's surface**. Both are gone; the arc is now sampled by anomaly and split by depth.

**Click a body to target it.** The plan then answers what a transfer is actually about: how close you pass, when, and **where the body will have moved to** by then — marked on its own orbit ring, with your position at that moment and the gap between them. A maneuver node can be **dragged along its orbit with the mouse**, its step is a ladder on the modifier keys (`Ctrl` ×0.1 up to `Ctrl+Shift` ×1000, moving the node's time by the same factor), a **NODE** autopilot button holds the nose on the burn — which is almost never prograde — and **WARP TO NODE** skips the wait, stopping one minute short. Warp itself now reaches ×10 000 000, for the transfers that take a year.

The burn readout **counts down to zero** while the engine is lit. It did not: the target was recomputed each frame from the trajectory you were currently on, so it moved with the ship and sat at the full delta-v from the first second of the burn to the last. It is measured against the frozen coasting plan now, which also stops gravity — a kilometre per second over a two-minute burn in low orbit — from being counted as thrust.

## Previously — F3c–F3g: Solid objects, and a body of your own

**Parts author their own collision hull.** What a part looks like and what it bumps into used to be the same list of primitives, which conflates two jobs: geometry wants cones and forty segments, collision wants as few boxes as will do. A `.swpart` now declares its hull as axis-aligned boxes, edited in Part Studio, shown in game with `F2` — and a part that declares none still falls back to its collider shapes, so every file written before this loads unchanged.

Those boxes are what you cannot walk through: buildings, rocket parts, the ground. They are also what `E` asks about — a ray from the eye against the hulls, instead of the nearest centre inside 18 m, which lost a solar field you were standing on to a silo behind your shoulder. **And the player has a body**: EV-1 is an ordinary prop, so the suit is redrawn in Part Studio like everything else and its ground clearance comes from its own hitbox rather than a constant in the game.

Two bugs from that work are worth keeping visible. Walking into a building **fired the player a hundred metres**, because the solver removed the velocity component into the wall — and on a planet that velocity carries 30 km/s of orbital motion. Collisions resolve position only now, and the regression test uses Terra's real carrier velocity. And the hitbox overlay swam hundreds of metres off the buildings it belonged to: **the 595 m rule** — anything positioned relative to a body on a moving planet must use the RENDERED pose, not the tick pose — for the fourth and fifth time. It has its own line in `docs/Architecture.md` now.

## Previously — F3: Generic production and energy

**A factory runs on the sun that is actually in the sky.** Solar output is the star's real elevation over that exact patch of ground, zero below the horizon and zero in eclipse — so a lunar site charges its banks by day, lives off them after dusk, stops honestly when they are flat, and starts again at dawn. Fourteen days is longer than any bank you can build, and that is the point: siting, storage and priorities are the game.

**Press `E` at a machine** and you get its front plate — what it is doing (OK / STARVED / BLOCKED / NO POWER), its share of the grid, the site's books, what is in the bin, and every recipe its category can run, each read as a sentence. That is where the **fuel chain** gets built: ice to water to hydrogen and oxygen (electrolysis, 480 kW, the reason energy is the constraint) to propellant. The synthesiser is fed both gases down one belt, and it is hydrogen-limited — which is a true thing about the chain, not a bug.

Priorities are simple and legible: a brownout stops the smelters and leaves the mines digging, because ore keeps overnight and a half-melted charge does not.

**And the grid is the cables you strung.** Buildings have a power connection authored on their geometry; a cable is laid between two of them with the same two clicks as a conveyor, and it hangs. A building takes **one** cable, a **power pole** takes as many as you like — so a factory's electrical layout is something you plan rather than something that happens. A grid is whatever the wires joined together, recomputed after every build and every demolition, and a solar farm you forgot to connect powers nothing. Wires on a grid that is short dim to red.

## Previously — F2: Ground build mode

**You build a factory by walking around it.** Arm a machine in the `F` catalogue, look at the ground, click — the ghost lands where your gaze meets the real heightfield, inside a reach you have to walk to extend; the wheel spins it, `R` razes it. Whether it may stand there comes from the `.swpart` itself: on land, flat enough for its own footprint, on enough ore, with room for it.

Conveyors get a **two-click tool**: pick the machine that ships, pick the one that receives, and the run is laid between their mouths — carrying goods from that frame on. The segments it lays are ordinary buildings, and what turns a row of them into a working link is that their mouths MEET: the network is derived from geometry after every build and every demolition, never stored. Take a tile out of the middle and the chain stops existing, because the ports no longer meet.

## Earlier — F1: The factory is data now, and it has belts

**EVA is first person**, because a factory is built at arm's length. **`F` opens a building catalogue** — every `.swpart` with an industrial block, with its footprint, power and ore requirement, and one of them armed and ready for the ground placement F2 brings. And the production chain has **real conveyors**: decks with rails and legs that follow the terrain, carrying visible crates whose spacing is the link's MEASURED throughput — a starving belt visibly thins out, and nothing about the cargo is simulated, it is a closed-form function of the clock.


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

157 tests, no window and no Vulkan device required — matter conservation across every recipe, warp exactness, orbital mechanics, aerodynamics against textbook shapes, collision, HUD layout, the `.swpart` / `.swrecipe` / `.swship` / `.aero.json` files as shipped.

```powershell
ctest --test-dir build/windows -C Debug --output-on-failure   # Windows
ctest --test-dir build/linux-debug --output-on-failure        # Linux
```

## Controls

**Ship (default mode):** `W`/`S` main engine forward/retro, `A`/`D` yaw, `↑`/`↓` pitch, `Q`/`E` roll, `X` kill rotation while held, `Shift`/`Ctrl` throttle up/down. `Tab` switches between pilot and free camera. `G` goes EVA. `V` toggles the reference frame between orbital and **surface-relative** — the speed readout, the navball's prograde/retrograde markers AND the autopilot all follow it, and the active frame is printed above the autopilot buttons. The artificial horizon (bottom center) shows attitude vs the local horizon; the chase camera auto-levels on the horizon when you are low and suborbital.

**Autopilot (`T` cycles, or click):** `SAS` holds the craft STILL — it drives the rotation rate to zero and keeps fighting, with the RCS's own authority and no more, so it settles a wobble in under a second and honestly loses to a real aerodynamic tumble. `PGD` / `RTG` point along the velocity **in whichever frame `V` has selected**; on a landing the orbital and surface retrogrades are around 100° apart, so this is the difference between flying the navball and fighting it. `NODE` holds the burn vector. Every button toggles: clicking the lit one switches the autopilot off, and any rotation input pauses it while you fly by hand.

**On foot (EVA, `G`):** first person. `W`/`S` walk, `A`/`D` SIDESTEP (the mouse turns you — the suit faces where you look), `Space` jumps. `F` opens the **building catalogue**: pick one, look at the ground, left-click to place it, mouse wheel to spin it, `R` to raze what you are looking at. Belts and cables are laid the same way and with the same two clicks — pick the machine that ships, then the one that receives. `E` at a machine opens its **front plate**: state, its share of the grid, what is in the bin, and the jobs its category can run (or, at a VAB, the saved designs it can build). `F2` shows the collision hulls.

**Free camera:** hold the right mouse button to look around, `WASD` to move, `Q`/`E` down/up, `Shift` to boost, mouse wheel to change speed.

**Star map:** `M` toggles the system view (real scale, beacon markers, patched-conics flight plan with encounter/impact/SOI-exit markers); mouse wheel zooms from LEO out to the whole Sol system. The map centers on your current SOI primary.

**Target (in map view):** left-click a body to target it, click it again to clear. The HUD then shows the CLOSEST APPROACH your plan makes to it — distance, time, relative speed — and the map marks **where that body will have moved to** by then, on its own orbit ring, with your own position at that moment and the gap between them. With a maneuver node up you get a second line for what the burn would achieve.

**Maneuver nodes (in map view):** `N` create/delete, `J`/`L` node time −/+, `I`/`K` prograde +/−, `U`/`O` normal −/+, `Y`/`H` radial +/−. The step is a ladder on the modifiers, and it moves the node's time by the same factor: nothing = 1 m/s / 10 s, `Ctrl` = 0.1 m/s / 1 s, `Shift` = 10 m/s / 100 s, `Alt` = 100 m/s / 1 000 s, `Ctrl+Shift` = 1 000 m/s / 10 000 s. The armed step is on the HUD under the node's vector. Or **grab the violet marker with the left mouse button and drag it along the orbit** — the node's time follows the pixel under the cursor, and the planned trajectory redraws as you move. Fly the burn with the **NODE** SAS button (it holds the nose on the burn vector, which is almost never prograde) and the **WARP TO NODE -1 MIN** button, then burn until DV reaches zero — the readout counts down against the trajectory you would have coasted, so gravity is not mistaken for thrust.

**Chase camera:** hold the right mouse button to orbit around the craft, mouse wheel to zoom, `C` to reset behind it.

**Hangar (VAB):** `B` opens/closes (sim paused; separate view). Right-drag orbits, wheel zooms. Click the palette to take a part IN HAND — it follows the mouse: cyan stack nodes snap it magnetically, or it glues to any hull surface under the cursor (radial parts). `W`/`S`/`A`/`D`/`Q`/`E` rotate the held part in 90° steps, left-click places, `ESC` puts it back, `DEL` discards. Click a placed part to grab its whole subtree. `X` cycles symmetry (x1/2/3/4/6/8, radial placements), `C` toggles the CoM (yellow) / thrust (violet) flags, UNDO removes the last placement (symmetry ring included). NEW starts a fresh design, LOAD cycles the world's vessels into the editor, **SAVE** writes it as a `.swship` and registers it so a VAB can build it, and BUILD is the test shortcut that puts it straight on the pad. `P` in flight switches the piloted vessel. `Space` (or `Z`) fires the decoupler (staging); on foot, `Space` jumps.

**Part Studio (`PartStudio.exe`):** the part authoring tool. Right-drag orbits, left-click selects a primitive or node, `G`/`R`/`S` move/rotate/scale with `X`/`Y`/`Z` axis constraint (`Shift` = fine, grid-snapped), `K` toggles the orange collision overlay, `DEL` removes. Buttons add primitives (box/cylinder/cone/sphere/tube), duplicate, set colors/emissive/segments, flag visible/collider, add stack/radial nodes (X/Y/Z sets a node's direction, SNAP SURF projects it onto the hull). SAVE writes the `.swpart` into the build AND the source `Assets/Parts/` — the game loads it at next launch.

**Time warp:** `.` faster / `,` slower, ×0 up to ×10,000,000 and altitude-limited — the top two rungs need you to be clear of Terra's sphere of influence, because a million times real time moves a craft 30 000 km per rendered frame. ×2 and ×5 are PHYSICS warp: everything stays simulated and the engines still work. Above ×5 the world rides analytic rails and engines cut out. `,` at ×1 stops time — pausing is the bottom rung of the same ladder, not a key of its own.

**Files:** `F5` saves, `F9` loads, `Esc` quits.

## Useful flags

`--frames N` exits after N frames (soak testing); `--log-file path.log` mirrors the log to a file; `--cpu` prefers a software (CPU) Vulkan device over hardware GPUs (and implies `--quality low`); `--quality low|medium|high` sets the shading tier — it drives the planet shader's octave budget, terrain self-shadowing, cloud shadows and the number of atmospheric scattering steps.

## Aerodynamic tables

Every vessel part carries a `.aero.json` beside its `.swpart`, solved offline. Re-run the forge after editing a part's geometry in Part Studio — nothing else needs to change, and a part whose table is missing simply falls back to the old isotropic drag.

```bash
build/linux-release/bin/AeroForge Assets/Parts --report
```

`--report` prints, per part, whether the forge classified it as a **wing** or a **body**, the area it presents nose-on and side-on, the drag coefficient referred to each, and the cross-flow coefficient at 15° of incidence — the numbers to check against a textbook. `--resolution N` sets the depth buffer (the integration element, default 192), `--theta`/`--phi` the direction grid, `--part <id>` re-solves one part, `--all` includes buildings.

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
