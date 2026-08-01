# StarWorks

An industrial space simulation game — mine, automate, build ships and stations, and grow a civilization that spans a star system. Powered by a bespoke C++20 / Vulkan engine written exclusively for this game.

<img width="1920" height="1000" alt="image" src="https://github.com/user-attachments/assets/442c16f1-5a56-48af-bfc6-8112033a005a" />
<img width="720" height="720" alt="image" src="https://github.com/user-attachments/assets/b8769305-65d8-4b8e-800f-66696fba902c" />
<img width="1915" height="996" alt="image" src="https://github.com/user-attachments/assets/ab9ed464-d503-4915-b172-38538bee09cc" />
<img width="1912" height="986" alt="image" src="https://github.com/user-attachments/assets/e04eaf62-2bc5-441b-aea4-9a941eabdab8" />
<img width="1917" height="987" alt="image" src="https://github.com/user-attachments/assets/1a2f7ef2-cc68-459a-86ee-86006fd0f714" />
<img width="1917" height="990" alt="image" src="https://github.com/user-attachments/assets/66c89760-41a3-432e-8cef-bcbf566c6e5f" />
<img width="1918" height="981" alt="image" src="https://github.com/user-attachments/assets/59994c49-183e-43aa-a082-5f5ae6ae61dd" />
<img width="1914" height="986" alt="image" src="https://github.com/user-attachments/assets/73f442f4-453d-447d-bfa3-9539f0e5cba3" />
<img width="1588" height="891" alt="image" src="https://github.com/user-attachments/assets/2451b716-d646-4fa3-beee-e75d8c3d7717" />
<img width="1920" height="998" alt="mars-close-orbit" src="https://github.com/user-attachments/assets/817e47e9-c4b9-4838-8d81-0a05d560ab3f" />
<img width="1914" height="993" alt="mars-sun-orbit" src="https://github.com/user-attachments/assets/6bd32046-2c16-44be-b572-778cdbe0f174" />

## Current state — F13c: A title screen worth the planet behind it

**The menu's background image is not an image — it is the game.** The engine has no texture pipeline and never needed one for this: the scene is already built before the menu appears, so the title screen renders the live world from a camera of its own — a slow orbit of Terra at ~6,700 km, parked near the terminator where the lit limb is at its most photogenic, drifting a full lap every thirteen minutes while the simulation stays paused. **STARWORKS** stands across the middle in three passes of the same 5x7 blockwork — glow, shadow, face — over a gradient scrim that darkens the space the title floats in and leaves the planet alone. The buttons are a centred column; a pause menu keeps your own frozen view behind it instead, because that is *your* game back there. Two small things the layout pass caught: labels now shrink to fit their row (the fifth HUD overflow found by arithmetic), and `(` never existed in the 5x7 charset — the pause menu had been silently rendering spaces for parentheses since F9.

## Previously — F13b: The code learns to be found

**`StarWorksGame.cpp` was 12,407 lines — a quarter of the project in one file.** It is now thirteen, one theme per translation unit, same class: the core keeps the constructor and the frame loop, and `GameShell`, `GameScene`, `GameTerrain`, `GameSaveLoad`, `GameFlight`, `GameHangar`, `GameFactory`, `GameFactoryUi`, `GameNetwork`, `GameHud` and `GameMap` each own the methods their name promises. The 750-line anonymous namespace that fed all of them became `GameInternal.hpp`. Nothing was rewritten — every line of the original landed exactly once, checked mechanically — and the map lives in `Game/Source/README.md` so the next method knows where it goes. Incremental builds stop paying for the whole game layer on every edit.

**And running the suite in both build types found the one place Debug and Release disagreed.** F12's adversarial test feeds the mirror an entity index of 4,294,967,295 and expects the refusal throw — but that value is also the null-handle sentinel, and an `SW_ASSERT` sat above the refusal: Release compiled it out and threw as designed, Debug trapped mid-suite. The assert is deleted; the existing bound check already refuses the value, in every build, which is the whole point of a guard on wire input. **223/223 tests green, Debug and Release both.** Milestones F10–F13 (packaging, firewall diagnosis, path independence, the jump gate, the shell) are documented in `docs/Architecture.md`.

## Previously — F9c: On foot, properly, and a check on the rocket that would not fall

**Jumping was losing about one press in three, and the reason is worth writing down.** `ShipControlsComponent` is cleared and rewritten once per *rendered frame*; the walker that reads it runs on the physics lane at a fixed **50 Hz**. Above sixty frames a second most frames tick that lane zero times, so a jump written as a one-frame edge was overwritten before any tick could see it. It is a **latch** now: the request stays set until a physics tick has actually run. Pressing twice inside one tick still jumps once, because the walker clears `isGrounded` as it goes and the second tick finds no ground to push off.

**Walking is twice as fast** — 8 m/s. The outpost is two hundred metres across and the pad is a hundred and twenty from the hub; at the old pace that was most of a minute of holding `W` to reach a rocket you had just paid for.

**`E` boards a rocket**, exactly as it opens a machine. One ray answers "what am I looking at", and the caller decides what to do with the hit: a building opens its panel, a vessel gets boarded. `P` still cycles between craft once you are flying — but getting *in* should not be a key that means something else when you are standing in front of the thing.

### The rocket that leaned and would not fall

Reported as tilted but apparently unaffected by gravity. Measured, on the real launch site, with the real terrain, the real hull and the game's own systems:

| Start lean | After 60 s | |
|---|---:|---|
| 0° | 0.25° | stands |
| 8° | 8.00° | stands |
| **15°** | **14.99°** | stands — the statics put the threshold at **15.1°** |
| **30°** | **88.43°** | falls flat |

So the physics is healthy, and the two ways it *looks* broken are both correct:

- **A lean under 15.1° is stable.** A body resting inside its own support polygon does not stand itself back up. For this rocket — a 1.2 m half-width on a 4.45 m half-length — that is fifteen degrees of perfectly legitimate lean.
- **A warp freezes it.** Above ×5 the simulation bubble converts a landed craft into a surface anchor at *whatever attitude it had*. Measured: frozen at 67.8° mid-topple, held there for the whole warp, and back to 91.1° — flat — sixty seconds after being released.

Both were invisible, so the HUD now says it: **`LANDED  LEAN 8 DEG  RESTING`**, or `TIPPING` in amber when the balance point has left the feet. The number comes from the same statics the ground contact uses, so the readout and the physics cannot disagree.

**One thing the probe taught about the probe.** Its first run showed the rocket climbing forty metres and never settling — which looked exactly like the reported fault. It was the harness: Terra's spin angle was never advanced, so the terrain was sampled in a frozen frame while the rocket co-rotated at 465 m/s, dragging it across nine kilometres of stationary landscape a minute. Measuring the wrong thing convincingly is the failure mode of measuring at all.

## Previously — F9b: The hangar draws, the VAB builds

Two rooms, two verbs, and the line between them is now the one the factory needs.

**The hangar only saves.** It has one button that touches the world and it does so once, on a press: `SAVE` writes the design as a `.swship` and registers it so a VAB can be told to build it. Not on every part placed — a drawing office that autosaved every click would fill the catalogue with forty variants of the same rocket. The button that used to manufacture is gone entirely, including the "rebuild this vessel in place" mode beside it, because a design office that can also manufacture makes the factory next door decorative.

**The VAB got a catalogue.** Its panel is split: the saved designs down the left, and on the right the one you have picked — **drawn, in three dimensions, turning** — with its part count, its dry mass, what it costs in each metal *against what is in the bin*, how long it will take at full power, and one `PRODUCE` button. Clicking a row now **selects**; ordering is a separate, deliberate press next to the price and the picture. Reading a catalogue is not placing an order.

**The preview is real geometry, and it needed a pipeline.** The HUD pass has no depth buffer and culls nothing, which is fine for flat panels and useless for a solid. So there is now a second screen-space pipeline that differs in exactly one flag: **back faces are culled**. That alone is a complete hidden-surface solution for a convex part — the front faces of a convex solid never overlap each other — and the parts are sorted back-to-front among themselves before submission. Two pipelines interleave in painter's order so the model sits *on* its panel and *under* the text.

The one trap is handedness. The camera negates Y once, in its projection, and the engine's counter-clockwise front-face convention was settled against that. A preview transform that skipped the flip would present the opposite winding and cull precisely the faces it should keep — the rocket would render inside out. Verified numerically rather than by eye, over a full turn of the model:

| | |
|---|---:|
| Determinant of the preview transform | **negative** — the same single flip the camera makes |
| Worst overflow past the preview box | **−0.014 NDC** (always inside) |
| One metre of design | **14.98 px across, 14.98 px down** — no distortion |
| Model height in a 205 px box | 188 px |

Framing fits the *shape*, not its bounding sphere: a rocket is long and thin, and a sphere around one is as wide as it is tall, which threw away a third of the height for a width nothing ever occupied.

## Previously — F9: You start on foot, and a rocket has to be paid for

**There is no starting rocket any more, and that is the whole milestone.** The game used to hand the player a nine-part vehicle parked in orbit, an asteroid to mine and eight cubes on rails pretending to be a station. All of it is gone. What is left is Terra, its moon, Mars, and one outpost on the ground — the mine, the smelter, the store, the solar farm, the battery bank, the beacon, the belts, the grid, the **VAB** and the **launch pad** — which were always the interesting half and were always decorative next to a free rocket.

**A vessel now exists only because a VAB built it.** The hangar's `BUILD` button used to instantiate a finished vehicle out of nothing; it is now `ORDER`. It writes the design to disk, registers it, and queues it at the nearest assembly hall, which pays for it **in iron and copper, at 40 kg a second, out of its own bin**, and ships the finished hull down the belt to a pad that unpacks it. That was already how the factory worked — it just had a free door beside it.

Measured on the shipped design and the outpost's own starting stock:

| | |
|---|---:|
| STARLING, 3 parts | **2,950 kg iron + 450 kg copper** |
| Build time at full power | **85 s** |
| The VAB's seeded stock | 3,000 kg iron, 500 kg copper — **exactly one** |
| With the copper missing | stalls at **87 %**, and the iron it did not need stays in the bin |

The outpost is stocked, not started: there is no standing order at start-up either, because seeding one would simply re-create the rocket that was just deleted. The second rocket comes from the mine.

**On foot is the normal state.** The player wakes up as a suit standing fourteen metres north of the hub, on the ground, at 464.62 m/s — Terra's equatorial spin, because a body standing on a planet is not still and one spawned at rest in the world frame watches the ground leave at half a kilometre a second. Piloting is something you **board**: `P` boards the nearest vessel, `G` steps back out beside it and co-moving with it. With nothing to board, both say so.

**Which meant every control had to stop assuming a cockpit.** The hangar key refused to open on foot — which would have put the design tool behind a vessel obtainable only through the design tool. `P` refused to act below two vessels, so the first rocket off the pad could never be boarded. And a dozen places dereferenced the ship without checking, from the throttle readout to the ship's own control block, each of them a crash on the first frame of a world that has no ship. `controlledEntity()` now falls back to the suit rather than to nothing, and the suit is created with the world and never destroyed.

**The HUD says why the warp key is doing nothing**, which matters more now that the answer is usually "you are standing on a planet with no vehicle".

## Previously — F8: A menu for the session, and a clock each

**Every player owns their own clock.** That is the decision the rest of this milestone hangs off. One pilot engages warp because *they* are waiting for an apoapsis; the person landing a rocket two hundred kilometres away is not, and dragging them along would make warp a thing you have to negotiate. So nobody is dragged. Two players in one session can be three hours apart, and the panel treats that as a reading rather than a fault.

**What crosses the gap is a stamped action.** An event does not happen "now", it happens AT AN INSTANT, and it carries that instant with it. A player in the future who separates a stage stamps it with the simulation second it occurred at, and every other player holds it — unopened — until their own clock reaches that second. Then it happens, for them, at exactly the moment it happened for him. Nobody's world is rewritten and nobody's world runs ahead of itself. The one case the rule cannot cover is an event that arrives stamped in the local *past*; there is no honest answer short of rewinding the simulation, so it lands at once and is **counted**, and the count is the diagnostic.

**The panel is on the right, on `F3`.** An address field you can actually type into, HOST and JOIN, the pilot list with each player's offset from your clock, and a `SYNC` chip on anyone ahead of you. The engine had no character input at all before this — no `glfwSetCharCallback`, no text buffer anywhere — so that is new too: typed characters come through the layout and the dead keys as *characters*, latched per frame exactly like the key-press events, and while a field has focus every gameplay key asks through one predicate rather than each guarding itself.

**Time warp is no longer free.** Past ×5 the integrator is switched off and everything goes on rails, which is exact when the motion already *is* a conic clear of the air and a fabrication otherwise. The old test was altitude, and altitude is not the same question — it happily allowed ×10,000 on a trajectory whose next event was the ground. The rule now is: **standing on something, or a closed orbit whose periapsis clears the atmosphere.** Everything else — an ascent, a reentry, an escape trajectory — is a situation whose outcome is decided by the integration the warp would skip.

**`SYNC` is the one warp that ignores the ladder.** Closing a three-hour gap from a 200 km orbit at the altitude-capped ×100 would take a real hour and nobody would ever press it, so the catch-up warp goes to ×10,000,000. It still obeys the orbit-or-ground rule, because that rule is about whether the world can be faked at all. Measured on the real servo — largest rung whose rate is under half the time remaining, re-chosen every frame:

| gap | real time to close | peak rate | overshoot |
|---|---:|---:|---:|
| 1 minute | 13.0 s | ×10 | 0.02 s |
| 3 hours | **45.7 s** | ×1,000 | 0.02 s |
| 1 day | 61.6 s | ×10,000 | 0.00 s |
| 1 year | 109.9 s | ×10,000,000 | 0.00 s |

At 30 fps the same gaps take 45.5 s and 109.2 s — the servo is frame-rate independent by construction, because it picks the rate from the time remaining rather than from a keypress count.

**Found by drawing it before running it.** The panel's height is computed from the roster length, and a mock of the layout — same NDC maths, same glyph advance, rendered to an image — put the footer verdict *outside* the panel. Five fixed rows, not four. Cheaper to find in a picture than in a screenshot.

## Previously — F7: Multiplayer, from the wire up

No design was handed down for this one, so every decision below is stated with the reason it was taken, because each of them closes doors.

**The host simulates; clients mirror.** Deterministic lockstep — every machine running the same simulation from the same inputs — sends almost nothing and was genuinely tempting for a game this size. It cannot work here, for three independent reasons, any one of which is fatal: this engine cannot promise bit-exact floating point across machines (the physics lane sums forces over parts in archetype order, the aerodynamic tables are interpolated, and the compiler may contract a multiply-add — one bit of divergence compounds into two different worlds within minutes, with no way to notice until it is enormous); **nobody could ever join a game in progress**, because lockstep starts everyone at frame zero and has no state to hand a latecomer; and time warp would have to be unanimous, which is not a mechanic anyone wants. So one machine owns the truth, and what a client sends back is **intent** — "pitch up, throttle 60 %" — never state. That is also the entire anti-cheat story: a client that lies about its inputs flies badly, and that is all it can do.

**A client's world carries the host's entity indices exactly.** The obvious design — a network id per entity, mapped to a locally allocated one — dies on the first component that stores an entity handle, and this game is full of them: a conveyor names its body and its link, a cable names its two poles, a cloud deck names its planet, a part names its vessel. Nothing declares which of a component's fields are handles, so nothing could rewrite them. The save file reached this conclusion years ago and restores indices exactly for precisely this reason; replication simply does it **incrementally**, through one new ECS primitive and type-erased component access — because a mirror learns its component types at runtime, from a table the host sends at the handshake.

**A delta is the difference from the last snapshot the client CONFIRMED**, never from the last one sent. On a lossy link those are not the same thing, and diffing against something the client never received produces a world that is wrong and stays wrong. Diffing against the last acknowledged state means a lost snapshot costs a few extra bytes in the next one and nothing else — there is no repair path because there is nothing to repair. A snapshot naming a baseline the client does not hold is refused whole, without touching a single entity.

**Change detection is a `memcmp`.** Components are trivially copyable by ECS rule, so "did this change" is a byte comparison: no per-component code, no dirty flags, and no chance of a system forgetting to raise one. The single place bytes are not enough is a **recycled entity index** — same index, new generation, possibly identical bytes — where the mirror must throw the old occupant away wholesale; that entity is forced to resend everything. There is a test that fails without it, and it fails by leaving an empty entity holding somebody else's links.

**Two delivery services over one UDP socket.** State deltas are **unreliable and sequenced**: a lost one is worthless because a newer one already exists, and resending it would deliver stale truth *behind* fresh truth. Everything else — handshake, world transfer, pilot input, disconnect — is **reliable and ordered**, with acknowledgement bitfields, timed resends and fragmentation. TCP was never an option, precisely because it forces ordering on the half that must not have it: one lost delta would stall every delta behind it, which is the freeze-then-teleport that makes a game feel broken.

**Measured**, on real UDP sockets with the real component layouts (`Tools/NetProbe`):

| | Co-op (5 craft, 100 entities) | Busy (20 craft, 500 entities) |
|---|---:|---:|
| Downstream, per client | **14.7 kB/s** | 56.6 kB/s |
| Upstream (input at 50 Hz) | **1.6 kB/s** | 1.6 kB/s |
| One delta | 606 B — **fits one datagram** | 2,316 B |
| Host cost per snapshot | 5.0 µs | 25.8 µs |
| World in the client's mirror | — | **0.9 ms** after connecting |

The encoder costs 112 µs at 2,000 entities; at twenty snapshots a second that is **0.2 % of one core** for a busy session. Bandwidth follows *what moved*, not how big the world is: at 500 entities, 40 records of 620 change.

**Eighteen new tests, and not one of them opens a socket.** The transport sits behind a two-line interface, so the suite runs against a simulated wire with a seeded generator and a clock that is passed in: "80 ms latency, 60 ms jitter, 20 % loss, four seconds" executes in microseconds and gives the same answer every time. After those four seconds of a genuinely bad link — packets lost, packets overtaking each other, an entity appearing halfway through — every value in the mirror still agrees to 1e-9 with the instant the client believes it is looking at.

**What is deliberately not here yet.** No interest management: every client is sent every replicated entity, which is why the 500-entity delta needs three datagrams instead of one. That is the next thing to build, and the game design already argues for it — a player on Terra does not need Luna's base at twenty hertz. And nothing is wired into the game layer: no avatars, no command vocabulary, no host/join screen. This milestone is the wire, proved.

```powershell
NetProbe        # stand a host and a client on real sockets, print what it costs
```

## Previously — F6: Real aerodynamics, a ground that holds, and an autopilot that can fly both

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

### And the ground had been ignoring rotation entirely

Contact touched a body's **velocity** and stopped there: the lowest point clamped to the terrain, the impact absorbed, the sliding rubbed off. Its rotation was never mentioned. A rocket that came down turning kept turning on the dirt for ever, and one that landed leaning stayed leaning — propped on the corner of its own bounding box, as though the planet under it were a rumour.

Two forces were missing, and they are the same two the linear case already had.

**Gravity**, through the oldest question in statics: does the centre of mass fall inside the **support polygon**? The hull corners actually touching the ground are the support; project the balance point onto the ground between them and the body stands, with the ground holding it up and nothing to explain. Push it past the edge and gravity gets a lever, the edge becomes a pivot, and it topples — faster as it turns, because the overhang grows. One rule, nothing scripted, and it produces the whole behaviour: measured on a 12 m rocket 2.4 m across, it stands at **11°** and goes over at **12°**, against a hand calculation that puts the threshold at 11.5°; the fall accelerates to 0.51 rad/s; and it comes to rest on its flank at 89° and stays there, because lying down the balance point is inside the support again.

**Friction**, which rubs the spin off the way it already rubbed off the slide. A landed rocket spinning at 1 rad/s is under 0.1 rad/s in one second and stopped dead in three.

And the regression that mattered more than either: an upright rocket left alone for thirty seconds is still upright, still at exactly 6.000 m, still doing 0.000 m/s and 0.000 rad/s. Contact that writes to a rotation must not make anything shiver.

### And the ground you saw was not the ground you stood on

The next report was a rocket half-buried in a hillside, and the player falling through the floor in some places and not others. Physics was innocent: it samples the analytic heightfield exactly. The **renderer** draws a grid of samples with flat triangles stretched between them, and those two surfaces agree only as well as that grid is fine.

The patch's resolution is chosen from your altitude — and it was measuring that **from sea level**. Standing on the launch site's 1,100 m plateau, the game sized the patch for somebody *flying* at 1,100 m and drew a 6.6 km square in 137 m cells. Terra's terrain reaches 9 km; almost nowhere interesting is at sea level, so this was not an edge case.

Measured against the collider at the launch site, over three sites and a quarter of a million sample points:

| | worst sink | worst hover | rms |
|---|---|---|---|
| before (sea altitude → 6.6 km patch, 137 m cells) | **8.77 m** | 17.72 m | 2.30 m |
| after (ground altitude → 1.5 km patch, 15.6 m cells) | **0.50 m** | 2.30 m | 0.14 m |

Two changes, and the measurement chose both. The level of detail now reads **height above the ground**, not above the sea. And the cell count is no longer a constant — what matters is the cell SIZE where somebody is standing, so the landing grid went from 96 cells to 192 while the four-hundred-kilometre patches keep the cheap one. The heightfield is a *ridged* fractal, so it has creases and the chord error across a cell falls with the cell width rather than its square: this is a resolution contract, not a tuning knob, and `TerrainTests` now pins it as one.

### Something to stand on, and something to measure it against

**Relief shading, baked.** Two terms computed on the height grid the patch already holds — no extra heightfield evaluations, and therefore no way to disagree with the surface being drawn. A **cast shadow** marched toward the sun one cell at a time, which is what puts a ridge's shadow in the valley beside it; and a **sky occlusion** term, the same march in six directions with no sun in it, which darkens gullies and crater floors and leaves ridges alone. Both are baked into the vertex albedo, so they multiply the shader's own lambert instead of replacing it. The sun is captured per rebuild: Terra turns 0.004° in the second between two of them, so a shadow one rebuild old is a shadow that is right. Measured on Terra's roughest ground the term runs **0.40 to 1.00**; on the launch plain it is flat at 1.00, because that ground really is flat at fifteen metres and honest shading of flat ground is no shading. 21 ms on the worker thread.

**Which is why there are plants.** What makes a plain read is the field standing on it: a hillside 400 m away and one 40 m away are the same wash of colour until something of known size is growing on both. Around **5,600 tufts** within 120 m, baked straight into the patch — one mesh, one draw call, no collision to author, rebuilt by the same worker that rebuilt the ground under them. The scatter is anchored to the **planet**, not to the patch: each tuft's lattice cell comes from an absolute latitude and longitude, so when the patch re-centres under a walking player the grass stays where it was growing. Where they grow is decided by the ground itself — the palette's own greenness, the slope, and whether it is above the surf — so a plant can never appear on a colour that would not support it, and no biome table has to be kept in step with the one the terrain already has. They inherit the ground's baked shadow with its colour.

**And the rim stopped showing what is under it.** The globe is a second ground surface beneath the patch, 133 km between vertices at that level of detail, and it has to stay — beyond the patch's rim it *is* the horizon. It should never be seen, and at the rim it was: the sheet simply stopped and the eye followed the cut down onto the surface below. A skirt dropped from the border vertices closes it, for 4 × 192 triangles. Nothing else about that second surface needed changing: front-to-back batching plus the early depth test already reject every one of its hidden fragments before it evaluates a single noise octave.

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

## Running it on another machine

> Every command, in order, from a bare machine to a LAN game: **[INSTALL.md](INSTALL.md)**.

**`build\windows\bin\Debug\StarWorks.exe` runs on the machine that built it and on no other**, and the reason has nothing to do with the game. A Debug build links the *debug* C++ runtime — `vcruntime140d.dll`, `msvcp140d.dll`, `ucrtbased.dll` — and Microsoft does not redistribute those: they ship with Visual Studio and with nothing else. Windows refuses to start the process before a single line of our code runs, which is exactly why the failure has no log, no message box and no clue in it.

```powershell
.\package.ps1            # -> dist\StarWorks\, copy the whole folder
.\package.ps1 -Zip       # ...and dist\StarWorks.zip
```

That builds RelWithDebInfo with the runtime linked **statically** (`-DSW_STATIC_RUNTIME=ON`) and stages the executable together with the compiled shaders and the assets. The target machine then needs **a Vulkan 1.3 graphics driver and nothing else** — no Visual Studio, no Visual C++ redistributable, no Vulkan SDK.

Two things that also stop it starting, both worth checking before anything else:

- **The folder, not the file.** The game resolves `Shaders/Mesh.vert.spv` and `Assets/Parts/*.swpart` relative to its own executable, never to the working directory. A lone `.exe` copied out of `bin\` will not run.
- **No Vulkan device.** A fresh Windows install using the Microsoft Basic Display Adapter has no Vulkan at all. `.\StarWorks.exe --log-file starworks.log` says so in the log; `--cpu` will run without a GPU if you have a software loader, very slowly.

## Playing over the local network

**The router is not involved and needs no configuration.** Two machines on `192.168.1.x` address each other directly; the router only forwards the frames, and there is nothing to port-forward because nothing is crossing the internet. What blocks a LAN game is the **hosting machine's own firewall**.

Windows Defender Firewall drops every *unsolicited* inbound datagram. The joining side is never the problem — it sends first, so the reply comes back through the flow its own packet opened. The host is the problem: it only ever waits to be spoken to, and the very first `ConnectRequest` is what gets dropped. The client retries, gives up, and reports a timeout; the host logs nothing, because its process never saw a byte.

**The game asks for this itself.** Press `HOST` and, if no rule covers it yet, Windows raises one UAC prompt; accept it and the rule is added. It is checked first — reading the firewall needs no privileges — so a player who accepted once is never asked again, and a prompt on every press would only train them to click through it.

**The game itself is never elevated.** What runs as administrator is a one-shot `netsh` that adds a rule and exits. A game running as administrator writes its saves, logs and crash dumps as administrator and refuses drag-and-drop from Explorer; none of that is a price worth paying for a firewall rule. Refusing the prompt is a legitimate answer: hosting continues, and the panel says `FIREWALL: RIGHTS REFUSED` so the timeout the other player is about to see is not a mystery.

**The rule follows the executable, not the port** — UDP inbound, Private and Domain profiles only, never Public. It stays valid if you host on another port, it opens nothing for anything else on the machine, and a rule Windows created through its own "allow this app?" prompt counts as satisfying it.

`firewall.ps1` does the same thing from a shell, for when you would rather not go through the game — setting up a machine ahead of time, or covering a build you are not about to run:

```powershell
.\firewall.ps1                        # add the rule (administrator)
.\firewall.ps1 -Exe <path>\StarWorks.exe # for a specific build
.\firewall.ps1 -Check                 # change nothing, just report
.\firewall.ps1 -Remove                # take it back out
```

### When it still times out

`ping` is a bad witness here: **Windows blocks inbound ICMP echo by default** (the "File and Printer Sharing (Echo Request - ICMPv4-In)" rule ships disabled), so two perfectly connected PCs on the same switch will refuse to ping each other out of the box. Ping failing proves nothing.

`netcheck.ps1` speaks plain UDP instead, with nothing of ours in it — so it can tell you whether the game is the problem or was never given a chance. Run the two halves at the same time, on the two machines:

```powershell
.\netcheck.ps1 -Listen              # on the machine that would host
.\netcheck.ps1 -Send 192.168.1.61   # on the other one
.\netcheck.ps1 -Local               # just describe this machine
.\netcheck.ps1 -AllowPing           # let this machine answer ping (diagnosis only)
```

The listener answers every datagram, so the sender learns whether the path works **in both directions** — a firewall is a one-way device and the return trip is the half that usually works. It also prints each machine's address *with its prefix*, and looks up ARP: two machines on different subnets will never reach each other whatever rules you write, and a router with client/AP isolation on (guest Wi-Fi, most commonly) blocks PC-to-PC traffic by design.

And the host now says what reached it. While hosting, the `F3` panel shows `RX n  REFUSED n` with a verdict under it:

- **`NOTHING HAS REACHED THIS PC`** — no datagram arrived at all. Firewall, Public profile, wrong address, or the two machines cannot see each other.
- **`PEER SPEAKS Vn - REBUILD IT`** — the packets arrive fine and carry a protocol version this build does not speak. Two different builds of the game; no firewall rule will ever fix it, and this failure is silent on the client, where it looks exactly like a blocked port.
- **`PACKETS ARE GETTING THROUGH`** — the network and the firewall are both fine.

`-Check` also prints the two other things that decide whether this works:

- **The address to type.** Found by asking the routing table, not by reading `ipconfig` — a machine with Docker, WSL, a VPN and a Bluetooth adapter has half a dozen addresses and only one of them is the right one. The game prints the same address in the `F3` panel when you press `HOST`, so you can just read it off the hosting screen.
- **The network profile.** Windows blocks inbound on a **Public** network regardless of the rule, and it silently picks Public for any network you never answered the "make this PC discoverable?" prompt for. `Set-NetConnectionProfile -InterfaceIndex N -NetworkCategory Private` fixes it.

The join panel distinguishes the two failures, because the cure for one is useless against the other: **`NO REPLY - CHECK FIREWALL`** means not a single datagram ever came back — nothing at that address answered, so the packets are dying before the host's process sees them (firewall, wrong address, nobody hosting). **`TIMED OUT - LINK LOST`** means the link was alive and then stopped, which is a real network fault and not a configuration one.

## Running the tests

223 tests, no window, no Vulkan device and no socket required — matter conservation across every recipe, warp exactness, orbital mechanics, aerodynamics against textbook shapes, collision, HUD layout, the whole network stack against a simulated lossy wire, the timeline that holds a future action until its instant arrives, the warp gate, a guard that fails if any script or source file hardcodes a drive letter, and the `.swpart` / `.swrecipe` / `.swship` / `.aero.json` files as shipped.

```powershell
ctest --test-dir build/windows -C Debug --output-on-failure   # Windows
ctest --test-dir build/linux-debug --output-on-failure        # Linux
```

## Controls

**Ship (default mode):** `W`/`S` main engine forward/retro, `A`/`D` yaw, `↑`/`↓` pitch, `Q`/`E` roll, `X` kill rotation while held, `Shift`/`Ctrl` throttle up/down. `Tab` switches between pilot and free camera. `G` goes EVA. `V` toggles the reference frame between orbital and **surface-relative** — the speed readout, the navball's prograde/retrograde markers AND the autopilot all follow it, and the active frame is printed above the autopilot buttons. The artificial horizon (bottom center) shows attitude vs the local horizon; the chase camera auto-levels on the horizon when you are low and suborbital.

**Autopilot (`T` cycles, or click):** `SAS` holds the craft STILL — it drives the rotation rate to zero and keeps fighting, with the RCS's own authority and no more, so it settles a wobble in under a second and honestly loses to a real aerodynamic tumble. `PGD` / `RTG` point along the velocity **in whichever frame `V` has selected**; on a landing the orbital and surface retrogrades are around 100° apart, so this is the difference between flying the navball and fighting it. `NODE` holds the burn vector. Every button toggles: clicking the lit one switches the autopilot off, and any rotation input pauses it while you fly by hand.

**On foot (EVA) — where you start, and the normal state.** First person. `W`/`S` walk, `A`/`D` SIDESTEP (the mouse turns you — the suit faces where you look), `Space` jumps. `E` boards the rocket you are looking at (the same key that opens a machine's panel); `P` boards the nearest one; `G` steps back out beside it. Everything else — the map, the hangar, the build catalogue, the machine panels, the multiplayer panel — is reachable from here. `F` opens the **building catalogue**: pick one, look at the ground, left-click to place it, mouse wheel to spin it, `R` to raze what you are looking at. Belts and cables are laid the same way and with the same two clicks — pick the machine that ships, then the one that receives. `E` at a machine opens its **front plate**: state, its share of the grid, what is in the bin, and the jobs its category can run (or, at a VAB, the saved designs it can build). `F2` shows the collision hulls.

**Free camera:** hold the right mouse button to look around, `WASD` to move, `Q`/`E` down/up, `Shift` to boost, mouse wheel to change speed.

**Star map:** `M` toggles the system view (real scale, beacon markers, patched-conics flight plan with encounter/impact/SOI-exit markers); mouse wheel zooms from LEO out to the whole Sol system. The map centers on your current SOI primary.

**Target (in map view):** left-click a body to target it, click it again to clear. The HUD then shows the CLOSEST APPROACH your plan makes to it — distance, time, relative speed — and the map marks **where that body will have moved to** by then, on its own orbit ring, with your own position at that moment and the gap between them. With a maneuver node up you get a second line for what the burn would achieve.

**Maneuver nodes (in map view):** `N` create/delete, `J`/`L` node time −/+, `I`/`K` prograde +/−, `U`/`O` normal −/+, `Y`/`H` radial +/−. The step is a ladder on the modifiers, and it moves the node's time by the same factor: nothing = 1 m/s / 10 s, `Ctrl` = 0.1 m/s / 1 s, `Shift` = 10 m/s / 100 s, `Alt` = 100 m/s / 1 000 s, `Ctrl+Shift` = 1 000 m/s / 10 000 s. The armed step is on the HUD under the node's vector. Or **grab the violet marker with the left mouse button and drag it along the orbit** — the node's time follows the pixel under the cursor, and the planned trajectory redraws as you move. Fly the burn with the **NODE** SAS button (it holds the nose on the burn vector, which is almost never prograde) and the **WARP TO NODE -1 MIN** button, then burn until DV reaches zero — the readout counts down against the trajectory you would have coasted, so gravity is not mistaken for thrust.

**Chase camera:** hold the right mouse button to orbit around the craft, mouse wheel to zoom, `C` to reset behind it.

**Hangar (VAB):** `B` opens/closes, on foot or in a cockpit (sim paused; separate view). Right-drag orbits, wheel zooms. Click the palette to take a part IN HAND — it follows the mouse: cyan stack nodes snap it magnetically, or it glues to any hull surface under the cursor (radial parts). `W`/`S`/`A`/`D`/`Q`/`E` rotate the held part in 90° steps, left-click places, `ESC` puts it back, `DEL` discards. Click a placed part to grab its whole subtree. `X` cycles symmetry (x1/2/3/4/6/8, radial placements), `C` toggles the CoM (yellow) / thrust (violet) flags, UNDO removes the last placement (symmetry ring included). NEW starts a fresh design, LOAD cycles the world's vessels into the editor, **SAVE** — the only button here that touches the world, and only on that press — writes it as a `.swship` and registers it so a VAB can build it. The hangar itself creates nothing; walk to the VAB, pick the design out of its catalogue, and press PRODUCE. `P` in flight switches the piloted vessel. `Space` (or `Z`) fires the decoupler (staging); on foot, `Space` jumps.

**Part Studio (`PartStudio.exe`):** the part authoring tool. Right-drag orbits, left-click selects a primitive or node, `G`/`R`/`S` move/rotate/scale with `X`/`Y`/`Z` axis constraint (`Shift` = fine, grid-snapped), `K` toggles the orange collision overlay, `DEL` removes. Buttons add primitives (box/cylinder/cone/sphere/tube), duplicate, set colors/emissive/segments, flag visible/collider, add stack/radial nodes (X/Y/Z sets a node's direction, SNAP SURF projects it onto the hull). SAVE writes the `.swpart` into the build AND the source `Assets/Parts/` — the game loads it at next launch.

**Time warp:** `.` faster / `,` slower, ×0 up to ×10,000,000. ×2 and ×5 are PHYSICS warp: everything stays simulated and the engines still work. Above ×5 the world rides analytic rails, engines cut out — and you must be **standing on something or in a closed orbit whose periapsis clears the atmosphere**, because rails cannot express a reentry, a suborbital arc or a decaying orbit, and warping one used to hand back a vehicle somewhere it could never have reached. The altitude ladder still caps the rate on top of that. `,` at ×1 stops time — pausing is the bottom rung of the same ladder, not a key of its own.

**Multiplayer (`F3`):** the panel on the right. Type an address into the field (click it, then digits, `.` and `:`), `HOST` or `JOIN`, and the pilot list shows everyone in the session **with the gap between their clock and yours** — because a warp here is personal, and two players can legitimately be hours apart. `SYNC` on anyone ahead of you warps you forward until you reach their instant, bypassing the altitude cap (up to ×10,000,000) but not the orbit-or-ground rule. Measured: a three-hour gap closes in 46 real seconds, a year in 110. `HOST` prints the address the other machine should type — read off the routing table, not guessed — and asks Windows once for the firewall rule that lets that machine through (see *Playing over the local network*).

**Menus:** the game boots into a **loading bar** that measures real work — seven named steps, one per frame — then a **main menu**: NEW GAME, LOAD GAME, SETTINGS, QUIT. `Esc` in flight opens that same menu rather than quitting (it used to close the window on the spot, with no chance to save); `Esc` again backs out. **SAVE GAME** writes a *named* save into `Saves\<name>.sav` — type a name, or keep the suggested one. `F5`/`F9` are still the quicksave pair, unchanged, into `starworks.sav`. The load list shows every save newest first, with its age and size.

**Files:** `F5` quicksaves, `F9` quickloads, `Esc` opens the menu.

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

See `docs/Architecture.md` for the module map, engineering conventions, and the development log, and `docs/Performance.md` for a measured cost breakdown — per-system tick times, terrain patch build times, geometry counts, network bandwidth per client and where the ceilings are.
