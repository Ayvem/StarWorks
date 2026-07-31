#pragma once

// ============================================================================
// Physics/PhysicsComponents.hpp
// The two mutually exclusive motion regimes of the engine:
//
//  - DynamicBodyComponent : TRULY simulated. Newtonian gravity integrated
//    every Physics tick (50 Hz). Reserved for objects near the player —
//    the BubbleSystem enforces this.
//  - OnRailsComponent     : analytic Kepler orbit. No integration ever; the
//    RailsSystem refreshes the transform at low frequency from the closed-
//    form solution. This is the default state of every distant object.
//
//  - GravitySourceComponent marks celestial bodies that generate gravity.
// ============================================================================

#include "ECS/Entity.hpp"
#include "Physics/Kepler.hpp"

#include <algorithm>

namespace sw::phys
{
    struct DynamicBodyComponent
    {
        WorldVec3 velocity{0.0}; // m/s, world frame
        f64 mass = 1000.0;       // kg (used by thrust; gravity is mass-free)
        /// Effective drag coefficient x area / mass (m^2/kg): atmosphere
        /// deceleration = 0.5 * rho * v^2 * ballistic. ~0.002 for a ship.
        f64 ballisticFactor = 0.002;
        /// BODY-FRAME SPIN, rad/s. It lives here and not on the game's
        /// ship because it is rigid-body state, and because the thing that
        /// most wants to change it is the ENGINE: aerodynamic moments are
        /// computed down here, and a weathercocking rocket that could not
        /// reach its own angular velocity would be a torque with nowhere to
        /// go. Attitude control writes it too; the two simply add.
        Vec3 angularVelocity{0.0f};
        /// Set by SurfaceInteractionSystem while resting on a body surface.
        u32 isGrounded = 0;
    };

    /// WHAT A BODY RESTS ON: an axis-aligned box in the entity's MODEL
    /// space, tight around its collision hull.
    ///
    /// Without one, ground contact has no idea how big the thing landing
    /// is, and the only position it can compute is "put the ORIGIN on the
    /// ground" — which buries every object up to its origin. A rocket
    /// modelled around its centre sinks half its length; the EVA capsule, a
    /// 2 m body centred on itself, sinks exactly 1 m and the player walks
    /// with their waist in the rock.
    ///
    /// A box rather than a radius, on purpose. A bounding SPHERE would
    /// float a 20 m rocket ten metres above the pad, because a rocket's
    /// bounding radius is half its LENGTH. The box knows the difference
    /// between the direction a vehicle is long in and the direction it is
    /// wide in — which is the entire question when it is standing on its
    /// tail.
    struct GroundHullComponent
    {
        Vec3 centre{0.0f};      // model space, metres
        Vec3 halfExtents{0.0f}; // model space, metres
    };

    inline constexpr u32 kMaxHullBoxes = 8;

    /// THE SOLID SHAPE of an object, in its own frame.
    ///
    /// `GroundHullComponent` above is the single box an object RESTS on. This
    /// is the several boxes it BUMPS INTO — the part's authored hitboxes,
    /// copied onto the entity at spawn so collision never has to go back to
    /// the catalogue, and so an entity's solidity is a fact about the entity
    /// rather than a lookup that could fail.
    ///
    /// `radius` is the bounding sphere around the entity origin, and it is
    /// the point of the whole component: the broad phase rejects a pair with
    /// one f64 subtraction and one comparison, so a base of a hundred
    /// buildings costs a hundred distance checks and then almost nothing.
    ///
    /// An entity with no HullComponent is not solid. That is how conveyor
    /// decks and cables stay walk-through: you step over a belt, you do not
    /// climb it.
    struct HullBoxComponent
    {
        Vec3 centre{0.0f};
        Vec3 halfExtents{0.5f};
    };

    struct HullComponent
    {
        HullBoxComponent boxes[kMaxHullBoxes]{};
        u32 count = 0;
        f32 radius = 0.0f;
    };

    /// Marks a hull that gets PUSHED OUT of the others. The player is one;
    /// buildings are not, because a building does not move for anybody.
    /// Keeping it a tag rather than a flag means the collision system's
    /// query is "every mover", not "every hull, then filter".
    struct HullMoverComponent
    {
        /// How far the mover may be displaced in one tick, metres. A cap:
        /// a body that somehow starts deep inside a building gets walked
        /// out over a few ticks instead of being flung across the map.
        f32 maxPushM = 1.5f;
    };

    /// How far the hull reaches HORIZONTALLY from the origin — the radius
    /// of its footprint on the ground. Ground contact samples the terrain
    /// out at this distance as well as under the centre, because on a slope
    /// it is the uphill edge of a footprint that touches first.
    ///
    /// Conservative: the corner of the box projected onto the horizontal
    /// plane. A null hull has no footprint.
    [[nodiscard]] inline f64 footprintReach(const GroundHullComponent* hull,
                                            const Quat& rotation, const Vec3& up)
    {
        if (hull == nullptr)
        {
            return 0.0;
        }
        const Vec3 localUp = glm::inverse(rotation) * up;
        // Total extent along each model axis, minus the part of it that
        // points straight up or down.
        const Vec3 extent = hull->halfExtents;
        const f32 vertical = std::abs(localUp.x) * extent.x +
                             std::abs(localUp.y) * extent.y +
                             std::abs(localUp.z) * extent.z;
        const f32 diagonal = glm::length(extent);
        const f32 horizontal =
            std::sqrt(std::max(0.0f, diagonal * diagonal - vertical * vertical));
        const Vec3 centre = hull->centre;
        const f32 centreVertical = glm::dot(centre, localUp);
        const f32 centreHorizontal = std::sqrt(std::max(
            0.0f, glm::dot(centre, centre) - centreVertical * centreVertical));
        return static_cast<f64>(centreHorizontal + horizontal);
    }

    /// Distance from the entity origin down to the lowest point of the
    /// hull, for a body with this rotation standing under this `up`. The
    /// resting radius is the terrain height plus this.
    ///
    /// The projection is done in MODEL space — a length is the same in
    /// either frame — so it is exact for the box at ANY attitude: a rocket
    /// lying on its side rests on its flank, not on its tail.
    [[nodiscard]] inline f64 groundClearance(const GroundHullComponent& hull,
                                             const Quat& rotation, const Vec3& up)
    {
        const Vec3 down = glm::inverse(rotation) * (-up);
        const f32 clearance = glm::dot(hull.centre, down) +
                              std::abs(down.x) * hull.halfExtents.x +
                              std::abs(down.y) * hull.halfExtents.y +
                              std::abs(down.z) * hull.halfExtents.z;
        // A hull whose box does not enclose its own origin can report a
        // negative clearance; resting "below" the ground is never what the
        // caller means.
        return static_cast<f64>(std::max(0.0f, clearance));
    }

    /// WHY A LANDED ROCKET FALLS OVER — and why one standing on its tail
    /// does not.
    ///
    /// Ground contact used to touch a body's VELOCITY and nothing else: the
    /// lowest point clamped to the terrain, the impact absorbed, the sliding
    /// rubbed off. Its ROTATION was never mentioned, and the result is the
    /// thing you cannot un-see once you have seen it — a rocket lying across
    /// a hillside at forty degrees, propped on the corner of its own
    /// bounding box, still turning at whatever rate you last left it, as
    /// though the planet under it were a rumour.
    ///
    /// What was missing is the oldest question in statics: does the centre
    /// of mass fall inside the SUPPORT POLYGON? The hull corners actually
    /// touching the ground are the support; project the centre of mass onto
    /// the ground between them and the body stands. Push it past the edge
    /// and gravity gets a lever, the edge becomes a pivot, and the thing
    /// topples — faster as it goes, because the overhang grows as it turns.
    ///
    /// That one rule produces all of the behaviour and none of it is
    /// scripted: a rocket on its tail is stable until it leans past its own
    /// base radius (a 2.4 m-wide vehicle balanced 4 m up: about 17 degrees),
    /// it accelerates over once it does, and it comes to rest on its flank
    /// because THERE the centre of mass is back inside the support again.
    ///
    /// Everything is computed in MODEL space — a torque does not care which
    /// frame it is written in, and the hull's corners are already there.
    /// `up` is the local vertical in WORLD space; `inertia` is the body's
    /// diagonal tensor in its own frame. Returns a BODY-FRAME angular
    /// acceleration, exactly zero while the body is balanced.
    [[nodiscard]] inline Vec3 topplingAcceleration(const GroundHullComponent& hull,
                                                   const Quat& rotation, const Vec3& up,
                                                   const Vec3& centreOfMass,
                                                   const Vec3& inertia, f64 massKg,
                                                   f64 gravity)
    {
        const Vec3 localUp = glm::normalize(glm::inverse(rotation) * up);

        Vec3 corners[8];
        f32 heights[8];
        f32 lowest = std::numeric_limits<f32>::max();
        for (int i = 0; i < 8; ++i)
        {
            corners[i] =
                hull.centre + Vec3((i & 1) ? hull.halfExtents.x : -hull.halfExtents.x,
                                   (i & 2) ? hull.halfExtents.y : -hull.halfExtents.y,
                                   (i & 4) ? hull.halfExtents.z : -hull.halfExtents.z);
            heights[i] = glm::dot(corners[i], localUp);
            lowest = std::min(lowest, heights[i]);
        }

        // WHICH CORNERS ARE TOUCHING. A tolerance and not an equality, both
        // because a body resting flat has four corners at one height only in
        // exact arithmetic, and because the hull is a box standing in for a
        // shape that is not one.
        //
        // AND IT IS THE FOOTPRINT IT SCALES WITH, not the diagonal. What the
        // tolerance has to cover is the height spread ACROSS THE FACE THE
        // BODY IS STANDING ON — that face's width times the sine of its lean
        // — because the whole face is what holds the body up and a rule that
        // sees only the lowest edge of it has a support polygon of zero
        // width, which means a NONZERO answer at every lean, in either
        // direction, for a body that is standing perfectly safely. The width
        // across the ground is therefore the right yardstick, and the box
        // diagonal is the wrong one twice over: on a tall body it is driven
        // by the LENGTH, which nothing rests on, and being an absolute length
        // it does not scale — the same shape twice the size needs twice the
        // slop and got the same 0.5 m.
        //
        // The cap is what keeps the slop from swallowing the hull it is
        // measuring: half the hull's own height above its lowest corner. It
        // is not a taste judgement, it is a proof. Write d_k for the height
        // spread each axis contributes, 2 * halfExtent_k * |up_k|; every
        // corner sits at a subset sum of those three above the lowest one,
        // the face we want is the one perpendicular to the LARGEST d, and
        // every corner NOT on it is at least d_max up. The cap is exactly
        // (d_x + d_y + d_z) / 4 <= (3/4) d_max, so the far face can never
        // join the contact set and `support` can never drift to the centroid
        // of the solid. On a 16 m x 16 m deck 0.8 m thick that is the whole
        // ballgame: the uncapped rule gave 0.9057 m of slop against 0.8 m of
        // hull, and MEASURED, the deck answered EXACTLY 0 at every lean from
        // 0 to 3.00 deg — not because the support had become the box centre,
        // but because SIX corners were in the set, their centroid sat
        // 2.667 m from the hull centre toward the low edge, and a 2.665 m
        // lean against a 10.670 m reach took the `overhang <= 0` early-out
        // every time. At 3.25 deg the set fell to the four corners of the
        // low edge face, reach collapsed from 10.670 m to 0.0227 m, and the
        // answer jumped to 0.9133 rad/s^2. Capped, the deck leaning 1 deg
        // gets 0.9164537 rad/s^2 and the curve is continuous from there out.
        //
        // What the two together buy, MEASURED by bisecting for the first lean
        // that is not exactly 0.0: it lands within 1e-5 deg of the true
        // atan(halfWidth/comHeight) threshold for every box at least sqrt(3)
        // times taller than it is wide — which is the condition
        // d_max >= 3 * (the other two) written out — and it does so at any
        // SIZE, a 2.4 m-wide hull and a 4 m-wide one of the same proportions
        // giving the same threshold to five decimals. The 12 m test rocket
        // and a 20 m one both come out at 11.309942 deg against a true
        // 11.309932, where the old absolute cap left the 20 m one answering
        // 0.0417 rad/s^2 at 8 deg of perfectly stable lean. At exactly
        // sqrt(3) the margin is 2.4e-4 deg; squatter than that the cap binds
        // first and the body gets a small RESTORING answer somewhat before
        // its true threshold (a 1:1 cube at 18.43 deg against a true 45) —
        // it settles flat, which is the right direction; it just does not get
        // there by standing perfectly still.
        const f32 verticalHalf = glm::dot(hull.centre, localUp) - lowest;
        const Vec3 axisUp = glm::abs(localUp);
        const Vec3 acrossGround =
            glm::sqrt(glm::max(Vec3(0.0f), Vec3(1.0f) - axisUp * axisUp));
        const f32 footprint = 2.0f * glm::dot(hull.halfExtents, acrossGround);
        const f32 tolerance =
            std::min(std::max(0.05f, 0.25f * footprint), 0.5f * verticalHalf);
        Vec3 support{0.0f};
        u32 touching = 0;
        for (int i = 0; i < 8; ++i)
        {
            if (heights[i] <= lowest + tolerance)
            {
                support += corners[i];
                touching += 1;
            }
        }
        if (touching == 0)
        {
            return Vec3(0.0f);
        }
        support /= static_cast<f32>(touching);

        // How far the centre of mass leans, measured ALONG the ground.
        const Vec3 offset = centreOfMass - support;
        const Vec3 lean = offset - localUp * glm::dot(offset, localUp);
        const f32 leanDistance = glm::length(lean);
        if (leanDistance < 1.0e-5f)
        {
            return Vec3(0.0f); // balanced over the centre of its own feet
        }
        const Vec3 direction = lean / leanDistance;

        // How far the SUPPORT reaches the same way. Inside it, the ground
        // holds the body up and there is nothing to explain.
        f32 reach = 0.0f;
        for (int i = 0; i < 8; ++i)
        {
            if (heights[i] <= lowest + tolerance)
            {
                reach = std::max(reach, glm::dot(corners[i] - support, direction));
            }
        }
        const f32 overhang = leanDistance - reach;
        if (overhang <= 0.0f)
        {
            return Vec3(0.0f);
        }

        // Past the edge: the edge is the pivot, the overhang is the lever.
        const Vec3 weight = -localUp * static_cast<f32>(massKg * gravity);
        const Vec3 torque = glm::cross(direction * overhang, weight);

        // ...AND THE INERTIA IS ABOUT THE PIVOT, NOT ABOUT THE CENTRE OF
        // MASS. `inertia` is the body's own tensor about its own centre,
        // which is the right number for a body turning in free flight and
        // the wrong one for a body hinged on the edge of its foot. The body
        // is not spinning in place: every gram of it is swinging on an arm
        // that reaches from the ground edge up to wherever it sits, and the
        // resistance to that is Huygens-Steiner, I_pivot = I_com + m d^2,
        // with d the distance from the centre of mass to the pivot EDGE —
        // the overhang across the ground and the height above it.
        //
        // Leaving it out is not a small error, because d is the size of the
        // vehicle and the vehicle is mostly tall. MEASURED on the 12 m test
        // rocket (2.4 m across, 4 t, centre of mass at its middle) leaning
        // 20 deg: it stands 6.048580 m above the ground and hangs 0.924490 m
        // outside the tail edge, so the arm to that edge is 6.118824 m and
        // m*d^2 is 1.4976e5 kg m^2 against an I_com of 4.992e4 — the true
        // resistance is 4.00x what was being used, and it is 4.00x across
        // the whole 12-30 deg range. The acceleration at 20 deg is 0.181675
        // rad/s^2 where it used to be 0.726702, and through the ground system
        // itself a rocket left leaning 12 deg now takes 14.98 s to end up on
        // its side instead of 5.46 s. It used to slam over.
        const f32 height = std::max(0.0f, glm::dot(centreOfMass, localUp) - lowest);
        const f32 pivotArmSq = overhang * overhang + height * height;
        const f32 huygens = static_cast<f32>(massKg) * pivotArmSq;
        return Vec3(torque.x / (std::max(inertia.x, 1.0f) + huygens),
                    torque.y / (std::max(inertia.y, 1.0f) + huygens),
                    torque.z / (std::max(inertia.z, 1.0f) + huygens));
    }

    /// v2 (hierarchical systems): the orbit is PRIMARY-RELATIVE and the
    /// component remembers which body it orbits. The RailsSystem adds the
    /// primary's current world position — so a station riding rails around
    /// Terra follows Terra around the Sun for free.
    struct OnRailsComponent
    {
        KeplerOrbit orbit{};   // primary-relative elements
        ecs::Entity primary{}; // gravity source orbited (null = world origin)
        /// DynamicBody payload preserved across regime conversions: when the
        /// bubble railifies a ship (time warp!) and later releases it, the
        /// ship must come back with ITS mass, not a default.
        f64 dynamicMass = 1000.0;
        f64 dynamicBallisticFactor = 0.002;
    };

    struct GravitySourceComponent
    {
        f64 mu = 0.0;         // GM, m^3/s^2
        f64 bodyRadius = 0.0; // meters — solid surface + altitude rules
        /// Sphere-of-influence radius: inside it, THIS body is the primary
        /// for rails conversions and trajectory patching. The default is
        /// effectively infinite (a lone body owns all of space); real
        /// systems set r_SOI = a * (mu/mu_parent)^(2/5).
        f64 soiRadius = 1.0e300;
        /// Current world-frame velocity of the body, stamped each Physics
        /// tick by whatever moves it (CelestialMotionSystem). Everything
        /// body-relative — atmosphere drag, ground friction, orbit
        /// conversions — measures velocities against this, so physics stays
        /// correct while the planet itself races around its star.
        WorldVec3 worldVelocity{0.0};
        /// Spin of the body: axis * rate (rad/s, world frame). The SURFACE
        /// (and the atmosphere) move at worldVelocity + angularVelocity x r:
        /// ground friction and EVA walking act in that ROTATING frame, so a
        /// landed ship co-rotates with its planet instead of having the
        /// ground slide away underneath it at hundreds of m/s.
        WorldVec3 angularVelocity{0.0};

        // ---- the body's attitude, in f64 ---------------------------------
        // TransformComponent::rotation is an f32 quaternion. That is right
        // for ORIENTING a mesh and ruinous for POSITIONING anything 6,371 km
        // from the axis: f32 carries ~7 digits, so that lever arm is
        // quantised to about 1.2 m — and the error CHANGES as the planet
        // turns, by up to 0.77 m from one frame to the next. Everything
        // anchored to the ground therefore shimmered, and so did the terrain
        // patch, while the camera (a f64 world position) held still.
        //
        // So the spin is kept here at full precision and stamped every tick
        // by whatever turns the body. Position math uses THIS; orientation
        // math can go on using the f32 quaternion, where a 1e-7 rad error is
        // a fraction of a pixel.
        WorldVec3 spinAxis{0.0, 1.0, 0.0};
        f64 spinAngle = 0.0;         // radians at the lane's present time
        f64 spinAnglePrevious = 0.0; // one tick earlier: the render's partner
    };

    /// The body's rotation at a given spin angle, as an exact f64 rotation.
    [[nodiscard]] inline glm::dquat spinRotation(const GravitySourceComponent& source,
                                                 f64 angle)
    {
        const f64 length = glm::length(source.spinAxis);
        if (length < 1.0e-12)
        {
            return glm::dquat{1.0, 0.0, 0.0, 0.0};
        }
        return glm::angleAxis(angle, source.spinAxis / length);
    }

    [[nodiscard]] inline glm::dquat spinRotation(const GravitySourceComponent& source)
    {
        return spinRotation(source, source.spinAngle);
    }

    /// The body's rotation partway through the current tick, for rendering.
    /// The angle is kept wrapped to [0, 2pi), so the two ends of a tick can
    /// straddle the wrap — interpolating them naively would sweep the planet
    /// the long way round in one frame. Take the SHORT arc, exactly as the
    /// f32 slerp beside it does.
    [[nodiscard]] inline glm::dquat spinRotationAt(const GravitySourceComponent& source,
                                                   f64 alpha)
    {
        constexpr f64 kTwoPi = 6.283185307179586;
        f64 delta = source.spinAngle - source.spinAnglePrevious;
        if (delta > kTwoPi * 0.5)
        {
            delta -= kTwoPi;
        }
        else if (delta < -kTwoPi * 0.5)
        {
            delta += kTwoPi;
        }
        return spinRotation(source, source.spinAnglePrevious + delta * alpha);
    }

    /// Rigidly attaches an entity to a celestial body's SURFACE: the local
    /// position is expressed in the body's rotating frame, so anchored
    /// structures (mines, factories, launch pads) co-rotate with the planet
    /// or asteroid they are built on. This — not absolute coordinates — is
    /// what makes surface bases save/load-proof: the body may have rotated
    /// arbitrarily far while the game was closed, the base stays exactly
    /// where it was built.
    /// IS THE ANALYTIC WORLD A GOOD ENOUGH STAND-IN FOR THE REAL ONE HERE?
    ///
    /// Time warp past the physics rate switches the integrator off and puts
    /// everything on rails. That is exact when the motion already IS a conic
    /// clear of the air, and it is a fabrication otherwise: a suborbital arc,
    /// a reentry or an escape trajectory all have their outcome decided by
    /// the integration the warp would skip, and warping one hands the player
    /// back a vehicle somewhere it could never have reached.
    ///
    /// Two situations qualify. Standing on something — the craft is not going
    /// anywhere and the ground holds it up. Or a CLOSED orbit whose lowest
    /// point clears the atmosphere: a periapsis inside the air means decay,
    /// and decay is precisely what rails cannot express.
    ///
    /// Altitude alone was the old test and it is not the same question: it
    /// happily permitted ten thousand times real time on a trajectory whose
    /// next event was the ground.
    /// How long a jump stays off the ground, with margin — the window over
    /// which a walker should still count as standing on the planet.
    ///
    /// Ballistic: up and back down is 2v/g. The half again on top covers
    /// landing on ground higher than the take-off point, and the drag a
    /// suit in air actually feels.
    ///
    /// IT IS COMPUTED AND NOT WRITTEN DOWN, because a constant is wrong
    /// nearly everywhere. The same legs and the same key give 0.92 s of air
    /// on Terra, 2.4 s on Mars and 5.6 s on Luna; a window tuned on Terra
    /// would leave a Luna walker "airborne" for most of every hop.
    [[nodiscard]] inline f64 jumpHangSeconds(f64 jumpSpeed, f64 surfaceGravity)
    {
        if (surfaceGravity <= 1.0e-6 || jumpSpeed <= 0.0)
        {
            return 1.0;
        }
        return std::clamp(2.0 * jumpSpeed / surfaceGravity * 1.5, 1.0, 60.0);
    }

    [[nodiscard]] inline bool warpPermitted(bool grounded, bool closedOrbit,
                                            f64 periapsisAltitude, f64 atmosphereTopAltitude)
    {
        return grounded || (closedOrbit && periapsisAltitude > atmosphereTopAltitude);
    }

    struct SurfaceAnchorComponent
    {
        ecs::Entity body{};           // the gravity source this is built on
        WorldVec3 localPosition{0.0}; // body-fixed frame, meters
        /// Orientation in the body-fixed frame (identity = align with the
        /// body, the historical behavior for built structures).
        Quat localRotation{1.0f, 0.0f, 0.0f, 0.0f};
        /// Payload preserved for auto-released anchors (see below).
        f64 dynamicMass = 0.0;
        f64 dynamicBallisticFactor = 0.0;
        /// 1 = this anchor was created by the bubble system for a LANDED
        /// dynamic craft (rails would fling it: a ground state converts to
        /// a degenerate ellipse). Released back to dynamic when the bubble
        /// focus returns. Hand-built structures use 0 and stay anchored.
        u8 autoRelease = 0;
    };

    /// Optional exponential atmosphere around a gravity source.
    struct AtmosphereComponent
    {
        f64 surfaceDensity = 1.225; // kg/m^3 at altitude 0 (Earth-like)
        f64 scaleHeight = 8500.0;   // meters (density /e per scale height)
        f64 topAltitude = 1.4e5;    // no drag above this altitude
    };

    /// A JUMP, taken from a moving surface.
    ///
    /// It is a function, in the engine, for one reason: it is THE CARRIER-
    /// VELOCITY RULE in three lines, and every time that rule has been
    /// written out by hand here it has cost a bug. A body standing on Terra
    /// is already doing 30 km/s around the Sun and 465 m/s with the spin; a
    /// jump changes ONE component of its velocity — the radial one, measured
    /// against the ground underneath it — and must leave the rest exactly
    /// where it was. Set the radial component rather than adding to it, so
    /// jumping while already rising cannot stack.
    ///
    /// `up` must be a unit vector. Returns the new WORLD velocity.
    [[nodiscard]] inline WorldVec3 surfaceJumpVelocity(const WorldVec3& worldVelocity,
                                                       const WorldVec3& surfaceVelocity,
                                                       const WorldVec3& up, f64 jumpSpeed)
    {
        const WorldVec3 relative = worldVelocity - surfaceVelocity;
        const f64 radialSpeed = glm::dot(relative, up);
        return surfaceVelocity + (relative - up * radialSpeed) + up * jumpSpeed;
    }

    static_assert(std::is_trivially_copyable_v<DynamicBodyComponent>);
    static_assert(std::is_trivially_copyable_v<OnRailsComponent>);
    static_assert(std::is_trivially_copyable_v<GravitySourceComponent>);
    static_assert(std::is_trivially_copyable_v<AtmosphereComponent>);
    static_assert(std::is_trivially_copyable_v<SurfaceAnchorComponent>);
} // namespace sw::phys
