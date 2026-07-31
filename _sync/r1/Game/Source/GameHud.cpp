// ============================================================================
// GameHud.cpp — The HUD: panels, text, navball, buttons and their click handling.
// Split out of StarWorksGame.cpp; same class, one theme per translation unit.
// ============================================================================

#include "StarWorksGame.hpp"

#include "GameInternal.hpp"
#include "Systems.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <format>
#include <limits>

namespace game
{

    void StarWorksGame::hudQuad(sw::f32 x0, sw::f32 y0, sw::f32 x1, sw::f32 y1,
                                const sw::Vec4& color)
    {
        sw::DrawItem item{};
        item.mesh = &m_meshes[m_navLineMeshIndex]; // unit quad
        item.transform =
            glm::translate(sw::Mat4{1.0f}, {(x0 + x1) * 0.5f, (y0 + y1) * 0.5f, 0.0f}) *
            glm::scale(sw::Mat4{1.0f}, {(x1 - x0) * 0.5f, (y1 - y0) * 0.5f, 1.0f});
        item.screenSpace = true;
        item.tint = color;
        m_drawItems.push_back(item);
    }

    void StarWorksGame::hudPanel(sw::f32 x0, sw::f32 y0, sw::f32 x1, sw::f32 y1,
                                 const sw::Vec4& fill)
    {
        constexpr sw::f32 kEdge = 0.004f;
        hudQuad(x0 - kEdge, y0 - kEdge, x1 + kEdge, y1 + kEdge, hud::kEdge);
        hudQuad(x0, y0, x1, y1, fill);
    }

    bool StarWorksGame::hudCursor(sw::f32& outX, sw::f32& outY)
    {
        sw::u32 width = 0;
        sw::u32 height = 0;
        window().framebufferSize(width, height);
        if (width == 0 || height == 0)
        {
            return false;
        }
        outX = input().mouseX() / static_cast<sw::f32>(width) * 2.0f - 1.0f;
        outY = input().mouseY() / static_cast<sw::f32>(height) * 2.0f - 1.0f;
        return true;
    }

    void StarWorksGame::hudDesignPreview(const sw::parts::ShipBlueprint& design,
                                         sw::f32 x0, sw::f32 y0, sw::f32 x1, sw::f32 y1,
                                         sw::f32 spinRadians)
    {
        if (design.parts.empty())
        {
            return;
        }

        // ---- 1. how big is it, and where is its middle --------------------
        sw::Vec3 low{1.0e9f};
        sw::Vec3 high{-1.0e9f};
        for (const sw::parts::BlueprintPartRecord& part : design.parts)
        {
            const auto* definition = sw::parts::findDefinition(part.definitionId);
            if (definition == nullptr)
            {
                continue;
            }
            const sw::f32 reach = sw::parts::partBoundsRadius(*definition);
            low = glm::min(low, part.localPosition - sw::Vec3{reach});
            high = glm::max(high, part.localPosition + sw::Vec3{reach});
        }
        if (low.x > high.x)
        {
            return; // nothing in the catalogue matched a part we have
        }
        const sw::Vec3 centre = (low + high) * 0.5f;
        const sw::Vec3 half = glm::max((high - low) * 0.5f, sw::Vec3{0.05f});

        // ---- 2. the view: proper rotations only ---------------------------
        // Stood upright by the same rotation the hangar uses, turned about
        // that vertical by the caller's angle, and tipped a little so the
        // thing reads as a solid rather than a silhouette.
        // The hangar's own display rotation: +90 deg about X puts the nose
        // (-Z) up. Written out rather than shared because the constant lives
        // in the hangar's translation unit section, further down this file.
        constexpr sw::f32 kPitch = 0.22f;
        const sw::Quat standUpright = glm::angleAxis(1.5707963f, sw::Vec3{1, 0, 0});
        const sw::Quat view = glm::angleAxis(-kPitch, sw::Vec3{1.0f, 0.0f, 0.0f}) *
                              glm::angleAxis(spinRadians, sw::Vec3{0.0f, 1.0f, 0.0f}) *
                              standUpright;

        // ---- 3. fit it to the rectangle -----------------------------------
        // NDC x is compressed by the aspect ratio and NDC y is not, so the
        // width available in "square" units is the half-width TIMES aspect.
        //
        // FIT THE SHAPE, NOT ITS BOUNDING SPHERE. A rocket is long and thin,
        // and a sphere around it is as wide as it is tall — framing by the
        // sphere throws away a third of the height for a width nothing ever
        // occupies. So: the vessel's own axis (+Z, stood upright, so it runs
        // up the screen) sets the vertical extent, its radial size sets the
        // horizontal one, and the pitch mixes a little of each into the
        // other. Worst case over a whole turn, so the model does not pulse
        // as it spins.
        const sw::f32 radial = std::max(half.x, half.y);
        const sw::f32 needHeight =
            half.z * std::cos(kPitch) + radial * std::sin(kPitch);
        const sw::f32 needWidth = radial;

        const sw::f32 aspect = renderer().aspectRatio();
        const sw::f32 halfWidth = std::abs(x1 - x0) * 0.5f;
        const sw::f32 halfHeight = std::abs(y1 - y0) * 0.5f;
        const sw::f32 scale = std::min(halfWidth * aspect / needWidth,
                                       halfHeight / needHeight) *
                              0.92f;
        const sw::Vec3 middle{(x0 + x1) * 0.5f, (y0 + y1) * 0.5f, 0.0f};

        // THE Y IS NEGATED, ONCE. The camera does the same thing in its
        // projection and the front-face convention was settled against it;
        // a preview that skipped the flip would be culled inside out.
        const sw::Mat4 frame =
            glm::translate(sw::Mat4{1.0f}, middle) *
            glm::scale(sw::Mat4{1.0f}, sw::Vec3{scale / aspect, -scale, scale}) *
            glm::mat4_cast(view);

        // ---- 4. back to front ---------------------------------------------
        // The view looks toward -Z of its own space, exactly as the camera
        // does, so the most negative depth is the farthest away.
        struct Ordered
        {
            sw::f32 depth;
            const sw::parts::BlueprintPartRecord* part;
        };
        std::vector<Ordered> ordered;
        ordered.reserve(design.parts.size());
        for (const sw::parts::BlueprintPartRecord& part : design.parts)
        {
            ordered.push_back({(view * (part.localPosition - centre)).z, &part});
        }
        std::sort(ordered.begin(), ordered.end(),
                  [](const Ordered& a, const Ordered& b) { return a.depth < b.depth; });

        // ---- 5. submit -----------------------------------------------------
        for (const Ordered& entry : ordered)
        {
            const auto meshIt = m_partMeshIds.find(entry.part->definitionId);
            if (meshIt == m_partMeshIds.end())
            {
                continue;
            }
            sw::DrawItem item{};
            item.mesh = &m_meshes[meshIt->second];
            item.transform =
                frame *
                glm::translate(sw::Mat4{1.0f}, entry.part->localPosition - centre) *
                glm::mat4_cast(entry.part->localRotation);
            item.screenSpace = true;
            item.hudSolid = true;
            item.hudLayer = static_cast<sw::u8>(sw::ui::HudLayer::Background);
            m_drawItems.push_back(item);
        }
    }

    void StarWorksGame::hudText(std::string_view text, sw::f32 x, sw::f32 y,
                                sw::f32 heightNdc, const sw::Vec4& color)
    {
        const sw::f32 aspect = renderer().aspectRatio();
        const sw::f32 scaleX = (5.0f / 7.0f) * heightNdc / aspect;
        const sw::f32 advance = sw::ui::kGlyphAdvance * heightNdc / aspect;

        sw::f32 penX = x;
        for (const char character : text)
        {
            const auto index = static_cast<sw::usize>(
                static_cast<unsigned char>(std::toupper(character)));
            const sw::u32 meshIndex =
                (index < m_glyphMeshIndex.size()) ? m_glyphMeshIndex[index] : 0xFFFFFFFFu;
            if (meshIndex != 0xFFFFFFFFu)
            {
                sw::DrawItem item{};
                item.mesh = &m_meshes[meshIndex];
                item.transform = glm::translate(sw::Mat4{1.0f}, {penX, y, 0.0f}) *
                                 glm::scale(sw::Mat4{1.0f}, {scaleX, heightNdc, 1.0f});
                item.screenSpace = true;
                // TEXT IS A LAYER, not a submission order to get right: a
                // glyph is never painted over by a panel, whatever else the
                // frame decided to draw. See UI/HudOrder.hpp.
                item.hudLayer = static_cast<sw::u8>(sw::ui::HudLayer::Text);
                item.tint = color;
                m_drawItems.push_back(item);
            }
            penX += advance;
        }
    }

    // ------------------------------------------------------------------------
    // Navigation beacons
    //
    // A surveyed site sits somewhere on 510 million square kilometres of
    // procedural ground, and from 30 km up one valley looks like the next.
    // A beacon fixes that: it draws a reticle at its own position with its
    // name and the LIVE distance under it — on the map always, and in the
    // cockpit once you are inside the beacon's declared range.
    //
    // The distance is measured from the CRAFT YOU CONTROL, not from the
    // camera: in free-cam or on the map the camera can be parked anywhere,
    // and "how far am I" has to mean the pilot, not the viewpoint.
    //
    // A beacon behind you or off the edge of the screen would be useless
    // exactly when you need it most (you are looking for it BECAUSE you
    // cannot see it), so an off-screen beacon is clamped to the border and
    // its reticle turns into an arrow pointing off that edge.
    // ------------------------------------------------------------------------
    void StarWorksGame::collectBeacons(const sw::Camera& activeCamera, bool mapView)
    {
        const sw::WorldVec3 cameraPosition = activeCamera.position();
        const sw::Mat4 viewProjection = activeCamera.viewProjectionCameraRelative();
        const sw::f32 aspect = renderer().aspectRatio();

        sw::WorldVec3 playerPosition = cameraPosition;
        if (const auto* controlled =
                m_world.tryGetComponent<TransformComponent>(controlledEntity()))
        {
            playerPosition = controlled->position;
        }

        auto centredText = [&](std::string_view text, sw::f32 centreX, sw::f32 y,
                               sw::f32 height, const sw::Vec4& color) {
            const sw::f32 advance = sw::ui::kGlyphAdvance * height / aspect;
            const sw::f32 halfWidth =
                advance * static_cast<sw::f32>(text.size()) * 0.5f;
            // A marker near the border would hang its label off the screen;
            // slide the text back on rather than truncating it.
            const sw::f32 clamped =
                glm::clamp(centreX, -0.98f + halfWidth, 0.98f - halfWidth);
            hudText(text, clamped - halfWidth, y, height, color);
        };
        auto bar = [&](sw::f32 x0, sw::f32 y0, sw::f32 x1, sw::f32 y1,
                       const sw::Vec4& color) {
            sw::DrawItem item{};
            item.mesh = &m_meshes[m_navLineMeshIndex]; // unit quad
            item.transform =
                glm::translate(sw::Mat4{1.0f},
                               {(x0 + x1) * 0.5f, (y0 + y1) * 0.5f, 0.0f}) *
                glm::scale(sw::Mat4{1.0f}, {(x1 - x0) * 0.5f, (y1 - y0) * 0.5f, 1.0f});
            item.screenSpace = true;
            item.tint = color;
            m_drawItems.push_back(item);
        };

        m_world.forEach<TransformComponent, sw::factory::BeaconComponent>(
            [&](sw::ecs::Entity, TransformComponent& transform,
                sw::factory::BeaconComponent& beacon) {
                const sw::f64 distance = glm::length(transform.position - playerPosition);
                if (!mapView &&
                    (distance > beacon.rangeM || distance < beacon.nearRangeM))
                {
                    // Too far to be lit for you, or so close that the pointer
                    // would just be covering the thing it points at.
                    return;
                }

                const sw::Vec3 relative = sw::Vec3(transform.position - cameraPosition);
                // Where the pointer goes — including the behind-you and
                // past-the-edge cases — is one tested engine function
                // (UI/ScreenMarker.hpp), not arithmetic repeated per HUD.
                const sw::ui::MarkerPlacement placement = sw::ui::placeScreenMarker(
                    viewProjection, relative, activeCamera.right(), activeCamera.up(),
                    activeCamera.forward());
                const sw::Vec2 ndc = placement.ndc;
                const bool offScreen = placement.offScreen;

                const sw::Vec4 color = offScreen ? sw::Vec4{1.0f, 0.62f, 0.20f, 0.85f}
                                                 : sw::Vec4{1.0f, 0.78f, 0.28f, 0.95f};

                // ---- the reticle: an open square, four thin bars ---------
                const sw::f32 half = (offScreen ? 0.020f : 0.028f);
                const sw::f32 halfX = half / aspect;
                constexpr sw::f32 kThick = 0.006f;
                const sw::f32 thickX = kThick / aspect;
                const sw::f32 arm = half * 0.55f;
                const sw::f32 armX = arm / aspect;
                // Corners only (an open reticle does not hide the thing it
                // is pointing at).
                for (const sw::f32 sx : {-1.0f, 1.0f})
                {
                    for (const sw::f32 sy : {-1.0f, 1.0f})
                    {
                        const sw::f32 cx = ndc.x + sx * halfX;
                        const sw::f32 cy = ndc.y + sy * half;
                        bar(cx - (sx < 0.0f ? 0.0f : armX), cy - kThick * 0.5f,
                            cx + (sx < 0.0f ? armX : 0.0f), cy + kThick * 0.5f, color);
                        bar(cx - thickX * 0.5f, cy - (sy < 0.0f ? 0.0f : arm),
                            cx + thickX * 0.5f, cy + (sy < 0.0f ? arm : 0.0f), color);
                    }
                }

                // ---- name, then the distance UNDER it --------------------
                // Glyphs are anchored at their TOP and grow downward, so
                // `textY` is the top of the block. Near the bottom border
                // there is no room under the reticle and the block flips
                // above it rather than falling off the screen.
                const sw::f32 textHeight = offScreen ? 0.030f : 0.036f;
                const bool showLabel = !offScreen;
                const sw::f32 lineStep = textHeight * 1.35f;
                const sw::f32 blockHeight = showLabel ? lineStep + textHeight : textHeight;
                sw::f32 textY = ndc.y + half + 0.016f;
                if (textY + blockHeight > 0.97f)
                {
                    textY = ndc.y - half - 0.016f - blockHeight;
                }
                const std::string_view label =
                    (beacon.label[0] != '\0') ? std::string_view{beacon.label}
                                              : std::string_view{"BEACON"};
                if (showLabel)
                {
                    centredText(label, ndc.x, textY, textHeight, color);
                    textY += lineStep;
                }
                const std::string distanceText =
                    (distance >= 10000.0)
                        ? std::format("{:.1f} KM", distance / 1000.0)
                        : ((distance >= 1000.0)
                               ? std::format("{:.2f} KM", distance / 1000.0)
                               : std::format("{:.0f} M", distance));
                centredText(distanceText, ndc.x, textY, textHeight,
                            {color.r, color.g, color.b, color.a * 0.9f});
            });
    }

    void StarWorksGame::collectHud()
    {
        constexpr sw::f32 kLine = 0.052f;
        constexpr sw::f32 kX = -0.98f;
        sw::f32 y = -0.97f;
        const sw::Vec4 main{0.65f, 0.95f, 0.75f, 0.95f};
        const sw::Vec4 dim{0.55f, 0.75f, 0.85f, 0.9f};

        // ---- mode ----------------------------------------------------------------
        const char* mode = m_mapView ? "MAP" : (m_evaMode ? "EVA" : "NAV");
        hudText(std::format("{} {}", mode, m_shipMode || m_mapView ? "" : "CAM LIBRE"), kX,
                y, kLine, main);
        y += kLine * 1.3f;

        // ---- speed & altitude, relative to the current SOI PRIMARY ---------------
        const sw::WorldVec3 velocity = controlledVelocity();
        const sw::WorldVec3 position =
            m_world.getComponent<TransformComponent>(controlledEntity()).position;

        const sw::f64 time = m_physicsLane->presentSeconds();
        const sw::i32 primaryIndex = controlledPrimaryIndex();
        const char* primaryName = "-";
        sw::WorldVec3 primaryPosition{0.0};
        sw::WorldVec3 primaryVelocity{0.0};
        sw::WorldVec3 primaryAngularVelocity{0.0};
        sw::f64 primaryRadius = 0.0;
        if (primaryIndex >= 0)
        {
            const auto& primary = m_celestialIndex.body(static_cast<sw::usize>(primaryIndex));
            primaryName = primary.name;
            primaryRadius = primary.bodyRadius;
            m_celestialIndex.stateAt(primaryIndex, time, primaryPosition,
                                     &primaryVelocity);
            if (const auto* source =
                    m_world.tryGetComponent<sw::phys::GravitySourceComponent>(
                        primary.entity))
            {
                primaryAngularVelocity = source->angularVelocity;
            }
        }

        sw::f64 speed = 0.0;
        if (m_speedSurfaceRelative)
        {
            // Surface velocity: the primary's own motion + its spin at this
            // position — works above any body, not just Terra.
            const sw::WorldVec3 surfaceVelocity =
                primaryVelocity +
                glm::cross(primaryAngularVelocity, position - primaryPosition);
            speed = glm::length(velocity - surfaceVelocity);
        }
        else
        {
            speed = glm::length(velocity - primaryVelocity); // orbital speed
        }
        hudText(std::format("SPD {} {:.1f} M/S", m_speedSurfaceRelative ? "SRF" : "ORB",
                            speed),
                kX, y, kLine, main);
        y += kLine * 1.3f;

        // ---- altitude above the primary -------------------------------------------
        const sw::f64 altitude =
            glm::length(position - primaryPosition) - primaryRadius;
        hudText(std::format("ALT {} {:.1f} KM", primaryName, altitude / 1000.0), kX, y,
                kLine, main);
        y += kLine * 1.3f;

        // ---- the air, while there is any -----------------------------------------
        //
        // Three numbers and a verdict. Dynamic pressure is what the airframe
        // feels and what a gravity turn is flown around; Mach is where the
        // drag lives; and the STABILITY MARGIN — how far the centre of
        // pressure sits behind the centre of mass, in vehicle diameters — is
        // the one number that says whether this rocket will fly straight or
        // swap ends. Positive is stable. It is shown because it is the thing
        // the player can actually fix, by moving fins or moving mass.
        const sw::ecs::Entity flown = controlledEntity();
        const auto* vesselFlown = m_world.tryGetComponent<sw::parts::VesselComponent>(flown);
        if (const auto* air = m_world.tryGetComponent<sw::aero::AeroStateComponent>(flown);
            air != nullptr && vesselFlown != nullptr && air->inAtmosphere != 0)
        {
            const sw::Vec4 warn{1.0f, 0.55f, 0.2f, 1.0f};
            const sw::f64 pressureKpa = air->dynamicPressurePa / 1000.0;
            hudText(std::format("Q {:.0f} KPA  M {:.2f}  AOA {:.0f}", pressureKpa,
                                air->machNumber,
                                air->angleOfAttackRad * 180.0 / 3.14159265358979),
                    kX, y, kLine, (pressureKpa > 45.0) ? warn : main);
            y += kLine * 1.15f;

            // +Z is the tail, so a pressure centre BEHIND the balance point
            // has the larger z. The margin is quoted in calibres — vehicle
            // widths — because that is the form the number is meaningful in:
            // one calibre of margin flies, a tenth of one is a coin toss.
            const sw::f32 calibre =
                std::max(0.5f, 2.0f * std::max(vesselFlown->halfExtents.x, 0.25f));
            const sw::f32 margin =
                (air->centreOfPressure.z - air->centreOfMass.z) / calibre;
            hudText(std::format("DRAG {:.0f} KN  MGN {:+.2f}{}", air->dragN / 1000.0,
                                margin, (margin > 0.05f) ? "" : " UNSTABLE"),
                    kX, y, kLine, (margin > 0.05f) ? main : warn);
            y += kLine * 1.3f;
        }

        // ---- current orbit around the primary: APO / PER / period ----------------
        auto formatEta = [](sw::f64 seconds) {
            return (seconds >= 3600.0) ? std::format("{:.1f} H", seconds / 3600.0)
                                       : std::format("{:.0f} S", seconds);
        };
        // A closest approach can be eight hundred metres or eight hundred
        // million kilometres, and both have to be readable at a glance.
        auto formatDistance = [](sw::f64 metres) {
            if (metres >= 1.0e9)
            {
                return std::format("{:.3f} GM", metres / 1.0e9);
            }
            if (metres >= 1000.0)
            {
                return std::format("{:.1f} KM", metres / 1000.0);
            }
            return std::format("{:.0f} M", metres);
        };
        const auto* controlledBody =
            m_world.tryGetComponent<sw::phys::DynamicBodyComponent>(controlledEntity());
        const bool grounded = controlledBody != nullptr && controlledBody->isGrounded != 0;
        sw::phys::KeplerOrbit currentOrbit{};
        if (primaryIndex >= 0 && !grounded &&
            sw::phys::kepler::fromStateVectors(
                m_celestialIndex.body(static_cast<sw::usize>(primaryIndex)).mu,
                position - primaryPosition, velocity - primaryVelocity, time,
                currentOrbit, /*allowHyperbolic=*/true))
        {
            constexpr sw::f64 kTwoPi = 6.283185307179586;
            const sw::f64 periapsisAltitude =
                sw::phys::kepler::periapsis(currentOrbit) - primaryRadius;
            if (!currentOrbit.isHyperbolic())
            {
                const sw::f64 meanAnomaly = currentOrbit.meanAnomalyAtEpoch;
                const sw::f64 timeToPeriapsis =
                    (kTwoPi - meanAnomaly) / currentOrbit.meanMotion;
                const sw::f64 timeToApoapsis =
                    std::fmod(3.0 * 3.14159265358979 - meanAnomaly, kTwoPi) /
                    currentOrbit.meanMotion;
                const sw::f64 apoapsisAltitude =
                    sw::phys::kepler::apoapsis(currentOrbit) - primaryRadius;
                hudText(std::format("APO {:.1f} KM T-{}", apoapsisAltitude / 1000.0,
                                    formatEta(timeToApoapsis)),
                        kX, y, kLine, dim);
                y += kLine * 1.3f;
                hudText(std::format("PER {:.1f} KM T-{}", periapsisAltitude / 1000.0,
                                    formatEta(timeToPeriapsis)),
                        kX, y, kLine, dim);
                y += kLine * 1.3f;
                hudText(std::format("ORB {}", formatEta(sw::phys::kepler::period(
                                        currentOrbit))),
                        kX, y, kLine, dim);
                y += kLine * 1.3f;
            }
            else // escape trajectory: periapsis (if ahead), no apoapsis
            {
                const sw::f64 meanAnomaly = currentOrbit.meanAnomalyAtEpoch;
                if (meanAnomaly < 0.0) // still inbound toward periapsis
                {
                    hudText(std::format("PER {:.1f} KM T-{}  ESC",
                                        periapsisAltitude / 1000.0,
                                        formatEta(-meanAnomaly /
                                                  currentOrbit.meanMotion)),
                            kX, y, kLine, dim);
                }
                else
                {
                    hudText("ESCAPE TRAJECTORY", kX, y, kLine, dim);
                }
                y += kLine * 1.3f;
            }
        }

        // ---- throttle + warp -----------------------------------------------------------
        // On foot there is no throttle, and there may be no vessel at all.
        const auto* ship = m_shipEntity.isNull()
                               ? nullptr
                               : m_world.tryGetComponent<ShipComponent>(m_shipEntity);
        const std::string warpLabel = m_simulation.isPaused()
                                          ? std::string("0")
                                          : warpText(kWarpLadder[m_warpIndex]);
        hudText((ship != nullptr && !m_evaMode)
                    ? std::format("THR {:.0f}%  WARP X{}", ship->throttle * 100.0f,
                                  warpLabel)
                    : std::format("WARP X{}", warpLabel),
                kX, y, kLine, dim);
        y += kLine * 1.3f;
        // STANDING ON SOMETHING: how far over it is leaning, and whether
        // the ground is still holding it. A rocket resting at eight degrees
        // is not a broken rocket — a body inside its own support polygon
        // does not stand itself back up — but there was no way to tell that
        // from a rocket that had stopped being simulated.
        if (m_flight.grounded)
        {
            hudText(std::format("LANDED  LEAN {:.0f} DEG  {}", m_flight.leanDegrees,
                                m_flight.tipping ? "TIPPING" : "RESTING"),
                    kX, y, kLine * 0.85f,
                    m_flight.tipping ? sw::Vec4{1.0f, 0.65f, 0.25f, 1.0f} : dim);
            y += kLine * 1.15f;
        }
        // WHY THE WARP KEY JUST DID NOTHING — shown for four seconds after
        // the gate actually refused something, and not one frame otherwise.
        //
        // It used to be drawn from `!warpAllowed()`, a standing condition:
        // true for every second of every ascent, every reentry and — because
        // `isGrounded` drops the moment the feet leave the dirt — every jump.
        // A warning that is on almost always is not a warning, it is
        // furniture, and the player reasonably read it as the thing that had
        // taken their controls away.
        if (clock().totalSeconds() < m_warpRefusedUntil && !m_warpRefusedReason.empty())
        {
            hudText(std::format("WARP LOCKED  {}", m_warpRefusedReason), kX, y,
                    kLine * 0.8f, sw::Vec4{0.95f, 0.45f, 0.35f, 1.0f});
            y += kLine * 1.1f;
        }

        // ---- vessel resources (parts carry them as real cargo) -----------------
        sw::f64 fuelUnits = 0.0;
        sw::f64 chargeUnits = 0.0;
        m_world.forEach<sw::parts::PartComponent, sw::factory::InventoryComponent>(
            [&](sw::ecs::Entity, sw::parts::PartComponent& part,
                sw::factory::InventoryComponent& inventory) {
                if (part.vessel != m_shipEntity)
                {
                    return;
                }
                fuelUnits +=
                    sw::factory::inventoryCount(inventory, sw::res::Resource::Fuel);
                chargeUnits += sw::factory::inventoryCount(
                    inventory, sw::res::Resource::ElectricCharge);
            });
        const sw::Vec4 fuelColor = (fuelUnits > 3000.0)
                                       ? dim
                                       : sw::Vec4{1.0f, 0.45f, 0.3f, 0.95f};
        hudText(std::format("FUEL {:.0f} KG  ELEC {:.0f} KJ", fuelUnits, chargeUnits),
                kX, y, kLine, fuelColor);
        y += kLine * 1.3f;

        // ---- flight-plan events (KSP style): the first upcoming transition --------
        for (const sw::space::TrajectorySegment& segment : m_prediction)
        {
            const char* label = nullptr;
            sw::Vec4 color = dim;
            switch (segment.endReason)
            {
            case sw::space::SegmentEnd::Encounter:
                label = "ENC";
                color = {0.4f, 1.0f, 0.9f, 0.95f};
                break;
            case sw::space::SegmentEnd::Impact:
                label = "IMPACT";
                color = {1.0f, 0.35f, 0.3f, 0.95f};
                break;
            case sw::space::SegmentEnd::SoiExit:
                label = "EXIT TO";
                color = {1.0f, 0.85f, 0.4f, 0.95f};
                break;
            default:
                break;
            }
            if (label == nullptr || segment.eventBodyIndex < 0)
            {
                continue;
            }
            const auto& eventBody =
                m_celestialIndex.body(static_cast<sw::usize>(segment.eventBodyIndex));
            const sw::f64 eta = segment.endTime - time;
            hudText(std::format("{} {} T-{:.0f} S", label, eventBody.name,
                                std::max(eta, 0.0)),
                    kX, y, kLine, color);
            y += kLine * 1.3f;
        }

        // ---- the target, and the closest approach to it -----------------------
        if (m_targetIndex >= 0 &&
            static_cast<sw::usize>(m_targetIndex) < m_celestialIndex.size())
        {
            const auto& target =
                m_celestialIndex.body(static_cast<sw::usize>(m_targetIndex));
            constexpr sw::Vec4 kTargetColor{1.0f, 0.55f, 0.88f, 0.95f};
            const sw::f64 distanceNow =
                glm::length(m_celestialIndex.positionAt(m_targetIndex, time) - position);
            hudText(std::format("TGT {}  {}", target.name, formatDistance(distanceNow)),
                    kX, y, kLine, kTargetColor);
            y += kLine * 1.3f;

            auto approachLine = [&](const sw::space::ClosestApproach& approach,
                                    const char* label, const sw::Vec4& color) {
                if (!approach.valid)
                {
                    return;
                }
                // ALTITUDE, not centre distance, once the pass is close
                // enough to be about the surface: 380 km above Luna and
                // 2 117 km from its centre are the same fact, and only one
                // of them tells you whether you hit it.
                const sw::f64 altitude = approach.distanceM - target.bodyRadius;
                const bool hits = altitude <= 0.0;
                hudText(std::format("{} {}  T-{}  REL {:.0f} M/S", label,
                                    hits ? std::string("IMPACT")
                                         : formatDistance(approach.distanceM),
                                    formatEta(std::max(approach.timeSeconds - time, 0.0)),
                                    approach.relativeSpeedMps),
                        kX, y, kLine, hits ? sw::Vec4{1.0f, 0.35f, 0.3f, 0.95f} : color);
                y += kLine * 1.3f;
            };
            approachLine(m_approach, "APPROACH", kTargetColor);
            approachLine(m_nodeApproach, "AFTER BURN",
                         sw::Vec4{0.85f, 0.75f, 1.0f, 0.95f});
        }

        // ---- maneuver node status --------------------------------------------
        if (m_nodeActive)
        {
            // Far from the node: show the PLANNED dv. Once the plan is
            // LOCKED — two minutes out — the LIVE remaining vector, which
            // now genuinely counts down as the engine burns.
            const sw::f64 timeToNode = m_nodeTime - time;
            const sw::f64 plannedDv =
                std::sqrt(m_nodePrograde * m_nodePrograde +
                          m_nodeNormal * m_nodeNormal + m_nodeRadial * m_nodeRadial);
            const sw::f64 remainingDv = glm::length(remainingBurnVector());
            const bool burnWindow = m_burnLocked;
            const sw::Vec4 nodeColor{0.8f, 0.55f, 1.0f, 0.95f};
            hudText(std::format("NODE T-{} DV {:.1f} M/S{}",
                                formatEta(std::max(timeToNode, 0.0)),
                                burnWindow ? remainingDv : plannedDv,
                                burnWindow ? " BURN" : ""),
                    kX, y, kLine, nodeColor);
            y += kLine * 1.3f;
            if (m_mapView)
            {
                hudText(std::format("PGD {:+.1f} NRM {:+.1f} RAD {:+.1f}",
                                    m_nodePrograde, m_nodeNormal, m_nodeRadial),
                        kX, y, kLine, nodeColor);
                y += kLine * 1.3f;
                // WHICH STEP IS ARMED. A ladder the player cannot see is a
                // ladder they have to remember; this line changes under
                // their thumb as they hold the modifier, which is the only
                // documentation a control like this needs.
                const bool shift = input().isKeyDown(sw::KeyCode::LeftShift) ||
                                   input().isKeyDown(sw::KeyCode::RightShift);
                const bool control = input().isKeyDown(sw::KeyCode::LeftControl) ||
                                     input().isKeyDown(sw::KeyCode::RightControl);
                const bool alt = input().isKeyDown(sw::KeyCode::LeftAlt) ||
                                 input().isKeyDown(sw::KeyCode::RightAlt);
                const sw::space::ManeuverStep step =
                    sw::space::maneuverStep(shift, control, alt);
                const char* held = (control && shift) ? "CTRL+SHIFT"
                                   : alt              ? "ALT"
                                   : shift            ? "SHIFT"
                                   : control          ? "CTRL"
                                                      : "-";
                hudText(std::format("STEP {:g} M/S  {:g} S   [{}]  {}", step.deltaVMps,
                                    step.seconds, held,
                                    m_nodeDragging ? "SLIDING" : "DRAG TO SLIDE"),
                        kX, y, kLine,
                        m_nodeDragging ? sw::Vec4{1.0f, 0.85f, 0.45f, 0.95f}
                        : (step.deltaVMps > 1.0)
                            ? sw::Vec4{1.0f, 0.78f, 0.30f, 0.95f}
                            : nodeColor);
                y += kLine * 1.3f;
            }
        }

        // ---- F2: what the ground cursor is about to do -------------------
        if (!m_mapView && !m_editorMode && m_evaMode && !m_buildMenu)
        {
            const auto* held = sw::parts::findDefinition(m_heldBuilding);
            auto nameOf = [&](sw::ecs::Entity entity) -> std::string {
                const auto* building =
                    m_world.tryGetComponent<sw::factory::BuildingComponent>(entity);
                const auto* definition =
                    (building != nullptr)
                        ? sw::parts::findDefinition(building->definitionId)
                        : nullptr;
                return (definition != nullptr) ? definition->name : std::string("?");
            };
            const bool beltMode =
                held != nullptr &&
                held->building.category == sw::factory::BuildingCategory::Conveyor;
            const bool cableMode =
                held != nullptr &&
                held->building.category == sw::factory::BuildingCategory::Cable;

            if (beltMode)
            {
                // Two clicks, and the HUD says which one you are on.
                if (m_beltSource.isNull())
                {
                    hudText("BELT  PICK AN OUTPUT", -0.36f, 0.70f, 0.038f, hud::kTitle);
                    hudText(m_buildCursor.target.isNull()
                                ? "LOOK AT A MACHINE"
                                : std::format("LCLICK  FROM {}",
                                              nameOf(m_buildCursor.target)),
                            -0.36f, 0.75f, 0.030f, hud::kTextDim);
                }
                else
                {
                    const bool ok = m_beltVerdict == sw::build::Verdict::Ok &&
                                    !m_beltPreview.empty();
                    hudText(std::format("BELT  FROM {}", nameOf(m_beltSource)), -0.36f,
                            0.70f, 0.038f, ok ? hud::kOk : hud::kBad);
                    hudText(m_beltPreview.empty()
                                ? "LOOK AT AN INPUT   R CANCEL"
                                : (ok ? std::format("LCLICK  {} SEGMENTS TO {}   R CANCEL",
                                                    m_beltPreview.size(),
                                                    nameOf(m_buildCursor.target))
                                      : std::string(sw::build::verdictText(m_beltVerdict))),
                            -0.36f, 0.75f, 0.030f, ok ? hud::kTextDim : hud::kBad);
                }
            }
            else if (cableMode)
            {
                if (m_cableSource.isNull())
                {
                    hudText("CABLE  PICK A CONNECTION", -0.36f, 0.70f, 0.038f,
                            hud::kTitle);
                    hudText(m_buildCursor.target.isNull()
                                ? std::string("LOOK AT A BUILDING OR A POLE")
                                : std::format("LCLICK  FROM {}   R CUT ITS CABLES",
                                              nameOf(m_buildCursor.target)),
                            -0.36f, 0.75f, 0.030f, hud::kTextDim);
                }
                else
                {
                    const bool ok = m_cableVerdict == sw::factory::CableVerdict::Ok;
                    hudText(std::format("CABLE  FROM {}", nameOf(m_cableSource)), -0.36f,
                            0.70f, 0.038f, ok ? hud::kOk : hud::kBad);
                    hudText(m_buildCursor.target.isNull()
                                ? std::string("LOOK AT THE OTHER END   R CANCEL")
                                : (ok ? std::format("LCLICK  WIRE TO {}   R CANCEL",
                                                    nameOf(m_buildCursor.target))
                                      : std::string(sw::factory::cableVerdictText(
                                            m_cableVerdict))),
                            -0.36f, 0.75f, 0.030f, ok ? hud::kTextDim : hud::kBad);
                }
            }
            else if (held != nullptr)
            {
                const bool ok = m_buildCursor.verdict == sw::build::Verdict::Ok;
                hudText(std::format("BUILD {}  {:.0f} M", held->name,
                                    m_buildCursor.rangeM),
                        -0.36f, 0.70f, 0.038f, ok ? hud::kOk : hud::kBad);
                hudText(ok ? "LCLICK BUILD   WHEEL ROTATE   F MENU"
                           : sw::build::verdictText(m_buildCursor.verdict),
                        -0.36f, 0.75f, 0.030f, ok ? hud::kTextDim : hud::kBad);
            }
            // F3: the machine panel. Only worth advertising when there IS a
            // machine within arm's reach, which is also exactly when E works.
            if (held == nullptr && m_configTarget.isNull() &&
                !m_buildCursor.target.isNull() &&
                m_buildCursor.rangeM <= kConfigRangeM)
            {
                hudText(std::format("E  CONFIGURE {}", nameOf(m_buildCursor.target)),
                        -0.36f, 0.70f, 0.034f, hud::kText);
            }
            if (!beltMode && !cableMode && !m_buildCursor.target.isNull())
            {
                hudText(std::format("R  DEMOLISH {}", nameOf(m_buildCursor.target)),
                        -0.36f, 0.80f, 0.030f, hud::kWarn);
            }
        }

        if (m_buildMenu && !m_editorMode)
        {
            // The catalogue takes the clickable UI over while it is open —
            // it owns m_hudButtons, so nothing behind it can be clicked
            // through.
            collectBuildMenu();
        }
        else if (!m_configTarget.isNull() && !m_editorMode && !m_mapView)
        {
            collectConfigMenu();
        }
        else if (!m_mapView && !m_editorMode)
        {
            collectNavball();
            collectSasButtons();
            // ...and the warp-to-node button in the cockpit too: the burn is
            // planned on the map but it is FLOWN here, and being sent back
            // to the map to skip four hours is a trip for nothing.
            collectWarpToNodeButton();
        }
        else if (m_mapView && !m_editorMode)
        {
            collectMapButtons();
            collectWarpToNodeButton(); // appends: collectMapButtons clears
        }
        // The multiplayer panel lives on the RIGHT and appends after
        // whichever collector above cleared the list, so it coexists with the
        // flight HUD, the map and even the build catalogue rather than
        // fighting any of them for the button table.
        if (m_netPanel && !m_editorMode)
        {
            collectNetPanel();
        }
        // (Hangar UI is not collected here: the hangar renders through its
        // own path — collectHangarItems -> collectEditorUi.)
    }

    void StarWorksGame::collectSasButtons()
    {
        // First clickable UI: three buttons above the bottom-left corner.
        m_hudButtons.clear();
        constexpr sw::f32 kHeight = 0.062f;
        constexpr sw::f32 kWidth = 0.115f;
        constexpr sw::f32 kGap = 0.018f;
        const sw::f32 y0 = 0.87f;
        sw::f32 x0 = -0.97f;

        // NODE is the fourth: a burn is almost never prograde — a plane
        // change is normal, a circularisation is prograde only by accident
        // — and flying one by eye means chasing a marker across the navball
        // with the throttle already open. It greys out with no node up.
        // SAS is a MODE now, not the absence of one: it holds the craft
        // still. Every button toggles — clicking the lit one switches the
        // autopilot off — so there is no longer a button whose only job is
        // to mean "none of the others".
        const char* labels[4] = {"SAS", "PGD", "RTG", "NODE"};
        const sw::u32 modes[4] = {SasComponent::kStability, SasComponent::kPrograde,
                                  SasComponent::kRetrograde, SasComponent::kNode};
        for (sw::u32 slot = 0; slot < 4; ++slot)
        {
            const sw::u32 id = modes[slot];
            const sw::f32 x1 = x0 + kWidth;
            const bool available = (id != SasComponent::kNode) || m_nodeActive;
            const bool active = (m_sasMode == id);
            const sw::Vec4 background =
                active      ? sw::Vec4{0.15f, 0.55f, 0.30f, 0.85f}
                : available ? sw::Vec4{0.16f, 0.22f, 0.30f, 0.65f}
                            : sw::Vec4{0.12f, 0.14f, 0.18f, 0.55f};

            sw::DrawItem panel{};
            panel.mesh = &m_meshes[m_navLineMeshIndex]; // unit quad
            panel.transform =
                glm::translate(sw::Mat4{1.0f},
                               {(x0 + x1) * 0.5f, y0 + kHeight * 0.5f, 0.0f}) *
                glm::scale(sw::Mat4{1.0f},
                           {kWidth * 0.5f, kHeight * 0.5f, 1.0f});
            panel.screenSpace = true;
            panel.tint = background;
            m_drawItems.push_back(panel);

            hudText(labels[slot], x0 + 0.022f, y0 + 0.015f, 0.036f,
                    active      ? sw::Vec4{0.9f, 1.0f, 0.9f, 1.0f}
                    : available ? sw::Vec4{0.7f, 0.8f, 0.9f, 0.9f}
                                : sw::Vec4{0.42f, 0.48f, 0.56f, 0.8f});

            if (available)
            {
                m_hudButtons.push_back({x0, y0, x1, y0 + kHeight, id});
            }
            x0 = x1 + kGap;
        }

        // WHICH PROGRADE. Under the row, because the two buttons above it
        // mean different directions depending on a toggle three metres away
        // on the other side of the screen — and on the way down they are
        // tens of degrees apart. `V` swaps it, the same key that swaps the
        // speed readout and the navball markers.
        hudText(std::format("{} FRAME (V)", m_speedSurfaceRelative ? "SRF" : "ORB"),
                -0.97f, y0 - 0.045f, 0.032f,
                m_speedSurfaceRelative ? sw::Vec4{0.55f, 0.85f, 0.65f, 0.9f}
                                       : sw::Vec4{0.6f, 0.72f, 0.9f, 0.9f});
    }

    // The map is where you look at your fleet, so it is where you should be
    // able to change which of it you are flying. `P` already cycled; this is
    // the same action with a surface you can find without knowing it exists.
    void StarWorksGame::collectMapButtons()
    {
        m_hudButtons.clear();

        std::vector<sw::ecs::Entity> pilotable;
        m_world.forEach<ShipComponent>([&](sw::ecs::Entity entity, ShipComponent&) {
            pilotable.push_back(entity);
        });
        if (pilotable.size() < 2)
        {
            return; // one ship: nothing to cycle between
        }
        sw::usize current = 0;
        for (sw::usize i = 0; i < pilotable.size(); ++i)
        {
            if (pilotable[i] == m_shipEntity)
            {
                current = i;
            }
        }

        constexpr sw::f32 kHeight = 0.062f;
        constexpr sw::f32 kWidth = 0.235f;
        const sw::f32 x0 = -0.97f;
        const sw::f32 y0 = 0.87f;
        const sw::f32 x1 = x0 + kWidth;

        sw::DrawItem panel{};
        panel.mesh = &m_meshes[m_navLineMeshIndex]; // unit quad
        panel.transform =
            glm::translate(sw::Mat4{1.0f}, {(x0 + x1) * 0.5f, y0 + kHeight * 0.5f, 0.0f}) *
            glm::scale(sw::Mat4{1.0f}, {kWidth * 0.5f, kHeight * 0.5f, 1.0f});
        panel.screenSpace = true;
        panel.tint = {0.16f, 0.22f, 0.30f, 0.65f};
        m_drawItems.push_back(panel);

        hudText("NEXT SHIP", x0 + 0.022f, y0 + 0.015f, 0.036f,
                {0.7f, 0.8f, 0.9f, 0.95f});
        hudText(std::format("SHIP {}/{}", current + 1, pilotable.size()), x0 + 0.022f,
                y0 - 0.052f, 0.034f, {0.55f, 0.72f, 0.88f, 0.9f});

        m_hudButtons.push_back({x0, y0, x1, y0 + kHeight, 300u});
    }

    // WARP TO THE NODE. A burn planned four hours out is four hours of
    // holding the warp key and watching for the moment to let go — and
    // overshooting by one rung of the ladder costs the whole orbit. This
    // stops one minute short, which is where a pilot wants to be anyway:
    // aligned, throttle hand ready, nothing to do but wait a little.
    void StarWorksGame::collectWarpToNodeButton()
    {
        if (!m_nodeActive)
        {
            return;
        }
        constexpr sw::f32 kHeight = 0.062f;
        // Wide enough for the LONGER of the two labels: a button whose text
        // runs off its own edge when you press it is a button that looks
        // broken at the exact moment you used it.
        constexpr sw::f32 kWidth = 0.40f;
        const sw::f32 x0 = -0.97f;
        const sw::f32 y0 = 0.78f;
        const sw::f32 x1 = x0 + kWidth;
        const bool running = m_warpToSeconds > 0.0;

        sw::DrawItem panel{};
        panel.mesh = &m_meshes[m_navLineMeshIndex]; // unit quad
        panel.transform =
            glm::translate(sw::Mat4{1.0f},
                           {(x0 + x1) * 0.5f, y0 + kHeight * 0.5f, 0.0f}) *
            glm::scale(sw::Mat4{1.0f}, {kWidth * 0.5f, kHeight * 0.5f, 1.0f});
        panel.screenSpace = true;
        panel.tint = running ? sw::Vec4{0.60f, 0.42f, 0.12f, 0.85f}
                             : sw::Vec4{0.16f, 0.22f, 0.30f, 0.65f};
        m_drawItems.push_back(panel);

        hudText(running ? "WARPING   CLICK TO STOP" : "WARP TO NODE -1 MIN",
                x0 + 0.020f, y0 + 0.016f, 0.034f,
                running ? sw::Vec4{1.0f, 0.92f, 0.75f, 1.0f}
                        : sw::Vec4{0.7f, 0.8f, 0.9f, 0.95f});
        m_hudButtons.push_back({x0, y0, x1, y0 + kHeight, 301u});
    }

    void StarWorksGame::handleHudClicks()
    {
        if (!input().wasMouseButtonPressed(sw::MouseButton::Left))
        {
            return;
        }
        sw::u32 width = 0;
        sw::u32 height = 0;
        window().framebufferSize(width, height);
        if (width == 0 || height == 0)
        {
            return;
        }
        const sw::f32 ndcX = input().mouseX() / static_cast<sw::f32>(width) * 2.0f - 1.0f;
        const sw::f32 ndcY = input().mouseY() / static_cast<sw::f32>(height) * 2.0f - 1.0f;
        for (const HudButton& button : m_hudButtons)
        {
            if (ndcX >= button.x0 && ndcX <= button.x1 && ndcY >= button.y0 &&
                ndcY <= button.y1)
            {
                // The multiplayer panel owns 1000+, tested first because
                // every other range below is open-ended upward.
                if (button.id >= 1100)
                {
                    const auto index = static_cast<sw::usize>(button.id - 1100u);
                    const std::vector<sw::net::PlayerView> roster = netRoster();
                    if (index < roster.size())
                    {
                        netSyncTo(roster[index].id, roster[index].simulatedSeconds);
                    }
                    break;
                }
                if (button.id == 1000)
                {
                    netHost();
                    break;
                }
                if (button.id == 1001)
                {
                    netJoin();
                    break;
                }
                if (button.id == 1002)
                {
                    netLeave();
                    break;
                }
                if (button.id == 1003)
                {
                    m_netAddressFocused = !m_netAddressFocused;
                    break;
                }
                // ---- the machine panel (E) ----------------------------------
                // Checked FIRST: its ids sit above the build menu's, and it
                // is the panel actually on screen when they are live.
                // The VAB's rows sit ABOVE the recipe ids, and are therefore
                // tested first: a recipe id is an arbitrary small number and
                // 610+id would happily swallow 900.
                if (button.id >= 900 && !m_configTarget.isNull())
                {
                    const std::span<const sw::parts::ShipBlueprint> designs =
                        sw::parts::blueprintCatalog();
                    const sw::usize index = button.id - 900u;
                    if (index < designs.size())
                    {
                        // A row SELECTS. Ordering is one deliberate press of
                        // PRODUCE, next to the price and the picture — not a
                        // side effect of reading the catalogue.
                        m_vabSelection = static_cast<sw::i32>(index);
                    }
                    return;
                }
                if (button.id == 898 && !m_configTarget.isNull())
                {
                    const std::span<const sw::parts::ShipBlueprint> catalogue =
                        sw::parts::blueprintCatalog();
                    if (m_vabSelection >= 0 &&
                        static_cast<sw::usize>(m_vabSelection) < catalogue.size())
                    {
                        orderVehicle(m_configTarget,
                                     catalogue[static_cast<sw::usize>(m_vabSelection)]);
                    }
                    break;
                }
                if (button.id == 899 && !m_configTarget.isNull())
                {
                    if (auto* assembly =
                            m_world.tryGetComponent<sw::factory::AssemblyComponent>(
                                m_configTarget))
                    {
                        sw::factory::assemblyOrder(*assembly, {}, 0.0, 0.0);
                    }
                    return;
                }
                if (button.id >= 610 && !m_configTarget.isNull())
                {
                    applyRecipeChoice(m_configTarget, button.id - 610u);
                    return;
                }
                if (button.id == 600 && !m_configTarget.isNull())
                {
                    applyRecipeChoice(m_configTarget, 0u); // STOP
                    return;
                }
                if (button.id == 601 && !m_configTarget.isNull())
                {
                    if (auto* power = m_world.tryGetComponent<sw::factory::PowerComponent>(
                            m_configTarget))
                    {
                        power->priority = (power->priority + 1u) % 5u;
                    }
                    return;
                }
                if (button.id >= 400) // build menu: arm this building
                {
                    const sw::u32 definitionId = button.id - 400u;
                    m_heldBuilding =
                        (m_heldBuilding == definitionId) ? 0u : definitionId;
                    return;
                }
                if (button.id == 300) // map: fly the next vessel
                {
                    cyclePilotedVessel();
                    return;
                }
                if (button.id == 301) // map: warp to one minute before the node
                {
                    if (m_warpToSeconds > 0.0)
                    {
                        m_warpToSeconds = 0.0;
                        m_warpIndex = 0;
                        SW_LOG_INFO("Game", "Warp to node cancelled");
                    }
                    else if (m_nodeActive)
                    {
                        m_warpToSeconds = m_nodeTime - 60.0;
                        m_simulation.setPaused(false);
                        SW_LOG_INFO("Game", "Warping to T-60 s on the node");
                    }
                    return;
                }
                if (m_mapView)
                {
                    return; // the map owns only its own buttons
                }
                // ---- hangar actions ------------------------------------------
                if (button.id >= 200)
                {
                    if (button.id == 201 && m_blueprint.size() > 1) // UNDO
                    {
                        // Remove the last placement: the trailing part plus
                        // any trailing symmetry siblings placed with it.
                        const sw::i32 group = m_blueprint.back().symmetryGroup;
                        m_blueprint.pop_back();
                        while (group >= 0 && m_blueprint.size() > 1 &&
                               m_blueprint.back().symmetryGroup == group)
                        {
                            m_blueprint.pop_back();
                        }
                    }
                    else if (button.id == 202) { hangarNewBlueprint(); }
                    else if (button.id == 203) { hangarLoadNextVessel(); }
                            else if (button.id == 205) // symmetry cycle
                    {
                        const sw::u32 options[6] = {1, 2, 3, 4, 6, 8};
                        for (sw::usize i = 0; i < 6; ++i)
                        {
                            if (options[i] == m_symmetryCount)
                            {
                                m_symmetryCount = options[(i + 1) % 6];
                                break;
                            }
                        }
                    }
                    else if (button.id == 206) { m_showCenters = !m_showCenters; }
                    else if (button.id == 207) { hangarSaveShip(); }
                    return;
                }
                if (button.id >= 100) // palette: take the part IN HAND
                {
                    const auto partCatalog = rocketPartPalette();
                    const sw::usize index = button.id - 100;
                    if (index < partCatalog.size())
                    {
                        if (!m_blueprintBackup.empty())
                        {
                            m_blueprint = m_blueprintBackup; // drop a pending grab
                            m_blueprintBackup.clear();
                        }
                        m_heldDefinition = partCatalog[index]->id;
                        m_heldSubtree.clear();
                        m_heldRotation = {1.0f, 0.0f, 0.0f, 0.0f};
                    }
                    return;
                }
                // EVERY autopilot button toggles: clicking the lit one puts
                // the autopilot back to OFF. There is no longer a button
                // that only means "none of the others" — SAS is a mode.
                m_sasMode = (m_sasMode == button.id) ? SasComponent::kOff : button.id;
                SW_LOG_INFO("Game", "SAS mode: {}", sasModeName(m_sasMode));
                break;
            }
        }

        // ---- hangar 3D click (no button consumed it) ----------------------------
        if (m_editorMode)
        {
            if (m_heldDefinition != 0)
            {
                commitGhost(); // no-op while the ghost is red/inactive
                return;
            }
            // Empty hand: ray-pick a placed part and grab its subtree.
            sw::Vec3 origin{};
            sw::Vec3 direction{};
            editorCursorRay(origin, direction);
            sw::f32 bestT = 1.0e30f;
            sw::i32 bestPart = -1;
            for (sw::usize i = 0; i < m_blueprint.size(); ++i)
            {
                const auto* definition =
                    sw::parts::findDefinition(m_blueprint[i].definitionId);
                if (definition == nullptr)
                {
                    continue;
                }
                const sw::Quat inverseRot = glm::inverse(m_blueprint[i].localRotation);
                const sw::Vec3 localOrigin =
                    inverseRot * (origin - m_blueprint[i].localPosition);
                const sw::Vec3 localDirection = inverseRot * direction;
                sw::parts::PartRayHit hit{};
                if (sw::parts::raycastPart(*definition, localOrigin, localDirection,
                                           500.0f, hit) &&
                    hit.t < bestT)
                {
                    bestT = hit.t;
                    bestPart = static_cast<sw::i32>(i);
                }
            }
            if (bestPart >= 0)
            {
                grabPartAt(static_cast<sw::usize>(bestPart));
            }
        }
    }

    void StarWorksGame::collectNavball()
    {
        const sw::i32 primaryIndex = controlledPrimaryIndex();
        if (primaryIndex < 0)
        {
            return; // no reference vertical in deep space
        }
        const sw::ecs::Entity entity = controlledEntity();
        const auto& transform = m_world.getComponent<TransformComponent>(entity);
        const sw::f64 time = m_physicsLane->presentSeconds();

        sw::WorldVec3 primaryPosition{};
        sw::WorldVec3 primaryVelocity{};
        m_celestialIndex.stateAt(primaryIndex, time, primaryPosition, &primaryVelocity);
        const sw::WorldVec3 radial = transform.position - primaryPosition;
        const sw::f64 distance = glm::length(radial);
        if (distance <= 1.0)
        {
            return;
        }
        const sw::Vec3 up = sw::Vec3(radial / distance);

        // ---- attitude vs the local horizon --------------------------------------
        const sw::Quat rotation = transform.rotation;
        const sw::Vec3 forward = rotation * sw::math::kWorldForward;
        const sw::Vec3 rightWing = rotation * sw::Vec3{1.0f, 0.0f, 0.0f};
        const sw::Vec3 shipUp = rotation * sw::Vec3{0.0f, 1.0f, 0.0f};
        const sw::f32 pitch =
            std::asin(std::clamp(glm::dot(forward, up), -1.0f, 1.0f));
        const sw::f32 roll = std::atan2(glm::dot(rightWing, up), glm::dot(shipUp, up));

        const sw::f32 aspect = renderer().aspectRatio();
        const sw::f32 ballRadius = kNavballRadius;

        // Isotropic instrument space: rotate/offset there, compress X by the
        // aspect ratio last (same convention as hudText).
        const sw::Mat4 base =
            glm::translate(sw::Mat4{1.0f}, {0.0f, kNavballCenterY, 0.0f}) *
            glm::scale(sw::Mat4{1.0f}, {1.0f / aspect, 1.0f, 1.0f});

        auto pushNav = [&](sw::u32 meshIndex, const sw::Mat4& local,
                           const sw::Vec4& color) {
            sw::DrawItem item{};
            item.mesh = &m_meshes[meshIndex];
            item.transform = base * local;
            item.screenSpace = true;
            item.tint = color;
            m_drawItems.push_back(item);
        };

        const sw::Vec4 frameColor{0.65f, 0.85f, 0.9f, 0.55f};
        const sw::Vec4 horizonColor{0.55f, 0.95f, 1.0f, 0.9f};
        const sw::Vec4 referenceColor{1.0f, 0.62f, 0.15f, 0.95f};

        // Outer ring.
        pushNav(m_navRingMeshIndex,
                glm::scale(sw::Mat4{1.0f}, sw::Vec3{ballRadius}), frameColor);

        // Horizon line: rotated by roll, shifted by pitch (nose up -> the
        // horizon drops on screen; screen Y grows downward). Each line is
        // clipped to the CHORD of the ball at its offset, so the instrument
        // never bleeds outside its ring (and degrades gracefully at the
        // straight-up/straight-down gimbal poles, where the chord vanishes).
        const sw::Mat4 rollRotation =
            glm::rotate(sw::Mat4{1.0f}, roll, {0.0f, 0.0f, 1.0f});
        auto pushHorizonLine = [&](sw::f32 angleFromHorizon, sw::f32 widthFactor,
                                   sw::f32 thickness, sw::f32 alpha) {
            const sw::f32 normalized =
                std::clamp((pitch + angleFromHorizon) / kHalfPi, -1.0f, 1.0f);
            const sw::f32 offset = normalized * ballRadius * 0.92f;
            const sw::f32 chord = std::sqrt(std::max(
                0.0f, 1.0f - normalized * normalized * 0.85f)); // 0 at the poles
            if (chord < 0.05f)
            {
                return;
            }
            pushNav(m_navLineMeshIndex,
                    rollRotation *
                        glm::translate(sw::Mat4{1.0f}, {0.0f, offset, 0.0f}) *
                        glm::scale(sw::Mat4{1.0f},
                                   {ballRadius * widthFactor * chord,
                                    ballRadius * thickness, 1.0f}),
                    {horizonColor.r, horizonColor.g, horizonColor.b, alpha});
        };
        pushHorizonLine(0.0f, 0.86f, 0.012f, 0.9f);
        // Short pitch ticks at +/- 30 degrees from the horizon.
        pushHorizonLine(-0.5235988f, 0.34f, 0.007f, 0.5f);
        pushHorizonLine(0.5235988f, 0.34f, 0.007f, 0.5f);

        // Fixed craft reference: center diamond + stub wings.
        pushNav(m_navDiamondMeshIndex,
                glm::scale(sw::Mat4{1.0f}, sw::Vec3{ballRadius * 0.05f}),
                referenceColor);
        for (const sw::f32 side : {-1.0f, 1.0f})
        {
            pushNav(m_navLineMeshIndex,
                    glm::translate(sw::Mat4{1.0f},
                                   {side * ballRadius * 0.17f, 0.0f, 0.0f}) *
                        glm::scale(sw::Mat4{1.0f},
                                   {ballRadius * 0.09f, ballRadius * 0.012f, 1.0f}),
                    referenceColor);
        }

        // ---- prograde / retrograde markers ---------------------------------------
        // Velocity in the HUD's current reference frame (ORB or SRF, like
        // the speed readout), projected into the craft's body frame.
        sw::WorldVec3 referenceVelocity = primaryVelocity;
        if (m_speedSurfaceRelative)
        {
            if (const auto* source =
                    m_world.tryGetComponent<sw::phys::GravitySourceComponent>(
                        m_celestialIndex.body(static_cast<sw::usize>(primaryIndex))
                            .entity))
            {
                referenceVelocity += glm::cross(source->angularVelocity, radial);
            }
        }
        const sw::WorldVec3 relativeVelocity =
            controlledVelocity() - referenceVelocity;
        const sw::f64 relativeSpeed = glm::length(relativeVelocity);
        if (relativeSpeed > 0.5)
        {
            const sw::Vec3 direction = sw::Vec3(relativeVelocity / relativeSpeed);
            const sw::Quat inverseRotation = glm::inverse(rotation);
            for (const sw::f32 sign : {1.0f, -1.0f}) // prograde, retrograde
            {
                const sw::Vec3 local = inverseRotation * (direction * sign);
                if (local.z >= 0.0f)
                {
                    continue; // behind the craft: not on the front hemisphere
                }
                const sw::Vec2 ballPosition{local.x * ballRadius,
                                            -local.y * ballRadius};
                const sw::Mat4 place = glm::translate(
                    sw::Mat4{1.0f}, {ballPosition.x, ballPosition.y, 0.0f});
                if (sign > 0.0f) // prograde: filled diamond
                {
                    pushNav(m_navDiamondMeshIndex,
                            place * glm::scale(sw::Mat4{1.0f},
                                               sw::Vec3{ballRadius * 0.085f}),
                            {0.55f, 1.0f, 0.35f, 0.95f});
                }
                else // retrograde: hollow ring
                {
                    pushNav(m_navRingMeshIndex,
                            place * glm::scale(sw::Mat4{1.0f},
                                               sw::Vec3{ballRadius * 0.085f}),
                            {1.0f, 0.5f, 0.25f, 0.95f});
                }
            }
        }

        // ---- maneuver burn marker: point the nose at it, burn, watch DV --
        if (m_nodeActive)
        {
            // The SAME vector the readout counts down and the SAS points
            // at: one answer to "where do I aim and how much is left",
            // never three that can disagree.
            const sw::WorldVec3 burnVector = remainingBurnVector();
            const sw::f64 burnLength = glm::length(burnVector);
            if (burnLength > 0.05)
            {
                const sw::Vec3 local = glm::inverse(rotation) *
                                       sw::Vec3(burnVector / burnLength);
                if (local.z < 0.0f) // front hemisphere
                {
                    const sw::Mat4 place = glm::translate(
                        sw::Mat4{1.0f},
                        {local.x * ballRadius, -local.y * ballRadius, 0.0f});
                    pushNav(m_navDiamondMeshIndex,
                            place * glm::scale(sw::Mat4{1.0f},
                                               sw::Vec3{ballRadius * 0.11f}),
                            {0.75f, 0.4f, 1.0f, 0.95f});
                    pushNav(m_navRingMeshIndex,
                            place * glm::scale(sw::Mat4{1.0f},
                                               sw::Vec3{ballRadius * 0.13f}),
                            {0.75f, 0.4f, 1.0f, 0.7f});
                }
            }
        }
    }
} // namespace game
