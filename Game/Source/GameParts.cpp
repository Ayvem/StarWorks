// ============================================================================
// GameParts.cpp — parts that move, and the pilot's menu that moves them.
//
// A part used to be one welded mesh: every shape's pose baked into its
// vertices, uploaded once at boot, shared by every instance of that part id.
// That is still exactly what most of a part is. What changed is that shapes
// can now belong to an ANIMATION GROUP, each group gets its own mesh, and a
// group is drawn with one extra transform in front of the part's own.
//
// Nothing about the vertex format, the draw item or the shader had to learn
// what an animation is. That is the whole reason it is a splitting rather than
// a skinning pass: the engine already draws a mesh at a matrix, and a hinged
// panel is a mesh at a matrix that changes.
// ============================================================================

#include "StarWorksGame.hpp"

#include "GameInternal.hpp"

#include <UI/HudRoute.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace game
{
    void StarWorksGame::attachPartAnimation(sw::ecs::Entity part, sw::u32 definitionId)
    {
        const auto* definition = sw::parts::findDefinition(definitionId);
        if (definition == nullptr || definition->animations.empty())
        {
            return; // thirty-four parts out of thirty-six take this branch
        }
        sw::parts::PartAnimationComponent state{};
        state.count = static_cast<sw::u32>(std::min<sw::usize>(
            definition->animations.size(),
            sw::parts::PartAnimationComponent::kMaxAnimations));
        for (sw::u32 i = 0; i < state.count; ++i)
        {
            // A solar wing folds into its fairing; a landing gear is down on
            // the pad. Both are the authored rest state, and both are what the
            // part is BUILT in — no opening animation plays on the launchpad.
            const sw::f32 initial = definition->animations[i].startsOpen ? 1.0f : 0.0f;
            state.phase[i] = initial;
            state.target[i] = initial;
        }
        m_world.addComponent(part, state);
    }

    void StarWorksGame::collectAnimatedGroups(sw::ecs::Entity entity,
                                              const sw::parts::PartComponent& part,
                                              const sw::Mat4& partModel,
                                              const sw::Vec3& relative,
                                              sw::f32 boundsRadius, const sw::Vec4& tint)
    {
        const auto* state =
            m_world.tryGetComponent<sw::parts::PartAnimationComponent>(entity);
        if (state == nullptr)
        {
            return;
        }
        const auto* definition = sw::parts::findDefinition(part.definitionId);
        if (definition == nullptr)
        {
            return;
        }
        const auto motions = m_partMotions.find(part.definitionId);
        if (motions == m_partMotions.end())
        {
            return;
        }
        // SW_ANIMPROBE=<definitionId>: what this part's motions actually DID
        // this frame, from inside the draw loop that does it — how many
        // motions, which mesh each found, what phase drove it and where its
        // matrix put it. Three rounds of "the animation is still buggy" went
        // past on evidence that stopped at the mesh grouping, which was
        // correct; the thing nobody had printed was the transform.
        std::FILE* probe = nullptr;
        if (const char* wanted = std::getenv("SW_ANIMPROBE");
            wanted != nullptr && !m_debugAnimProbed &&
            part.definitionId == static_cast<sw::u32>(std::strtoul(wanted, nullptr, 10)) &&
            ++m_debugAnimDelay > 150u)
        {
            // LATE, and that is the point. The first version fired on the
            // first frame this part was drawn, which is a hundred frames
            // before the hook presses the button, and reported phase 0.000
            // for an animation that had not been asked to move yet -- a
            // measurement of the probe rather than of the feature.
            m_debugAnimProbed = true;
            probe = std::fopen("/tmp/sw_animprobe.txt", "w");
            if (probe != nullptr)
            {
                std::fprintf(probe, "definition %u: %zu motions, %u animation states\n",
                             part.definitionId, motions->second.size(), state->count);
                for (sw::u32 a = 0; a < state->count; ++a)
                {
                    std::fprintf(probe, "  A%u phase %.3f target %.3f\n", a,
                                 static_cast<double>(state->phase[a]),
                                 static_cast<double>(state->target[a]));
                }
            }
        }
        for (sw::u32 index = 0; index < motions->second.size(); ++index)
        {
            const sw::parts::PartMotionGroup& motion = motions->second[index];
            const auto slot = m_partGroupMeshIds.find(partGroupKey(part.definitionId, index));
            if (slot == m_partGroupMeshIds.end())
            {
                continue;
            }
            // ONE HINGE PER MOTION, and the motion is what the poses say it
            // is rather than what the animation index says.
            //
            // Shapes that share a hinge share a mesh and one transform — an
            // array and the four struts holding it swing together or they
            // come apart in mid-air, and they really do share a hinge. Shapes
            // that DO NOT share one are separate motions, which is how a
            // telescoping array whose four segments deploy to four different
            // places comes out as four segments rather than as one carrying
            // three passengers. `partMotionGroups` derives both from the
            // authored poses; nothing here has to be told which it is.
            const sw::u32 group = (motion.animation >= 0)
                                      ? static_cast<sw::u32>(motion.animation)
                                      : 0u;
            if (group >= state->count || motion.driver >= definition->shapes.size())
            {
                continue;
            }
            const sw::parts::PartShape* driver = &definition->shapes[motion.driver];
            const sw::Quat restRotation{glm::radians(driver->rotationDeg)};
            const sw::Quat endRotation{glm::radians(driver->endRotationDeg)};
            const sw::parts::HingeMotion hinge = sw::parts::hingeBetween(
                driver->position, restRotation, driver->endPosition, endRotation);
            // SMOOTHSTEP, not the raw phase. A hinge that starts and stops
            // instantly reads as a mechanism being yanked; real deployments
            // ease in and out because they are driven by a motor with inertia.
            const sw::f32 raw = glm::clamp(state->phase[group], 0.0f, 1.0f);
            const sw::f32 eased = raw * raw * (3.0f - 2.0f * raw);
            sw::Vec3 position{};
            sw::Quat rotation{};
            sw::parts::poseAlongHinge(hinge, driver->position, restRotation, eased,
                                      position, rotation);

            // The group's mesh was baked at the REST pose, so the transform
            // that moves it is "undo the rest pose, apply the live one".
            const sw::Mat4 rest = glm::translate(sw::Mat4{1.0f}, driver->position) *
                                  glm::mat4_cast(restRotation);
            const sw::Mat4 live = glm::translate(sw::Mat4{1.0f}, position) *
                                  glm::mat4_cast(rotation);
            sw::DrawItem item{&m_meshes[slot->second],
                              partModel * live * glm::inverse(rest), relative,
                              boundsRadius};
            item.tint = tint;
            // A NOZZLE THAT LIGHTS UP. The one animation that does not move at
            // all: the glow cone of an engine, whose emissive goes from dark to
            // white with the throttle. It rides the tint's rgb, which the
            // emissive branch of the shader already reads as a radiance.
            if (driver->endEmissive >= 0.0f)
            {
                const sw::f32 glow =
                    glm::mix(driver->emissive, driver->endEmissive, eased);
                const sw::f32 scale = (driver->emissive > 1.0e-4f)
                                          ? (glow / driver->emissive)
                                          : (1.0f + glow * 6.0f);
                item.tint = {tint.r * scale, tint.g * scale, tint.b * scale, tint.a};
            }
            m_drawItems.push_back(item);
            if (probe != nullptr)
            {
                // ...AND WHERE IT LANDS IN THE WORLD. The part-local answer
                // was right three times running while the picture stayed
                // wrong, so the probe follows the matrix all the way to the
                // camera-relative point the renderer will actually place.
                // The mesh is baked at the REST pose, so the point that
                // lands where the shape is DRAWN is the rest point put
                // through the item's own matrix. Putting the END pose
                // through it applies the motion twice, which is how the
                // first version of this line produced numbers that were
                // wrong by exactly one deployment.
                const sw::Vec4 tip = item.transform * sw::Vec4(driver->position, 1.0f);
                std::fprintf(probe, "        drawn at %8.3f %8.3f %8.3f\n",
                             static_cast<double>(tip.x), static_cast<double>(tip.y),
                             static_cast<double>(tip.z));
                const sw::Mat4 delta = live * glm::inverse(rest);
                std::fprintf(probe,
                             "  motion %2u A%d driver %2u x%zu mesh %u  phase %.3f  "
                             "rest %6.3f %6.3f %6.3f -> now %6.3f %6.3f %6.3f\n",
                             index, motion.animation, motion.driver,
                             motion.shapes.size(), slot->second,
                             static_cast<double>(raw),
                             static_cast<double>(driver->position.x),
                             static_cast<double>(driver->position.y),
                             static_cast<double>(driver->position.z),
                             static_cast<double>((delta * sw::Vec4(driver->position, 1.0f)).x),
                             static_cast<double>((delta * sw::Vec4(driver->position, 1.0f)).y),
                             static_cast<double>((delta * sw::Vec4(driver->position, 1.0f)).z));
            }
        }
        if (probe != nullptr)
        {
            std::fclose(probe);
        }
    }

    void StarWorksGame::updateThrottleAnimations()
    {
        // THE THROTTLE IS NOT A PART'S BUSINESS, so the engine's own system
        // cannot read it: ShipComponent lives in the game and the animation
        // system lives in the engine, which is the right way round. What comes
        // down here is one number per vessel, written into the target of every
        // throttle-triggered animation on it.
        m_world.forEach<sw::parts::PartComponent, sw::parts::PartAnimationComponent>(
            [&](sw::ecs::Entity entity, sw::parts::PartComponent& part,
                sw::parts::PartAnimationComponent& state) {
                const auto* definition = sw::parts::findDefinition(part.definitionId);
                if (definition == nullptr)
                {
                    return;
                }
                const auto* ship = m_world.tryGetComponent<ShipComponent>(part.vessel);
                const auto* controls =
                    m_world.tryGetComponent<ShipControlsComponent>(part.vessel);
                const sw::f32 throttle =
                    (ship != nullptr && controls != nullptr)
                        ? ship->throttle * std::abs(controls->thrustAxis)
                        : 0.0f;
                const sw::u32 count = std::min<sw::u32>(
                    state.count, static_cast<sw::u32>(definition->animations.size()));
                for (sw::u32 i = 0; i < count; ++i)
                {
                    if (definition->animations[i].trigger !=
                        sw::parts::AnimationTrigger::Throttle)
                    {
                        continue;
                    }
                    // ...and a shut-down engine's nozzle stays dark however
                    // far the throttle is pushed, which is the one place the
                    // two kinds of animation on a part have to agree.
                    const sw::f64 armed =
                        sw::parts::engineExhaustBlocked(m_world, entity)
                            ? 0.0
                            : sw::parts::animationGate(
                                  *definition, &state, sw::parts::AnimationGates::Thrust);
                    state.target[i] = throttle * static_cast<sw::f32>(armed);
                }
            });
    }

    // ------------------------------------------------------------------------
    // THE PILOT'S MENU
    // ------------------------------------------------------------------------

    // WHERE A THING IS DRAWN, WHICH IS NOT WHERE IT IS SIMULATED.
    //
    // Physics runs at 50 Hz and the screen does not, so every mesh is drawn at
    // a pose mixed between the last two ticks by the lane's alpha. The chase
    // camera has always done the same — and the part picker did not. In orbit
    // at nine and a half kilometres a second one physics step is a hundred and
    // ninety metres, so the sphere the click tested against sat up to a
    // hundred and ninety metres from the pixels the pilot was aiming at, drifting
    // in and out of alignment sixty times a second. A click that lands is then
    // a coincidence, which is exactly what "possible but extremely difficult"
    // describes.
    //
    // The menu's anchor had the same fault with a louder symptom: the part
    // could be computed as being BEHIND a camera that was looking straight at
    // it, and a part behind the camera closes the menu.
    sw::WorldVec3 StarWorksGame::renderPosition(sw::ecs::Entity entity) const
    {
        sw::ecs::World& world = const_cast<sw::ecs::World&>(m_world);
        const auto* transform = world.tryGetComponent<TransformComponent>(entity);
        if (transform == nullptr)
        {
            return sw::WorldVec3{0.0};
        }
        const auto* previous = world.tryGetComponent<PreviousTransformComponent>(entity);
        if (previous == nullptr || m_physicsLane == nullptr)
        {
            return transform->position;
        }
        return glm::mix(previous->position, transform->position,
                        static_cast<sw::f64>(m_physicsLane->alpha()));
    }

    sw::ecs::Entity StarWorksGame::pickPartUnderCursor(const sw::Camera& camera) const
    {
        sw::f32 cursorX = 0.0f;
        sw::f32 cursorY = 0.0f;
        if (!const_cast<StarWorksGame*>(this)->hudCursor(cursorX, cursorY))
        {
            return {};
        }
        // The cursor as a ray out of the eye. The projection is
        // camera-relative, so the inverse takes NDC straight to a direction.
        const sw::Mat4 inverse = glm::inverse(camera.viewProjectionCameraRelative());
        const sw::Vec4 far = inverse * sw::Vec4{cursorX, cursorY, 0.5f, 1.0f};
        if (!(std::abs(far.w) > 1.0e-9f))
        {
            return {};
        }
        return pickPartAlongRay(camera.position(), glm::normalize(sw::Vec3(far) / far.w));
    }

    sw::ecs::Entity StarWorksGame::pickPartAlongRay(const sw::WorldVec3& eye,
                                                    const sw::Vec3& direction) const
    {
        sw::ecs::Entity best{};
        sw::f32 bestDistance = 1.0e30f;
        sw::ecs::World& world = const_cast<sw::ecs::World&>(m_world);
        world.forEach<TransformComponent, sw::parts::PartComponent>(
            [&](sw::ecs::Entity entity, TransformComponent&,
                sw::parts::PartComponent& part) {
                const auto* definition = sw::parts::findDefinition(part.definitionId);
                if (definition == nullptr || definition->animations.empty())
                {
                    return; // only parts with something to click are clickable
                }
                // The DRAWN pose — see renderPosition. Testing the simulated
                // one aims the click at where the part will be in a fiftieth
                // of a second, which in orbit is a different postcode.
                const sw::Vec3 relative = sw::Vec3(renderPosition(entity) - eye);
                // THREE HUNDRED METRES, and that is a gameplay rule rather
                // than a rendering one: you may work a switch on a craft you
                // are flying beside, not on one in another orbit.
                if (glm::length(relative) > 300.0f)
                {
                    return;
                }
                // ONE RAY, AND WHATEVER IS IN ITS WAY. Not the exact collider
                // test — see rayEntersSphere for why that one is wrong twice
                // over — but the part's bounding sphere, which already covers
                // its deployed pose. Nearest along the ray wins, so a part in
                // front of another shadows it exactly as it looks like it
                // should.
                const sw::f32 entry = sw::parts::rayEntersSphere(
                    relative, direction,
                    std::max(sw::parts::partBoundsRadius(*definition), 0.75f));
                if (entry >= 0.0f && entry < bestDistance)
                {
                    bestDistance = entry;
                    best = entity;
                }
            });
        return best;
    }

    void StarWorksGame::collectPartMenu(const sw::Camera& camera)
    {
        // A QUICK CLICK OPENS THE MENU; HOLDING TURNS THE CAMERA. The right
        // button does both jobs and the only thing separating them is how long
        // it was down and how far the mouse went — see Input::isQuickClick,
        // which is where the two bounds and the reasoning live.
        //
        // THE FIRST VERSION USED SIX PIXELS OF TRAVEL AND NO TIME BOUND AT
        // ALL, and it never fired once: six pixels is inside the slop of an
        // ordinary click on an ordinary mouse, so every tap was read as a drag
        // and the feature looked as though it had not been written.
        if (input().wasMouseButtonPressed(sw::MouseButton::Right))
        {
            m_rightDragPixels = 0.0f;
            m_rightHeldSeconds = 0.0f;
        }
        if (input().isMouseButtonDown(sw::MouseButton::Right))
        {
            m_rightDragPixels += std::abs(input().mouseDeltaX()) +
                                 std::abs(input().mouseDeltaY());
            m_rightHeldSeconds += clock().deltaSeconds();
        }
        const bool clicked = input().wasMouseButtonReleased(sw::MouseButton::Right) &&
                             sw::Input::isQuickClick(m_rightHeldSeconds,
                                                     m_rightDragPixels);
        if (clicked)
        {
            const sw::ecs::Entity hit = pickPartUnderCursor(camera);
            if (hit.isNull())
            {
                // SAY SO. A click that finds nothing used to leave the screen
                // exactly as it was, which is indistinguishable from a click
                // that was never registered — and that is how a working pick
                // and a broken one look the same.
                m_menuPart = {};
                m_partMenuMissUntil = clock().totalSeconds() + 1.5;
            }
            else
            {
                m_menuPart = (hit == m_menuPart) ? sw::ecs::Entity{} : hit;
                m_partMenuMissUntil = 0.0;
            }
        }
        if (clock().totalSeconds() < m_partMenuMissUntil)
        {
            hudTextCentered("NOTHING TO OPERATE THERE", 0.0f, 0.34f, 0.030f,
                            sw::Vec4{0.85f, 0.72f, 0.45f, 0.95f});
        }
        if (m_menuPart.isNull())
        {
            return;
        }
        const auto* transform = m_world.tryGetComponent<TransformComponent>(m_menuPart);
        const auto* part = m_world.tryGetComponent<sw::parts::PartComponent>(m_menuPart);
        // THE STATE IS OPTIONAL NOW. A docking port has no animations at all,
        // and requiring one closed its menu the instant it opened — so the one
        // part in the game whose menu has something urgent on it was the one
        // part whose menu could not exist.
        const auto* state =
            m_world.tryGetComponent<sw::parts::PartAnimationComponent>(m_menuPart);
        if (transform == nullptr || part == nullptr)
        {
            m_menuPart = {};
            return;
        }
        const auto* definition = sw::parts::findDefinition(part->definitionId);
        if (definition == nullptr)
        {
            m_menuPart = {};
            return;
        }
        const bool docked =
            !sw::parts::dockingJointOf(m_world, m_menuPart).isNull();
        const auto* shell = m_world.tryGetComponent<sw::parts::FairingComponent>(m_menuPart);
        const bool canJettison =
            shell != nullptr && shell->closed != 0 && shell->jettisoned == 0;
        if (state == nullptr && !docked && !canJettison)
        {
            m_menuPart = {}; // nothing on this part can be operated
            return;
        }
        // Where the part is on screen. Behind the camera closes the menu:
        // a panel anchored to something you have turned away from would slide
        // to the wrong edge and stay there.
        const sw::Vec3 relative = sw::Vec3(renderPosition(m_menuPart) - camera.position());
        const sw::Vec4 clip =
            camera.viewProjectionCameraRelative() * sw::Vec4(relative, 1.0f);
        if (!(clip.w > 0.0f))
        {
            m_menuPart = {};
            return;
        }
        m_menuAnchor = {clip.x / clip.w, clip.y / clip.w};

        sw::f32 cursorX = 0.0f;
        sw::f32 cursorY = 0.0f;
        const bool cursorValid = hudCursor(cursorX, cursorY);

        constexpr sw::f32 kWidth = 0.30f;
        constexpr sw::f32 kRow = 0.052f;
        constexpr sw::f32 kPad = 0.010f;
        const sw::u32 animationRows =
            (state != nullptr)
                ? std::min<sw::u32>(state->count,
                                    static_cast<sw::u32>(definition->animations.size()))
                : 0u;
        const sw::u32 rows =
            animationRows + (docked ? 1u : 0u) + (canJettison ? 1u : 0u);
        const sw::f32 x0 = glm::clamp(m_menuAnchor.x + 0.03f, -0.98f, 0.98f - kWidth);
        const sw::f32 y0 =
            glm::clamp(m_menuAnchor.y - 0.02f, -0.98f,
                       0.98f - (kRow * static_cast<sw::f32>(rows + 1) + kPad));
        hudQuad(x0, y0, x0 + kWidth,
                y0 + kRow * static_cast<sw::f32>(rows + 1) + kPad,
                {0.06f, 0.08f, 0.11f, 0.92f});
        hudText(definition->name.empty() ? "PART" : definition->name, x0 + 0.012f,
                y0 + 0.012f, 0.030f, sw::Vec4{0.75f, 0.82f, 0.92f, 1.0f});

        // ONE ROW PER THING THE PILOT MAY DO, and the label is the VERB for
        // where it is going, not for where it is. "CLOSE" on an open panel is
        // the button you press to close it; a label that read "OPEN" while the
        // panel was open would be a status light wearing a button's clothes.
        for (sw::u32 i = 0; i < animationRows; ++i)
        {
            const sw::parts::PartAnimation& animation = definition->animations[i];
            if (animation.trigger != sw::parts::AnimationTrigger::Toggle)
            {
                continue; // the throttle drives this one; there is nothing to press
            }
            const bool open = state->target[i] > 0.5f;
            const bool moving = std::abs(state->phase[i] - state->target[i]) > 1.0e-3f;
            // AN ENGINE WITH A PART OVER ITS NOZZLE HAS NO SWITCH WORTH
            // PRESSING, and saying TURN ON to a pilot whose engine cannot fire
            // is the panel arguing with the physics. The row states the fact
            // instead, and goes back to being a switch the moment the thing in
            // the way is staged off.
            if (animation.gates == sw::parts::AnimationGates::Thrust &&
                sw::parts::engineExhaustBlocked(m_world, m_menuPart))
            {
                const sw::f32 blockedY =
                    y0 + kRow * static_cast<sw::f32>(i + 1) + kPad * 0.5f;
                hudQuad(x0 + 0.008f, blockedY, x0 + kWidth - 0.008f,
                        blockedY + kRow - 0.008f, {0.24f, 0.16f, 0.12f, 0.95f});
                hudText("EXHAUST BLOCKED", x0 + 0.020f, blockedY + 0.010f, 0.030f,
                        sw::Vec4{1.0f, 0.62f, 0.35f, 1.0f});
                continue;
            }
            const char* verb = "";
            switch (animation.verbs)
            {
            case sw::parts::AnimationVerbs::OnOff:
                verb = open ? "TURN OFF" : "TURN ON";
                break;
            case sw::parts::AnimationVerbs::ExtendRetract:
                verb = open ? "RETRACT" : "EXTEND";
                break;
            case sw::parts::AnimationVerbs::DeployStow:
                verb = open ? "STOW" : "DEPLOY";
                break;
            default:
                verb = open ? "CLOSE" : "OPEN";
                break;
            }
            const sw::f32 rowY = y0 + kRow * static_cast<sw::f32>(i + 1) + kPad * 0.5f;
            const bool hot = cursorValid && cursorX >= x0 + 0.008f &&
                             cursorX <= x0 + kWidth - 0.008f && cursorY >= rowY &&
                             cursorY <= rowY + kRow - 0.008f;
            hudQuad(x0 + 0.008f, rowY, x0 + kWidth - 0.008f, rowY + kRow - 0.008f,
                    hot ? sw::Vec4{0.22f, 0.30f, 0.40f, 0.95f}
                        : sw::Vec4{0.14f, 0.18f, 0.24f, 0.95f});
            hudText(std::format("{}{}", verb, moving ? " ..." : ""), x0 + 0.020f,
                    rowY + 0.010f, 0.030f,
                    moving ? sw::Vec4{0.95f, 0.80f, 0.45f, 1.0f}
                           : sw::Vec4{0.85f, 0.92f, 1.0f, 1.0f});
            m_hudButtons.push_back({x0 + 0.008f, rowY, x0 + kWidth - 0.008f,
                                    rowY + kRow - 0.008f, 900u + i});
        }

        // ---- and the one row a docked port always has --------------------
        //
        // On the port itself, not on a keyboard shortcut and not on the HUD:
        // a craft can carry several ports and only one of them is holding the
        // thing you want to let go of. Pressing the button on the ring you are
        // looking at cannot mean anything else.
        if (docked)
        {
            const sw::f32 rowY =
                y0 + kRow * static_cast<sw::f32>(animationRows + 1) + kPad * 0.5f;
            const bool hot = cursorValid && cursorX >= x0 + 0.008f &&
                             cursorX <= x0 + kWidth - 0.008f && cursorY >= rowY &&
                             cursorY <= rowY + kRow - 0.008f;
            hudQuad(x0 + 0.008f, rowY, x0 + kWidth - 0.008f, rowY + kRow - 0.008f,
                    hot ? sw::Vec4{0.42f, 0.24f, 0.24f, 0.95f}
                        : sw::Vec4{0.28f, 0.16f, 0.18f, 0.95f});
            hudText("UNDOCK", x0 + 0.020f, rowY + 0.010f, 0.030f,
                    sw::Vec4{1.0f, 0.82f, 0.72f, 1.0f});
            m_hudButtons.push_back({x0 + 0.008f, rowY, x0 + kWidth - 0.008f,
                                    rowY + kRow - 0.008f,
                                    900u + sw::ui::kHudPartAnimationSlots});
        }

        // ---- and the one a closed shroud always has ----------------------
        if (canJettison)
        {
            const sw::f32 rowY =
                y0 + kRow * static_cast<sw::f32>(rows) + kPad * 0.5f;
            const bool hot = cursorValid && cursorX >= x0 + 0.008f &&
                             cursorX <= x0 + kWidth - 0.008f && cursorY >= rowY &&
                             cursorY <= rowY + kRow - 0.008f;
            hudQuad(x0 + 0.008f, rowY, x0 + kWidth - 0.008f, rowY + kRow - 0.008f,
                    hot ? sw::Vec4{0.42f, 0.30f, 0.16f, 0.95f}
                        : sw::Vec4{0.28f, 0.20f, 0.12f, 0.95f});
            hudText("JETTISON", x0 + 0.020f, rowY + 0.010f, 0.030f,
                    sw::Vec4{1.0f, 0.88f, 0.66f, 1.0f});
            m_hudButtons.push_back({x0 + 0.008f, rowY, x0 + kWidth - 0.008f,
                                    rowY + kRow - 0.008f,
                                    901u + sw::ui::kHudPartAnimationSlots});
        }
    }

    void StarWorksGame::togglePartAnimation(sw::u32 index)
    {
        auto* state =
            m_world.tryGetComponent<sw::parts::PartAnimationComponent>(m_menuPart);
        if (state == nullptr || index >= state->count)
        {
            return;
        }
        // The TARGET flips; the phase walks there from wherever it is. A panel
        // caught halfway through opening closes again from halfway, which is
        // both what a mechanism does and the reason both numbers are stored.
        state->target[index] = (state->target[index] > 0.5f) ? 0.0f : 1.0f;
    }
} // namespace game
