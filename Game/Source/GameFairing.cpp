// ============================================================================
// GameFairing.cpp — F48: the shell the player draws.
//
// KSP's fairing tool, reproduced from its own rules rather than from memory:
//
//   * placing the base puts you straight into construct mode — you do not go
//     looking for a tool;
//   * the wall FOLLOWS THE CURSOR between clicks, and a left click plants a
//     ring where it is;
//   * a right click takes the last ring back off, and takes you out of the
//     mode when there is nothing left to undo;
//   * bringing the radius in to the axis turns the prompt from PLACE SECTION
//     into CLOSE FAIRING, and that click caps the nose and ends the job;
//   * the radius is bounded — a shell twice the base's own width is the widest
//     thing that base can hold up.
//
// What the cursor means is the one piece of arithmetic here worth naming. The
// shell is a solid of revolution, so a point on it is (height along the stack,
// radius from the axis) and nothing else; the cursor's ray is intersected with
// the plane that CONTAINS THE AXIS and turns to face the camera, which is what
// makes the wall track the mouse the same way from any angle you have orbited
// the hangar to.
// ============================================================================

#include "StarWorksGame.hpp"

#include "GameInternal.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <format>

namespace game
{
    namespace
    {
        /// The widest a shell may get, as a multiple of its base's rim. KSP
        /// bounds it at the base's diameter and so does this: past that the
        /// shroud is holding the rocket up rather than the other way round.
        constexpr sw::f32 kMaxRadiusFactor = 2.0f;
        /// A ring has to be above the one below it by at least this, or a
        /// double click leaves two rings at the same height and a band with
        /// no area.
        constexpr sw::f32 kMinRingStep = 0.12f;
        /// Below this radius the prompt becomes CLOSE FAIRING. Wide enough to
        /// hit with a mouse, narrow enough that the cap is a nose and not a
        /// flat lid.
        constexpr sw::f32 kCloseRadius = 0.22f;
        constexpr sw::Vec4 kShellColor{0.86f, 0.87f, 0.90f, 1.0f};
    } // namespace

    // ------------------------------------------------------------------------
    // Entering and leaving
    // ------------------------------------------------------------------------
    void StarWorksGame::beginFairing(sw::usize blueprintIndex)
    {
        if (blueprintIndex >= m_blueprint.size())
        {
            return;
        }
        const auto* definition =
            sw::parts::findDefinition(m_blueprint[blueprintIndex].definitionId);
        if (definition == nullptr || definition->type != sw::parts::PartType::Fairing)
        {
            return;
        }
        m_fairingDrawing = true;
        m_fairingIndex = blueprintIndex;
        m_fairingDraft = sw::parts::FairingComponent{};
        // RING ZERO IS THE BASE'S OWN RIM, and it is not a decision the player
        // makes: a shell that started anywhere else would either float above
        // its base or eat it.
        const sw::parts::AttachNode* top = nullptr;
        for (const sw::parts::AttachNode& node : definition->nodes)
        {
            if (top == nullptr || node.position.z < top->position.z)
            {
                top = &node; // -Z is up the stack
            }
        }
        const sw::f32 rimHeight = (top != nullptr) ? -top->position.z : 0.4f;
        const sw::f32 rimRadius = (top != nullptr) ? top->size : 1.25f;
        m_fairingDraft.rings[0] = {rimHeight, rimRadius};
        m_fairingDraft.ringCount = 1;
        m_fairingDraft.sides = 8;
        m_blueprint[blueprintIndex].fairing = m_fairingDraft;
        m_fairingPreviewDirty = true;
        SW_LOG_INFO("Game", "FAIRING: drawing on part {}", blueprintIndex);
    }

    void StarWorksGame::endFairing(bool keep)
    {
        if (m_fairingDrawing && m_fairingIndex < m_blueprint.size())
        {
            // AN UNCLOSED PROFILE IS A DRAFT AND NOTHING ELSE. It shields
            // nothing, weighs nothing and is not drawn in flight, so leaving
            // one behind by pressing escape cannot half-build a fairing.
            m_blueprint[m_fairingIndex].fairing =
                keep ? m_fairingDraft : sw::parts::FairingComponent{};
        }
        m_fairingDrawing = false;
        m_fairingDraft = {};
        m_fairingCursorValid = false;
        m_fairingPreviewDirty = true;
    }

    // ------------------------------------------------------------------------
    // WHERE THE CURSOR IS ON THE SHELL
    // ------------------------------------------------------------------------
    bool StarWorksGame::fairingCursor(sw::Vec2& outRing)
    {
        if (!m_fairingDrawing || m_fairingIndex >= m_blueprint.size())
        {
            return false;
        }
        const BlueprintPart& base = m_blueprint[m_fairingIndex];
        sw::Vec3 origin{};
        sw::Vec3 direction{};
        editorCursorRay(origin, direction);

        const sw::Vec3 axis = glm::normalize(base.localRotation * sw::Vec3{0, 0, -1});
        // The plane that contains the axis and turns to face the camera: its
        // normal is the view direction with the axis taken out of it.
        sw::Vec3 normal = direction - axis * glm::dot(direction, axis);
        if (glm::length(normal) < 1.0e-4f)
        {
            return false; // looking straight down the stack: no plane to draw on
        }
        normal = glm::normalize(normal);
        const sw::f32 denominator = glm::dot(direction, normal);
        if (std::abs(denominator) < 1.0e-5f)
        {
            return false;
        }
        const sw::f32 t = glm::dot(base.localPosition - origin, normal) / denominator;
        if (!(t > 0.0f))
        {
            return false; // the plane is behind the camera
        }
        const sw::Vec3 hit = origin + direction * t;
        const sw::Vec3 fromBase = hit - base.localPosition;
        const sw::f32 height = glm::dot(fromBase, axis);
        const sw::f32 radius = glm::length(fromBase - axis * height);

        const sw::Vec2 last = m_fairingDraft.rings[m_fairingDraft.ringCount - 1];
        const sw::f32 maxRadius = m_fairingDraft.rings[0].y * kMaxRadiusFactor;
        outRing = {std::max(height, last.x + kMinRingStep),
                   std::clamp(radius, 0.0f, maxRadius)};
        return true;
    }

    void StarWorksGame::updateFairing()
    {
        if (!m_fairingDrawing)
        {
            return;
        }
        if (m_fairingIndex >= m_blueprint.size())
        {
            endFairing(false);
            return;
        }
        m_fairingCursorValid = fairingCursor(m_fairingCursor);
        // A SCRIPTED CURSOR REPLACES THE RAY AND NOTHING ELSE. One ring a
        // frame, so a capture can be aimed at a half-drawn shell as easily as
        // at a finished one, and every ring still goes in through the click
        // handler the mouse calls.
        const bool scriptedRing = m_fairingScripted && !m_fairingScript.empty();
        if (scriptedRing)
        {
            const sw::Vec2 last = m_fairingDraft.rings[m_fairingDraft.ringCount - 1];
            const sw::f32 maxRadius = m_fairingDraft.rings[0].y * kMaxRadiusFactor;
            m_fairingCursor = {std::max(m_fairingScript.front().x, last.x + kMinRingStep),
                               std::clamp(m_fairingScript.front().y, 0.0f, maxRadius)};
            m_fairingScript.erase(m_fairingScript.begin());
            m_fairingCursorValid = true;
        }
        m_fairingCanClose =
            m_fairingCursorValid && m_fairingDraft.ringCount >= 2 &&
            m_fairingCursor.y <= kCloseRadius;
        // ONLY THE SCRIPTED RINGS ARE CLICKED IN. A script that runs out with
        // the nose still open leaves the tool exactly as a player who stopped
        // to think leaves it — which is the picture the half-drawn shell needs,
        // and clicking on past the end would fill the profile with rings the
        // hook never asked for.
        if (scriptedRing)
        {
            if (!fairingClick())
            {
                SW_LOG_ERROR("Game",
                             "SW_FAIRING: the click at {:.2f} m / {:.2f} m planted nothing",
                             static_cast<double>(m_fairingCursor.x),
                             static_cast<double>(m_fairingCursor.y));
            }
            if (!m_fairingDrawing)
            {
                m_fairingScripted = false; // the last ring closed it
                m_fairingScript.clear();
                return;
            }
        }

        if (input().wasKeyPressed(sw::KeyCode::Escape))
        {
            endFairing(false);
            return;
        }
        // A QUICK RIGHT CLICK UNDOES; HOLDING TURNS THE CAMERA. The right
        // button already does both jobs in the cockpit and the two are told
        // apart the same way here — how long it was down and how far it went.
        if (input().wasMouseButtonPressed(sw::MouseButton::Right))
        {
            m_rightDragPixels = 0.0f;
            m_rightHeldSeconds = 0.0f;
        }
        if (input().isMouseButtonDown(sw::MouseButton::Right))
        {
            m_rightDragPixels +=
                std::abs(input().mouseDeltaX()) + std::abs(input().mouseDeltaY());
            m_rightHeldSeconds += clock().deltaSeconds();
        }
        if (input().wasMouseButtonReleased(sw::MouseButton::Right) &&
            sw::Input::isQuickClick(m_rightHeldSeconds, m_rightDragPixels))
        {
            if (m_fairingDraft.ringCount > 1)
            {
                --m_fairingDraft.ringCount;
                m_fairingPreviewDirty = true;
            }
            else
            {
                endFairing(false); // nothing left to take back
            }
        }
    }

    // ------------------------------------------------------------------------
    // DRAWING ONE WITHOUT A MOUSE
    //
    // A capture has no cursor, and the tool is nothing BUT a cursor — so the
    // hook feeds the rings the ray would have produced into the same
    // m_fairingCursor and lets updateFairing() click them in. What it must not
    // do is write the profile into the component: that would photograph a
    // shape the tool might not be able to draw, which is the failure mode this
    // project has already paid for twice.
    // ------------------------------------------------------------------------
    void StarWorksGame::debugFairingScript(const char* spec)
    {
        if (spec == nullptr || !m_editorMode)
        {
            SW_LOG_ERROR("Game", "SW_FAIRING: not in the hangar — set SW_HANGAR too");
            return;
        }
        sw::usize base = m_blueprint.size();
        for (sw::usize i = 0; i < m_blueprint.size(); ++i)
        {
            const auto* definition =
                sw::parts::findDefinition(m_blueprint[i].definitionId);
            if (definition != nullptr &&
                definition->type == sw::parts::PartType::Fairing)
            {
                base = i;
                break;
            }
        }
        if (base == m_blueprint.size())
        {
            // LOUD, per the rule: "the design has no fairing base" and "the
            // tool is broken" produce the same empty picture.
            SW_LOG_ERROR("Game", "SW_FAIRING: no FR-1 on the deck ({} parts) — load a "
                                 "design that has one", m_blueprint.size());
            return;
        }

        // <height>:<radius>, in metres above the mounting face. A final radius
        // inside the closing threshold is what caps the nose, exactly as the
        // last click of a real one does.
        m_fairingScript.clear();
        // A bare `1` is "draw the usual one" — the same convention every other
        // hook here uses, and a height with no radius after it is a typo
        // rather than a profile.
        const char* cursor = (std::strchr(spec, ':') != nullptr) ? spec : "";
        while (*cursor != '\0')
        {
            char* after = nullptr;
            const sw::f32 height = std::strtof(cursor, &after);
            if (after == cursor)
            {
                break;
            }
            cursor = (*after == ':' || *after == ',') ? after + 1 : after;
            const sw::f32 radius = std::strtof(cursor, &after);
            cursor = (after == cursor) ? cursor : after;
            m_fairingScript.push_back({height, radius});
            while (*cursor == ',' || *cursor == ' ')
            {
                ++cursor;
            }
        }
        if (m_fairingScript.empty())
        {
            // The default shroud: a barrel and a nose, the shape the tool is
            // for. Written as rings so it goes in through the same clicks.
            m_fairingScript = {{1.4f, 1.25f}, {3.2f, 1.25f}, {4.6f, 0.7f}, {5.6f, 0.0f}};
        }
        m_fairingScripted = true;
        beginFairing(base); // the real entry: placing the base opens the tool
        SW_LOG_INFO("Game", "SW_FAIRING: drawing {} rings on part {}",
                    m_fairingScript.size(), base);
    }

    // ------------------------------------------------------------------------
    // THE CLICK THAT PLANTS A RING
    // ------------------------------------------------------------------------
    bool StarWorksGame::fairingClick()
    {
        if (!m_fairingDrawing || !m_fairingCursorValid)
        {
            return false;
        }
        if (m_fairingCanClose)
        {
            // THE NOSE. Closing is a ring of zero radius rather than a special
            // cap, so the lathe, the mass, the drag and the panels all keep
            // working off one list with no case for "the last one".
            if (m_fairingDraft.ringCount < sw::parts::FairingComponent::kMaxRings)
            {
                m_fairingDraft.rings[m_fairingDraft.ringCount] = {m_fairingCursor.x,
                                                                  0.0f};
                ++m_fairingDraft.ringCount;
            }
            else
            {
                m_fairingDraft.rings[m_fairingDraft.ringCount - 1].y = 0.0f;
            }
            m_fairingDraft.closed = 1;
            SW_LOG_INFO("Game", "FAIRING: closed, {} rings, {:.2f} m tall, {:.0f} kg",
                        m_fairingDraft.ringCount,
                        static_cast<double>(
                            m_fairingDraft.rings[m_fairingDraft.ringCount - 1].x),
                        sw::parts::fairingMassKg(m_fairingDraft));
            endFairing(true);
            return true;
        }
        if (m_fairingDraft.ringCount >= sw::parts::FairingComponent::kMaxRings)
        {
            return true; // full: the only thing left to do is close it
        }
        m_fairingDraft.rings[m_fairingDraft.ringCount] = m_fairingCursor;
        ++m_fairingDraft.ringCount;
        m_fairingPreviewDirty = true;
        return true;
    }

    // ------------------------------------------------------------------------
    // ONE MESH FOR EVERY SHELL ON THE DESIGN
    //
    // Rebuilt only when a profile changes, into two slots used in turn: a mesh
    // replaced the frame after it was drawn is the pipeline bubble the terrain
    // patch learnt about once, and this is the same lesson rather than a new
    // one. One combined mesh rather than one per fairing keeps that bookkeeping
    // to a single pair of slots however many shells a design carries.
    // ------------------------------------------------------------------------
    void StarWorksGame::rebuildFairingPreview()
    {
        m_fairingPreviewDirty = false;
        sw::MeshData combined;
        auto append = [&combined](const sw::parts::FairingComponent& fairing,
                                  const BlueprintPart& part, sw::f32 alpha) {
            const sw::MeshData shell = sw::parts::buildFairingMesh(
                fairing, {kShellColor.r, kShellColor.g, kShellColor.b, alpha});
            const sw::u32 base = static_cast<sw::u32>(combined.vertices.size());
            for (const sw::Vertex& vertex : shell.vertices)
            {
                sw::Vertex moved = vertex;
                moved.position = part.localPosition + part.localRotation * vertex.position;
                moved.normal = part.localRotation * vertex.normal;
                combined.vertices.push_back(moved);
            }
            for (const sw::u32 index : shell.indices)
            {
                combined.indices.push_back(base + index);
            }
        };

        for (sw::usize i = 0; i < m_blueprint.size(); ++i)
        {
            const BlueprintPart& part = m_blueprint[i];
            if (m_fairingDrawing && i == m_fairingIndex)
            {
                append(m_fairingDraft, part, 0.45f); // the draft, see-through
                continue;
            }
            if (sw::parts::fairingIsFlying(part.fairing))
            {
                append(part.fairing, part, 1.0f);
            }
        }

        m_fairingPreviewIndexCount = static_cast<sw::u32>(combined.indices.size());
        if (m_fairingPreviewIndexCount == 0)
        {
            return;
        }
        if (m_fairingPreviewSlots[0] == 0xFFFFFFFFu)
        {
            m_fairingPreviewSlots[0] = registerMesh(renderer().createMesh(combined));
            m_fairingPreviewSlots[1] = registerMesh(renderer().createMesh(combined));
            m_fairingPreviewSlot = 0;
            return;
        }
        m_fairingPreviewSlot ^= 1u;
        m_meshes[m_fairingPreviewSlots[m_fairingPreviewSlot]] =
            renderer().createMesh(combined);
    }

    // ------------------------------------------------------------------------
    // What it looks like while you are drawing it
    // ------------------------------------------------------------------------
    void StarWorksGame::collectFairingPreview(const sw::WorldVec3& cameraPosition,
                                              const sw::Quat& display)
    {
        if (m_fairingPreviewIndexCount > 0 && m_fairingPreviewSlots[0] != 0xFFFFFFFFu)
        {
            const sw::Vec3 relative = -sw::Vec3(cameraPosition);
            sw::DrawItem shell{};
            shell.mesh = &m_meshes[m_fairingPreviewSlots[m_fairingPreviewSlot]];
            shell.transform = glm::translate(sw::Mat4{1.0f}, relative) *
                              glm::mat4_cast(display);
            shell.boundsCenter = relative;
            shell.boundsRadius = 60.0f;
            shell.tint = {1.0f, 1.0f, 1.0f, 1.0f};
            shell.transparent = m_fairingDrawing;
            m_drawItems.push_back(shell);
        }
        if (!m_fairingDrawing || !m_fairingCursorValid ||
            m_fairingIndex >= m_blueprint.size())
        {
            return;
        }

        // THE WALL THAT FOLLOWS THE MOUSE: the segment from the last ring to
        // where the cursor is, drawn on both sides of the axis so it reads as
        // a profile rather than as a stray line.
        const BlueprintPart& base = m_blueprint[m_fairingIndex];
        const sw::Vec3 axis = glm::normalize(base.localRotation * sw::Vec3{0, 0, -1});
        sw::Vec3 out = base.localRotation * sw::Vec3{1, 0, 0};
        out = glm::normalize(out - axis * glm::dot(out, axis));
        const sw::Vec2 last = m_fairingDraft.rings[m_fairingDraft.ringCount - 1];
        const sw::Vec4 colour = m_fairingCanClose ? sw::Vec4{0.4f, 0.75f, 1.0f, 1.0f}
                                                  : sw::Vec4{0.45f, 1.0f, 0.55f, 1.0f};
        for (const sw::f32 side : {1.0f, -1.0f})
        {
            const sw::Vec3 from =
                base.localPosition + axis * last.x + out * (last.y * side);
            const sw::Vec3 to = base.localPosition + axis * m_fairingCursor.x +
                                out * (m_fairingCursor.y * side);
            const sw::Vec3 middle = (from + to) * 0.5f;
            const sw::Vec3 delta = to - from;
            const sw::f32 length = glm::length(delta);
            if (!(length > 1.0e-4f))
            {
                continue;
            }
            const sw::Vec3 forward = delta / length;
            sw::Vec3 reference{0.0f, 1.0f, 0.0f};
            if (std::abs(glm::dot(forward, reference)) > 0.99f)
            {
                reference = sw::Vec3{1.0f, 0.0f, 0.0f};
            }
            const sw::Vec3 right = glm::normalize(glm::cross(reference, forward));
            const sw::Vec3 up = glm::cross(forward, right);
            const sw::Vec3 relative =
                sw::Vec3(sw::WorldVec3(display * middle) - cameraPosition);
            sw::DrawItem line{};
            line.mesh = &m_meshes[m_orbitLineMeshIndex];
            line.transform =
                glm::translate(sw::Mat4{1.0f}, relative) *
                glm::mat4_cast(display * glm::quat_cast(sw::Mat3{right, up, forward})) *
                glm::scale(sw::Mat4{1.0f}, sw::Vec3{0.05f, 0.05f, length});
            line.boundsCenter = relative;
            line.boundsRadius = length * 0.6f;
            line.tint = {colour.r, colour.g, colour.b, 2.0f};
            m_drawItems.push_back(line);
        }
        // ...and a mark on every ring already placed, so the shape reads as a
        // sequence of decisions rather than as one curve.
        for (sw::u32 i = 0; i < m_fairingDraft.ringCount; ++i)
        {
            for (const sw::f32 side : {1.0f, -1.0f})
            {
                const sw::Vec3 point = base.localPosition +
                                       axis * m_fairingDraft.rings[i].x +
                                       out * (m_fairingDraft.rings[i].y * side);
                const sw::Vec3 relative =
                    sw::Vec3(sw::WorldVec3(display * point) - cameraPosition);
                sw::DrawItem mark{};
                mark.mesh = &m_meshes[m_markerMeshIndex];
                mark.transform = glm::translate(sw::Mat4{1.0f}, relative) *
                                 glm::scale(sw::Mat4{1.0f}, sw::Vec3{0.16f});
                mark.boundsCenter = relative;
                mark.boundsRadius = 0.2f;
                mark.tint = {0.95f, 0.85f, 0.35f, 2.0f};
                m_drawItems.push_back(mark);
            }
        }
    }

    // ------------------------------------------------------------------------
    // IN FLIGHT: one mesh per instance, and the release
    // ------------------------------------------------------------------------
    void StarWorksGame::buildFairingShellMesh(sw::ecs::Entity fairingPart,
                                              const sw::parts::FairingComponent& fairing)
    {
        auto* mesh = m_world.tryGetComponent<MeshComponent>(fairingPart);
        const auto* part = m_world.tryGetComponent<sw::parts::PartComponent>(fairingPart);
        if (part == nullptr)
        {
            // WHICH PART IS THIS? The shell is welded onto the base's own mesh,
            // so the base's definition has to be readable — and a caller that
            // builds the shell before the part knows what it is gets told,
            // rather than getting a rocket with no nose on it.
            SW_LOG_ERROR("Game", "FAIRING: entity {} has a shell but no part",
                         fairingPart.index);
            return;
        }
        const auto* definition = sw::parts::findDefinition(part->definitionId);
        if (definition == nullptr)
        {
            return;
        }
        // BASE AND SHELL IN ONE BUFFER. The base's own mesh is shared by every
        // instance of the part and cannot be touched; the shell is this
        // instance's alone. Welding them costs one mesh per fairing built and
        // saves a second entity that would have to be moved, culled, saved and
        // thrown away in step with the first.
        sw::MeshData combined = sw::parts::buildPartMeshGroup(*definition, -1);
        const sw::MeshData shell = sw::parts::buildFairingMesh(fairing, kShellColor);
        const sw::u32 base = static_cast<sw::u32>(combined.vertices.size());
        combined.vertices.insert(combined.vertices.end(), shell.vertices.begin(),
                                 shell.vertices.end());
        for (const sw::u32 index : shell.indices)
        {
            combined.indices.push_back(base + index);
        }
        const sw::u32 slot = registerMesh(renderer().createMesh(combined));
        if (mesh != nullptr)
        {
            mesh->meshIndex = slot;
        }
        else
        {
            m_world.addComponent(fairingPart, MeshComponent{slot});
        }
        if (auto* bounds = m_world.tryGetComponent<BoundsComponent>(fairingPart))
        {
            const sw::Vec2 tip = fairing.rings[fairing.ringCount - 1];
            bounds->localRadius =
                std::max(bounds->localRadius, std::max(tip.x, fairing.rings[0].y) + 0.5f);
        }
    }

    void StarWorksGame::jettisonFairing(sw::ecs::Entity fairingPart)
    {
        auto* fairing = m_world.tryGetComponent<sw::parts::FairingComponent>(fairingPart);
        const auto* part = m_world.tryGetComponent<sw::parts::PartComponent>(fairingPart);
        if (fairing == nullptr || part == nullptr || fairing->jettisoned != 0 ||
            fairing->closed == 0)
        {
            return;
        }
        const auto* transform = m_world.tryGetComponent<TransformComponent>(fairingPart);
        const auto* vesselBody =
            m_world.tryGetComponent<sw::phys::DynamicBodyComponent>(part->vessel);
        if (transform == nullptr)
        {
            return;
        }
        const sw::parts::FairingComponent shell = *fairing;
        // The shell stops existing for the drag, the shielding and the mesh in
        // the same instant — the payload feels the wind from here.
        fairing->jettisoned = 1;
        if (auto* mesh = m_world.tryGetComponent<MeshComponent>(fairingPart))
        {
            if (const auto found = m_partMeshIds.find(part->definitionId);
                found != m_partMeshIds.end())
            {
                mesh->meshIndex = found->second;
            }
        }

        const sw::u32 sides = std::clamp(shell.sides,
                                         sw::parts::FairingComponent::kMinSides,
                                         sw::parts::FairingComponent::kMaxSides);
        const sw::f64 panelMass =
            std::max(sw::parts::fairingMassKg(shell) / static_cast<sw::f64>(sides), 1.0);
        constexpr sw::f32 kTwoPi = 6.28318530717958647692f;
        for (sw::u32 side = 0; side < sides; ++side)
        {
            const sw::MeshData panel =
                sw::parts::buildFairingPanelMesh(shell, side, kShellColor);
            if (panel.indices.empty())
            {
                continue;
            }
            const sw::ecs::Entity debris = m_world.createEntity();
            TransformComponent pose{};
            pose.position = transform->position;
            pose.rotation = transform->rotation;
            m_world.addComponent(debris, pose);
            m_world.addComponent(debris,
                                 PreviousTransformComponent{pose.position, pose.rotation});
            m_world.addComponent(debris, BoundsComponent{
                                             shell.rings[shell.ringCount - 1].x + 1.0f});
            m_world.addComponent(debris,
                                 MeshComponent{registerMesh(renderer().createMesh(panel))});
            // OUTWARD AND A LITTLE UP, about the panel's own middle: a real
            // shroud hinges off its base and is thrown clear of the payload,
            // and a panel that fell straight down the stack would hit it.
            const sw::f32 angle =
                kTwoPi * (static_cast<sw::f32>(side) + 0.5f) / static_cast<sw::f32>(sides);
            const sw::Vec3 outward{std::cos(angle), std::sin(angle), 0.0f};
            sw::phys::DynamicBodyComponent body{};
            body.mass = panelMass;
            body.velocity = (vesselBody != nullptr) ? vesselBody->velocity
                                                    : sw::WorldVec3{0.0};
            body.velocity += sw::WorldVec3(pose.rotation *
                                           (outward * 6.0f + sw::Vec3{0, 0, -1.5f}));
            body.angularVelocity = outward * 1.2f;
            body.ballisticFactor = 0.02;
            m_world.addComponent(debris, body);
            m_world.addComponent(debris, DebrisComponent{});
        }
        SW_LOG_INFO("Game", "FAIRING: jettisoned into {} panels", sides);
    }

    // ------------------------------------------------------------------------
    // The one line of instructions the tool needs
    // ------------------------------------------------------------------------
    void StarWorksGame::collectFairingUi()
    {
        if (!m_fairingDrawing)
        {
            return;
        }
        const sw::f32 mass = static_cast<sw::f32>(
            sw::parts::fairingMassKg(m_fairingDraft));
        // LOW ON THE SCREEN, because the thing the player is looking at is the
        // shell they are drawing and the first version of this put the prompt
        // straight through the middle of it.
        hudTextCentered(m_fairingCanClose ? "CLOSE FAIRING" : "PLACE SECTION", 0.0f,
                        0.60f, 0.048f,
                        m_fairingCanClose ? sw::Vec4{0.45f, 0.8f, 1.0f, 1.0f}
                                          : sw::Vec4{0.5f, 1.0f, 0.6f, 1.0f});
        hudTextCentered(
            std::format("{} RINGS   {:.2f} M   {:.0f} KG", m_fairingDraft.ringCount,
                        static_cast<double>(m_fairingCursor.x),
                        static_cast<double>(mass)),
            0.0f, 0.66f, 0.030f, hud::kTextDim);
        hudTextCentered("LEFT CLICK PLACES   RIGHT CLICK UNDOES   ESC CANCELS", 0.0f,
                        0.71f, 0.026f, hud::kTextDim);
    }
} // namespace game
