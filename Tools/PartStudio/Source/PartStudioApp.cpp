#include "PartStudioApp.hpp"

#include <algorithm>
#include <cmath>
#include <format>

namespace studio
{
    namespace
    {
        /// `PrimitiveFactory::makeCube(1)` is a cube ONE METRE ACROSS, so
        /// its half extent is 0.5. To draw a box of half extents `h` with
        /// it you scale by `2h`. The constant exists so that fact is
        /// written down once instead of being re-derived — wrongly, as it
        /// turned out — at each call site.
        constexpr sw::f32 kUnitBoxScale = 2.0f;

        /// Three letters for the node list; the inspector spells it out.
        [[nodiscard]] std::string_view nodeTypeShortName(sw::parts::NodeType type)
        {
            switch (type)
            {
            case sw::parts::NodeType::Stack: return "STK";
            case sw::parts::NodeType::Radial: return "RAD";
            case sw::parts::NodeType::ConveyorIn: return "CIN";
            case sw::parts::NodeType::ConveyorOut: return "COU";
            default: return "?";
            }
        }
    } // namespace

    namespace
    {
        constexpr const char* kGlyphCharset =
            "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.,-+/%:";
        constexpr sw::f32 kSnapCoarse = 0.05f;  // meters
        constexpr sw::f32 kSnapFine = 0.005f;
        constexpr sw::f32 kAngleCoarse = 5.0f;  // degrees
        constexpr sw::f32 kAngleFine = 1.0f;

        constexpr const char* kKindNames[5] = {"BOX", "CYL", "CONE", "SPH", "TUBE"};

        constexpr sw::Vec3 kPalette[12] = {
            {0.88f, 0.88f, 0.90f}, {0.65f, 0.66f, 0.70f}, {0.35f, 0.35f, 0.38f},
            {0.15f, 0.15f, 0.17f}, {0.70f, 0.15f, 0.12f}, {0.85f, 0.45f, 0.12f},
            {0.85f, 0.70f, 0.15f}, {0.20f, 0.60f, 0.30f}, {0.30f, 0.85f, 0.80f},
            {0.10f, 0.25f, 0.60f}, {0.08f, 0.12f, 0.40f}, {0.45f, 0.30f, 0.20f}};

        [[nodiscard]] sw::f32 snapValue(sw::f32 value, sw::f32 step)
        {
            return std::round(value / step) * step;
        }
    } // namespace

    PartStudioApp::PartStudioApp(const sw::ApplicationConfig& config)
        : sw::Application(config)
    {
        m_camera.setPerspective(sw::math::toRadians(50.0f), 0.05f, 400.0f);
        m_glyphMeshIndex.fill(0xFFFFFFFFu);
        for (const char* c = kGlyphCharset; *c != '\0'; ++c)
        {
            const sw::MeshData glyph = sw::ui::buildGlyphMesh(*c);
            if (!glyph.empty())
            {
                m_glyphMeshIndex[static_cast<sw::usize>(*c)] =
                    registerMesh(renderer().createMesh(glyph));
            }
        }
        m_floorMeshIndex = registerMesh(renderer().createMesh(
            sw::PrimitiveFactory::makeGridPlane(12.0f, 24, {0.16f, 0.24f, 0.3f, 1.0f})));
        m_markerMeshIndex = registerMesh(renderer().createMesh(
            sw::PrimitiveFactory::makeOctahedron(1.0f, {1.0f, 1.0f, 1.0f, 1.0f})));
        // A 1 m CUBE — so its HALF extent is 0.5, not 1. Anything drawing a
        // box of known half extents with it must therefore scale by the FULL
        // extent (2 * halfExtents), which is what `kUnitBoxScale` below is
        // for. Getting this wrong drew every collider and every hitbox in
        // this editor at half its real size for months, and hulls were then
        // authored to LOOK right against a lie — the suit came out twice as
        // tall as the body inside it.
        m_unitBoxMeshIndex = registerMesh(renderer().createMesh(
            sw::PrimitiveFactory::makeCube(1.0f, {1.0f, 1.0f, 1.0f, 1.0f})));
        m_shapeMeshBase = m_meshes.size();

        m_directory = sw::FileSystem::executableDirectory() / "Assets" / "Parts";
        refreshFileList();
        if (m_files.empty())
        {
            newPart();
        }
        else
        {
            loadCurrentFile();
        }
        SW_LOG_INFO("Studio", "Part Studio ready - {} part files in '{}'",
                    m_files.size(), m_directory.string());
    }

    sw::u32 PartStudioApp::registerMesh(sw::Mesh mesh)
    {
        m_meshes.push_back(std::move(mesh));
        return static_cast<sw::u32>(m_meshes.size() - 1);
    }

    // ========================= files =========================================
    void PartStudioApp::refreshFileList()
    {
        m_files.clear();
        std::error_code errorCode;
        std::filesystem::create_directories(m_directory, errorCode);
        for (const auto& entry :
             std::filesystem::directory_iterator(m_directory, errorCode))
        {
            if (entry.is_regular_file() && entry.path().extension() == ".swpart")
            {
                m_files.push_back(entry.path());
            }
        }
        std::sort(m_files.begin(), m_files.end());
        if (m_fileIndex >= m_files.size())
        {
            m_fileIndex = 0;
        }
    }

    void PartStudioApp::loadCurrentFile()
    {
        if (m_files.empty())
        {
            return;
        }
        sw::parts::PartDefinition loaded{};
        if (sw::parts::loadPartFile(m_files[m_fileIndex], loaded))
        {
            m_part = std::move(loaded);
            m_untitled = false;
            m_selectedShape = m_part.shapes.empty() ? -1 : 0;
            m_selectedNode = -1;
            m_selectedHitbox = -1;
            m_mode = Mode::Idle;
            markGeometryDirty();
            setStatus(std::format("LOADED {}", m_files[m_fileIndex].filename().string()));
        }
        else
        {
            setStatus(std::format("LOAD FAILED {}", m_files[m_fileIndex].filename().string()));
        }
    }

    void PartStudioApp::saveCurrentFile()
    {
        std::filesystem::path path;
        if (m_untitled || m_files.empty())
        {
            path = m_directory / std::format("part_{}.swpart", m_part.id);
        }
        else
        {
            path = m_files[m_fileIndex];
        }
        if (sw::parts::savePartFile(m_part, path))
        {
            // The executable-side Assets/ is a BUILD MIRROR: also write the
            // SOURCE tree copy when there is one, so a rebuild never
            // clobbers the user's work.
            //
            // The root is ASKED FOR, not walked to by hand. The previous
            // version climbed a fixed five levels looking for any directory
            // called Assets/Parts — from a project at G:\StarWorks that
            // reaches G:\ itself, and would write into G:\Assets\Parts if
            // such a folder happened to exist. projectRoot() requires
            // CMakeLists.txt AND Assets together, and returns nothing for a
            // packaged build, where there is correctly no source to update.
            const std::filesystem::path root = sw::FileSystem::projectRoot();
            if (!root.empty())
            {
                const std::filesystem::path source = root / "Assets" / "Parts";
                std::error_code errorCode;
                if (std::filesystem::is_directory(source, errorCode) &&
                    !std::filesystem::equivalent(source, m_directory, errorCode))
                {
                    (void)sw::parts::savePartFile(m_part, source / path.filename());
                }
            }
            setStatus(std::format("SAVED {}", path.filename().string()));
            const bool wasUntitled = m_untitled;
            m_untitled = false;
            refreshFileList();
            if (wasUntitled)
            {
                for (sw::usize i = 0; i < m_files.size(); ++i)
                {
                    if (m_files[i] == path)
                    {
                        m_fileIndex = i;
                    }
                }
            }
        }
        else
        {
            setStatus("SAVE FAILED");
        }
    }

    void PartStudioApp::newPart()
    {
        sw::u32 nextId = 100; // user parts start above the built-in range
        for (const auto& file : m_files)
        {
            sw::parts::PartDefinition probe{};
            if (sw::parts::loadPartFile(file, probe))
            {
                nextId = std::max(nextId, probe.id + 1);
            }
        }
        m_part = sw::parts::PartDefinition{};
        m_part.id = nextId;
        m_part.type = sw::parts::PartType::Structural;
        m_part.name = std::format("NEW PART {}", nextId);
        sw::parts::PartShape shape{};
        shape.kind = sw::parts::ShapeKind::Box;
        shape.size = {0.5f, 0.5f, 0.5f};
        shape.color = {0.7f, 0.72f, 0.76f};
        shape.collider = true;
        m_part.shapes.push_back(shape);
        m_part.nodes.push_back(
            {"top", {0.0f, 0.0f, -0.5f}, {0.0f, 0.0f, -1.0f}, sw::parts::NodeType::Stack, 0.5f});
        m_part.nodes.push_back(
            {"bottom", {0.0f, 0.0f, 0.5f}, {0.0f, 0.0f, 1.0f}, sw::parts::NodeType::Stack, 0.5f});
        m_untitled = true;
        m_selectedShape = 0;
        m_selectedNode = -1;
        m_mode = Mode::Idle;
        markGeometryDirty();
        setStatus(std::format("NEW PART ID {}", nextId));
    }

    // ========================= selection helpers =============================
    bool PartStudioApp::hasShapeSelection() const
    {
        return m_selectedShape >= 0 &&
               m_selectedShape < static_cast<sw::i32>(m_part.shapes.size());
    }

    bool PartStudioApp::hasNodeSelection() const
    {
        return m_selectedNode >= 0 &&
               m_selectedNode < static_cast<sw::i32>(m_part.nodes.size());
    }

    bool PartStudioApp::hasHitboxSelection() const
    {
        return m_selectedHitbox >= 0 &&
               m_selectedHitbox < static_cast<sw::i32>(m_part.hitboxes.size());
    }

    // ========================= editing actions ===============================
    // THE HULL IS AUTHORED NOW. A new box starts around the part's own
    // bounds rather than at a unit cube in the middle of it: the first thing
    // you want after pressing the button is almost always "that, but tighter".
    void PartStudioApp::addHitbox()
    {
        sw::parts::HitBox box{};
        constexpr sw::f32 kHuge = 1.0e9f;
        sw::Vec3 low{kHuge, kHuge, kHuge};
        sw::Vec3 high{-kHuge, -kHuge, -kHuge};
        sw::parts::expandPartHullBounds(m_part, sw::Vec3{0.0f},
                                        sw::Quat{1.0f, 0.0f, 0.0f, 0.0f}, low, high);
        if (low.x <= high.x)
        {
            box.center = (low + high) * 0.5f;
            box.halfExtents = glm::max((high - low) * 0.5f, sw::Vec3{0.05f});
        }
        m_part.hitboxes.push_back(box);
        m_selectedHitbox = static_cast<sw::i32>(m_part.hitboxes.size() - 1);
        m_selectedShape = -1;
        m_selectedNode = -1;
        m_showHitboxes = true;
        setStatus("HITBOX ADDED");
    }

    // One box per collider shape — a hull good enough to EDIT from rather
    // than to draw from scratch. It is the same call the shipped catalogue
    // was seeded with, so a generated hull and an authored one are the same
    // kind of object.
    void PartStudioApp::fitHitboxes()
    {
        m_part.hitboxes = sw::parts::hitboxesFromColliders(m_part);
        m_selectedHitbox = m_part.hitboxes.empty() ? -1 : 0;
        m_selectedShape = -1;
        m_selectedNode = -1;
        m_showHitboxes = true;
        setStatus(std::format("HULL FITTED: {} BOXES", m_part.hitboxes.size()));
    }

    void PartStudioApp::addShape(sw::parts::ShapeKind kind)
    {
        sw::parts::PartShape shape{};
        shape.kind = kind;
        switch (kind)
        {
        case sw::parts::ShapeKind::Box: shape.size = {0.5f, 0.5f, 0.5f}; break;
        case sw::parts::ShapeKind::Cylinder: shape.size = {0.5f, 0.5f, 0.0f}; break;
        case sw::parts::ShapeKind::Cone: shape.size = {0.5f, 0.5f, 0.25f}; break;
        case sw::parts::ShapeKind::Sphere: shape.size = {0.5f, 0.5f, 0.5f}; break;
        case sw::parts::ShapeKind::Tube: shape.size = {0.5f, 0.2f, 0.35f}; break;
        }
        shape.color = {0.7f, 0.72f, 0.76f};
        shape.collider = true;
        m_part.shapes.push_back(shape);
        m_selectedShape = static_cast<sw::i32>(m_part.shapes.size() - 1);
        m_selectedNode = -1;
        m_selectedHitbox = -1;
        markGeometryDirty();
    }

    void PartStudioApp::addNode(sw::parts::NodeType type)
    {
        sw::parts::AttachNode node{};
        node.type = type;
        if (sw::parts::isConveyorNode(type))
        {
            // A conveyor PORT: the mouth a belt docks against. Direction is
            // the contract — an OUT ships, an IN receives — so the two start
            // on opposite faces and the author moves them onto the geometry
            // with SNAP SURF like any other node.
            const bool out = type == sw::parts::NodeType::ConveyorOut;
            node.name = out ? "out" : "in";
            node.position = {0.0f, 0.6f, out ? -1.0f : 1.0f};
            node.direction = {0.0f, 0.0f, out ? -1.0f : 1.0f};
            node.size = 0.55f;
        }
        else if (type == sw::parts::NodeType::Stack)
        {
            node.name = std::format("stack{}", m_part.nodes.size());
            node.position = {0.0f, 0.0f, -1.0f};
            node.direction = {0.0f, 0.0f, -1.0f};
            node.size = 0.6f;
        }
        else
        {
            node.name = std::format("radial{}", m_part.nodes.size());
            node.position = {1.0f, 0.0f, 0.0f};
            node.direction = {1.0f, 0.0f, 0.0f};
            node.size = 0.4f;
        }
        m_part.nodes.push_back(std::move(node));
        m_selectedNode = static_cast<sw::i32>(m_part.nodes.size() - 1);
        m_selectedShape = -1;
        snapSelectedNodeToSurface();
    }

    void PartStudioApp::snapSelectedNodeToSurface()
    {
        if (!hasNodeSelection())
        {
            return;
        }
        sw::parts::AttachNode& node = m_part.nodes[static_cast<sw::usize>(m_selectedNode)];
        const sw::f32 reach = sw::parts::partBoundsRadius(m_part) * 2.0f + 1.0f;
        const sw::Vec3 origin = node.position + node.direction * reach;
        sw::parts::PartRayHit hit{};
        if (sw::parts::raycastPart(m_part, origin, -node.direction, reach * 2.0f, hit))
        {
            node.position = origin - node.direction * hit.t;
            setStatus("NODE SNAPPED TO SURFACE");
        }
        else
        {
            setStatus("NO SURFACE ALONG NODE AXIS");
        }
    }

    // ========================= modal manipulation ============================
    void PartStudioApp::beginMode(Mode mode)
    {
        if (hasShapeSelection())
        {
            m_shapeBackup = m_part.shapes[static_cast<sw::usize>(m_selectedShape)];
        }
        else if (hasNodeSelection())
        {
            if (mode == Mode::Rotate)
            {
                return; // nodes have no orientation to rotate; use X/Y/Z keys
            }
            m_nodeBackup = m_part.nodes[static_cast<sw::usize>(m_selectedNode)];
        }
        else if (hasHitboxSelection())
        {
            if (mode == Mode::Rotate)
            {
                return; // an AABB has no orientation, by definition
            }
            m_hitboxBackup = m_part.hitboxes[static_cast<sw::usize>(m_selectedHitbox)];
        }
        else
        {
            return;
        }
        m_mode = mode;
        m_modeAxis = -1;
        m_modalAccumX = 0.0f;
        m_modalAccumY = 0.0f;
    }

    void PartStudioApp::applyMode()
    {
        m_mode = Mode::Idle;
        setStatus("APPLIED");
    }

    void PartStudioApp::cancelMode()
    {
        if (hasShapeSelection())
        {
            m_part.shapes[static_cast<sw::usize>(m_selectedShape)] = m_shapeBackup;
        }
        else if (hasNodeSelection())
        {
            m_part.nodes[static_cast<sw::usize>(m_selectedNode)] = m_nodeBackup;
        }
        else if (hasHitboxSelection())
        {
            m_part.hitboxes[static_cast<sw::usize>(m_selectedHitbox)] = m_hitboxBackup;
        }
        m_mode = Mode::Idle;
        markGeometryDirty();
        setStatus("CANCELLED");
    }

    void PartStudioApp::updateModal()
    {
        // Axis constraint toggles.
        for (sw::i32 axis = 0; axis < 3; ++axis)
        {
            const sw::KeyCode keys[3] = {sw::KeyCode::X, sw::KeyCode::Y, sw::KeyCode::Z};
            if (input().wasKeyPressed(keys[axis]))
            {
                m_modeAxis = (m_modeAxis == axis) ? -1 : axis;
                m_modalAccumX = 0.0f;
                m_modalAccumY = 0.0f;
                // Restore the backup so the constraint starts clean.
                if (hasShapeSelection())
                {
                    m_part.shapes[static_cast<sw::usize>(m_selectedShape)] = m_shapeBackup;
                }
                else if (hasNodeSelection())
                {
                    m_part.nodes[static_cast<sw::usize>(m_selectedNode)] = m_nodeBackup;
                }
            }
        }
        m_modalAccumX += input().mouseDeltaX();
        m_modalAccumY += input().mouseDeltaY();

        const bool fine = input().isKeyDown(sw::KeyCode::LeftShift) ||
                          input().isKeyDown(sw::KeyCode::RightShift);
        const sw::f32 positionSnap = fine ? kSnapFine : kSnapCoarse;
        const sw::f32 angleSnap = fine ? kAngleFine : kAngleCoarse;
        const sw::f32 unitsPerPixel = worldPerPixel();

        const sw::Vec3 axes[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        const sw::Vec3 right = m_camera.right();
        const sw::Vec3 up = m_camera.up();

        // The manipulated position/size/rotation, rebuilt from the backup
        // each frame (stable snapping, exact cancel).
        if (m_mode == Mode::Grab)
        {
            sw::Vec3 delta{0.0f};
            if (m_modeAxis < 0)
            {
                delta = (right * m_modalAccumX - up * m_modalAccumY) * unitsPerPixel;
            }
            else
            {
                const sw::Vec3 axis = axes[m_modeAxis];
                // Mouse motion projected on the axis's SCREEN direction.
                sw::Vec2 screenAxis{glm::dot(axis, right), -glm::dot(axis, up)};
                const sw::f32 length = glm::length(screenAxis);
                screenAxis = length > 1.0e-4f ? screenAxis / length : sw::Vec2{1.0f, 0.0f};
                const sw::f32 travel =
                    (m_modalAccumX * screenAxis.x + m_modalAccumY * screenAxis.y) *
                    unitsPerPixel;
                delta = axis * travel;
            }
            if (hasShapeSelection())
            {
                sw::parts::PartShape& shape =
                    m_part.shapes[static_cast<sw::usize>(m_selectedShape)];
                shape.position = m_shapeBackup.position + delta;
                for (sw::i32 axis = 0; axis < 3; ++axis)
                {
                    shape.position[axis] = snapValue(shape.position[axis], positionSnap);
                }
            }
            else if (hasNodeSelection())
            {
                sw::parts::AttachNode& node =
                    m_part.nodes[static_cast<sw::usize>(m_selectedNode)];
                node.position = m_nodeBackup.position + delta;
                for (sw::i32 axis = 0; axis < 3; ++axis)
                {
                    node.position[axis] = snapValue(node.position[axis], positionSnap);
                }
            }
            else if (hasHitboxSelection())
            {
                sw::parts::HitBox& box =
                    m_part.hitboxes[static_cast<sw::usize>(m_selectedHitbox)];
                box.center = m_hitboxBackup.center + delta;
                for (sw::i32 axis = 0; axis < 3; ++axis)
                {
                    box.center[axis] = snapValue(box.center[axis], positionSnap);
                }
            }
            markGeometryDirty();
        }
        else if (m_mode == Mode::Rotate && hasShapeSelection())
        {
            const sw::i32 axis = m_modeAxis < 0 ? 2 : m_modeAxis;
            const sw::f32 angle =
                snapValue(m_modalAccumX * 0.4f, angleSnap);
            sw::parts::PartShape& shape =
                m_part.shapes[static_cast<sw::usize>(m_selectedShape)];
            shape.rotationDeg = m_shapeBackup.rotationDeg;
            shape.rotationDeg[axis] = m_shapeBackup.rotationDeg[axis] + angle;
            markGeometryDirty();
        }
        else if (m_mode == Mode::Scale)
        {
            const sw::f32 factor = std::exp(m_modalAccumX * 0.005f);
            if (hasShapeSelection())
            {
                sw::parts::PartShape& shape =
                    m_part.shapes[static_cast<sw::usize>(m_selectedShape)];
                shape.size = m_shapeBackup.size;
                if (m_modeAxis < 0)
                {
                    shape.size = glm::max(m_shapeBackup.size * factor, sw::Vec3{0.0f});
                }
                else
                {
                    shape.size[m_modeAxis] =
                        std::max(m_shapeBackup.size[m_modeAxis] * factor, 0.0f);
                }
                // Snap dimensions too: parts line up on the same grid.
                for (sw::i32 axis = 0; axis < 3; ++axis)
                {
                    shape.size[axis] =
                        std::max(snapValue(shape.size[axis], kSnapFine), 0.0f);
                }
            }
            else if (hasNodeSelection())
            {
                sw::parts::AttachNode& node =
                    m_part.nodes[static_cast<sw::usize>(m_selectedNode)];
                node.size = std::clamp(m_nodeBackup.size * factor, 0.05f, 3.0f);
            }
            else if (hasHitboxSelection())
            {
                sw::parts::HitBox& box =
                    m_part.hitboxes[static_cast<sw::usize>(m_selectedHitbox)];
                box.halfExtents = m_hitboxBackup.halfExtents;
                if (m_modeAxis < 0)
                {
                    box.halfExtents = m_hitboxBackup.halfExtents * factor;
                }
                else
                {
                    box.halfExtents[m_modeAxis] =
                        m_hitboxBackup.halfExtents[m_modeAxis] * factor;
                }
                for (sw::i32 axis = 0; axis < 3; ++axis)
                {
                    box.halfExtents[axis] =
                        std::max(snapValue(box.halfExtents[axis], kSnapFine), 0.02f);
                }
            }
            markGeometryDirty();
        }

        if (input().wasMouseButtonPressed(sw::MouseButton::Left) ||
            input().wasKeyPressed(sw::KeyCode::Enter))
        {
            applyMode();
        }
        else if (input().wasKeyPressed(sw::KeyCode::Escape) ||
                 input().wasMouseButtonPressed(sw::MouseButton::Right))
        {
            cancelMode();
        }
    }

    // ========================= camera / picking ==============================
    void PartStudioApp::updateCameraControls()
    {
        if (m_mode == Mode::Idle && input().isMouseButtonDown(sw::MouseButton::Right))
        {
            m_yaw -= input().mouseDeltaX() * 0.005f;
            m_pitch = std::clamp(m_pitch - input().mouseDeltaY() * 0.005f, -1.35f, 1.45f);
        }
        if (const sw::f32 scroll = input().scrollDeltaY(); scroll != 0.0f)
        {
            m_distance = std::clamp(m_distance * std::pow(1.15f, -scroll), 1.2f, 60.0f);
        }
        const sw::f32 cosPitch = std::cos(m_pitch);
        const sw::Vec3 offset{cosPitch * std::sin(m_yaw) * m_distance,
                              std::sin(m_pitch) * m_distance,
                              cosPitch * std::cos(m_yaw) * m_distance};
        m_camera.setPosition(sw::WorldVec3(offset));
        const sw::Vec3 forward = glm::normalize(-offset);
        const sw::Vec3 right = glm::normalize(glm::cross(forward, sw::Vec3{0, 1, 0}));
        const sw::Vec3 up = glm::cross(right, forward);
        m_camera.setOrientation(glm::quat_cast(sw::Mat3{right, up, -forward}));
        m_camera.setAspectRatio(renderer().aspectRatio());
    }

    sw::f32 PartStudioApp::worldPerPixel()
    {
        sw::u32 width = 0;
        sw::u32 height = 0;
        window().framebufferSize(width, height);
        const sw::f32 spanY = 2.0f * m_distance * std::tan(m_camera.verticalFov() * 0.5f);
        return height > 0 ? spanY / static_cast<sw::f32>(height) : 0.01f;
    }

    void PartStudioApp::cursorRay(sw::Vec3& outOrigin, sw::Vec3& outDirection)
    {
        sw::u32 width = 0;
        sw::u32 height = 0;
        window().framebufferSize(width, height);
        const sw::f32 ndcX =
            input().mouseX() / static_cast<sw::f32>(std::max(width, 1u)) * 2.0f - 1.0f;
        const sw::f32 ndcY =
            input().mouseY() / static_cast<sw::f32>(std::max(height, 1u)) * 2.0f - 1.0f;

        // Unproject two depths through the inverse view-projection: works
        // for any convention (reverse-Z included).
        const sw::Mat4 inverse = glm::inverse(m_camera.viewProjectionCameraRelative());
        const sw::Vec4 nearPoint = inverse * sw::Vec4{ndcX, ndcY, 0.9f, 1.0f};
        const sw::Vec4 farPoint = inverse * sw::Vec4{ndcX, ndcY, 0.1f, 1.0f};
        const sw::Vec3 a = sw::Vec3(nearPoint) / nearPoint.w;
        const sw::Vec3 b = sw::Vec3(farPoint) / farPoint.w;
        outOrigin = sw::Vec3(m_camera.position()); // part lives at the origin
        outDirection = glm::normalize(b - a);
    }

    void PartStudioApp::pickAtCursor()
    {
        sw::Vec3 origin{};
        sw::Vec3 direction{};
        cursorRay(origin, direction);

        // Nodes first (they are small and often sit ON shape surfaces).
        sw::f32 bestT = 1.0e30f;
        sw::i32 bestNode = -1;
        for (sw::usize i = 0; i < m_part.nodes.size(); ++i)
        {
            const sw::Vec3 toNode = m_part.nodes[i].position - origin;
            const sw::f32 along = glm::dot(toNode, direction);
            if (along <= 0.0f)
            {
                continue;
            }
            const sw::f32 distance = glm::length(toNode - direction * along);
            const sw::f32 pickRadius =
                std::max(0.1f, m_part.nodes[i].size * 0.22f) + worldPerPixel() * 4.0f;
            if (distance < pickRadius && along < bestT)
            {
                bestT = along;
                bestNode = static_cast<sw::i32>(i);
            }
        }
        if (bestNode >= 0)
        {
            m_selectedNode = bestNode;
            m_selectedShape = -1;
            return;
        }

        // Shapes: exact ray cast against EVERY shape (visible or not).
        sw::i32 bestShape = -1;
        bestT = 1.0e30f;
        for (sw::usize i = 0; i < m_part.shapes.size(); ++i)
        {
            sw::parts::PartDefinition probe{};
            probe.shapes.push_back(m_part.shapes[i]);
            probe.shapes[0].visible = true;
            probe.shapes[0].collider = false;
            sw::parts::PartRayHit hit{};
            if (sw::parts::raycastPart(probe, origin, direction, 500.0f, hit) &&
                hit.t < bestT)
            {
                bestT = hit.t;
                bestShape = static_cast<sw::i32>(i);
            }
        }
        m_selectedShape = bestShape;
        if (bestShape >= 0)
        {
            m_selectedNode = -1;
            m_selectedHitbox = -1;
        }
    }

    // ========================= frame update ==================================
    void PartStudioApp::onUpdate(sw::f32 deltaSeconds)
    {
        m_statusAge += deltaSeconds;
        updateCameraControls();

        if (m_mode != Mode::Idle)
        {
            updateModal();
            return; // modal owns the mouse and the letter keys
        }

        // ---- keyboard shortcuts ------------------------------------------------
        if (input().wasKeyPressed(sw::KeyCode::G)) { beginMode(Mode::Grab); }
        if (input().wasKeyPressed(sw::KeyCode::R)) { beginMode(Mode::Rotate); }
        if (input().wasKeyPressed(sw::KeyCode::S)) { beginMode(Mode::Scale); }
        if (input().wasKeyPressed(sw::KeyCode::K)) { m_showColliders = !m_showColliders; }
        if (input().wasKeyPressed(sw::KeyCode::H)) { m_showHitboxes = !m_showHitboxes; }
        if (input().wasKeyPressed(sw::KeyCode::Delete))
        {
            if (hasShapeSelection() && m_part.shapes.size() > 1)
            {
                m_part.shapes.erase(m_part.shapes.begin() + m_selectedShape);
                m_selectedShape = -1;
                markGeometryDirty();
            }
            else if (hasNodeSelection())
            {
                m_part.nodes.erase(m_part.nodes.begin() + m_selectedNode);
                m_selectedNode = -1;
            }
            else if (hasHitboxSelection())
            {
                m_part.hitboxes.erase(m_part.hitboxes.begin() + m_selectedHitbox);
                m_selectedHitbox = -1;
            }
        }
        // Node direction: X/Y/Z aligns, pressing the same axis again flips.
        if (hasNodeSelection() && !hasHitboxSelection())
        {
            sw::parts::AttachNode& node =
                m_part.nodes[static_cast<sw::usize>(m_selectedNode)];
            const sw::KeyCode keys[3] = {sw::KeyCode::X, sw::KeyCode::Y, sw::KeyCode::Z};
            for (sw::i32 axis = 0; axis < 3; ++axis)
            {
                if (input().wasKeyPressed(keys[axis]))
                {
                    sw::Vec3 direction{0.0f};
                    direction[axis] = 1.0f;
                    node.direction = (glm::dot(node.direction, direction) > 0.99f)
                                         ? -direction
                                         : direction;
                }
            }
        }

        handleClicks();
    }

    void PartStudioApp::handleClicks()
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
        for (const HudButton& button : m_buttons)
        {
            if (ndcX < button.x0 || ndcX > button.x1 || ndcY < button.y0 ||
                ndcY > button.y1)
            {
                continue;
            }
            const sw::u32 id = button.id;
            if (id == 200) { newPart(); }
            else if (id == 201 && !m_files.empty())
            {
                m_fileIndex = (m_fileIndex + m_files.size() - 1) % m_files.size();
                loadCurrentFile();
            }
            else if (id == 202 && !m_files.empty())
            {
                m_fileIndex = (m_fileIndex + 1) % m_files.size();
                loadCurrentFile();
            }
            else if (id == 203) { saveCurrentFile(); }
            else if (id >= 210 && id < 215)
            {
                addShape(static_cast<sw::parts::ShapeKind>(id - 210));
            }
            else if (id == 220 && hasShapeSelection())
            {
                m_part.shapes.push_back(
                    m_part.shapes[static_cast<sw::usize>(m_selectedShape)]);
                m_selectedShape = static_cast<sw::i32>(m_part.shapes.size() - 1);
                markGeometryDirty();
                setStatus("DUPLICATED");
            }
            else if (id == 221 && hasShapeSelection() && m_part.shapes.size() > 1)
            {
                m_part.shapes.erase(m_part.shapes.begin() + m_selectedShape);
                m_selectedShape = -1;
                markGeometryDirty();
            }
            else if (id == 222 && hasShapeSelection())
            {
                auto& shape = m_part.shapes[static_cast<sw::usize>(m_selectedShape)];
                shape.visible = !shape.visible;
                markGeometryDirty();
            }
            else if (id == 223 && hasShapeSelection())
            {
                auto& shape = m_part.shapes[static_cast<sw::usize>(m_selectedShape)];
                shape.collider = !shape.collider;
                markGeometryDirty();
            }
            else if ((id == 224 || id == 225) && hasShapeSelection())
            {
                auto& shape = m_part.shapes[static_cast<sw::usize>(m_selectedShape)];
                shape.emissive = std::clamp(
                    shape.emissive + (id == 224 ? 0.1f : -0.1f), 0.0f, 1.0f);
                markGeometryDirty();
            }
            else if ((id == 226 || id == 227) && hasShapeSelection())
            {
                auto& shape = m_part.shapes[static_cast<sw::usize>(m_selectedShape)];
                const sw::i32 delta = (id == 226) ? 4 : -4;
                shape.segments = static_cast<sw::u32>(std::clamp(
                    static_cast<sw::i32>(shape.segments) + delta, 3, 64));
                markGeometryDirty();
            }
            else if (id == 230) { addNode(sw::parts::NodeType::Stack); }
            else if (id == 231) { addNode(sw::parts::NodeType::Radial); }
            else if (id == 235) { addNode(sw::parts::NodeType::ConveyorIn); }
            else if (id == 236) { addNode(sw::parts::NodeType::ConveyorOut); }
            else if (id == 232) { snapSelectedNodeToSurface(); markGeometryDirty(); }
            else if (id == 233 && hasNodeSelection())
            {
                // Cycle through every type rather than toggling two: the
                // conveyor ports are node types now, and a two-way switch
                // could not reach them.
                auto& node = m_part.nodes[static_cast<sw::usize>(m_selectedNode)];
                node.type = static_cast<sw::parts::NodeType>(
                    (static_cast<sw::u8>(node.type) + 1) %
                    static_cast<sw::u8>(sw::parts::NodeType::Count));
            }
            else if (id == 234 && hasNodeSelection())
            {
                m_part.nodes.erase(m_part.nodes.begin() + m_selectedNode);
                m_selectedNode = -1;
            }
            else if (id >= 240 && id < 252 && hasShapeSelection())
            {
                m_part.shapes[static_cast<sw::usize>(m_selectedShape)].color =
                    kPalette[id - 240];
                markGeometryDirty();
            }
            else if (id >= 260 && id < 266 && hasShapeSelection())
            {
                auto& color = m_part.shapes[static_cast<sw::usize>(m_selectedShape)].color;
                const sw::i32 channel = static_cast<sw::i32>(id - 260) / 2;
                const sw::f32 delta = ((id - 260) % 2 == 0) ? 0.05f : -0.05f;
                color[channel] = std::clamp(color[channel] + delta, 0.0f, 1.0f);
                markGeometryDirty();
            }
            else if (id == 270) { addHitbox(); }
            else if (id == 271) { fitHitboxes(); }
            else if (id == 272)
            {
                if (hasHitboxSelection())
                {
                    m_part.hitboxes.erase(m_part.hitboxes.begin() + m_selectedHitbox);
                    m_selectedHitbox = -1;
                    setStatus("HITBOX DELETED");
                }
            }
            else if (id == 273) { m_showHitboxes = !m_showHitboxes; }
            else if (id >= 500 && id < 500 + m_part.hitboxes.size())
            {
                m_selectedHitbox = static_cast<sw::i32>(id - 500);
                m_selectedShape = -1;
                m_selectedNode = -1;
            }
            else if (id >= 300 && id < 300 + m_part.shapes.size())
            {
                m_selectedShape = static_cast<sw::i32>(id - 300);
                m_selectedNode = -1;
                m_selectedHitbox = -1;
            }
            else if (id >= 400 && id < 400 + m_part.nodes.size())
            {
                m_selectedNode = static_cast<sw::i32>(id - 400);
                m_selectedShape = -1;
                m_selectedHitbox = -1;
            }
            return; // click consumed by the UI
        }
        pickAtCursor(); // fell through: 3D selection
    }

    // ========================= rendering ======================================
    void PartStudioApp::rebuildShapeMeshes()
    {
        renderer().waitIdle(); // slots are re-created in place
        for (sw::usize i = 0; i < m_part.shapes.size(); ++i)
        {
            sw::MeshData data = sw::parts::buildShapeMesh(m_part.shapes[i]);
            if (data.empty())
            {
                // Never leave a hole: a degenerate shape renders as a tiny box.
                data = sw::PrimitiveFactory::makeCube(0.02f, {1.0f, 0.2f, 0.2f, 1.0f});
            }
            const sw::usize slot = m_shapeMeshBase + i;
            if (slot < m_meshes.size())
            {
                m_meshes[slot] = renderer().createMesh(data);
            }
            else
            {
                registerMesh(renderer().createMesh(data));
            }
        }
        m_shapeMeshCount = m_part.shapes.size();
        m_geometryDirty = false;
    }

    void PartStudioApp::onRender()
    {
        if (m_geometryDirty)
        {
            rebuildShapeMeshes();
        }

        renderer().setSunPosition({50.0f, 80.0f, 30.0f});
        renderer().setShadowSpheres({});
        m_drawItems.clear();
        collectScene();
        collectUi();
        renderer().renderFrame(m_camera, m_drawItems);
    }

    void PartStudioApp::collectScene()
    {
        const sw::Vec3 cameraPosition = sw::Vec3(m_camera.position());
        const auto push = [&](sw::u32 meshIndex, const sw::Vec3& position,
                              const sw::Mat4& local, const sw::Vec4& tint,
                              bool transparent, sw::f32 boundsRadius) {
            sw::DrawItem item{};
            item.mesh = &m_meshes[meshIndex];
            item.transform =
                glm::translate(sw::Mat4{1.0f}, position - cameraPosition) * local;
            item.boundsCenter = position - cameraPosition;
            item.boundsRadius = boundsRadius;
            item.tint = tint;
            item.transparent = transparent;
            m_drawItems.push_back(item);
        };

        // Floor + world axes (X red, Y green, Z blue — the STACK axis).
        push(m_floorMeshIndex, {0.0f, -3.0f, 0.0f}, sw::Mat4{1.0f},
             {1.0f, 1.0f, 1.0f, 1.0f}, false, 20.0f);
        const sw::Vec3 axisColors[3] = {{0.9f, 0.25f, 0.2f}, {0.2f, 0.8f, 0.3f},
                                        {0.25f, 0.5f, 1.0f}};
        for (sw::i32 axis = 0; axis < 3; ++axis)
        {
            sw::Vec3 scale{0.012f};
            scale[axis] = 1.6f;
            sw::Vec3 center{0.0f};
            center[axis] = 1.6f;
            push(m_unitBoxMeshIndex, center, glm::scale(sw::Mat4{1.0f}, scale),
                 sw::Vec4{axisColors[axis], 2.0f}, false, 2.0f);
        }

        // Shapes.
        for (sw::usize i = 0; i < m_shapeMeshCount && i < m_part.shapes.size(); ++i)
        {
            const sw::parts::PartShape& shape = m_part.shapes[i];
            const bool selected = static_cast<sw::i32>(i) == m_selectedShape;
            if (shape.visible)
            {
                const sw::Vec4 tint =
                    selected ? sw::Vec4{1.35f, 1.3f, 0.95f, 1.0f} : sw::Vec4{1.0f};
                push(static_cast<sw::u32>(m_shapeMeshBase + i), {0.0f, 0.0f, 0.0f},
                     sw::Mat4{1.0f}, tint, false,
                     sw::parts::partBoundsRadius(m_part) + 0.5f);
            }
            // Collision overlay: the OBB stand-in the game validates with.
            if (shape.collider && (m_showColliders || !shape.visible))
            {
                sw::Vec3 halfExtents = shape.size;
                switch (shape.kind)
                {
                case sw::parts::ShapeKind::Cylinder:
                case sw::parts::ShapeKind::Tube:
                    halfExtents = {shape.size.x, shape.size.x, shape.size.y};
                    break;
                case sw::parts::ShapeKind::Cone:
                {
                    const sw::f32 radius = std::max(shape.size.x, shape.size.z);
                    halfExtents = {radius, radius, shape.size.y};
                    break;
                }
                default:
                    break;
                }
                const sw::Mat4 local =
                    glm::mat4_cast(sw::parts::shapeRotation(shape)) *
                    glm::scale(sw::Mat4{1.0f},
                               glm::max(halfExtents, sw::Vec3{0.01f}) * kUnitBoxScale);
                push(m_unitBoxMeshIndex, shape.position, local,
                     {1.0f, 0.55f, 0.15f, selected ? 0.4f : 0.22f}, true,
                     glm::length(halfExtents) + 0.2f);
            }
        }

        // THE HULL. Drawn in a colour nothing else uses, because the whole
        // point of looking at it is to see where it does NOT match the model
        // — a hitbox that reads as part of the geometry is a hitbox you
        // stop checking.
        if (m_showHitboxes)
        {
            for (sw::usize i = 0; i < m_part.hitboxes.size(); ++i)
            {
                const sw::parts::HitBox& box = m_part.hitboxes[i];
                const bool selected = static_cast<sw::i32>(i) == m_selectedHitbox;
                const sw::Vec3 halfExtents = glm::max(box.halfExtents, sw::Vec3{0.01f});
                // The SAME arithmetic the game's F2 overlay uses, so a box
                // drawn here is the box you bump into there.
                const sw::Mat4 local =
                    glm::scale(sw::Mat4{1.0f}, halfExtents * kUnitBoxScale);
                push(m_unitBoxMeshIndex, box.center, local,
                     {0.25f, 0.95f, 0.85f, selected ? 0.42f : 0.18f}, true,
                     glm::length(halfExtents) + 0.2f);
            }
        }

        // Attach nodes: octahedron + direction spike.
        for (sw::usize i = 0; i < m_part.nodes.size(); ++i)
        {
            const sw::parts::AttachNode& node = m_part.nodes[i];
            const bool selected = static_cast<sw::i32>(i) == m_selectedNode;
            // Green stack, blue radial, amber conveyor IN, orange OUT: the
            // port's DIRECTION is the thing you have to get right, so the
            // two ends of a belt never share a colour.
            sw::Vec4 color{0.3f, 0.8f, 1.0f, 2.0f};
            switch (node.type)
            {
            case sw::parts::NodeType::Stack: color = {0.3f, 1.0f, 0.5f, 2.0f}; break;
            case sw::parts::NodeType::ConveyorIn: color = {1.0f, 0.82f, 0.30f, 2.0f}; break;
            case sw::parts::NodeType::ConveyorOut: color = {1.0f, 0.45f, 0.15f, 2.0f}; break;
            default: break;
            }
            if (selected) { color = {1.0f, 1.0f, 1.0f, 2.0f}; }
            const sw::f32 radius = std::max(0.09f, node.size * 0.18f);
            push(m_markerMeshIndex, node.position,
                 glm::scale(sw::Mat4{1.0f}, sw::Vec3{radius}), color, false, radius * 2);

            // Direction spike: a thin box from the node outward.
            const sw::Vec3 spikeCenter = node.position + node.direction * 0.3f;
            const sw::Vec3 forward = node.direction;
            const sw::Vec3 reference =
                std::abs(forward.z) > 0.9f ? sw::Vec3{0, 1, 0} : sw::Vec3{0, 0, 1};
            const sw::Vec3 side = glm::normalize(glm::cross(reference, forward));
            const sw::Vec3 up = glm::cross(forward, side);
            const sw::Mat4 basis{sw::Vec4{side, 0.0f}, sw::Vec4{up, 0.0f},
                                 sw::Vec4{forward, 0.0f}, sw::Vec4{0, 0, 0, 1}};
            push(m_unitBoxMeshIndex, spikeCenter,
                 basis * glm::scale(sw::Mat4{1.0f}, sw::Vec3{0.02f, 0.02f, 0.3f}),
                 color, false, 0.6f);
        }
    }

    void PartStudioApp::hudText(std::string_view text, sw::f32 x, sw::f32 y,
                                sw::f32 heightNdc, const sw::Vec4& color)
    {
        const sw::f32 aspect = renderer().aspectRatio();
        const sw::f32 scaleX = (5.0f / 7.0f) * heightNdc / aspect;
        const sw::f32 advance = sw::ui::kGlyphAdvance * heightNdc / aspect;
        sw::f32 penX = x;
        for (const char character : text)
        {
            const auto index = static_cast<sw::usize>(static_cast<unsigned char>(
                std::toupper(static_cast<unsigned char>(character))));
            const sw::u32 meshIndex =
                index < m_glyphMeshIndex.size() ? m_glyphMeshIndex[index] : 0xFFFFFFFFu;
            if (meshIndex != 0xFFFFFFFFu)
            {
                sw::DrawItem item{};
                item.mesh = &m_meshes[meshIndex];
                item.transform = glm::translate(sw::Mat4{1.0f}, {penX, y, 0.0f}) *
                                 glm::scale(sw::Mat4{1.0f}, {scaleX, heightNdc, 1.0f});
                item.screenSpace = true;
                item.tint = color;
                m_drawItems.push_back(item);
            }
            penX += advance;
        }
    }

    void PartStudioApp::collectUi()
    {
        m_buttons.clear();
        const sw::Vec4 textColor{0.8f, 0.9f, 1.0f, 0.95f};
        const sw::Vec4 dimColor{0.55f, 0.65f, 0.75f, 0.85f};

        const auto panel = [&](sw::f32 x0, sw::f32 y0, sw::f32 x1, sw::f32 y1,
                               const sw::Vec4& color) {
            sw::DrawItem item{};
            item.mesh = &m_meshes[m_unitBoxMeshIndex];
            item.transform =
                glm::translate(sw::Mat4{1.0f},
                               {(x0 + x1) * 0.5f, (y0 + y1) * 0.5f, 0.0f}) *
                glm::scale(sw::Mat4{1.0f},
                           {(x1 - x0) * 0.5f, (y1 - y0) * 0.5f, 0.001f});
            item.screenSpace = true;
            item.tint = color;
            m_drawItems.push_back(item);
        };
        const auto button = [&](sw::f32 x0, sw::f32 y0, sw::f32 width,
                                std::string_view label, sw::u32 id, bool highlighted) {
            const sw::f32 height = 0.058f;
            panel(x0, y0, x0 + width, y0 + height,
                  highlighted ? sw::Vec4{0.2f, 0.5f, 0.3f, 0.85f}
                              : sw::Vec4{0.14f, 0.2f, 0.28f, 0.7f});
            hudText(label, x0 + 0.012f, y0 + 0.014f, 0.028f,
                    highlighted ? sw::Vec4{0.9f, 1.0f, 0.9f, 1.0f} : textColor);
            m_buttons.push_back({x0, y0, x0 + width, y0 + height, id});
        };

        // ---- header --------------------------------------------------------------
        const std::string fileName =
            m_untitled ? std::format("UNTITLED (ID {})", m_part.id)
                       : (m_files.empty() ? "-" : m_files[m_fileIndex].filename().string());
        hudText(std::format("PART STUDIO - {}", fileName), -0.97f, -0.97f, 0.045f,
                {1.0f, 0.85f, 0.35f, 1.0f});
        hudText(std::format("{}  ID {}  DRY {:.0f} KG  SHAPES {}  NODES {}  HULL {}",
                            m_part.name, m_part.id, m_part.dryMassKg,
                            m_part.shapes.size(), m_part.nodes.size(),
                            m_part.hitboxes.empty()
                                ? std::string("AUTO")
                                : std::format("{}", m_part.hitboxes.size())),
                -0.97f, -0.90f, 0.032f, textColor);
        hudText("RCLICK ORBIT  WHEEL ZOOM  LCLICK SELECT  G/R/S +X/Y/Z  SHIFT FINE  "
                "K COLLIDERS  H HULL  DEL REMOVE",
                -0.97f, -0.845f, 0.026f, dimColor);

        // ---- file row --------------------------------------------------------------
        button(-0.97f, 0.90f, 0.14f, "NEW", 200, false);
        button(-0.81f, 0.90f, 0.10f, "PREV", 201, false);
        button(-0.69f, 0.90f, 0.10f, "NEXT", 202, false);
        button(-0.57f, 0.90f, 0.14f, "SAVE", 203, true);

        // ---- add-shape row ---------------------------------------------------------
        sw::f32 x = -0.38f;
        for (sw::u32 kind = 0; kind < 5; ++kind)
        {
            button(x, 0.90f, 0.115f, std::format("+{}", kKindNames[kind]), 210 + kind,
                   false);
            x += 0.13f;
        }
        button(x, 0.90f, 0.10f, "DUP", 220, false);
        x += 0.115f;
        button(x, 0.90f, 0.10f, "DEL", 221, false);

        // ---- node row ----------------------------------------------------------------
        button(-0.66f, 0.82f, 0.13f, "+STACK", 230, false);
        button(-0.51f, 0.82f, 0.14f, "+RADIAL", 231, false);
        button(-0.35f, 0.82f, 0.15f, "+CONV IN", 235, false);
        button(-0.18f, 0.82f, 0.16f, "+CONV OUT", 236, false);
        button(0.00f, 0.82f, 0.16f, "SNAP SURF", 232, false);
        button(0.18f, 0.82f, 0.15f, "ND TYPE", 233, false);
        button(0.35f, 0.82f, 0.14f, "ND DEL", 234, false);

        // ---- hull row ----------------------------------------------------------------
        // The HITBOX is what the part bumps into: the ground it rests on,
        // the parts it may not overlap in the VAB, the box a building's
        // footprint is checked against. It is edited here rather than
        // inferred from the model, so a hull can be fixed without redrawing
        // anything and a forty-segment nose cone still collides as one box.
        button(-0.66f, 0.74f, 0.15f, "+HITBOX", 270, false);
        button(-0.49f, 0.74f, 0.15f, "FIT HULL", 271, false);
        button(-0.32f, 0.74f, 0.13f, "HB DEL", 272, false);
        button(-0.17f, 0.74f, 0.16f, "H HULL VIEW", 273, m_showHitboxes);

        // ---- shape list (left column) --------------------------------------------------
        sw::f32 y = -0.76f;
        for (sw::usize i = 0; i < m_part.shapes.size() && i < 18; ++i)
        {
            const sw::parts::PartShape& shape = m_part.shapes[i];
            const bool selected = static_cast<sw::i32>(i) == m_selectedShape;
            panel(-0.98f, y, -0.70f, y + 0.052f,
                  selected ? sw::Vec4{0.2f, 0.5f, 0.3f, 0.8f}
                           : sw::Vec4{0.12f, 0.17f, 0.24f, 0.55f});
            hudText(std::format("{} {}{}{}", kKindNames[static_cast<sw::usize>(shape.kind)],
                                shape.visible ? "V" : "-", shape.collider ? "C" : "-",
                                shape.emissive > 0.0f ? "E" : ""),
                    -0.965f, y + 0.013f, 0.026f,
                    selected ? sw::Vec4{0.9f, 1.0f, 0.9f, 1.0f} : textColor);
            m_buttons.push_back({-0.98f, y, -0.70f, y + 0.052f,
                                 300u + static_cast<sw::u32>(i)});
            y += 0.058f;
        }
        // Node list under the shapes.
        y += 0.02f;
        for (sw::usize i = 0; i < m_part.nodes.size() && i < 8; ++i)
        {
            const sw::parts::AttachNode& node = m_part.nodes[i];
            const bool selected = static_cast<sw::i32>(i) == m_selectedNode;
            panel(-0.98f, y, -0.70f, y + 0.052f,
                  selected ? sw::Vec4{0.45f, 0.42f, 0.15f, 0.85f}
                           : sw::Vec4{0.16f, 0.16f, 0.2f, 0.55f});
            hudText(std::format("ND {} {}", node.name,
                                nodeTypeShortName(node.type)),
                    -0.965f, y + 0.013f, 0.026f,
                    selected ? sw::Vec4{1.0f, 1.0f, 0.85f, 1.0f} : textColor);
            m_buttons.push_back({-0.98f, y, -0.70f, y + 0.052f,
                                 400u + static_cast<sw::u32>(i)});
            y += 0.058f;
        }
        // Hull list under the nodes.
        y += 0.02f;
        for (sw::usize i = 0; i < m_part.hitboxes.size() && i < 8; ++i)
        {
            const sw::parts::HitBox& box = m_part.hitboxes[i];
            const bool selected = static_cast<sw::i32>(i) == m_selectedHitbox;
            panel(-0.98f, y, -0.70f, y + 0.052f,
                  selected ? sw::Vec4{0.16f, 0.45f, 0.48f, 0.85f}
                           : sw::Vec4{0.12f, 0.2f, 0.22f, 0.55f});
            hudText(std::format("HB {:.1f}x{:.1f}x{:.1f}", box.halfExtents.x * 2.0f,
                                box.halfExtents.y * 2.0f, box.halfExtents.z * 2.0f),
                    -0.965f, y + 0.013f, 0.026f,
                    selected ? sw::Vec4{0.85f, 1.0f, 1.0f, 1.0f} : textColor);
            m_buttons.push_back({-0.98f, y, -0.70f, y + 0.052f,
                                 500u + static_cast<sw::u32>(i)});
            y += 0.058f;
        }

        // ---- selection details + color tools (right column) ------------------------------
        if (hasHitboxSelection())
        {
            const sw::parts::HitBox& box =
                m_part.hitboxes[static_cast<sw::usize>(m_selectedHitbox)];
            hudText(std::format("HITBOX  CENTRE {:.2f} {:.2f} {:.2f}", box.center.x,
                                box.center.y, box.center.z),
                    0.30f, -0.97f, 0.028f, textColor);
            hudText(std::format("SIZE {:.2f} {:.2f} {:.2f}   G MOVE   S SIZE",
                                box.halfExtents.x * 2.0f, box.halfExtents.y * 2.0f,
                                box.halfExtents.z * 2.0f),
                    0.30f, -0.93f, 0.028f, textColor);
        }
        else if (hasShapeSelection())
        {
            const sw::parts::PartShape& shape =
                m_part.shapes[static_cast<sw::usize>(m_selectedShape)];
            hudText(std::format("{}  POS {:.2f} {:.2f} {:.2f}",
                                kKindNames[static_cast<sw::usize>(shape.kind)],
                                shape.position.x, shape.position.y, shape.position.z),
                    0.30f, -0.97f, 0.028f, textColor);
            hudText(std::format("SIZE {:.2f} {:.2f} {:.2f}  ROT {:.0f} {:.0f} {:.0f}",
                                shape.size.x, shape.size.y, shape.size.z,
                                shape.rotationDeg.x, shape.rotationDeg.y,
                                shape.rotationDeg.z),
                    0.30f, -0.92f, 0.028f, textColor);
            hudText(std::format("RGB {:.2f} {:.2f} {:.2f}  EMIT {:.1f}  SEG {}",
                                shape.color.x, shape.color.y, shape.color.z,
                                shape.emissive, shape.segments),
                    0.30f, -0.87f, 0.028f, textColor);

            // Palette swatches.
            sw::f32 swatchX = 0.30f;
            for (sw::u32 i = 0; i < 12; ++i)
            {
                panel(swatchX, -0.82f, swatchX + 0.045f, -0.755f,
                      sw::Vec4{kPalette[i], 1.0f});
                m_buttons.push_back({swatchX, -0.82f, swatchX + 0.045f, -0.755f, 240u + i});
                swatchX += 0.055f;
            }
            // Fine channel buttons + emissive/segments.
            const char* channelLabels[6] = {"R+", "R-", "G+", "G-", "B+", "B-"};
            sw::f32 channelX = 0.30f;
            for (sw::u32 i = 0; i < 6; ++i)
            {
                button(channelX, -0.73f, 0.062f, channelLabels[i], 260 + i, false);
                channelX += 0.072f;
            }
            button(0.30f, -0.65f, 0.09f, "EMIT+", 224, shape.emissive > 0.0f);
            button(0.40f, -0.65f, 0.09f, "EMIT-", 225, false);
            button(0.50f, -0.65f, 0.09f, "SEG+", 226, false);
            button(0.60f, -0.65f, 0.09f, "SEG-", 227, false);
            button(0.70f, -0.65f, 0.09f, shape.visible ? "VIS:Y" : "VIS:N", 222,
                   shape.visible);
            button(0.80f, -0.65f, 0.09f, shape.collider ? "COL:Y" : "COL:N", 223,
                   shape.collider);
        }
        else if (hasNodeSelection())
        {
            const sw::parts::AttachNode& node =
                m_part.nodes[static_cast<sw::usize>(m_selectedNode)];
            hudText(std::format("NODE {}  {}  POS {:.2f} {:.2f} {:.2f}", node.name,
                                sw::parts::nodeTypeName(node.type),
                                node.position.x, node.position.y, node.position.z),
                    0.30f, -0.97f, 0.028f, textColor);
            hudText(std::format("DIR {:.0f} {:.0f} {:.0f}  SIZE {:.2f}   X/Y/Z SET DIR",
                                node.direction.x, node.direction.y, node.direction.z,
                                node.size),
                    0.30f, -0.92f, 0.028f, textColor);
        }

        // ---- modal + status footer ----------------------------------------------------
        if (m_mode != Mode::Idle)
        {
            const char* modeNames[4] = {"", "GRAB", "ROTATE", "SCALE"};
            const char* axisNames[4] = {"FREE", "X", "Y", "Z"};
            hudText(std::format("{} [{}]  CLICK/ENTER APPLY  ESC CANCEL",
                                modeNames[static_cast<sw::usize>(m_mode)],
                                axisNames[m_modeAxis + 1]),
                    -0.30f, 0.72f, 0.04f, {1.0f, 0.9f, 0.4f, 1.0f});
        }
        if (m_statusAge < 3.0f)
        {
            hudText(m_status, -0.97f, 0.845f, 0.03f, {0.6f, 1.0f, 0.7f, 0.9f});
        }
    }

    void PartStudioApp::setStatus(std::string message)
    {
        m_status = std::move(message);
        m_statusAge = 0.0f;
    }
} // namespace studio
