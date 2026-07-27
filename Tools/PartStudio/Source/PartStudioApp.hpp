#pragma once

// ============================================================================
// PartStudioApp.hpp
// PART STUDIO — the in-engine authoring tool for .swpart files.
//
// This is where parts are MADE: compose primitives (box / cylinder / cone /
// sphere / tube), color them, flag which ones form the collision hull, and
// place the attach nodes ON the surfaces. The game and the tool read and
// write the exact same JSON files through sw::parts — there is no export
// step, no drift: save in the Studio, launch the game, fly the part.
//
// Interaction model (Blender-style modals + clickable UI):
//   right-drag orbit, wheel zoom
//   left-click        select shape / node (ray cast against real geometry)
//   G / R / S         grab / rotate / scale the selection (mouse moves it)
//     + X / Y / Z     constrain to an axis, Shift = fine snap
//     left-click/Enter confirm, Esc cancel
//   K                 collision overlay (orange boxes = collider hull)
//   Buttons           add shapes, dup/delete, colors, flags, nodes, files
// ============================================================================

#include <Engine.hpp>

#include <array>
#include <filesystem>
#include <string>
#include <vector>

namespace studio
{
    class PartStudioApp final : public sw::Application
    {
    public:
        explicit PartStudioApp(const sw::ApplicationConfig& config);

    protected:
        void onUpdate(sw::f32 deltaSeconds) override;
        void onRender() override;

    private:
        enum class Mode : sw::u8
        {
            Idle = 0,
            Grab,
            Rotate,
            Scale,
        };

        // ---- files ------------------------------------------------------------
        void refreshFileList();
        void loadCurrentFile();
        void saveCurrentFile();
        void newPart();

        // ---- editing ----------------------------------------------------------
        void markGeometryDirty() { m_geometryDirty = true; }
        void rebuildShapeMeshes();
        void beginMode(Mode mode);
        void applyMode();
        void cancelMode();
        void updateModal();
        void updateCameraControls();
        void pickAtCursor();
        void addShape(sw::parts::ShapeKind kind);
        void addNode(sw::parts::NodeType type);
        void snapSelectedNodeToSurface();
        [[nodiscard]] bool hasShapeSelection() const;
        [[nodiscard]] bool hasNodeSelection() const;
        [[nodiscard]] bool hasHitboxSelection() const;
        void addHitbox();
        void fitHitboxes();

        /// Ray through the cursor, origin at the camera (world units;
        /// the edited part sits at the world origin).
        void cursorRay(sw::Vec3& outOrigin, sw::Vec3& outDirection);
        /// World-units-per-pixel at the orbit pivot distance.
        [[nodiscard]] sw::f32 worldPerPixel();

        // ---- presentation -------------------------------------------------------
        void collectScene();
        void collectUi();
        void hudText(std::string_view text, sw::f32 x, sw::f32 y, sw::f32 heightNdc,
                     const sw::Vec4& color);
        void handleClicks();
        void setStatus(std::string message);
        [[nodiscard]] sw::u32 registerMesh(sw::Mesh mesh);

        // ---- data ---------------------------------------------------------------
        std::filesystem::path m_directory;
        std::vector<std::filesystem::path> m_files;
        sw::usize m_fileIndex = 0;
        bool m_untitled = false;

        sw::parts::PartDefinition m_part;
        sw::i32 m_selectedShape = -1;
        sw::i32 m_selectedNode = -1;
        /// THE HULL, selected the same way shapes and nodes are. A hitbox is
        /// an axis-aligned box, so it has no rotation to edit — grab and
        /// scale only, and Rotate refuses like it does for nodes.
        sw::i32 m_selectedHitbox = -1;

        Mode m_mode = Mode::Idle;
        sw::i32 m_modeAxis = -1; // -1 free, 0/1/2 = X/Y/Z
        sw::f32 m_modalAccumX = 0.0f;
        sw::f32 m_modalAccumY = 0.0f;
        sw::parts::PartShape m_shapeBackup{};
        sw::parts::AttachNode m_nodeBackup{};
        sw::parts::HitBox m_hitboxBackup{};

        bool m_geometryDirty = true;
        bool m_showHitboxes = true;
        bool m_showColliders = true;

        // ---- camera --------------------------------------------------------------
        sw::Camera m_camera;
        sw::f32 m_yaw = 0.7f;
        sw::f32 m_pitch = 0.35f;
        sw::f32 m_distance = 7.0f;

        // ---- rendering resources ---------------------------------------------------
        std::vector<sw::Mesh> m_meshes;
        std::vector<sw::DrawItem> m_drawItems;
        std::array<sw::u32, 128> m_glyphMeshIndex{};
        sw::u32 m_floorMeshIndex = 0;
        sw::u32 m_markerMeshIndex = 0;
        sw::u32 m_unitBoxMeshIndex = 0; // unit cube: overlays, axes, HUD panels
        sw::usize m_shapeMeshBase = 0;  // first shape mesh slot
        sw::usize m_shapeMeshCount = 0;

        struct HudButton
        {
            sw::f32 x0, y0, x1, y1;
            sw::u32 id;
        };
        std::vector<HudButton> m_buttons;

        std::string m_status;
        sw::f32 m_statusAge = 100.0f;
    };
} // namespace studio
