#pragma once

// ============================================================================
// Gameplay/Blueprint.hpp
// A SAVED DESIGN, and what it costs to build one.
//
// The hangar has always been able to assemble a rocket out of thin air. F5
// makes that an industrial act instead: a design is a FILE, a design has a
// BILL OF MATERIALS, and the metal that pays it has to arrive on a belt.
//
// Two ideas, both deliberately small:
//
//   * `ShipBlueprint` is the hangar's part list, saved. It is the same data
//     the editor already holds — definition id, local pose, symmetry group —
//     written to a `.swship` JSON on the same contract as `.swpart` and
//     `.swrecipe`: stable ids, a per-file loader that refuses garbage, and a
//     catalogue the game reads at startup.
//
//   * `BillOfMaterials` is what it costs, and the rule comes from what the
//     part IS. Electrical parts are mostly copper — batteries, panels, the
//     wiring and pumps of an engine — and structure is mostly iron. That is
//     both the physically obvious answer and the one that gives copper a
//     reason to exist in the chain.
//
// MATTER IS CONSERVED, as everywhere else here: iron + copper equals the
// part's dry mass exactly. Twelve tonnes of metal go into the VAB and a
// twelve-tonne rocket comes out of it, and `BlueprintTests` weighs it.
// ============================================================================

#include "Gameplay/Fairing.hpp"
#include "Gameplay/Parts.hpp"

#include <filesystem>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace sw::parts
{
    /// One part of a saved design, in the vessel's own frame.
    struct BlueprintPartRecord
    {
        u32 definitionId = 0;
        Vec3 localPosition{0.0f};
        Quat localRotation{1.0f, 0.0f, 0.0f, 0.0f};
        /// THE JOINT that holds this part on. A pose alone would spawn a
        /// rocket as a pile of parts flying in formation: the joints are
        /// what breaks under load, what a decoupler cuts, and what makes a
        /// vessel a vessel. `parentIndex` is an index into `parts` (-1 for
        /// the root), `parentPoint` is the parent's node — 255 means a
        /// SURFACE attachment, which is radial by nature.
        i32 parentIndex = -1;
        u8 parentPoint = 0;
        u8 childPoint = 0;
        /// Which radial-symmetry batch it was placed with, or -1. Kept so a
        /// design round-trips through disk and back into the editor without
        /// losing the grouping its UNDO depends on.
        i32 symmetryGroup = -1;
        /// F48: the profile of the shell drawn on this part, when it is a
        /// fairing base. Zero rings on everything else.
        parts::FairingComponent fairing{};
    };

    struct ShipBlueprint
    {
        std::string name;
        std::vector<BlueprintPartRecord> parts;
    };

    /// What one vehicle costs, in kilograms of each metal.
    struct BillOfMaterials
    {
        f64 ironKg = 0.0;
        f64 copperKg = 0.0;

        [[nodiscard]] f64 totalKg() const { return ironKg + copperKg; }
    };

    /// The COPPER fraction of a part's dry mass, by what the part is.
    ///
    /// A solar wing is mostly conductor and cell; a battery is mostly plate;
    /// an engine is a steel bell wrapped in pumps and wiring; a fuel tank is
    /// a steel drum. The numbers are a balance decision, but the SHAPE of
    /// them is not — it is why a rocket needs a copper chain at all.
    [[nodiscard]] f64 copperFraction(PartType type);

    /// One part's bill. Iron + copper is exactly its dry mass.
    [[nodiscard]] BillOfMaterials partCost(const PartDefinition& definition);

    /// The whole design's bill. Parts whose definition is missing from the
    /// catalogue contribute nothing rather than silently costing zero-mass
    /// metal — a blueprint referring to a deleted part is a broken blueprint,
    /// and `blueprintIsBuildable` says so.
    [[nodiscard]] BillOfMaterials blueprintCost(const ShipBlueprint& blueprint);

    /// False when the design names a part this build does not have.
    [[nodiscard]] bool blueprintIsBuildable(const ShipBlueprint& blueprint);

    /// Dry mass of the design, kg — what the finished vehicle weighs.
    [[nodiscard]] f64 blueprintDryMassKg(const ShipBlueprint& blueprint);

    // ---- the catalogue, on the .swpart contract ---------------------------
    [[nodiscard]] std::span<const ShipBlueprint> blueprintCatalog();
    [[nodiscard]] const ShipBlueprint* findBlueprint(std::string_view name);
    /// Replaces the registry with every *.swship in `directory`, sorted by
    /// name. Returns false and KEEPS the previous catalogue when the folder
    /// holds nothing valid — an empty design folder is not a reason to lose
    /// the designs you had.
    bool loadBlueprintCatalog(const std::filesystem::path& directory);
    [[nodiscard]] bool loadBlueprintFile(const std::filesystem::path& path,
                                         ShipBlueprint& out);
    [[nodiscard]] bool saveBlueprintFile(const ShipBlueprint& blueprint,
                                         const std::filesystem::path& path);
    /// Adds or replaces one design in the live catalogue — what the hangar's
    /// SAVE does, so a design is orderable at the VAB without a restart.
    void registerBlueprint(const ShipBlueprint& blueprint);
} // namespace sw::parts
