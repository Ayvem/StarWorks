#pragma once

// ============================================================================
// Factory/Recipes.hpp
// The production catalogue: what a machine can turn into what, at what rate,
// for how much power.
//
// F1's founding rule is the project's oldest one applied to industry: MATTER
// IS CONSERVED. A recipe declares its inputs and outputs in resource UNITS,
// the resource catalogue gives every unit a real mass, and `recipeMassLoss`
// makes the difference explicit. A recipe that creates matter is not a
// balance decision, it is a bug — `RecipeTests` fails the build over it.
//
// Recipes are DATA: `.swrecipe` JSON files loaded from Assets/Recipes, with
// the same contract as the part catalogue (stable numeric ids, a built-in
// fallback so tests and a broken install still run, per-file error logging).
// Nothing in the engine hard-codes a production chain.
//
// Rates are UNITS PER SECOND, deliberately: the executor multiplies them by
// the bulk-catch-up lane's dt, so eight hours of time warp produce exactly
// eight hours of goods. Anything expressed per-tick would not survive warp.
// ============================================================================

#include "Core/Types.hpp"
#include "Resources/ResourceTypes.hpp"

#include <filesystem>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace sw::factory
{
    /// What KIND of building a recipe needs. A building declares the same
    /// category, and that is the whole compatibility rule for now — richer
    /// tagging can arrive the day a machine needs to belong to two families.
    enum class BuildingCategory : u8
    {
        Miner = 0,   // extracts from a deposit (no material input)
        Refinery,    // smelting, chemistry, electrolysis
        Storage,     // holds, produces nothing
        Solar,       // produces power only
        Fabricator,  // makes PARTS (F6)
        Pad,         // landing / loading pad (F5)
        Hub,         // the site's centre: defines it, holds its books
        Beacon,      // produces nothing but VISIBILITY: a site you can find
        Conveyor,    // a belt segment: moves goods between the above
        Battery,     // stores ElectricCharge: what a 14-day night is fought with
        Pole,        // a power pole: the only thing a grid may branch at
        Cable,       // one span of wire between two power nodes
        Count
    };

    [[nodiscard]] std::string_view categoryName(BuildingCategory category);
    [[nodiscard]] bool categoryFromName(std::string_view name,
                                        BuildingCategory& outCategory);

    inline constexpr u32 kMaxRecipeIngredients = 4;

    struct Ingredient
    {
        res::Resource resource = res::Resource::Count; // Count == unused
        f64 unitsPerSecond = 0.0;
    };

    struct RecipeDefinition
    {
        u32 id = 0; // STABLE id (saved inside RecipeStateComponent)
        std::string name;
        BuildingCategory requiredCategory = BuildingCategory::Refinery;
        Ingredient inputs[kMaxRecipeIngredients]{};
        Ingredient outputs[kMaxRecipeIngredients]{};
        /// Power drawn while running, kW. Electrolysis is the reason the
        /// whole energy system exists, so this is a first-class field.
        f64 powerKw = 0.0;
        /// Declared mass loss, as a fraction of the input mass (slag, vented
        /// volatiles). The conservation test allows exactly this much and
        /// not a gram more.
        f64 massLossFraction = 0.0;
    };

    // ---- catalogue --------------------------------------------------------
    [[nodiscard]] std::span<const RecipeDefinition> recipeCatalog();
    [[nodiscard]] const RecipeDefinition* findRecipe(u32 id);
    /// Every recipe a building of this category may run, in catalogue order.
    [[nodiscard]] std::vector<u32> recipesForCategory(BuildingCategory category);

    /// Replaces the registry with every *.swrecipe in `directory` (sorted by
    /// id). Returns false and KEEPS the previous catalogue when the folder
    /// holds no valid recipe.
    bool loadRecipeCatalog(const std::filesystem::path& directory);
    [[nodiscard]] bool loadRecipeFile(const std::filesystem::path& path,
                                      RecipeDefinition& out);
    [[nodiscard]] bool saveRecipeFile(const RecipeDefinition& recipe,
                                      const std::filesystem::path& path);

    // ---- conservation -----------------------------------------------------
    /// Mass entering / leaving per second of running, kg/s.
    [[nodiscard]] f64 recipeInputMassKgps(const RecipeDefinition& recipe);
    [[nodiscard]] f64 recipeOutputMassKgps(const RecipeDefinition& recipe);
    /// Mass that vanishes per second. Negative means the recipe CREATES
    /// matter — which is what the test is looking for.
    [[nodiscard]] f64 recipeMassLossKgps(const RecipeDefinition& recipe);

    // Stable recipe ids (never renumber).
    inline constexpr u32 kRecipeMineIronOre = 1;
    inline constexpr u32 kRecipeMineCopperOre = 2;
    inline constexpr u32 kRecipeMineWaterIce = 3;
    inline constexpr u32 kRecipeSmeltIron = 4;
    inline constexpr u32 kRecipeSmeltCopper = 5;
    inline constexpr u32 kRecipeMeltWater = 6;
    inline constexpr u32 kRecipeElectrolysis = 7;
    inline constexpr u32 kRecipeSynthesizeFuel = 8;
} // namespace sw::factory
