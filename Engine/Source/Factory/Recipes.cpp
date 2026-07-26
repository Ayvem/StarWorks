#include "Factory/Recipes.hpp"

#include "Core/FileSystem.hpp"
#include "Core/Json.hpp"
#include "Core/Log.hpp"

#include <algorithm>
#include <array>
#include <vector>

namespace sw::factory
{
    namespace
    {
        constexpr const char* kLogCat = "Recipes";

        constexpr std::array<std::string_view, static_cast<usize>(BuildingCategory::Count)>
            kCategoryNames = {"miner",  "refinery",   "storage",  "solar",
                              "fabricator", "pad", "hub", "beacon",
                              "conveyor"};

        Ingredient make(res::Resource resource, f64 unitsPerSecond)
        {
            return Ingredient{resource, unitsPerSecond};
        }

        /// The BUILT-IN chain. It is the fallback when Assets/Recipes is
        /// missing (tests, a broken install) and, just as importantly, the
        /// reference the shipped .swrecipe files are written from.
        ///
        /// Every resource in the catalogue is 1 kg per unit, so the ratios
        /// ARE the mass balance and there is nowhere to hide: 3 kg of iron
        /// ore yields 1.8 kg of iron and 1.2 kg of slag, which is roughly
        /// what a 60%-Fe ore really gives. Electrolysis splits water into
        /// 11.1% hydrogen and 88.9% oxygen BY MASS — the real numbers.
        /// RecipeTests recomputes all of it.
        std::vector<RecipeDefinition> builtinCatalog()
        {
            std::vector<RecipeDefinition> recipes;

            auto extraction = [&](u32 id, const char* name, res::Resource ore,
                                  f64 rate, f64 powerKw) {
                RecipeDefinition r{};
                r.id = id;
                r.name = name;
                r.requiredCategory = BuildingCategory::Miner;
                r.outputs[0] = make(ore, rate);
                r.powerKw = powerKw;
                // Extraction takes matter OUT of the ground, so the deposit
                // is the input the books balance against — not the machine.
                r.massLossFraction = 0.0;
                return r;
            };
            recipes.push_back(extraction(kRecipeMineIronOre, "Iron Ore Extraction",
                                         res::Resource::IronOre, 1.0, 45.0));
            recipes.push_back(extraction(kRecipeMineCopperOre, "Copper Ore Extraction",
                                         res::Resource::CopperOre, 0.8, 45.0));
            recipes.push_back(extraction(kRecipeMineWaterIce, "Ice Harvesting",
                                         res::Resource::WaterIce, 1.2, 30.0));

            {
                RecipeDefinition r{};
                r.id = kRecipeSmeltIron;
                r.name = "Iron Smelting";
                r.requiredCategory = BuildingCategory::Refinery;
                r.inputs[0] = make(res::Resource::IronOre, 3.0);
                r.outputs[0] = make(res::Resource::Iron, 1.8);
                r.powerKw = 120.0;
                r.massLossFraction = 0.40; // slag: a 60%-Fe ore
                recipes.push_back(r);
            }
            {
                RecipeDefinition r{};
                r.id = kRecipeSmeltCopper;
                r.name = "Copper Smelting";
                r.requiredCategory = BuildingCategory::Refinery;
                r.inputs[0] = make(res::Resource::CopperOre, 3.0);
                r.outputs[0] = make(res::Resource::Copper, 1.5);
                r.powerKw = 110.0;
                r.massLossFraction = 0.50; // copper ore is a lean ore
                recipes.push_back(r);
            }
            {
                RecipeDefinition r{};
                r.id = kRecipeMeltWater;
                r.name = "Ice Melting";
                r.requiredCategory = BuildingCategory::Refinery;
                r.inputs[0] = make(res::Resource::WaterIce, 2.0);
                r.outputs[0] = make(res::Resource::Water, 2.0);
                r.powerKw = 25.0;
                r.massLossFraction = 0.0;
                recipes.push_back(r);
            }
            {
                // THE recipe the whole energy system exists for: it is what
                // makes a polar site worth founding, and what makes a lunar
                // night hurt.
                RecipeDefinition r{};
                r.id = kRecipeElectrolysis;
                r.name = "Water Electrolysis";
                r.requiredCategory = BuildingCategory::Refinery;
                r.inputs[0] = make(res::Resource::Water, 1.0);
                r.outputs[0] = make(res::Resource::Hydrogen, 0.111);
                r.outputs[1] = make(res::Resource::Oxygen, 0.889);
                r.powerKw = 480.0;
                r.massLossFraction = 0.0;
                recipes.push_back(r);
            }
            {
                RecipeDefinition r{};
                r.id = kRecipeSynthesizeFuel;
                r.name = "Fuel Synthesis";
                r.requiredCategory = BuildingCategory::Refinery;
                r.inputs[0] = make(res::Resource::Hydrogen, 0.25);
                r.inputs[1] = make(res::Resource::Oxygen, 0.75);
                r.outputs[0] = make(res::Resource::Fuel, 1.0);
                r.powerKw = 60.0;
                r.massLossFraction = 0.0;
                recipes.push_back(r);
            }
            return recipes;
        }

        std::vector<RecipeDefinition>& registry()
        {
            static std::vector<RecipeDefinition> recipes = builtinCatalog();
            return recipes;
        }
    } // namespace

    std::string_view categoryName(BuildingCategory category)
    {
        const auto index = static_cast<usize>(category);
        return index < kCategoryNames.size() ? kCategoryNames[index] : "unknown";
    }

    bool categoryFromName(std::string_view name, BuildingCategory& outCategory)
    {
        for (usize i = 0; i < kCategoryNames.size(); ++i)
        {
            if (kCategoryNames[i] == name)
            {
                outCategory = static_cast<BuildingCategory>(i);
                return true;
            }
        }
        return false;
    }

    std::span<const RecipeDefinition> recipeCatalog() { return registry(); }

    const RecipeDefinition* findRecipe(u32 id)
    {
        for (const RecipeDefinition& recipe : registry())
        {
            if (recipe.id == id)
            {
                return &recipe;
            }
        }
        return nullptr;
    }

    std::vector<u32> recipesForCategory(BuildingCategory category)
    {
        std::vector<u32> ids;
        for (const RecipeDefinition& recipe : registry())
        {
            if (recipe.requiredCategory == category)
            {
                ids.push_back(recipe.id);
            }
        }
        return ids;
    }

    // ---- conservation -------------------------------------------------------

    namespace
    {
        f64 massOf(const Ingredient (&list)[kMaxRecipeIngredients])
        {
            f64 total = 0.0;
            for (const Ingredient& ingredient : list)
            {
                if (ingredient.resource == res::Resource::Count)
                {
                    continue;
                }
                total += ingredient.unitsPerSecond *
                         res::definition(ingredient.resource).massPerUnitKg;
            }
            return total;
        }
    } // namespace

    f64 recipeInputMassKgps(const RecipeDefinition& recipe)
    {
        return massOf(recipe.inputs);
    }

    f64 recipeOutputMassKgps(const RecipeDefinition& recipe)
    {
        return massOf(recipe.outputs);
    }

    f64 recipeMassLossKgps(const RecipeDefinition& recipe)
    {
        return recipeInputMassKgps(recipe) - recipeOutputMassKgps(recipe);
    }

    // ---- .swrecipe files ----------------------------------------------------

    namespace
    {
        bool resourceFromName(std::string_view name, res::Resource& outResource)
        {
            for (usize r = 0; r < res::kResourceCount; ++r)
            {
                const auto resource = static_cast<res::Resource>(r);
                if (res::definition(resource).name == name)
                {
                    outResource = resource;
                    return true;
                }
            }
            return false;
        }

        bool readIngredients(const json::Value* array,
                             Ingredient (&out)[kMaxRecipeIngredients],
                             const std::filesystem::path& path)
        {
            if (array == nullptr || !array->isArray())
            {
                return true; // absent is legal: a miner has no inputs
            }
            usize slot = 0;
            for (const json::Value& entry : array->asArray())
            {
                if (slot >= kMaxRecipeIngredients)
                {
                    SW_LOG_WARN(kLogCat, "'{}': more than {} ingredients, extra ignored",
                                path.string(), kMaxRecipeIngredients);
                    break;
                }
                res::Resource resource = res::Resource::Count;
                if (!resourceFromName(entry.string("resource"), resource))
                {
                    SW_LOG_ERROR(kLogCat, "'{}': unknown resource '{}'", path.string(),
                                 entry.string("resource"));
                    return false;
                }
                out[slot].resource = resource;
                out[slot].unitsPerSecond = entry.number("unitsPerSecond", 0.0);
                ++slot;
            }
            return true;
        }

        json::Value writeIngredients(const Ingredient (&list)[kMaxRecipeIngredients])
        {
            json::Value array = json::Value::makeArray();
            for (const Ingredient& ingredient : list)
            {
                if (ingredient.resource == res::Resource::Count)
                {
                    continue;
                }
                json::Value entry = json::Value::makeObject();
                entry.set("resource", json::Value(std::string(
                                          res::definition(ingredient.resource).name)));
                entry.set("unitsPerSecond", json::Value(ingredient.unitsPerSecond));
                array.push(std::move(entry));
            }
            return array;
        }
    } // namespace

    bool loadRecipeFile(const std::filesystem::path& path, RecipeDefinition& out)
    {
        std::string text;
        try
        {
            const std::vector<u8> bytes = FileSystem::readBinaryFile(path);
            text.assign(bytes.begin(), bytes.end());
        }
        catch (...)
        {
            SW_LOG_ERROR(kLogCat, "Cannot read '{}'", path.string());
            return false;
        }

        std::string parseError;
        const json::Value root = json::parse(text, parseError);
        if (!parseError.empty() || !root.isObject())
        {
            SW_LOG_ERROR(kLogCat, "'{}': {}", path.string(),
                         parseError.empty() ? "not a JSON object" : parseError);
            return false;
        }

        RecipeDefinition recipe{};
        recipe.id = static_cast<u32>(root.number("id", 0.0));
        if (recipe.id == 0)
        {
            SW_LOG_ERROR(kLogCat, "'{}': missing or zero id", path.string());
            return false;
        }
        recipe.name = root.string("name");
        if (!categoryFromName(root.string("requiredCategory"), recipe.requiredCategory))
        {
            SW_LOG_ERROR(kLogCat, "'{}': unknown category '{}'", path.string(),
                         root.string("requiredCategory"));
            return false;
        }
        if (!readIngredients(root.find("inputs"), recipe.inputs, path) ||
            !readIngredients(root.find("outputs"), recipe.outputs, path))
        {
            return false;
        }
        recipe.powerKw = root.number("powerKw", 0.0);
        recipe.massLossFraction = root.number("massLossFraction", 0.0);

        // A recipe that creates matter is refused AT LOAD — the same
        // treatment a part with a bad node gets. Balance is data; physics
        // is not.
        const f64 inputMass = recipeInputMassKgps(recipe);
        if (inputMass > 0.0 && recipeOutputMassKgps(recipe) > inputMass * 1.0001)
        {
            SW_LOG_ERROR(kLogCat,
                         "'{}': outputs {:.3f} kg/s from {:.3f} kg/s of inputs — "
                         "matter is conserved, fix the ratios",
                         path.string(), recipeOutputMassKgps(recipe), inputMass);
            return false;
        }

        out = std::move(recipe);
        return true;
    }

    bool saveRecipeFile(const RecipeDefinition& recipe,
                        const std::filesystem::path& path)
    {
        json::Value root = json::Value::makeObject();
        root.set("id", json::Value(recipe.id));
        root.set("name", json::Value(recipe.name));
        root.set("requiredCategory",
                 json::Value(std::string(categoryName(recipe.requiredCategory))));
        root.set("inputs", writeIngredients(recipe.inputs));
        root.set("outputs", writeIngredients(recipe.outputs));
        root.set("powerKw", json::Value(recipe.powerKw));
        root.set("massLossFraction", json::Value(recipe.massLossFraction));

        const std::string text = json::serialize(root);
        try
        {
            FileSystem::writeBinaryFile(
                path, std::vector<u8>(text.begin(), text.end()));
        }
        catch (...)
        {
            SW_LOG_ERROR(kLogCat, "Cannot write '{}'", path.string());
            return false;
        }
        return true;
    }

    bool loadRecipeCatalog(const std::filesystem::path& directory)
    {
        std::error_code error;
        if (!std::filesystem::is_directory(directory, error))
        {
            SW_LOG_WARN(kLogCat, "No recipe directory at '{}' — built-in chain kept",
                        directory.string());
            return false;
        }

        std::vector<RecipeDefinition> loaded;
        for (const auto& entry : std::filesystem::directory_iterator(directory, error))
        {
            if (!entry.is_regular_file() || entry.path().extension() != ".swrecipe")
            {
                continue;
            }
            RecipeDefinition recipe{};
            if (loadRecipeFile(entry.path(), recipe))
            {
                loaded.push_back(std::move(recipe));
            }
        }
        if (loaded.empty())
        {
            SW_LOG_WARN(kLogCat, "'{}' holds no valid recipe — built-in chain kept",
                        directory.string());
            return false;
        }

        std::sort(loaded.begin(), loaded.end(),
                  [](const RecipeDefinition& a, const RecipeDefinition& b) {
                      return a.id < b.id;
                  });
        registry() = std::move(loaded);
        SW_LOG_INFO(kLogCat, "Loaded {} recipes from '{}'", registry().size(),
                    directory.string());
        return true;
    }
} // namespace sw::factory
