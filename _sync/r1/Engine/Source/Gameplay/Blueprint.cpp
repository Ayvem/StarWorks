#include "Gameplay/Blueprint.hpp"

#include "Core/FileSystem.hpp"
#include "Core/Json.hpp"
#include "Core/Log.hpp"

#include <algorithm>

namespace sw::parts
{
    namespace
    {
        constexpr const char* kLogCat = "Blueprint";

        [[nodiscard]] std::vector<ShipBlueprint>& registry()
        {
            static std::vector<ShipBlueprint> designs;
            return designs;
        }

        [[nodiscard]] json::Value vec3ToJson(const Vec3& v)
        {
            json::Value array = json::Value::makeArray();
            array.push(json::Value(static_cast<f64>(v.x)));
            array.push(json::Value(static_cast<f64>(v.y)));
            array.push(json::Value(static_cast<f64>(v.z)));
            return array;
        }

        [[nodiscard]] Vec3 vec3FromJson(const json::Value* value, const Vec3& fallback)
        {
            if (value == nullptr || !value->isArray() || value->asArray().size() != 3)
            {
                return fallback;
            }
            const json::Array& array = value->asArray();
            return {static_cast<f32>(array[0].asNumber()),
                    static_cast<f32>(array[1].asNumber()),
                    static_cast<f32>(array[2].asNumber())};
        }

        [[nodiscard]] json::Value quatToJson(const Quat& q)
        {
            json::Value array = json::Value::makeArray();
            array.push(json::Value(static_cast<f64>(q.w)));
            array.push(json::Value(static_cast<f64>(q.x)));
            array.push(json::Value(static_cast<f64>(q.y)));
            array.push(json::Value(static_cast<f64>(q.z)));
            return array;
        }

        [[nodiscard]] Quat quatFromJson(const json::Value* value)
        {
            if (value == nullptr || !value->isArray() || value->asArray().size() != 4)
            {
                return Quat{1.0f, 0.0f, 0.0f, 0.0f};
            }
            const json::Array& array = value->asArray();
            const Quat q{static_cast<f32>(array[0].asNumber()),
                         static_cast<f32>(array[1].asNumber()),
                         static_cast<f32>(array[2].asNumber()),
                         static_cast<f32>(array[3].asNumber())};
            // A design edited by hand should not be able to introduce a
            // non-unit rotation that quietly scales every part it touches.
            const f32 length = glm::length(q);
            return (length > 1.0e-5f) ? q / length : Quat{1.0f, 0.0f, 0.0f, 0.0f};
        }
    } // namespace

    f64 copperFraction(PartType type)
    {
        switch (type)
        {
        case PartType::Battery:
            return 0.55; // plate and electrolyte, wrapped in a thin case
        case PartType::SolarPanel:
            return 0.60; // cell and conductor; the frame is the small part
        case PartType::Engine:
            return 0.25; // a steel bell, but the pumps and harness are copper
        case PartType::DockingPort:
            return 0.20; // latches, sensors, the power pass-through
        default:
            // Tanks, wings, decouplers, bays, structure: steel with a loom
            // run through it.
            return 0.05;
        }
    }

    BillOfMaterials partCost(const PartDefinition& definition)
    {
        const f64 copper = std::clamp(copperFraction(definition.type), 0.0, 1.0);
        BillOfMaterials bill{};
        bill.copperKg = definition.dryMassKg * copper;
        // Iron is the REMAINDER, never an independent number: that is what
        // makes iron + copper equal the dry mass to the last gram, whatever
        // anyone does to the fractions later.
        bill.ironKg = definition.dryMassKg - bill.copperKg;
        return bill;
    }

    BillOfMaterials blueprintCost(const ShipBlueprint& blueprint)
    {
        BillOfMaterials total{};
        for (const BlueprintPartRecord& record : blueprint.parts)
        {
            const PartDefinition* definition = findDefinition(record.definitionId);
            if (definition == nullptr)
            {
                continue; // see blueprintIsBuildable
            }
            const BillOfMaterials bill = partCost(*definition);
            total.ironKg += bill.ironKg;
            total.copperKg += bill.copperKg;
        }
        return total;
    }

    bool blueprintIsBuildable(const ShipBlueprint& blueprint)
    {
        if (blueprint.parts.empty())
        {
            return false;
        }
        for (const BlueprintPartRecord& record : blueprint.parts)
        {
            if (findDefinition(record.definitionId) == nullptr)
            {
                return false;
            }
        }
        return true;
    }

    f64 blueprintDryMassKg(const ShipBlueprint& blueprint)
    {
        f64 mass = 0.0;
        for (const BlueprintPartRecord& record : blueprint.parts)
        {
            if (const PartDefinition* definition = findDefinition(record.definitionId))
            {
                mass += definition->dryMassKg;
            }
        }
        return mass;
    }

    std::span<const ShipBlueprint> blueprintCatalog() { return registry(); }

    const ShipBlueprint* findBlueprint(std::string_view name)
    {
        for (const ShipBlueprint& blueprint : registry())
        {
            if (blueprint.name == name)
            {
                return &blueprint;
            }
        }
        return nullptr;
    }

    void registerBlueprint(const ShipBlueprint& blueprint)
    {
        for (ShipBlueprint& existing : registry())
        {
            if (existing.name == blueprint.name)
            {
                existing = blueprint;
                return;
            }
        }
        registry().push_back(blueprint);
        std::sort(registry().begin(), registry().end(),
                  [](const ShipBlueprint& a, const ShipBlueprint& b) {
                      return a.name < b.name;
                  });
    }

    bool loadBlueprintFile(const std::filesystem::path& path, ShipBlueprint& out)
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

        ShipBlueprint blueprint{};
        blueprint.name = root.string("name");
        if (blueprint.name.empty())
        {
            blueprint.name = path.stem().string();
        }
        if (const json::Value* parts = root.find("parts"))
        {
            for (const json::Value& entry : parts->asArray())
            {
                BlueprintPartRecord record{};
                record.definitionId =
                    static_cast<u32>(entry.number("definitionId", 0.0));
                record.localPosition = vec3FromJson(entry.find("position"), Vec3{0.0f});
                record.localRotation = quatFromJson(entry.find("rotation"));
                record.parentIndex = static_cast<i32>(entry.number("parent", -1.0));
                record.parentPoint =
                    static_cast<u8>(std::clamp(entry.number("parentPoint", 0.0), 0.0, 255.0));
                record.childPoint =
                    static_cast<u8>(std::clamp(entry.number("childPoint", 0.0), 0.0, 255.0));
                record.symmetryGroup =
                    static_cast<i32>(entry.number("symmetryGroup", -1.0));
                // A part is always written after its parent, so a parent
                // index that does not point BACKWARDS is a broken file. It
                // costs a loose part, never a read off the end of the vector
                // when the design is instantiated.
                if (record.parentIndex >=
                    static_cast<i32>(blueprint.parts.size()))
                {
                    record.parentIndex = -1;
                }
                blueprint.parts.push_back(record);
            }
        }
        if (blueprint.parts.empty())
        {
            SW_LOG_ERROR(kLogCat, "'{}': a design needs at least one part",
                         path.string());
            return false;
        }
        out = std::move(blueprint);
        return true;
    }

    bool saveBlueprintFile(const ShipBlueprint& blueprint,
                           const std::filesystem::path& path)
    {
        json::Value root = json::Value::makeObject();
        root.set("name", json::Value(blueprint.name));
        json::Value parts = json::Value::makeArray();
        for (const BlueprintPartRecord& record : blueprint.parts)
        {
            json::Value entry = json::Value::makeObject();
            entry.set("definitionId", json::Value(static_cast<f64>(record.definitionId)));
            entry.set("position", vec3ToJson(record.localPosition));
            entry.set("rotation", quatToJson(record.localRotation));
            entry.set("parent", json::Value(static_cast<f64>(record.parentIndex)));
            entry.set("parentPoint", json::Value(static_cast<f64>(record.parentPoint)));
            entry.set("childPoint", json::Value(static_cast<f64>(record.childPoint)));
            entry.set("symmetryGroup",
                      json::Value(static_cast<f64>(record.symmetryGroup)));
            parts.push(std::move(entry));
        }
        root.set("parts", std::move(parts));

        const std::string text = json::serialize(root);
        try
        {
            FileSystem::writeBinaryFile(path, std::vector<u8>(text.begin(), text.end()));
        }
        catch (...)
        {
            SW_LOG_ERROR(kLogCat, "Cannot write '{}'", path.string());
            return false;
        }
        return true;
    }

    bool loadBlueprintCatalog(const std::filesystem::path& directory)
    {
        std::vector<ShipBlueprint> loaded;
        std::error_code error{};
        if (std::filesystem::is_directory(directory, error))
        {
            std::vector<std::filesystem::path> files;
            for (const auto& entry : std::filesystem::directory_iterator(directory, error))
            {
                if (entry.path().extension() == ".swship")
                {
                    files.push_back(entry.path());
                }
            }
            std::sort(files.begin(), files.end());
            for (const std::filesystem::path& file : files)
            {
                ShipBlueprint blueprint{};
                if (loadBlueprintFile(file, blueprint))
                {
                    loaded.push_back(std::move(blueprint));
                }
            }
        }
        if (loaded.empty())
        {
            SW_LOG_WARN(kLogCat, "No designs in '{}': keeping the {} already loaded",
                        directory.string(), registry().size());
            return false;
        }
        registry() = std::move(loaded);
        SW_LOG_INFO(kLogCat, "Designs loaded: {} from '{}'", registry().size(),
                    directory.string());
        return true;
    }
} // namespace sw::parts
