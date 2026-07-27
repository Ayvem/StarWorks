#include "Gameplay/Parts.hpp"

#include "Core/FileSystem.hpp"
#include "Core/Json.hpp"
#include "Core/Log.hpp"
#include "ECS/World.hpp"
#include "Gameplay/PartGeometry.hpp"
#include "Physics/PhysicsComponents.hpp"

#include <algorithm>
#include <array>
#include <iterator>
#include <fstream>
#include <unordered_map>
#include <utility>
#include <vector>

namespace sw::parts
{
    namespace
    {
        /// Stack node helper: top/bottom joints on the part axis (-Z nose).
        [[nodiscard]] AttachNode stackTop(f32 z, f32 size = 0.8f)
        {
            return {"top", {0.0f, 0.0f, z}, {0.0f, 0.0f, -1.0f}, NodeType::Stack, size};
        }
        [[nodiscard]] AttachNode stackBottom(f32 z, f32 size = 0.8f)
        {
            return {"bottom", {0.0f, 0.0f, z}, {0.0f, 0.0f, 1.0f}, NodeType::Stack, size};
        }
        [[nodiscard]] AttachNode radial(f32 x, f32 z)
        {
            return {"radial", {x, 0.0f, z},
                    {x >= 0.0f ? 1.0f : -1.0f, 0.0f, 0.0f}, NodeType::Radial, 0.4f};
        }
        /// Fallback geometry: one gray box that is both render and collider,
        /// sized to REACH the part's nodes (surfaces match attachment).
        [[nodiscard]] PartShape fallbackBox(const Vec3& halfExtents, const Vec3& color)
        {
            PartShape shape{};
            shape.kind = ShapeKind::Box;
            shape.size = halfExtents;
            shape.color = color;
            shape.collider = true;
            return shape;
        }

        std::vector<PartDefinition> buildFallbackCatalog()
        {
            {
                std::array<PartDefinition, 9> defs{};

                PartDefinition& tank = defs[0];
                tank.id = kPartFuelTankMedium;
                tank.type = PartType::FuelTank;
                tank.name = "FT-16 Fuel Tank";
                tank.dryMassKg = 900.0;
                tank.costCredits = 1200.0;
                tank.volumeM3 = 21.0;
                tank.capacities[0] = {res::Resource::Fuel, 16000.0}; // 16 t
                tank.crashToleranceMps = 8.0;
                tank.breakingForceN = 4.0e5;
                tank.dragCoefficientArea = 1.6;
                tank.shapes.push_back(fallbackBox({1.2f, 1.2f, 2.1f}, {0.8f, 0.82f, 0.86f}));
                tank.nodes = {stackTop(-2.1f, 1.2f), stackBottom(2.1f, 1.2f),
                              radial(1.2f, 0.0f), radial(-1.2f, 0.0f)};

                PartDefinition& engine = defs[1];
                engine.id = kPartEngineVector;
                engine.type = PartType::Engine;
                engine.name = "V-400 Engine";
                engine.dryMassKg = 1400.0;
                engine.costCredits = 4200.0;
                engine.volumeM3 = 3.5;
                engine.crashToleranceMps = 10.0;
                engine.breakingForceN = 6.0e5;
                engine.dragCoefficientArea = 1.2;
                engine.thrustNewtons = 4.0e5; // 400 kN
                engine.specificImpulseS = 345.0;
                engine.shapes.push_back(fallbackBox({0.9f, 0.9f, 1.1f}, {0.35f, 0.35f, 0.38f}));
                engine.nodes = {stackTop(-1.1f, 0.9f)};

                PartDefinition& fin = defs[2];
                fin.id = kPartFinBasic;
                fin.type = PartType::Wing;
                fin.name = "AV-F1 Fin";
                fin.dryMassKg = 90.0;
                fin.costCredits = 350.0;
                fin.volumeM3 = 0.4;
                fin.crashToleranceMps = 12.0;
                fin.breakingForceN = 1.5e5;
                fin.dragCoefficientArea = 0.25;
                fin.liftCoefficient = 1.1; // the future aero pass reads this
                fin.shapes.push_back(fallbackBox({0.9f, 0.08f, 0.7f}, {0.7f, 0.3f, 0.25f}));
                fin.nodes = {radial(-0.9f, 0.0f)};

                PartDefinition& battery = defs[3];
                battery.id = kPartBatteryPack;
                battery.type = PartType::Battery;
                battery.name = "B-800 Battery";
                battery.dryMassKg = 60.0;
                battery.costCredits = 900.0;
                battery.volumeM3 = 0.12;
                battery.capacities[0] = {res::Resource::ElectricCharge, 800.0}; // kJ
                battery.crashToleranceMps = 10.0;
                battery.dragCoefficientArea = 0.1;
                battery.shapes.push_back(fallbackBox({0.35f, 0.3f, 0.3f}, {0.2f, 0.25f, 0.3f}));
                battery.nodes = {radial(-0.35f, 0.0f)};

                PartDefinition& solar = defs[4];
                solar.id = kPartSolarWing;
                solar.type = PartType::SolarPanel;
                solar.name = "SP-2 Solar Wing";
                solar.dryMassKg = 75.0;
                solar.costCredits = 1500.0;
                solar.volumeM3 = 0.3;
                solar.crashToleranceMps = 6.0;
                solar.breakingForceN = 4.0e4;
                solar.dragCoefficientArea = 0.9;
                solar.chargeRateKw = 4.0; // kJ/s in sunlight
                solar.shapes.push_back(fallbackBox({0.15f, 1.6f, 0.5f}, {0.1f, 0.15f, 0.45f}));
                solar.nodes = {radial(0.15f, 0.0f)};

                PartDefinition& dock = defs[5];
                dock.id = kPartDockingRing;
                dock.type = PartType::DockingPort;
                dock.name = "DR-1 Docking Ring";
                dock.dryMassKg = 320.0;
                dock.costCredits = 2200.0;
                dock.volumeM3 = 1.0;
                dock.crashToleranceMps = 8.0;
                dock.breakingForceN = 3.0e5;
                dock.dragCoefficientArea = 0.7;
                dock.shapes.push_back(fallbackBox({0.8f, 0.8f, 0.4f}, {0.6f, 0.6f, 0.65f}));
                dock.nodes = {stackBottom(0.4f, 0.8f), stackTop(-0.4f, 0.8f)};

                PartDefinition& decoupler = defs[6];
                decoupler.id = kPartDecouplerFlat;
                decoupler.type = PartType::Decoupler;
                decoupler.name = "DC-2 Decoupler";
                decoupler.dryMassKg = 140.0;
                decoupler.costCredits = 600.0;
                decoupler.volumeM3 = 0.5;
                decoupler.crashToleranceMps = 9.0;
                decoupler.breakingForceN = 2.5e5;
                decoupler.dragCoefficientArea = 0.4;
                decoupler.shapes.push_back(fallbackBox({1.1f, 1.1f, 0.25f}, {0.75f, 0.6f, 0.2f}));
                decoupler.nodes = {stackTop(-0.25f, 1.1f), stackBottom(0.25f, 1.1f)};

                PartDefinition& cargo = defs[7];
                cargo.id = kPartCargoBaySmall;
                cargo.type = PartType::CargoBay;
                cargo.name = "CB-8 Cargo Bay";
                cargo.dryMassKg = 650.0;
                cargo.costCredits = 1800.0;
                cargo.volumeM3 = 8.0; // usable hold volume
                cargo.crashToleranceMps = 10.0;
                cargo.breakingForceN = 5.0e5;
                cargo.dragCoefficientArea = 1.4;
                cargo.shapes.push_back(fallbackBox({1.25f, 1.25f, 1.1f}, {0.55f, 0.58f, 0.6f}));
                cargo.nodes = {stackTop(-1.1f, 1.25f), stackBottom(1.1f, 1.25f),
                               radial(1.25f, 0.0f), radial(-1.25f, 0.0f)};

                PartDefinition& core = defs[8];
                core.id = kPartCoreStructural;
                core.type = PartType::Structural;
                core.name = "SC-1 Command Core";
                core.dryMassKg = 1100.0;
                core.costCredits = 5200.0;
                core.volumeM3 = 5.0;
                core.crashToleranceMps = 14.0;
                core.breakingForceN = 8.0e5;
                core.dragCoefficientArea = 1.1;
                core.shapes.push_back(fallbackBox({1.1f, 1.1f, 1.3f}, {0.85f, 0.85f, 0.88f}));
                core.nodes = {stackTop(-1.3f, 1.1f), stackBottom(1.3f, 1.1f),
                              radial(1.1f, 0.0f), radial(-1.1f, 0.0f)};

                return std::vector<PartDefinition>(defs.begin(), defs.end());
            }
        }

        /// The mutable registry: starts as the built-in fallback, replaced
        /// wholesale by loadCatalog(). Never shrinks mid-frame — call sites
        /// hold pointers into it only transiently.
        std::vector<PartDefinition>& registry()
        {
            static std::vector<PartDefinition> kRegistry = buildFallbackCatalog();
            return kRegistry;
        }

        // ------------------------- JSON mapping --------------------------------
        constexpr std::string_view kPartTypeNames[] = {
            "FuelTank", "Engine", "Wing", "Battery", "SolarPanel",
            "DockingPort", "Decoupler", "CargoBay", "Structural"};
        constexpr std::string_view kShapeKindNames[] = {
            "box", "cylinder", "cone", "sphere", "tube"};
        // Node type names as they appear in .swpart JSON. Kept lower-case and
        // hyphenated so a hand-edited file reads the way the tool writes it.
        constexpr std::string_view kNodeTypeNames[] = {"stack", "radial", "conveyor-in",
                                                       "conveyor-out", "power"};

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

        template <typename Enum, usize N>
        [[nodiscard]] bool enumFromName(const std::string_view (&names)[N],
                                        const std::string& text, Enum& out)
        {
            for (usize i = 0; i < N; ++i)
            {
                if (names[i] == text)
                {
                    out = static_cast<Enum>(i);
                    return true;
                }
            }
            return false;
        }
    } // namespace

    std::string_view nodeTypeName(NodeType type)
    {
        const usize index = static_cast<usize>(type);
        return (index < std::size(kNodeTypeNames)) ? kNodeTypeNames[index]
                                                   : kNodeTypeNames[0];
    }

    bool nodeTypeFromName(std::string_view name, NodeType& outType)
    {
        for (usize i = 0; i < std::size(kNodeTypeNames); ++i)
        {
            if (kNodeTypeNames[i] == name)
            {
                outType = static_cast<NodeType>(i);
                return true;
            }
        }
        return false;
    }

    const AttachNode* findConveyorNode(const PartDefinition& definition, NodeType type)
    {
        for (const AttachNode& node : definition.nodes)
        {
            if (node.type == type)
            {
                return &node;
            }
        }
        return nullptr;
    }

    std::vector<const AttachNode*> conveyorNodes(const PartDefinition& definition,
                                                 NodeType type)
    {
        std::vector<const AttachNode*> found;
        for (const AttachNode& node : definition.nodes)
        {
            if (node.type == type)
            {
                found.push_back(&node);
            }
        }
        return found;
    }

    u32 conveyorNodeCount(const PartDefinition& definition, NodeType type)
    {
        u32 count = 0;
        for (const AttachNode& node : definition.nodes)
        {
            count += (node.type == type) ? 1u : 0u;
        }
        return count;
    }

    std::span<const PartDefinition> catalog() { return registry(); }

    const PartDefinition* findDefinition(u32 id)
    {
        for (const PartDefinition& definition : registry())
        {
            if (definition.id == id)
            {
                return &definition;
            }
        }
        return nullptr;
    }

    bool loadPartFile(const std::filesystem::path& path, PartDefinition& out)
    {
        std::string text;
        try
        {
            const std::vector<u8> bytes = FileSystem::readBinaryFile(path);
            text.assign(bytes.begin(), bytes.end());
        }
        catch (...)
        {
            SW_LOG_ERROR("Parts", "Cannot read '{}'", path.string());
            return false;
        }

        std::string parseError;
        const json::Value root = json::parse(text, parseError);
        if (!parseError.empty() || !root.isObject())
        {
            SW_LOG_ERROR("Parts", "'{}': {}", path.string(),
                         parseError.empty() ? "not a JSON object" : parseError);
            return false;
        }

        PartDefinition definition{};
        definition.id = static_cast<u32>(root.number("id", 0.0));
        if (definition.id == 0)
        {
            SW_LOG_ERROR("Parts", "'{}': missing or zero id", path.string());
            return false;
        }
        if (!enumFromName(kPartTypeNames, root.string("type"), definition.type))
        {
            SW_LOG_ERROR("Parts", "'{}': unknown part type '{}'", path.string(),
                         root.string("type"));
            return false;
        }
        definition.name = root.string("name");
        definition.dryMassKg = root.number("dryMassKg", 100.0);
        definition.costCredits = root.number("costCredits", 100.0);
        definition.volumeM3 = root.number("volumeM3", 1.0);
        definition.crashToleranceMps = root.number("crashToleranceMps", 12.0);
        definition.breakingForceN = root.number("breakingForceN", 2.0e5);
        definition.dragCoefficientArea = root.number("dragCoefficientArea", 0.8);
        definition.liftCoefficient = root.number("liftCoefficient", 0.0);
        definition.thrustNewtons = root.number("thrustNewtons", 0.0);
        definition.specificImpulseS = root.number("specificImpulseS", 0.0);
        definition.chargeRateKw = root.number("chargeRateKw", 0.0);

        // ---- the optional industrial block (F1) ---------------------------
        if (const json::Value* building = root.find("building");
            building != nullptr && building->isObject())
        {
            BuildingSpec spec{};
            if (!factory::categoryFromName(building->string("category"), spec.category))
            {
                SW_LOG_ERROR("Parts", "'{}': unknown building category '{}'",
                             path.string(), building->string("category"));
                return false;
            }
            spec.valid = true;
            if (const json::Value* footprint = building->find("footprintM");
                footprint != nullptr && footprint->isArray() &&
                footprint->asArray().size() >= 2)
            {
                spec.footprintM[0] = footprint->asArray()[0].asNumber(8.0);
                spec.footprintM[1] = footprint->asArray()[1].asNumber(8.0);
            }
            spec.powerKw = building->number("powerKw", 0.0);
            spec.inventoryVolumeM3 = building->number("inventoryVolumeM3", 0.0);
            spec.maxSlopeTangent = building->number("maxSlopeTangent", 0.25);
            spec.minOreDensity = building->number("minOreDensity", 0.0);
            definition.building = spec;
        }

        if (const json::Value* capacities = root.find("capacities"))
        {
            usize slot = 0;
            for (const json::Value& entry : capacities->asArray())
            {
                if (slot >= kMaxResourceCapacities)
                {
                    break;
                }
                const std::string& resourceName = entry.string("resource");
                for (usize r = 0; r < res::kResourceCount; ++r)
                {
                    const auto resource = static_cast<res::Resource>(r);
                    if (res::definition(resource).name == resourceName)
                    {
                        definition.capacities[slot] = {resource,
                                                       entry.number("units", 0.0)};
                        ++slot;
                        break;
                    }
                }
            }
        }

        if (const json::Value* shapes = root.find("shapes"))
        {
            for (const json::Value& entry : shapes->asArray())
            {
                PartShape shape{};
                if (!enumFromName(kShapeKindNames, entry.string("kind"), shape.kind))
                {
                    SW_LOG_ERROR("Parts", "'{}': unknown shape kind '{}'",
                                 path.string(), entry.string("kind"));
                    return false;
                }
                shape.position = vec3FromJson(entry.find("position"), Vec3{0.0f});
                shape.rotationDeg = vec3FromJson(entry.find("rotationDeg"), Vec3{0.0f});
                shape.size = vec3FromJson(entry.find("size"), Vec3{0.5f});
                shape.color = vec3FromJson(entry.find("color"), Vec3{0.8f});
                shape.emissive = static_cast<f32>(entry.number("emissive", 0.0));
                shape.specular = static_cast<f32>(entry.number("specular", 0.32));
                shape.gloss = static_cast<f32>(entry.number("gloss", 0.60));
                shape.segments = static_cast<u32>(entry.number("segments", 24.0));
                shape.visible = entry.boolean("visible", true);
                shape.collider = entry.boolean("collider", false);
                definition.shapes.push_back(shape);
            }
        }

        definition.prop = root.boolean("prop", false);

        if (const json::Value* nodes = root.find("nodes"))
        {
            for (const json::Value& entry : nodes->asArray())
            {
                AttachNode node{};
                node.name = entry.string("name");
                node.position = vec3FromJson(entry.find("position"), Vec3{0.0f});
                node.direction = vec3FromJson(entry.find("direction"), {0.0f, 0.0f, 1.0f});
                const f32 length = glm::length(node.direction);
                node.direction = length > 1.0e-5f ? node.direction / length
                                                  : Vec3{0.0f, 0.0f, 1.0f};
                if (!nodeTypeFromName(entry.string("type"), node.type))
                {
                    node.type = NodeType::Stack; // unknown/absent: the old default
                }
                node.size = static_cast<f32>(entry.number("size", 0.6));
                definition.nodes.push_back(std::move(node));
            }
        }

        // THE HULL. Absent means "derive it from the collider shapes" —
        // every .swpart written before hitboxes existed keeps its old
        // behaviour, and a part whose hull happens to match its geometry
        // never has to say so twice.
        if (const json::Value* boxes = root.find("hitboxes"))
        {
            for (const json::Value& entry : boxes->asArray())
            {
                HitBox box{};
                box.center = vec3FromJson(entry.find("center"), Vec3{0.0f});
                box.halfExtents = glm::abs(
                    vec3FromJson(entry.find("halfExtents"), Vec3{0.5f}));
                definition.hitboxes.push_back(box);
            }
        }

        if (definition.shapes.empty())
        {
            SW_LOG_ERROR("Parts", "'{}': a part needs at least one shape", path.string());
            return false;
        }
        out = std::move(definition);
        return true;
    }

    bool savePartFile(const PartDefinition& definition, const std::filesystem::path& path)
    {
        json::Value root = json::Value::makeObject();
        root.set("id", json::Value(definition.id));
        root.set("type", json::Value(std::string(
                             kPartTypeNames[static_cast<usize>(definition.type)])));
        root.set("name", json::Value(definition.name));
        root.set("dryMassKg", json::Value(definition.dryMassKg));
        root.set("costCredits", json::Value(definition.costCredits));
        root.set("volumeM3", json::Value(definition.volumeM3));
        root.set("crashToleranceMps", json::Value(definition.crashToleranceMps));
        root.set("breakingForceN", json::Value(definition.breakingForceN));
        root.set("dragCoefficientArea", json::Value(definition.dragCoefficientArea));
        root.set("liftCoefficient", json::Value(definition.liftCoefficient));
        root.set("thrustNewtons", json::Value(definition.thrustNewtons));
        root.set("specificImpulseS", json::Value(definition.specificImpulseS));
        root.set("chargeRateKw", json::Value(definition.chargeRateKw));

        if (definition.building.valid)
        {
            const BuildingSpec& spec = definition.building;
            json::Value building = json::Value::makeObject();
            building.set("category", json::Value(std::string(
                                         factory::categoryName(spec.category))));
            json::Value footprint = json::Value::makeArray();
            footprint.push(json::Value(spec.footprintM[0]));
            footprint.push(json::Value(spec.footprintM[1]));
            building.set("footprintM", std::move(footprint));
            building.set("powerKw", json::Value(spec.powerKw));
            building.set("inventoryVolumeM3", json::Value(spec.inventoryVolumeM3));
            building.set("maxSlopeTangent", json::Value(spec.maxSlopeTangent));
            building.set("minOreDensity", json::Value(spec.minOreDensity));
            root.set("building", std::move(building));
        }

        json::Value capacities = json::Value::makeArray();
        for (const ResourceCapacity& capacity : definition.capacities)
        {
            if (capacity.resource == res::Resource::Count)
            {
                continue;
            }
            json::Value entry = json::Value::makeObject();
            entry.set("resource",
                      json::Value(std::string(res::definition(capacity.resource).name)));
            entry.set("units", json::Value(capacity.units));
            capacities.push(std::move(entry));
        }
        root.set("capacities", std::move(capacities));

        json::Value shapes = json::Value::makeArray();
        for (const PartShape& shape : definition.shapes)
        {
            json::Value entry = json::Value::makeObject();
            entry.set("kind", json::Value(std::string(
                                  kShapeKindNames[static_cast<usize>(shape.kind)])));
            entry.set("position", vec3ToJson(shape.position));
            entry.set("rotationDeg", vec3ToJson(shape.rotationDeg));
            entry.set("size", vec3ToJson(shape.size));
            entry.set("color", vec3ToJson(shape.color));
            entry.set("emissive", json::Value(static_cast<f64>(shape.emissive)));
            entry.set("specular", json::Value(static_cast<f64>(shape.specular)));
            entry.set("gloss", json::Value(static_cast<f64>(shape.gloss)));
            entry.set("segments", json::Value(shape.segments));
            entry.set("visible", json::Value(shape.visible));
            entry.set("collider", json::Value(shape.collider));
            shapes.push(std::move(entry));
        }
        root.set("shapes", std::move(shapes));

        if (definition.prop)
        {
            root.set("prop", json::Value(true));
        }

        json::Value nodes = json::Value::makeArray();
        for (const AttachNode& node : definition.nodes)
        {
            json::Value entry = json::Value::makeObject();
            entry.set("name", json::Value(node.name));
            entry.set("position", vec3ToJson(node.position));
            entry.set("direction", vec3ToJson(node.direction));
            entry.set("type", json::Value(std::string(nodeTypeName(node.type))));
            entry.set("size", json::Value(static_cast<f64>(node.size)));
            nodes.push(std::move(entry));
        }
        root.set("nodes", std::move(nodes));

        if (!definition.hitboxes.empty())
        {
            json::Value boxes = json::Value::makeArray();
            for (const HitBox& box : definition.hitboxes)
            {
                json::Value entry = json::Value::makeObject();
                entry.set("center", vec3ToJson(box.center));
                entry.set("halfExtents", vec3ToJson(box.halfExtents));
                boxes.push(std::move(entry));
            }
            root.set("hitboxes", std::move(boxes));
        }

        const std::string text = json::serialize(root);
        try
        {
            FileSystem::writeBinaryFile(path, std::vector<u8>(text.begin(), text.end()));
        }
        catch (...)
        {
            SW_LOG_ERROR("Parts", "Cannot write '{}'", path.string());
            return false;
        }
        return true;
    }

    bool loadCatalog(const std::filesystem::path& directory)
    {
        std::error_code errorCode;
        if (!std::filesystem::is_directory(directory, errorCode))
        {
            SW_LOG_WARN("Parts", "Part directory '{}' missing - using built-in catalog",
                        directory.string());
            return false;
        }
        std::vector<PartDefinition> loaded;
        for (const auto& entry : std::filesystem::directory_iterator(directory, errorCode))
        {
            if (!entry.is_regular_file() || entry.path().extension() != ".swpart")
            {
                continue;
            }
            PartDefinition definition{};
            if (loadPartFile(entry.path(), definition))
            {
                loaded.push_back(std::move(definition));
            }
        }
        if (loaded.empty())
        {
            SW_LOG_WARN("Parts", "No valid .swpart in '{}' - using built-in catalog",
                        directory.string());
            return false;
        }
        std::sort(loaded.begin(), loaded.end(),
                  [](const PartDefinition& a, const PartDefinition& b) {
                      return a.id < b.id;
                  });
        registry() = std::move(loaded);
        SW_LOG_INFO("Parts", "Catalog loaded: {} parts from '{}'", registry().size(),
                    directory.string());
        return true;
    }

    // ------------------------------------------------------------------------
    // Vessel operations
    // ------------------------------------------------------------------------
    ecs::Entity connectParts(ecs::World& world, ecs::Entity partA, ecs::Entity partB,
                             u8 attachPointA, u8 attachPointB, JointType type,
                             f64 strengthN, f64 breakForceN)
    {
        const ecs::Entity joint = world.createEntity();
        JointComponent component{};
        component.partA = partA;
        component.partB = partB;
        component.attachPointA = attachPointA;
        component.attachPointB = attachPointB;
        component.type = type;
        component.strengthN = strengthN;
        component.breakForceN = breakForceN;
        world.addComponent(joint, component);
        return joint;
    }

    ecs::Entity splitVessel(ecs::World& world, ecs::Entity vessel,
                            std::span<const ecs::Entity> partsToDetach)
    {
        if (partsToDetach.empty())
        {
            return {};
        }
        const auto* rootTransform = world.tryGetComponent<TransformComponent>(vessel);
        const auto* rootBody = world.tryGetComponent<phys::DynamicBodyComponent>(vessel);
        if (rootTransform == nullptr)
        {
            return {};
        }

        // New root: same frame as the old one, so the detached parts keep
        // their local poses untouched.
        const ecs::Entity newRoot = world.createEntity();
        world.addComponent(newRoot, *rootTransform);
        world.addComponent(newRoot, PreviousTransformComponent{rootTransform->position,
                                                               rootTransform->rotation});
        phys::DynamicBodyComponent body{};
        if (rootBody != nullptr)
        {
            body = *rootBody;
        }
        // Gentle separation shove tail-ward (+Z of the vessel frame).
        body.velocity += WorldVec3(rootTransform->rotation * Vec3{0.0f, 0.0f, 1.5f});
        world.addComponent(newRoot, body);
        world.addComponent(newRoot, VesselComponent{});

        for (const ecs::Entity part : partsToDetach)
        {
            if (auto* component = world.tryGetComponent<PartComponent>(part))
            {
                component->vessel = newRoot;
            }
        }
        return newRoot;
    }

    ecs::Entity decoupleAt(ecs::World& world, ecs::Entity decouplerPart)
    {
        const auto* decoupler = world.tryGetComponent<PartComponent>(decouplerPart);
        if (decoupler == nullptr)
        {
            return {};
        }
        const ecs::Entity vessel = decoupler->vessel;

        // Snapshot this vessel's parts and joints.
        std::vector<ecs::Entity> parts;
        world.forEach<PartComponent>([&](ecs::Entity entity, PartComponent& part) {
            if (part.vessel == vessel)
            {
                parts.push_back(entity);
            }
        });
        struct JointRecord
        {
            ecs::Entity entity;
            ecs::Entity a, b;
        };
        std::vector<JointRecord> joints;
        world.forEach<JointComponent>([&](ecs::Entity entity, JointComponent& joint) {
            const auto* partA = world.tryGetComponent<PartComponent>(joint.partA);
            if (partA != nullptr && partA->vessel == vessel)
            {
                joints.push_back({entity, joint.partA, joint.partB});
            }
        });

        // The joint to sever: the decoupler's TAIL-side link (largest +Z
        // local offset of the far part — away from the nose/root).
        ecs::Entity severed{};
        f32 bestZ = -1.0e9f;
        for (const JointRecord& joint : joints)
        {
            const ecs::Entity other = (joint.a == decouplerPart) ? joint.b
                                      : (joint.b == decouplerPart) ? joint.a
                                                                   : ecs::Entity{};
            if (other.isNull())
            {
                continue;
            }
            if (const auto* otherPart = world.tryGetComponent<PartComponent>(other);
                otherPart != nullptr && otherPart->localPosition.z > bestZ)
            {
                bestZ = otherPart->localPosition.z;
                severed = joint.entity;
            }
        }
        if (severed.isNull())
        {
            return {};
        }

        // Connectivity WITHOUT the severed joint, seeded from the
        // nose-most part (the de-facto root of the stack).
        ecs::Entity rootPart = parts.front();
        f32 rootZ = 1.0e9f;
        for (const ecs::Entity part : parts)
        {
            const auto& component = world.getComponent<PartComponent>(part);
            if (component.localPosition.z < rootZ)
            {
                rootZ = component.localPosition.z;
                rootPart = part;
            }
        }
        std::vector<ecs::Entity> reachable{rootPart};
        for (usize scan = 0; scan < reachable.size(); ++scan)
        {
            for (const JointRecord& joint : joints)
            {
                if (joint.entity == severed)
                {
                    continue;
                }
                ecs::Entity next{};
                if (joint.a == reachable[scan]) { next = joint.b; }
                else if (joint.b == reachable[scan]) { next = joint.a; }
                if (next.isNull())
                {
                    continue;
                }
                bool known = false;
                for (const ecs::Entity seen : reachable)
                {
                    if (seen == next) { known = true; break; }
                }
                if (!known)
                {
                    reachable.push_back(next);
                }
            }
        }

        std::vector<ecs::Entity> detached;
        for (const ecs::Entity part : parts)
        {
            bool kept = false;
            for (const ecs::Entity seen : reachable)
            {
                if (seen == part) { kept = true; break; }
            }
            if (!kept)
            {
                detached.push_back(part);
            }
        }
        if (detached.empty())
        {
            return {};
        }
        world.destroyEntity(severed);
        return splitVessel(world, vessel, detached);
    }

    bool dockVessels(ecs::World& world, ecs::Entity portPartA, ecs::Entity portPartB)
    {
        auto* portA = world.tryGetComponent<PartComponent>(portPartA);
        auto* portB = world.tryGetComponent<PartComponent>(portPartB);
        if (portA == nullptr || portB == nullptr || portA->vessel == portB->vessel)
        {
            return false;
        }
        const ecs::Entity vesselA = portA->vessel;
        const ecs::Entity vesselB = portB->vessel;
        const auto* rootA = world.tryGetComponent<TransformComponent>(vesselA);
        if (rootA == nullptr)
        {
            return false;
        }
        const Quat inverseRotation = glm::inverse(rootA->rotation);

        // Re-localize every part of B into A's frame (their WORLD poses are
        // current — the attachment system ran this tick).
        world.forEach<PartComponent, TransformComponent>(
            [&](ecs::Entity, PartComponent& part, TransformComponent& transform) {
                if (part.vessel != vesselB)
                {
                    return;
                }
                part.vessel = vesselA;
                part.localPosition =
                    inverseRotation * Vec3(transform.position - rootA->position);
                part.localRotation = inverseRotation * transform.rotation;
            });

        connectParts(world, portPartA, portPartB, 1, 1, JointType::Docking, 3.0e5,
                     3.0e5);
        world.destroyEntity(vesselB); // the absorbed root vanishes
        return true;
    }

    // ------------------------------------------------------------------------
    // VesselAssemblySystem
    // ------------------------------------------------------------------------
    void VesselAssemblySystem::update(ecs::World& world, f32 /*deltaSeconds*/)
    {
        // Zero the accumulators.
        world.forEach<VesselComponent>([](ecs::Entity, VesselComponent& vessel) {
            vessel = VesselComponent{};
        });

        // The vessel's GROUND HULL, in vessel space. It is accumulated from
        // the same collider shapes the VAB validates placement against, so
        // what a rocket rests on is what a rocket is made of — the day a
        // stage separates, the remaining hull shrinks with it.
        std::unordered_map<ecs::Entity, std::pair<Vec3, Vec3>> hulls;

        // Accumulate every part into its vessel.
        world.forEach<PartComponent>([&world, &hulls](ecs::Entity entity,
                                                      PartComponent& part) {
            auto* vessel = world.tryGetComponent<VesselComponent>(part.vessel);
            const PartDefinition* definition = findDefinition(part.definitionId);
            if (vessel == nullptr || definition == nullptr)
            {
                return;
            }

            constexpr f32 kHuge = 1.0e9f;
            auto [entry, inserted] = hulls.try_emplace(
                part.vessel, Vec3{kHuge, kHuge, kHuge}, Vec3{-kHuge, -kHuge, -kHuge});
            expandPartHullBounds(*definition, part.localPosition, part.localRotation,
                                     entry->second.first, entry->second.second);
            vessel->dryMassKg += definition->dryMassKg;
            vessel->totalCostCredits += definition->costCredits;
            vessel->dragCoefficientArea += definition->dragCoefficientArea;
            vessel->solarChargeRateKw += definition->chargeRateKw;
            vessel->partCount += 1;
            if (definition->thrustNewtons > 0.0 && definition->specificImpulseS > 0.0)
            {
                vessel->maxThrustNewtons += definition->thrustNewtons;
                vessel->maxMassFlowKgps += definition->thrustNewtons /
                                           (definition->specificImpulseS * 9.80665);
            }

            // Carried resources weigh the vessel down (fuel above all).
            if (const auto* inventory =
                    world.tryGetComponent<factory::InventoryComponent>(entity))
            {
                for (const factory::InventorySlot& slot : inventory->slots)
                {
                    if (slot.resource != res::Resource::Count)
                    {
                        vessel->totalMassKg +=
                            slot.units * res::definition(slot.resource).massPerUnitKg;
                    }
                }
            }
        });

        // Push the aggregate into the vessel's rigid body.
        std::vector<std::pair<ecs::Entity, phys::GroundHullComponent>> pendingHulls;
        world.forEach<VesselComponent, phys::DynamicBodyComponent>(
            [&hulls, &pendingHulls](ecs::Entity entity, VesselComponent& vessel,
                                    phys::DynamicBodyComponent& body) {
                if (vessel.partCount == 0)
                {
                    return; // not part-built (EVA capsule, asteroid...)
                }
                vessel.totalMassKg += vessel.dryMassKg;
                body.mass = vessel.totalMassKg;
                body.ballisticFactor =
                    vessel.dragCoefficientArea / std::max(vessel.totalMassKg, 1.0);

                const auto found = hulls.find(entity);
                if (found == hulls.end() ||
                    found->second.first.x > found->second.second.x)
                {
                    return; // no collider geometry: leave the hull alone
                }
                phys::GroundHullComponent hull{};
                hull.centre = (found->second.first + found->second.second) * 0.5f;
                hull.halfExtents = (found->second.second - found->second.first) * 0.5f;
                pendingHulls.emplace_back(entity, hull);
            });

        // Applied AFTER the iteration: adding a component moves the entity
        // between archetypes, and doing that mid-forEach is how an ECS eats
        // its own iterator.
        for (const auto& [entity, hull] : pendingHulls)
        {
            if (auto* existing = world.tryGetComponent<phys::GroundHullComponent>(entity))
            {
                *existing = hull;
            }
            else
            {
                world.addComponent(entity, hull);
            }
        }
    }

    // ------------------------------------------------------------------------
    // PartAttachmentSystem
    // ------------------------------------------------------------------------
    void PartAttachmentSystem::update(ecs::World& world, f32 /*deltaSeconds*/)
    {
        world.forEach<TransformComponent, PreviousTransformComponent, PartComponent>(
            [&world](ecs::Entity, TransformComponent& transform,
                     PreviousTransformComponent& previous, PartComponent& part) {
                const auto* vessel =
                    world.tryGetComponent<TransformComponent>(part.vessel);
                if (vessel == nullptr)
                {
                    return; // orphaned part: floats where it was
                }
                transform.position =
                    vessel->position + WorldVec3(vessel->rotation * part.localPosition);
                transform.rotation = vessel->rotation * part.localRotation;

                if (const auto* vesselPrevious =
                        world.tryGetComponent<PreviousTransformComponent>(part.vessel))
                {
                    previous.position =
                        vesselPrevious->position +
                        WorldVec3(vesselPrevious->rotation * part.localPosition);
                    previous.rotation = vesselPrevious->rotation * part.localRotation;
                }
                else
                {
                    previous.position = transform.position;
                    previous.rotation = transform.rotation;
                }
            });
    }
} // namespace sw::parts
