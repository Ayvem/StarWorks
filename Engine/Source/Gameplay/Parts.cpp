#include "Gameplay/Parts.hpp"

#include <cstring>

#include "Core/FileSystem.hpp"
#include "Core/Json.hpp"
#include "Core/Log.hpp"
#include "ECS/World.hpp"
#include "Gameplay/PartGeometry.hpp"
#include "Physics/Aerodynamics.hpp"
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
        constexpr std::string_view kAnimationTriggerNames[] = {"toggle", "throttle"};
        constexpr std::string_view kAnimationVerbNames[] = {"open-close", "on-off",
                                                           "extend-retract", "deploy-stow"};
        constexpr std::string_view kAnimationGateNames[] = {"nothing", "power", "thrust"};
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
        definition.thrustDirection =
            vec3FromJson(root.find("thrustDirection"), Vec3{0.0f, 0.0f, -1.0f});
        definition.flexStiffnessNmPerRad = root.number("flexStiffnessNmPerRad", 0.0);
        definition.flexYieldNm = root.number("flexYieldNm", 0.0);
        if (glm::length(definition.thrustDirection) < 1.0e-6f)
        {
            definition.thrustDirection = Vec3{0.0f, 0.0f, -1.0f};
        }
        definition.thrustDirection = glm::normalize(definition.thrustDirection);
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
                // The animation fields are absent from every file written
                // before they existed, and `find` returning nullptr is what
                // makes that a non-event in both directions.
                shape.animation = static_cast<i32>(entry.number("animation", -1.0));
                shape.endPosition = vec3FromJson(entry.find("endPosition"), shape.position);
                shape.endRotationDeg =
                    vec3FromJson(entry.find("endRotationDeg"), shape.rotationDeg);
                shape.endEmissive = static_cast<f32>(entry.number("endEmissive", -1.0));
                definition.shapes.push_back(shape);
            }
        }

        if (const json::Value* animations = root.find("animations"))
        {
            for (const json::Value& entry : animations->asArray())
            {
                if (definition.animations.size() >= kMaxPartAnimations)
                {
                    SW_LOG_WARN("Parts", "'{}': more than {} animations, ignoring the rest",
                                path.string(), kMaxPartAnimations);
                    break;
                }
                PartAnimation animation{};
                const std::string name = entry.string("name");
                std::strncpy(animation.name, name.c_str(),
                             PartAnimation::kNameCapacity - 1);
                (void)enumFromName(kAnimationTriggerNames, entry.string("trigger"),
                                   animation.trigger);
                (void)enumFromName(kAnimationVerbNames, entry.string("verbs"),
                                   animation.verbs);
                (void)enumFromName(kAnimationGateNames, entry.string("gates"),
                                   animation.gates);
                animation.durationSeconds =
                    static_cast<f32>(entry.number("durationSeconds", 3.0));
                animation.startsOpen = entry.boolean("startsOpen", false);
                definition.animations.push_back(animation);
            }
        }
        // A shape pointing at an animation that is not there would be a
        // silent invisible group, so it is corrected here rather than
        // discovered later as a panel that never appears.
        for (PartShape& shape : definition.shapes)
        {
            if (shape.animation >= static_cast<i32>(definition.animations.size()))
            {
                SW_LOG_WARN("Parts", "'{}': shape references animation {}, which does "
                                     "not exist; treating it as static",
                            path.string(), shape.animation);
                shape.animation = -1;
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
        if (definition.thrustNewtons > 0.0)
        {
            root.set("thrustDirection", vec3ToJson(definition.thrustDirection));
        }
        if (definition.flexStiffnessNmPerRad > 0.0)
        {
            root.set("flexStiffnessNmPerRad",
                     json::Value(definition.flexStiffnessNmPerRad));
            root.set("flexYieldNm", json::Value(definition.flexYieldNm));
        }
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
            // WRITTEN ONLY WHEN THEY SAY SOMETHING. Thirty-four shipped parts
            // have no animation, and emitting four dead keys on every shape of
            // every one of them would put a thousand lines of noise into the
            // next diff of an unrelated edit.
            if (shape.animation >= 0)
            {
                entry.set("animation", json::Value(static_cast<f64>(shape.animation)));
                entry.set("endPosition", vec3ToJson(shape.endPosition));
                entry.set("endRotationDeg", vec3ToJson(shape.endRotationDeg));
                if (shape.endEmissive >= 0.0f)
                {
                    entry.set("endEmissive",
                              json::Value(static_cast<f64>(shape.endEmissive)));
                }
            }
            shapes.push(std::move(entry));
        }
        root.set("shapes", std::move(shapes));

        if (!definition.animations.empty())
        {
            json::Value animations = json::Value::makeArray();
            for (const PartAnimation& animation : definition.animations)
            {
                json::Value entry = json::Value::makeObject();
                entry.set("name", json::Value(std::string(animation.name)));
                entry.set("trigger",
                          json::Value(std::string(kAnimationTriggerNames[static_cast<usize>(
                              animation.trigger)])));
                entry.set("verbs",
                          json::Value(std::string(
                              kAnimationVerbNames[static_cast<usize>(animation.verbs)])));
                entry.set("gates",
                          json::Value(std::string(
                              kAnimationGateNames[static_cast<usize>(animation.gates)])));
                entry.set("durationSeconds",
                          json::Value(static_cast<f64>(animation.durationSeconds)));
                entry.set("startsOpen", json::Value(animation.startsOpen));
                animations.push(std::move(entry));
            }
            root.set("animations", std::move(animations));
        }

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
        // A DISCARDED STAGE STILL FLIES. It inherits the aerodynamic slot
        // for the same reason it inherits a velocity: it is a vehicle now,
        // with its own balance point and its own fins or lack of them, and
        // watching a spent booster tumble away is the whole reward for
        // having modelled moments at all.
        if (world.tryGetComponent<aero::AeroStateComponent>(vessel) != nullptr)
        {
            world.addComponent(newRoot, aero::AeroStateComponent{});
        }

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
        // Zero the accumulators — but NOT the motion history.
        //
        // This pass rebuilds the vessel from its parts every tick, and it did
        // so by assigning a default-constructed component over the top, which
        // is exactly right for every field that is a SUM and catastrophic for
        // the two that are a MEMORY. PartFlexSystem differences the velocity
        // across a tick to get an acceleration; with the previous velocity
        // wiped each pass, that difference was the vessel's whole orbital
        // speed divided by the step — nine kilometres a second over a
        // fiftieth of one, fifty thousand g — and every flexible part on the
        // Endurance yielded to the clamp on the first frame and stayed there.
        world.forEach<VesselComponent>([](ecs::Entity, VesselComponent& vessel) {
            const WorldVec3 previousVelocity = vessel.previousVelocity;
            const Vec3 previousAngular = vessel.previousAngularVelocity;
            vessel = VesselComponent{};
            vessel.previousVelocity = previousVelocity;
            vessel.previousAngularVelocity = previousAngular;
        });

        // The vessel's GROUND HULL, in vessel space. It is accumulated from
        // the same collider shapes the VAB validates placement against, so
        // what a rocket rests on is what a rocket is made of — the day a
        // stage separates, the remaining hull shrinks with it.
        std::unordered_map<ecs::Entity, std::pair<Vec3, Vec3>> hulls;

        /// One part reduced to what the balance and the spin care about.
        /// Collected here rather than folded straight into a running sum
        /// because the moment of inertia is taken about the CENTRE OF MASS,
        /// and the centre of mass is not known until the last part has been
        /// weighed.
        struct MassPoint
        {
            f64 massKg = 0.0;
            Vec3 position{0.0f};
            Vec3 halfExtents{0.5f};
        };
        std::unordered_map<ecs::Entity, std::vector<MassPoint>> massPoints;

        /// One engine reduced to what a force needs: how hard, which way and
        /// WHERE. Collected rather than summed for the same reason the mass
        /// points are — the torque is taken about the centre of mass, and the
        /// centre of mass is not known until the last part has been weighed.
        struct ThrustPoint
        {
            Vec3 position{0.0f};
            Vec3 forceN{0.0f};
        };
        std::unordered_map<ecs::Entity, std::vector<ThrustPoint>> thrustPoints;

        // Accumulate every part into its vessel.
        world.forEach<PartComponent>([&world, &hulls, &massPoints, &thrustPoints](
                                         ecs::Entity entity, PartComponent& part) {
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
            // WHAT A STOWED PART DOES NOT DO. This is the line that separates
            // an animation from a decoration: a folded solar wing makes no
            // power and a shut-down engine makes no thrust, and both fall out
            // of the same one number — the phase of whichever animation the
            // part's author said gates them.
            //
            // Half open is half of it, deliberately. A panel caught mid-travel
            // really is presenting half its area to the sun, and a rule that
            // waited for the phase to reach exactly 1 would make the last
            // instant of a three-second deployment carry all of the effect.
            const auto* animation =
                world.tryGetComponent<PartAnimationComponent>(entity);
            const f64 powerGate = animationGate(*definition, animation,
                                                AnimationGates::Power);
            const f64 thrustGate = animationGate(*definition, animation,
                                                 AnimationGates::Thrust);
            vessel->solarChargeRateKw += definition->chargeRateKw * powerGate;
            vessel->partCount += 1;
            if (definition->thrustNewtons > 0.0 && definition->specificImpulseS > 0.0)
            {
                // WHERE IT PUSHES AND FROM WHERE. The scalar below is still
                // the total for the fuel sums and the HUD; this is the same
                // thrust as a vector at a point, which is what a torque needs.
                thrustPoints[part.vessel].push_back(
                    {part.localPosition,
                     part.localRotation *
                         (definition->thrustDirection *
                          static_cast<f32>(definition->thrustNewtons * thrustGate))});
                vessel->maxThrustNewtons += definition->thrustNewtons * thrustGate;
                vessel->maxMassFlowKgps += definition->thrustNewtons * thrustGate /
                                           (definition->specificImpulseS * 9.80665);
            }

            // Carried resources weigh the vessel down (fuel above all).
            f64 partMassKg = definition->dryMassKg;
            if (const auto* inventory =
                    world.tryGetComponent<factory::InventoryComponent>(entity))
            {
                for (const factory::InventorySlot& slot : inventory->slots)
                {
                    if (slot.resource != res::Resource::Count)
                    {
                        const f64 carried =
                            slot.units * res::definition(slot.resource).massPerUnitKg;
                        vessel->totalMassKg += carried;
                        partMassKg += carried;
                    }
                }
            }

            // ...and it weighs it down SOMEWHERE. A tank draining at the
            // tail moves the balance point forward; the fuel has to be
            // where the tank is for that to happen at all.
            constexpr f32 kHugeLocal = 1.0e9f;
            Vec3 partMin{kHugeLocal, kHugeLocal, kHugeLocal};
            Vec3 partMax{-kHugeLocal, -kHugeLocal, -kHugeLocal};
            expandPartHullBounds(*definition, part.localPosition, part.localRotation,
                                 partMin, partMax);
            MassPoint point{};
            point.massKg = partMassKg;
            point.position = part.localPosition;
            if (partMin.x <= partMax.x)
            {
                point.halfExtents = (partMax - partMin) * 0.5f;
            }
            massPoints[part.vessel].push_back(point);
        });

        // Push the aggregate into the vessel's rigid body.
        std::vector<std::pair<ecs::Entity, phys::GroundHullComponent>> pendingHulls;
        world.forEach<VesselComponent, phys::DynamicBodyComponent>(
            [&hulls, &massPoints, &thrustPoints, &pendingHulls](ecs::Entity entity,
                                                 VesselComponent& vessel,
                                                 phys::DynamicBodyComponent& body) {
                if (vessel.partCount == 0)
                {
                    return; // not part-built (EVA capsule, asteroid...)
                }
                vessel.totalMassKg += vessel.dryMassKg;
                body.mass = vessel.totalMassKg;
                body.ballisticFactor =
                    vessel.dragCoefficientArea / std::max(vessel.totalMassKg, 1.0);

                // ---- balance and spin -------------------------------------
                if (const auto points = massPoints.find(entity);
                    points != massPoints.end() && !points->second.empty())
                {
                    f64 total = 0.0;
                    glm::dvec3 weighted{0.0};
                    for (const MassPoint& point : points->second)
                    {
                        total += point.massKg;
                        weighted += glm::dvec3(point.position) * point.massKg;
                    }
                    if (total > 1.0e-6)
                    {
                        vessel.centreOfMass = Vec3(weighted / total);
                    }
                    // Parallel axes: each part's own inertia about its own
                    // centre, plus its mass times how far off the vessel's
                    // axis it sits. Diagonal only — a rocket's principal
                    // axes are its own, near enough, and the products of
                    // inertia of a symmetric stack are zero anyway.
                    glm::dvec3 inertia{0.0};
                    for (const MassPoint& point : points->second)
                    {
                        const glm::dvec3 own(aero::boxInertia(point.massKg,
                                                              point.halfExtents));
                        const glm::dvec3 offset(point.position - vessel.centreOfMass);
                        inertia += own + point.massKg *
                                             glm::dvec3(offset.y * offset.y +
                                                            offset.z * offset.z,
                                                        offset.x * offset.x +
                                                            offset.z * offset.z,
                                                        offset.x * offset.x +
                                                            offset.y * offset.y);
                    }
                    vessel.inertiaKgM2 = Vec3(glm::max(inertia, glm::dvec3(1.0)));
                }

                // ---- what the engines do, now that the balance is known ----
                //
                // A SYMMETRIC CRAFT GETS ZERO TORQUE FOR FREE. Nothing here
                // special-cases it: four modules spaced around a ring produce
                // four arms that cancel, and shutting one off leaves three
                // that do not. That is the whole feature, and it is one cross
                // product.
                if (const auto engines = thrustPoints.find(entity);
                    engines != thrustPoints.end())
                {
                    glm::dvec3 force{0.0};
                    glm::dvec3 torque{0.0};
                    for (const ThrustPoint& engine : engines->second)
                    {
                        const glm::dvec3 f(engine.forceN);
                        force += f;
                        torque += glm::cross(
                            glm::dvec3(engine.position - vessel.centreOfMass), f);
                    }
                    vessel.thrustForceN = Vec3(force);
                    vessel.thrustTorqueNm = Vec3(torque);
                }

                const auto found = hulls.find(entity);
                if (found == hulls.end() ||
                    found->second.first.x > found->second.second.x)
                {
                    return; // no collider geometry: leave the hull alone
                }
                phys::GroundHullComponent hull{};
                hull.centre = (found->second.first + found->second.second) * 0.5f;
                hull.halfExtents = (found->second.second - found->second.first) * 0.5f;
                vessel.halfExtents = hull.halfExtents;
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
    // PartFlexSystem
    // ------------------------------------------------------------------------
    void PartFlexSystem::update(ecs::World& world, f32 deltaSeconds)
    {
        if (!(deltaSeconds > 0.0f))
        {
            return;
        }
        // ---- this tick's acceleration, by difference ------------------------
        //
        // Thrust, the ground, the air and a collision all change a vessel's
        // velocity and none of them records what it did anywhere in common.
        // Differencing across the tick catches all four and cannot fall out of
        // step with any of them.
        world.forEach<VesselComponent, TransformComponent, phys::DynamicBodyComponent>(
            [deltaSeconds](ecs::Entity, VesselComponent& vessel,
                           TransformComponent& transform,
                           phys::DynamicBodyComponent& body) {
                // THE FIRST DIFFERENCE IS NOT AN ACCELERATION. A vessel
                // seen for the first time has no previous velocity, and
                // subtracting zero from nine kilometres a second is a number
                // that bends every strut on it flat before the first frame is
                // drawn.
                if (vessel.previousVelocity == WorldVec3{0.0} &&
                    glm::dot(body.velocity, body.velocity) > 1.0)
                {
                    vessel.previousVelocity = body.velocity;
                    vessel.previousAngularVelocity = body.angularVelocity;
                    vessel.accelerationMps2 = Vec3{0.0f};
                    vessel.angularAccelRadPerS2 = Vec3{0.0f};
                    return;
                }
                const Quat toVessel = glm::inverse(transform.rotation);
                const WorldVec3 deltaV = body.velocity - vessel.previousVelocity;
                vessel.accelerationMps2 =
                    toVessel * (Vec3(deltaV) / deltaSeconds);
                vessel.angularAccelRadPerS2 =
                    (body.angularVelocity - vessel.previousAngularVelocity) /
                    deltaSeconds;
                vessel.previousVelocity = body.velocity;
                vessel.previousAngularVelocity = body.angularVelocity;
            });

        // ---- one damped spring per flexible part ----------------------------
        world.forEach<PartComponent, PartFlexComponent>(
            [&world, deltaSeconds](ecs::Entity, PartComponent& part,
                                   PartFlexComponent& flex) {
                const PartDefinition* definition = findDefinition(part.definitionId);
                const auto* vessel = world.tryGetComponent<VesselComponent>(part.vessel);
                if (definition == nullptr || vessel == nullptr ||
                    !(definition->flexStiffnessNmPerRad > 0.0))
                {
                    return;
                }

                // WHAT THIS PART IS BEING ASKED TO CARRY. Its own mass times
                // the acceleration it is being given at ITS location, which on
                // a turning vessel is not the acceleration at the centre of
                // mass: alpha x r is what a part far from the balance point
                // feels and it is the whole reason a long craft bends when a
                // single engine is shut down.
                const Vec3 arm = part.localPosition - vessel->centreOfMass;
                const Vec3 properAccel =
                    vessel->accelerationMps2 +
                    glm::cross(vessel->angularAccelRadPerS2, arm);

                // ...AND ONLY ITS LATERAL PART BENDS ANYTHING. A beam pushed
                // along its own length is in compression, not in bending. The
                // part's own long axis is the direction it sticks out from the
                // vessel's balance point, which is also the lever its mass acts
                // through.
                const f32 armLength = glm::length(arm);
                const Vec3 outward =
                    (armLength > 1.0e-3f) ? arm / armLength : Vec3{0.0f, 0.0f, -1.0f};
                const Vec3 lateral =
                    properAccel - outward * glm::dot(properAccel, outward);

                // Moment = mass x lateral acceleration x lever, and the lever
                // is the part's own reach rather than its distance from the
                // balance point: a strut bends about ITS root, not about the
                // ship's middle.
                const f32 lever = std::max(partBoundsRadius(*definition), 0.25f);
                const f32 moment =
                    static_cast<f32>(definition->dryMassKg) * glm::length(lateral) *
                    lever;
                const Vec3 axis = glm::cross(outward, lateral);
                const f32 axisLength = glm::length(axis);
                const Vec3 target =
                    (axisLength > 1.0e-6f)
                        ? (axis / axisLength) *
                              (moment / static_cast<f32>(definition->flexStiffnessNmPerRad))
                        : Vec3{0.0f};

                // A DAMPED SPRING, not a direct assignment, and that is where
                // the vibration comes from: a panel handed its steady
                // deflection instantly would simply be bent, and one that
                // reaches it through a second-order system overshoots and
                // rings, which is what a big flexible thing does after the
                // engines light.
                constexpr f32 kOmega = 6.0f;      // ~1 Hz, a big structure
                constexpr f32 kDamping = 0.12f;   // lightly damped: it rings
                flex.rate += ((target - flex.elastic) * (kOmega * kOmega) -
                              flex.rate * (2.0f * kDamping * kOmega)) *
                             deltaSeconds;
                flex.elastic += flex.rate * deltaSeconds;

                // YIELD: past this the part stops springing back. The excess
                // moves into the permanent set, which is the difference
                // between a panel that rings and a leg that stays bent.
                if (definition->flexYieldNm > 0.0)
                {
                    const f32 yieldAngle =
                        static_cast<f32>(definition->flexYieldNm /
                                         definition->flexStiffnessNmPerRad);
                    const f32 bend = glm::length(flex.elastic);
                    if (bend > yieldAngle)
                    {
                        const Vec3 direction = flex.elastic / bend;
                        flex.permanent += direction * (bend - yieldAngle);
                        flex.elastic = direction * yieldAngle;
                        flex.rate *= 0.25f; // the energy went into the metal
                    }
                }
                // A bend of more than a right angle is a part that has come
                // off, not a part that is bent. Held here until breakage
                // lands; without the clamp a spike would spin the pose.
                constexpr f32 kMaxBend = 1.2f;
                if (glm::length(flex.permanent) > kMaxBend)
                {
                    flex.permanent = glm::normalize(flex.permanent) * kMaxBend;
                }
                if (glm::length(flex.elastic) > kMaxBend)
                {
                    flex.elastic = glm::normalize(flex.elastic) * kMaxBend;
                    flex.rate = Vec3{0.0f};
                }
            });
    }

    // ------------------------------------------------------------------------
    // PartAttachmentSystem
    // ------------------------------------------------------------------------
    void PartAttachmentSystem::update(ecs::World& world, f32 /*deltaSeconds*/)
    {
        world.forEach<TransformComponent, PreviousTransformComponent, PartComponent>(
            [&world](ecs::Entity entity, TransformComponent& transform,
                     PreviousTransformComponent& previous, PartComponent& part) {
                const auto* vessel =
                    world.tryGetComponent<TransformComponent>(part.vessel);
                if (vessel == nullptr)
                {
                    return; // orphaned part: floats where it was
                }
                // ---- THE BEND, applied where the pose is made -------------
                //
                // This is the line that turns three numbers into a structure
                // that visibly flexes. The part is rotated about the vessel's
                // balance point by its total deflection, so it both LOOKS bent
                // and, if it carries an engine, PUSHES bent — which is the
                // feedback that makes a long craft wobble instead of settling.
                //
                // About the balance point and not about the part's own origin,
                // because a deflection is a rotation of the member relative to
                // the ship, and a member that rotated about its own centre
                // would pivot in place without going anywhere.
                Vec3 localPosition = part.localPosition;
                Quat localRotation = part.localRotation;
                if (const auto* flex = world.tryGetComponent<PartFlexComponent>(entity))
                {
                    const Vec3 bend = flex->elastic + flex->permanent;
                    const f32 angle = glm::length(bend);
                    if (angle > 1.0e-5f)
                    {
                        const auto* aggregate =
                            world.tryGetComponent<VesselComponent>(part.vessel);
                        const Vec3 pivot =
                            (aggregate != nullptr) ? aggregate->centreOfMass : Vec3{0.0f};
                        const Quat deflection = glm::angleAxis(angle, bend / angle);
                        localPosition =
                            pivot + deflection * (part.localPosition - pivot);
                        localRotation = deflection * part.localRotation;
                    }
                }
                transform.position =
                    vessel->position + WorldVec3(vessel->rotation * localPosition);
                transform.rotation = vessel->rotation * localRotation;

                if (const auto* vesselPrevious =
                        world.tryGetComponent<PreviousTransformComponent>(part.vessel))
                {
                    previous.position =
                        vesselPrevious->position +
                        WorldVec3(vesselPrevious->rotation * localPosition);
                    previous.rotation = vesselPrevious->rotation * localRotation;
                }
                else
                {
                    previous.position = transform.position;
                    previous.rotation = transform.rotation;
                }
            });
    }
    f64 animationGate(const PartDefinition& definition,
                      const PartAnimationComponent* state, AnimationGates gate)
    {
        // No animation gates this: the part works, as every part did before
        // animations existed. The default has to be 1 and not 0, or adding the
        // feature would silently switch off every engine in the game.
        f64 scale = 1.0;
        if (state == nullptr)
        {
            return scale;
        }
        const u32 count =
            std::min<u32>(state->count, static_cast<u32>(definition.animations.size()));
        for (u32 i = 0; i < count; ++i)
        {
            if (definition.animations[i].gates == gate)
            {
                scale = std::min(scale, static_cast<f64>(
                                            glm::clamp(state->phase[i], 0.0f, 1.0f)));
            }
        }
        return scale;
    }

    void PartAnimationSystem::update(ecs::World& world, f32 deltaSeconds)
    {
        world.forEach<PartComponent, PartAnimationComponent>(
            [&](ecs::Entity, PartComponent& part, PartAnimationComponent& state) {
                const PartDefinition* definition = findDefinition(part.definitionId);
                if (definition == nullptr)
                {
                    return;
                }
                const u32 count = std::min<u32>(
                    state.count, static_cast<u32>(definition->animations.size()));
                for (u32 i = 0; i < count; ++i)
                {
                    const PartAnimation& animation = definition->animations[i];
                    if (animation.trigger == AnimationTrigger::Throttle)
                    {
                        // The throttle IS the phase, so a nozzle brightens as
                        // the engine is opened up. No travel time: the target
                        // is written straight in by whoever knows the throttle.
                        state.phase[i] = state.target[i];
                        continue;
                    }
                    const f32 rate =
                        1.0f / std::max(animation.durationSeconds, 0.05f);
                    const f32 step = rate * deltaSeconds;
                    const f32 remaining = state.target[i] - state.phase[i];
                    state.phase[i] = (std::abs(remaining) <= step)
                                         ? state.target[i]
                                         : state.phase[i] + std::copysign(step, remaining);
                }
            });
    }

    // ------------------------------------------------------------------------
    // CHASLES' THEOREM, or why a hinge is not a lerp
    // ------------------------------------------------------------------------
    HingeMotion hingeBetween(const Vec3& positionA, const Quat& rotationA,
                             const Vec3& positionB, const Quat& rotationB)
    {
        HingeMotion hinge{};
        // The rotation that takes A's orientation to B's.
        const Quat delta = glm::normalize(rotationB * glm::inverse(rotationA));
        f32 angle = 2.0f * std::acos(glm::clamp(delta.w, -1.0f, 1.0f));
        Vec3 axis = Vec3(delta.x, delta.y, delta.z);
        const f32 axisLength = glm::length(axis);
        if (!(angle > 1.0e-4f) || !(axisLength > 1.0e-6f))
        {
            // No rotation at all: the whole motion is a slide. A landing gear
            // dropping straight out of a bay is this case, and it must not go
            // through the pivot solve below — (I - R) is the zero matrix there
            // and there is no pivot to find.
            hinge.angleRadians = 0.0f;
            hinge.axis = Vec3{0.0f, 1.0f, 0.0f};
            hinge.slide = positionB - positionA;
            hinge.pivot = positionA;
            return hinge;
        }
        axis /= axisLength;
        if (angle > 3.14159265f)
        {
            // Take the short way round, and take the axis with it.
            angle = 6.28318531f - angle;
            axis = -axis;
        }
        hinge.axis = axis;
        hinge.angleRadians = angle;

        // The motion is x -> R*x + t with t = pB - R*pA. A fixed point c
        // satisfies (I - R) c = t. (I - R) is singular along the axis — every
        // point on the axis moves only by the slide — so the system is solved
        // in the PLANE perpendicular to it, where (I - R) is invertible for
        // any angle that is not a full turn.
        const Mat3 rotation = glm::mat3_cast(delta);
        const Vec3 translation = positionB - rotation * positionA;
        // Split the translation: the part along the axis is the screw's slide
        // and has no bearing on where the pivot is.
        const f32 along = glm::dot(translation, axis);
        hinge.slide = axis * along;
        const Vec3 planar = translation - hinge.slide;
        // In the perpendicular plane, (I - R) acting on c has a closed form:
        // its inverse is (I/2 + cot(angle/2)/2 * [axis]x) restricted there.
        const f32 half = angle * 0.5f;
        const f32 cot = std::cos(half) / std::max(std::sin(half), 1.0e-6f);
        hinge.pivot = 0.5f * planar + 0.5f * cot * glm::cross(axis, planar);
        return hinge;
    }

    void poseAlongHinge(const HingeMotion& hinge, const Vec3& positionA,
                        const Quat& rotationA, f32 phase, Vec3& outPosition,
                        Quat& outRotation)
    {
        const f32 t = glm::clamp(phase, 0.0f, 1.0f);
        if (!(std::abs(hinge.angleRadians) > 1.0e-4f))
        {
            outPosition = positionA + hinge.slide * t;
            outRotation = rotationA;
            return;
        }
        const Quat step = glm::angleAxis(hinge.angleRadians * t, hinge.axis);
        outPosition = hinge.pivot + step * (positionA - hinge.pivot) + hinge.slide * t;
        outRotation = glm::normalize(step * rotationA);
    }
} // namespace sw::parts
