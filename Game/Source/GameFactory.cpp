// ============================================================================
// GameFactory.cpp — Ground construction and the factory: buildings, hulls, cables, belts, pads.
// Split out of StarWorksGame.cpp; same class, one theme per translation unit.
// ============================================================================

#include "StarWorksGame.hpp"

#include "GameInternal.hpp"
#include "Systems.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <format>
#include <limits>

namespace game
{

    void StarWorksGame::orderVehicle(sw::ecs::Entity hall,
                                     const sw::parts::ShipBlueprint& design)
    {
        auto* assembly = m_world.tryGetComponent<sw::factory::AssemblyComponent>(hall);
        if (assembly == nullptr)
        {
            return;
        }
        const sw::parts::BillOfMaterials bill = sw::parts::blueprintCost(design);
        sw::factory::assemblyOrder(*assembly, design.name, bill.ironKg, bill.copperKg);
        SW_LOG_INFO("Game", "VAB: ordered '{}' — {:.0f} kg iron, {:.0f} kg copper",
                    design.name, bill.ironKg, bill.copperKg);
    }

    sw::f64 StarWorksGame::fuelVessel(sw::ecs::Entity vessel, sw::f64 availableUnits)
    {
        if (availableUnits <= 0.0)
        {
            return 0.0;
        }
        sw::f64 poured = 0.0;
        m_world.forEach<sw::parts::PartComponent, sw::factory::InventoryComponent>(
            [&](sw::ecs::Entity, sw::parts::PartComponent& part,
                sw::factory::InventoryComponent& inventory) {
                if (part.vessel != vessel || poured >= availableUnits)
                {
                    return;
                }
                const auto* definition = sw::parts::findDefinition(part.definitionId);
                if (definition == nullptr ||
                    definition->capacities[0].resource != sw::res::Resource::Fuel)
                {
                    return;
                }
                poured += sw::factory::inventoryAdd(inventory, sw::res::Resource::Fuel,
                                                    availableUnits - poured);
            });
        return poured;
    }

    // ------------------------------------------------------------------------
    // THE PAD
    //
    // A crate of vehicle arrives on the belt like any other good, and the
    // pad's job is to turn it back into a thing: pop the design's name off
    // the hall's queue — reached through the belt's own link channel, which
    // already records which machine is at the far end — build it standing on
    // the deck, and pour whatever fuel the pad has been stockpiling into it.
    //
    // It runs in the GAME, not the factory lane, because it makes entities:
    // parts, joints, a vessel. The factory layer's contract is that it moves
    // matter and nothing else.
    // ------------------------------------------------------------------------
    // ------------------------------------------------------------------------
    // IS SOMETHING STANDING ON THE PAD?
    //
    // Asked of the VESSELS, not of the hulls: a rocket is a root entity with
    // a swarm of parts hanging off it, and testing the root against the
    // deck's own footprint is one distance per vessel — cheap enough to ask
    // every frame, and it answers the question a launch director would ask.
    //
    // The radius comes from the pad's own deck boxes, so a wider LP-1
    // redrawn in Part Studio guards a wider deck. The vertical reach is
    // deliberately generous: a rocket fifty metres up is still ON the pad as
    // far as dropping another one there is concerned.
    // ------------------------------------------------------------------------
    bool StarWorksGame::padIsOccupied(sw::ecs::Entity pad)
    {
        const auto* padTransform = m_world.tryGetComponent<TransformComponent>(pad);
        if (padTransform == nullptr)
        {
            return false;
        }
        sw::f64 deckRadius = 12.0;
        if (const auto* building =
                m_world.tryGetComponent<sw::factory::BuildingComponent>(pad))
        {
            if (const auto* definition =
                    sw::parts::findDefinition(building->definitionId))
            {
                for (const sw::parts::HitBox& box :
                     sw::parts::effectiveHull(*definition))
                {
                    if (std::abs(box.center.x) <= std::abs(box.halfExtents.x) &&
                        std::abs(box.center.z) <= std::abs(box.halfExtents.z))
                    {
                        deckRadius = std::max(
                            deckRadius, static_cast<sw::f64>(std::max(
                                            std::abs(box.halfExtents.x),
                                            std::abs(box.halfExtents.z))));
                    }
                }
            }
        }
        // ...plus the height a launching rocket has to clear before the pad
        // counts as free again.
        constexpr sw::f64 kClearanceM = 60.0;
        const sw::f64 reach = deckRadius + kClearanceM;

        bool occupied = false;
        m_world.forEach<sw::parts::VesselComponent, TransformComponent>(
            [&](sw::ecs::Entity, sw::parts::VesselComponent&,
                TransformComponent& transform) {
                if (occupied)
                {
                    return;
                }
                occupied = glm::length(transform.position - padTransform->position) <
                           reach;
            });
        return occupied;
    }

    void StarWorksGame::updateLaunchPads()
    {
        std::vector<sw::ecs::Entity> pads;
        m_world.forEach<sw::factory::BuildingComponent, sw::factory::InventoryComponent>(
            [&](sw::ecs::Entity entity, sw::factory::BuildingComponent& building,
                sw::factory::InventoryComponent& inventory) {
                if (building.category == sw::factory::BuildingCategory::Pad &&
                    sw::factory::inventoryCount(inventory, sw::res::Resource::Vehicle) >=
                        1.0)
                {
                    pads.push_back(entity);
                }
            });

        for (const sw::ecs::Entity pad : pads)
        {
            // IS THERE ALREADY A ROCKET ON IT?
            //
            // A pad holds one vehicle. Unpacking a second one on top of the
            // first put two rockets in the same cubic metre — which the hull
            // solver then resolved by throwing one of them off the pad. The
            // crate simply waits: it is on the pad's own belt, it is not
            // going anywhere, and the panel says the deck is occupied.
            if (padIsOccupied(pad))
            {
                continue;
            }

            // WHICH design is in the crate. The belt that brought it names
            // its source, and the source is the hall that built it.
            std::string name;
            sw::ecs::Entity hall{};
            if (const auto* link =
                    m_world.tryGetComponent<sw::factory::ItemLinkComponent>(pad))
            {
                for (const sw::factory::LinkChannel& channel : link->channels)
                {
                    if (channel.resource != sw::res::Resource::Vehicle)
                    {
                        continue;
                    }
                    if (auto* queue =
                            m_world.tryGetComponent<sw::factory::VehicleQueueComponent>(
                                channel.source))
                    {
                        const std::string_view front =
                            sw::factory::vehicleQueueFront(*queue);
                        if (!front.empty())
                        {
                            name = std::string(front);
                            hall = channel.source;
                            break;
                        }
                    }
                }
            }
            const sw::parts::ShipBlueprint* design =
                name.empty() ? nullptr : sw::parts::findBlueprint(name);
            if (design == nullptr || !sw::parts::blueprintIsBuildable(*design))
            {
                // An unidentified crate is not destroyed and not unpacked:
                // it sits on the pad, and the panel shows it sitting there.
                // Silently deleting a rocket would be the worse answer.
                continue;
            }

            // ---- unpack it ------------------------------------------------
            std::vector<BlueprintPart> saved;
            saved.swap(m_blueprint);
            m_blueprint = partsFromDesign(*design);
            const sw::ecs::Entity vessel = instantiateBlueprint({}, pad);
            m_blueprint.swap(saved);
            if (vessel.isNull())
            {
                continue;
            }

            auto& inventory =
                m_world.getComponent<sw::factory::InventoryComponent>(pad);
            sw::factory::inventoryRemove(inventory, sw::res::Resource::Vehicle, 1.0);
            if (auto* queue =
                    m_world.tryGetComponent<sw::factory::VehicleQueueComponent>(hall))
            {
                sw::factory::vehicleQueuePop(*queue);
            }

            // ...and fuel it from the pad's own tanks. A rocket that arrives
            // dry is a rocket you have to feed by hand; the pad's second
            // conveyor mouth exists precisely so you do not.
            const sw::f64 fuel =
                sw::factory::inventoryCount(inventory, sw::res::Resource::Fuel);
            const sw::f64 poured = fuelVessel(vessel, fuel);
            if (poured > 0.0)
            {
                sw::factory::inventoryRemove(inventory, sw::res::Resource::Fuel, poured);
            }
            rebuildHulls();
            SW_LOG_INFO("Game", "PAD: '{}' rolled out — {:.0f} kg of fuel aboard", name,
                        poured);
        }
    }

    namespace
    {
        // WHAT A MACHINE DIGS FOR.
        //
        // A miner's yield is its recipe rate times the density of the thing
        // it is digging for, so the sample has to ask the ground about THAT
        // resource. Sampling a fixed one instead is not a rounding error: an
        // ice harvester on a lunar polar site reads the IRON field there,
        // which is barren, so `groundDensity` comes out at or near zero and
        // ProductionSystem parks the machine in kStarved for the rest of the
        // save. The harvester would never produce a gram, on the one site
        // the whole colony is meant to be founded on.
        //
        // MEASURED, on the richest ice direction near Luna's north pole
        // (0.0533, 0.9982, 0.0272): the ice field reads 0.824894 there and
        // the iron field reads 0.000000. Sixty seconds of the real
        // ProductionSystem on an Ice Harvesting recipe (1.2 u/s) then gives
        // 0.0000 units in state kStarved the old way, and 59.3924 units in
        // state kRunning this way. An IRON mine is untouched: over 2000
        // sampled directions on Terra the largest difference between the old
        // expression and this one is 0.000000000.
        //
        // An extraction recipe has exactly one output and no inputs (see
        // factory::builtinCatalog), so outputs[0] IS what the mine pulls out
        // of the rock. Anything that is not an extraction recipe — a
        // smelter, a silo, a solar farm, or a machine with no recipe picked
        // yet — has no deposit to speak of and reports Count.
        [[nodiscard]] sw::res::Resource minedResourceFor(sw::u32 recipeId)
        {
            const sw::factory::RecipeDefinition* recipe =
                sw::factory::findRecipe(recipeId);
            if (recipe == nullptr ||
                recipe->requiredCategory != sw::factory::BuildingCategory::Miner)
            {
                return sw::res::Resource::Count;
            }
            return recipe->outputs[0].resource;
        }

        /// Density of whatever `recipeId` extracts, under `up`. 0 when the
        /// body has no geology, or when the recipe digs for nothing.
        [[nodiscard]] sw::f32 depositDensityFor(
            const sw::planet::DepositComponent* deposits, const sw::Vec3& up,
            sw::u32 recipeId)
        {
            const sw::res::Resource resource = minedResourceFor(recipeId);
            if (deposits == nullptr || resource == sw::res::Resource::Count)
            {
                return 0.0f;
            }
            return sw::planet::oreDensity(*deposits, up, resource);
        }
    } // namespace

    // ------------------------------------------------------------------------
    // PLACING A BUILDING
    //
    // ONE function. The scene builder lays the starting outpost with it and
    // the player's build cursor commits with it, so a machine you put down
    // and a machine the game put down are the same object, made the same
    // way — there is no "scripted" variant that quietly differs.
    //
    // `direction` is a UNIT vector in the body's rotating frame; `yaw` spins
    // the building about its own local vertical. Everything else — the
    // footprint, the power, the storage, the recipes it may run — is read
    // from the .swpart.
    // ------------------------------------------------------------------------
    sw::ecs::Entity StarWorksGame::placeBuilding(sw::u32 definitionId,
                                                 sw::ecs::Entity body,
                                                 const sw::Vec3& direction,
                                                 sw::f32 yawRadians, sw::u32 recipeId,
                                                 sw::ecs::Entity site,
                                                 const sw::Vec4& marker)
    {
        const sw::parts::PartDefinition* definition =
            sw::parts::findDefinition(definitionId);
        if (definition == nullptr || !sw::parts::isBuilding(*definition))
        {
            SW_LOG_WARN("Game", "Building definition {} missing from the catalog",
                        definitionId);
            return sw::ecs::Entity::null();
        }
        const auto* terrain = m_world.tryGetComponent<sw::planet::TerrainComponent>(body);
        const auto* gravity =
            m_world.tryGetComponent<sw::phys::GravitySourceComponent>(body);
        if (terrain == nullptr || gravity == nullptr)
        {
            SW_LOG_WARN("Game", "Cannot build on a body with no ground");
            return sw::ecs::Entity::null();
        }
        const auto* deposits = m_world.tryGetComponent<sw::planet::DepositComponent>(body);

        const sw::parts::BuildingSpec& spec = definition->building;
        const sw::Vec3 up = glm::normalize(direction);
        const sw::f64 elevation = sw::planet::terrainElevation(*terrain, up);

        // Stand the model upright: its +Y onto the local vertical, then the
        // requested yaw about that same axis (applied in MODEL space, which
        // is why it is a spin on the spot and not a tilt).
        const sw::Quat standUp = standUpFor(up);

        const sw::ecs::Entity e = m_world.createEntity();
        m_world.addComponent(e, TransformComponent{}); // metres: parts are life-size
        m_world.addComponent(e, PreviousTransformComponent{});
        m_world.addComponent(e, BoundsComponent{static_cast<sw::f32>(
                                    std::max(spec.footprintM[0], spec.footprintM[1]))});
        m_world.addComponent(e, MeshComponent{m_partMeshIds.at(definitionId)});
        if (marker.a > 0.0f)
        {
            m_world.addComponent(e, MapMarkerComponent{marker});
        }

        // SOLID. Straight from the .swpart's hitboxes — belts and cables
        // excepted, which is why you can walk a factory floor at all.
        sw::phys::HullComponent hull{};
        if (hullFor(*definition, hull))
        {
            m_world.addComponent(e, hull);
        }

        sw::phys::SurfaceAnchorComponent anchor{};
        anchor.body = body;
        anchor.localPosition = sw::WorldVec3(up) * (gravity->bodyRadius + elevation);
        anchor.localRotation =
            standUp * glm::angleAxis(yawRadians, sw::Vec3{0.0f, 1.0f, 0.0f});
        m_world.addComponent(e, anchor);

        sw::factory::BuildingComponent building{};
        building.definitionId = definitionId;
        building.site = site;
        building.category = spec.category;
        // The ground is asked about the resource THIS machine's recipe
        // extracts, not about iron. See minedResourceFor above.
        building.groundDensity = depositDensityFor(deposits, up, recipeId);
        m_world.addComponent(e, building);

        sw::factory::RecipeStateComponent state{};
        state.recipeId = recipeId;
        m_world.addComponent(e, state);

        sw::factory::PowerComponent power{};
        power.producedKw = std::max(0.0, spec.powerKw);
        power.consumedKw = std::max(0.0, -spec.powerKw);
        if (const auto* recipe = sw::factory::findRecipe(recipeId))
        {
            power.consumedKw += recipe->powerKw;
        }
        // Who the grid drops first when the sun goes down. A default, not a
        // law: the E panel will let the player promote their electrolyser.
        power.priority = sw::factory::defaultPowerPriority(spec.category);
        m_world.addComponent(e, power);

        // A battery bank is a building that happens to hold joules. Giving
        // it the component here (rather than a flag in the .swpart) keeps
        // the part file about GEOMETRY, which is what the Part Studio edits.
        if (spec.category == sw::factory::BuildingCategory::Battery)
        {
            sw::factory::BatteryComponent battery{};
            m_world.addComponent(e, battery);
        }

        // ...and an assembly hall is a building that happens to hold an
        // ORDER, plus the little queue of names that leaves with its crates.
        // Same reasoning: the .swpart says what it looks like and what
        // category it is; what that category implies lives here.
        if (spec.category == sw::factory::BuildingCategory::Assembly)
        {
            m_world.addComponent(e, sw::factory::AssemblyComponent{});
            m_world.addComponent(e, sw::factory::VehicleQueueComponent{});
        }

        if (spec.inventoryVolumeM3 > 0.0)
        {
            sw::factory::InventoryComponent inventory{};
            inventory.volumeCapacityM3 = spec.inventoryVolumeM3;
            m_world.addComponent(e, inventory);
        }
        return e;
    }

    bool StarWorksGame::conveyorPortOf(sw::ecs::Entity entity, sw::parts::NodeType type,
                                       sw::WorldVec3& outLocal)
    {
        return conveyorPortOf(entity, type, 0, outLocal);
    }

    bool StarWorksGame::conveyorPortOf(sw::ecs::Entity entity, sw::parts::NodeType type,
                                       sw::u32 index, sw::WorldVec3& outLocal)
    {
        const auto* building =
            m_world.tryGetComponent<sw::factory::BuildingComponent>(entity);
        const auto* anchor =
            m_world.tryGetComponent<sw::phys::SurfaceAnchorComponent>(entity);
        if (building == nullptr || anchor == nullptr)
        {
            return false;
        }
        const auto* definition = sw::parts::findDefinition(building->definitionId);
        if (definition == nullptr)
        {
            return false;
        }
        const std::vector<const sw::parts::AttachNode*> ports =
            sw::parts::conveyorNodes(*definition, type);
        if (index >= ports.size())
        {
            return false;
        }
        outLocal = anchor->localPosition +
                   sw::WorldVec3(anchor->localRotation * ports[index]->position);
        return true;
    }

    // WHICH MOUTH DID YOU MEAN?
    //
    // A machine with two out ports needs a port picked, and the least
    // intrusive way to ask is not to ask: the player is already aiming at
    // the machine, so take the mouth nearest their aim. It reads as "click
    // the side you want", which is what you would do with real plumbing.
    //
    // Mouths that already have a belt on them are skipped — that is what
    // makes the SECOND port reachable at all once the first is wired, and
    // it means clicking the same machine twice lays two different runs
    // rather than refusing the second.
    sw::u32 StarWorksGame::chooseConveyorPort(sw::ecs::Entity entity,
                                              sw::parts::NodeType type,
                                              const sw::WorldVec3& aimLocal, bool& outAny)
    {
        outAny = false;
        sw::u32 best = 0;
        sw::f64 bestDistance = 1.0e30;
        for (sw::u32 index = 0; index < sw::factory::kMaxMachinePorts; ++index)
        {
            sw::WorldVec3 port{};
            if (!conveyorPortOf(entity, type, index, port))
            {
                break;
            }
            // Taken? A belt mouth within the snap radius already owns it.
            bool taken = false;
            m_world.forEach<sw::factory::BuildingComponent,
                            sw::phys::SurfaceAnchorComponent>(
                [&](sw::ecs::Entity other, sw::factory::BuildingComponent& building,
                    sw::phys::SurfaceAnchorComponent&) {
                    if (taken || other == entity ||
                        building.category != sw::factory::BuildingCategory::Conveyor)
                    {
                        return;
                    }
                    // A belt's mouth of the OPPOSITE kind is what would meet
                    // this one: our out port is met by a belt's in port.
                    const sw::parts::NodeType facing =
                        (type == sw::parts::NodeType::ConveyorOut)
                            ? sw::parts::NodeType::ConveyorIn
                            : sw::parts::NodeType::ConveyorOut;
                    sw::WorldVec3 beltPort{};
                    if (conveyorPortOf(other, facing, 0, beltPort) &&
                        glm::length(beltPort - port) < sw::factory::kConveyorPortSnapM)
                    {
                        taken = true;
                    }
                });
            if (taken)
            {
                continue;
            }
            const sw::f64 distance = glm::length(port - aimLocal);
            if (distance < bestDistance)
            {
                bestDistance = distance;
                best = index;
                outAny = true;
            }
        }
        return best;
    }

    // SOLIDITY, from the .swpart. A definition's authored hitboxes become
    // the entity's HullComponent at spawn, so collision never goes back to
    // the catalogue and an entity's solidity is a fact about the entity.
    //
    // Belts and cables get none, on purpose: you step over a conveyor deck
    // and duck under a wire. Making them solid would turn a factory floor
    // into an obstacle course, and they are the two things a player walks
    // among most.
    bool StarWorksGame::hullFor(const sw::parts::PartDefinition& definition,
                                sw::phys::HullComponent& outHull)
    {
        if (!sw::parts::isSolid(definition))
        {
            return false;
        }
        const std::vector<sw::parts::HitBox> boxes = sw::parts::effectiveHull(definition);
        if (boxes.empty())
        {
            return false;
        }
        outHull = sw::phys::HullComponent{};
        for (const sw::parts::HitBox& box : boxes)
        {
            if (outHull.count >= sw::phys::kMaxHullBoxes)
            {
                break;
            }
            outHull.boxes[outHull.count++] = {box.center, glm::abs(box.halfExtents)};
            outHull.radius = std::max(outHull.radius,
                                      sw::phys::obbRadius(box.center, box.halfExtents));
        }
        return outHull.count > 0;
    }

    // Hulls are DERIVED from the .swpart, so they are not saved — a hull in
    // a save file would be a second copy of the model's own answer, free to
    // drift the moment a part is redrawn. They are rebuilt after a load
    // instead, exactly like the conveyor chains and the power grids. Without
    // this a loaded world had no solid objects at all and E hit nothing.
    void StarWorksGame::rebuildHulls()
    {
        std::vector<std::pair<sw::ecs::Entity, sw::phys::HullComponent>> hulls;
        auto collect = [&](sw::ecs::Entity entity, sw::u32 definitionId) {
            const auto* definition = sw::parts::findDefinition(definitionId);
            sw::phys::HullComponent hull{};
            if (definition != nullptr && hullFor(*definition, hull))
            {
                hulls.emplace_back(entity, hull);
            }
        };
        m_world.forEach<sw::factory::BuildingComponent>(
            [&](sw::ecs::Entity entity, sw::factory::BuildingComponent& building) {
                collect(entity, building.definitionId);
            });
        m_world.forEach<sw::parts::PartComponent>(
            [&](sw::ecs::Entity entity, sw::parts::PartComponent& part) {
                collect(entity, part.definitionId);
            });
        for (const auto& [entity, hull] : hulls)
        {
            if (m_world.hasComponent<sw::phys::HullComponent>(entity))
            {
                m_world.getComponent<sw::phys::HullComponent>(entity) = hull;
            }
            else
            {
                m_world.addComponent(entity, hull);
            }
        }
        // ...and the player, who is the one thing that gets pushed.
        if (!m_capsuleEntity.isNull())
        {
            if (const auto* suit = sw::parts::findDefinition(sw::parts::kPropEvaSuit))
            {
                sw::phys::HullComponent hull{};
                if (hullFor(*suit, hull))
                {
                    if (m_world.hasComponent<sw::phys::HullComponent>(m_capsuleEntity))
                    {
                        m_world.getComponent<sw::phys::HullComponent>(m_capsuleEntity) =
                            hull;
                    }
                    else
                    {
                        m_world.addComponent(m_capsuleEntity, hull);
                    }
                    if (!m_world.hasComponent<sw::phys::HullMoverComponent>(
                            m_capsuleEntity))
                    {
                        m_world.addComponent(m_capsuleEntity,
                                             sw::phys::HullMoverComponent{});
                    }
                }
            }
        }
        SW_LOG_INFO("Game", "Hulls rebuilt: {} solid objects", hulls.size());
    }

    // WHAT AM I LOOKING AT? A ray from the eye against the solid hulls —
    // which is exactly the question "near enough, and in front of me", and
    // exactly what a distance-to-centre check could not answer. A 16 m solar
    // field whose centre is 20 m away is still right there in front of you;
    // a silo behind your shoulder is not, however close its centre is.
    sw::ecs::Entity StarWorksGame::hullUnderCrosshair(sw::f64 maxDistanceM)
    {
        // THE CAMERA IS IN THE RENDERED WORLD, so the boxes must be too.
        //
        // The first version read each building's TransformComponent raw —
        // its TICK pose — and cast an 18 m ray at it from a camera sitting
        // in the interpolated world. One physics step of Terra's orbit is
        // 595 m, so the buildings were never anywhere near the ray and E
        // simply stopped working: not "sometimes wrong", never right.
        //
        // Same interpolation as the mesh pass, the hull overlay, the belt
        // cargo and the build ghost. FIFTH time. On this project, anything
        // that compares a camera-space quantity against a body on a moving
        // planet goes through `mix(previous, current, alpha)`, full stop.
        const sw::f32 alpha = m_physicsLane->alpha();
        const sw::f64 alpha64 = static_cast<sw::f64>(alpha);
        const sw::WorldVec3 eye = m_camera.position();
        const sw::Vec3 forward = m_camera.forward();

        sw::ecs::Entity best{};
        sw::f32 bestT = static_cast<sw::f32>(maxDistanceM);
        // ONE RAY, TWO KINDS OF THING. `E` asks "what am I looking at", and
        // the answer is a machine or a vessel; casting twice and comparing
        // afterwards would be two answers to one question. The caller sorts
        // out what to DO with whatever it hit.
        auto cast = [&](sw::ecs::Entity entity, const TransformComponent& transform,
                        const PreviousTransformComponent& previous,
                        const sw::phys::HullComponent& hull) {
                const sw::WorldVec3 position =
                    glm::mix(previous.position, transform.position, alpha64);
                const sw::Quat rotation =
                    glm::slerp(previous.rotation, transform.rotation, alpha);

                // Broad phase first, same as the collision system: one f64
                // subtraction rejects everything but the neighbours.
                const sw::WorldVec3 offset = position - eye;
                const sw::f64 reach = maxDistanceM + static_cast<sw::f64>(hull.radius);
                if (glm::dot(offset, offset) > reach * reach)
                {
                    return;
                }
                const sw::Vec3 relative = sw::Vec3(offset);
                for (sw::u32 i = 0; i < hull.count; ++i)
                {
                    const sw::phys::Obb box =
                        sw::phys::makeObb(relative + rotation * hull.boxes[i].centre,
                                          hull.boxes[i].halfExtents, rotation);
                    sw::f32 t = 0.0f;
                    sw::Vec3 normal{};
                    if (sw::phys::rayObb(sw::Vec3{0.0f}, forward, box, bestT, t, normal) &&
                        t < bestT)
                    {
                        bestT = t;
                        best = entity;
                    }
                }
        };
        m_world.forEach<TransformComponent, PreviousTransformComponent,
                        sw::phys::HullComponent, sw::factory::BuildingComponent>(
            [&](sw::ecs::Entity entity, TransformComponent& transform,
                PreviousTransformComponent& previous, sw::phys::HullComponent& hull,
                sw::factory::BuildingComponent&) { cast(entity, transform, previous, hull); });
        m_world.forEach<TransformComponent, PreviousTransformComponent,
                        sw::phys::HullComponent, sw::parts::PartComponent>(
            [&](sw::ecs::Entity entity, TransformComponent& transform,
                PreviousTransformComponent& previous, sw::phys::HullComponent& hull,
                sw::parts::PartComponent&) { cast(entity, transform, previous, hull); });
        return best;
    }

    // Where a cable hooks onto this entity, in the body's rotating frame.
    // Same shape as `conveyorPortOf` and for the same reason: the node is
    // authored on the geometry, so there is exactly one way to ask where it
    // ended up once the building was stood on a sphere.
    bool StarWorksGame::powerNodeOf(sw::ecs::Entity entity, sw::WorldVec3& outLocal)
    {
        const auto* building =
            m_world.tryGetComponent<sw::factory::BuildingComponent>(entity);
        const auto* anchor =
            m_world.tryGetComponent<sw::phys::SurfaceAnchorComponent>(entity);
        if (building == nullptr || anchor == nullptr)
        {
            return false;
        }
        const auto* definition = sw::parts::findDefinition(building->definitionId);
        if (definition == nullptr)
        {
            return false;
        }
        const sw::parts::AttachNode* node = sw::parts::findPowerNode(*definition);
        if (node == nullptr)
        {
            return false;
        }
        outLocal =
            anchor->localPosition + sw::WorldVec3(anchor->localRotation * node->position);
        return true;
    }

    // ------------------------------------------------------------------------
    // THE GRID, DERIVED FROM THE CABLES
    //
    // Run after every build and every demolition, exactly like the conveyor
    // network — and for the same reason: there must not be a second copy of
    // "who is connected to whom" that can fall out of step with the objects
    // the player can see.
    //
    // What it does, in order:
    //   1. drops cables whose endpoints have gone (demolished under the wire);
    //   2. numbers every building, unions the ones a cable joins, and writes
    //      the resulting component id into each PowerComponent;
    //   3. re-hangs every surviving cable's curve from its endpoints' CURRENT
    //      power nodes, so a span can never be left pointing at where a
    //      machine used to be.
    // ------------------------------------------------------------------------
    void StarWorksGame::rebuildPowerNetwork()
    {
        // ---- 1. every building, in a stable order ------------------------
        std::vector<sw::ecs::Entity> nodes;
        std::unordered_map<sw::ecs::Entity, sw::u32> indexOf;
        m_world.forEach<sw::factory::BuildingComponent, sw::factory::PowerComponent>(
            [&](sw::ecs::Entity entity, sw::factory::BuildingComponent&,
                sw::factory::PowerComponent&) {
                indexOf[entity] = static_cast<sw::u32>(nodes.size());
                nodes.push_back(entity);
            });

        // ---- 2. the cables, minus the ones left dangling -----------------
        std::vector<sw::factory::PowerLink> links;
        std::vector<sw::ecs::Entity> cables;
        std::vector<sw::ecs::Entity> orphans;
        m_world.forEach<sw::factory::PowerLinkComponent>(
            [&](sw::ecs::Entity entity, sw::factory::PowerLinkComponent& link) {
                const auto endA = indexOf.find(link.a);
                const auto endB = indexOf.find(link.b);
                if (endA == indexOf.end() || endB == indexOf.end())
                {
                    orphans.push_back(entity); // one end was demolished
                    return;
                }
                links.push_back({endA->second, endB->second});
                cables.push_back(entity);
            });
        for (const sw::ecs::Entity entity : orphans)
        {
            m_world.destroyEntity(entity);
        }

        // ---- 3. the components, into the components ----------------------
        const std::vector<sw::u32> grid =
            sw::factory::traceGrids(nodes.size(), links);
        for (sw::usize i = 0; i < nodes.size(); ++i)
        {
            if (auto* power =
                    m_world.tryGetComponent<sw::factory::PowerComponent>(nodes[i]))
            {
                power->gridId = grid[i];
            }
        }

        // ---- 4. re-hang every span ---------------------------------------
        for (sw::usize i = 0; i < cables.size(); ++i)
        {
            const auto& link =
                m_world.getComponent<sw::factory::PowerLinkComponent>(cables[i]);
            auto* cable = m_world.tryGetComponent<CableComponent>(cables[i]);
            if (cable == nullptr)
            {
                continue;
            }
            sw::WorldVec3 from{};
            sw::WorldVec3 to{};
            if (!powerNodeOf(link.a, from) || !powerNodeOf(link.b, to))
            {
                continue;
            }
            hangCable(*cable, from, to);
        }
        SW_LOG_INFO("Game", "Power network: {} buildings, {} cables", nodes.size(),
                    cables.size());
    }

    // The sagging curve, sampled into the component. `up` is the local
    // vertical at the middle of the span — on a 6,371 km sphere the two ends
    // of a 40 m cable have verticals 0.0004 degrees apart, so one is enough,
    // and using the midpoint's keeps the sag symmetric.
    void StarWorksGame::hangCable(CableComponent& cable, const sw::WorldVec3& from,
                                  const sw::WorldVec3& to)
    {
        const sw::Vec3 up = sw::Vec3(glm::normalize((from + to) * 0.5));
        cable.pointCount = CableComponent::kMaxPoints;
        sw::f64 length = 0.0;
        for (sw::u32 i = 0; i < cable.pointCount; ++i)
        {
            const sw::f64 t = static_cast<sw::f64>(i) /
                              static_cast<sw::f64>(cable.pointCount - 1);
            cable.points[i] =
                sw::factory::cablePointAt(from, to, up, kCableSagFraction, t);
            if (i > 0)
            {
                length += glm::length(cable.points[i] - cable.points[i - 1]);
            }
        }
        cable.lengthM = static_cast<sw::f32>(length);
    }

    // The whole answer to "may this cable be laid", for the preview and for
    // the commit. One function, so the green line you were shown and the
    // wire you get cannot disagree.
    sw::factory::CableVerdict StarWorksGame::planCable(sw::ecs::Entity from,
                                                       sw::ecs::Entity to,
                                                       sw::WorldVec3& outFrom,
                                                       sw::WorldVec3& outTo)
    {
        if (from.isNull() || to.isNull() || from == to)
        {
            return sw::factory::CableVerdict::SameNode;
        }
        if (!powerNodeOf(from, outFrom) || !powerNodeOf(to, outTo))
        {
            return sw::factory::CableVerdict::NoPowerNode;
        }
        const auto* buildingA =
            m_world.tryGetComponent<sw::factory::BuildingComponent>(from);
        const auto* buildingB =
            m_world.tryGetComponent<sw::factory::BuildingComponent>(to);
        const auto* powerA = m_world.tryGetComponent<sw::factory::PowerComponent>(from);
        const auto* powerB = m_world.tryGetComponent<sw::factory::PowerComponent>(to);
        if (buildingA == nullptr || buildingB == nullptr || powerA == nullptr ||
            powerB == nullptr)
        {
            return sw::factory::CableVerdict::NoPowerNode;
        }

        // How many wires already meet at each end.
        sw::u32 onA = 0;
        sw::u32 onB = 0;
        m_world.forEach<sw::factory::PowerLinkComponent>(
            [&](sw::ecs::Entity, sw::factory::PowerLinkComponent& link) {
                if (link.a == from || link.b == from) { onA += 1; }
                if (link.a == to || link.b == to) { onB += 1; }
            });

        return sw::factory::validateCable(
            true, true, buildingA->category, buildingB->category, onA, onB,
            powerA->gridId, powerB->gridId, glm::length(outTo - outFrom),
            kMaxCableLengthM);
    }

    void StarWorksGame::layCable(sw::ecs::Entity body, sw::ecs::Entity from,
                                 sw::ecs::Entity to)
    {
        sw::WorldVec3 fromNode{};
        sw::WorldVec3 toNode{};
        if (planCable(from, to, fromNode, toNode) != sw::factory::CableVerdict::Ok)
        {
            return;
        }
        const sw::ecs::Entity entity = m_world.createEntity();
        m_world.addComponent(entity, TransformComponent{});
        m_world.addComponent(entity, PreviousTransformComponent{});
        m_world.addComponent(entity, sw::factory::PowerLinkComponent{from, to});
        CableComponent cable{};
        cable.body = body;
        hangCable(cable, fromNode, toNode);
        m_world.addComponent(entity, cable);
        // ...and only NOW is the grid what the cables say it is.
        rebuildPowerNetwork();
    }

    // ------------------------------------------------------------------------
    // PLANNING A BELT
    //
    // The player's operation is "feed THIS from THAT", not "put a tile here,
    // then another". So the tool takes two machines and produces the run
    // between their mouths — and what it produces is ordinary CV-1 buildings,
    // so afterwards there is nothing special about a belt the tool laid
    // versus one placed by hand. The network is still derived from where the
    // ports ended up.
    //
    // Same routine for the preview and the commit: a run you were shown in
    // green cannot come out different when you click.
    // ------------------------------------------------------------------------
    sw::build::Verdict StarWorksGame::planBelt(sw::ecs::Entity body, sw::ecs::Entity from,
                                               sw::ecs::Entity to,
                                               std::vector<BeltTile>& outTiles)
    {
        return planBelt(body, from, 0, to, 0, outTiles);
    }

    sw::build::Verdict StarWorksGame::planBelt(sw::ecs::Entity body, sw::ecs::Entity from,
                                               sw::u32 fromPortIndex, sw::ecs::Entity to,
                                               sw::u32 toPortIndex,
                                               std::vector<BeltTile>& outTiles)
    {
        outTiles.clear();
        const auto* terrain = m_world.tryGetComponent<sw::planet::TerrainComponent>(body);
        const auto* gravity =
            m_world.tryGetComponent<sw::phys::GravitySourceComponent>(body);
        const auto* segment = sw::parts::findDefinition(sw::parts::kBuildingConveyor);
        if (terrain == nullptr || gravity == nullptr || segment == nullptr ||
            from == to || from.isNull() || to.isNull())
        {
            return sw::build::Verdict::NoDefinition;
        }

        sw::WorldVec3 fromPort{};
        sw::WorldVec3 toPort{};
        if (!conveyorPortOf(from, sw::parts::NodeType::ConveyorOut, fromPortIndex,
                            fromPort) ||
            !conveyorPortOf(to, sw::parts::NodeType::ConveyorIn, toPortIndex, toPort))
        {
            return sw::build::Verdict::NoDefinition; // one of them has no mouth
        }

        sw::WorldVec3 path[sw::factory::kMaxConveyorPoints]{};
        sw::u32 count = sw::factory::kMaxConveyorPoints;
        const sw::f64 length = sw::factory::buildConveyorPath(
            *terrain, gravity->bodyRadius, fromPort, toPort, 0.0, path, count);
        if (length > kMaxBeltLengthM)
        {
            return sw::build::Verdict::OutOfRange;
        }

        const sw::f64 span = static_cast<sw::f64>(m_conveyorSegmentM);
        const sw::i32 tiles =
            std::max(1, static_cast<sw::i32>(std::lround(length / span)));
        const std::vector<sw::build::Footprint> occupied = footprintsOn(body);
        static const sw::planet::DepositComponent kNoDeposits{};
        const auto* deposits = m_world.tryGetComponent<sw::planet::DepositComponent>(body);

        sw::build::Verdict worst = sw::build::Verdict::Ok;
        for (sw::i32 i = 0; i < tiles; ++i)
        {
            sw::WorldVec3 local{};
            sw::Vec3 heading{};
            sw::factory::conveyorPointAt(
                path, count, (static_cast<sw::f64>(i) + 0.5) * (length / tiles), local,
                heading);
            const sw::Vec3 up = sw::Vec3(glm::normalize(local));
            // Model -Z is the direction of travel: aim it down the run.
            outTiles.push_back({up, yawToFace(up, sw::Vec3{0.0f, 0.0f, -1.0f}, heading)});

            const sw::build::Verdict verdict = sw::build::validatePlacement(
                *terrain, (deposits != nullptr) ? *deposits : kNoDeposits,
                gravity->bodyRadius, *segment, up, occupied);
            if (verdict != sw::build::Verdict::Ok && worst == sw::build::Verdict::Ok)
            {
                worst = verdict; // the FIRST reason the run cannot be laid
            }
        }
        return worst;
    }

    sw::u32 StarWorksGame::defaultRecipeFor(sw::factory::BuildingCategory category)
    {
        const std::vector<sw::u32> recipes = sw::factory::recipesForCategory(category);
        return recipes.empty() ? 0u : recipes.front();
    }

    // ------------------------------------------------------------------------
    // THE CONVEYOR NETWORK, DERIVED FROM GEOMETRY
    //
    // A belt segment is an ordinary building: the player places them one at a
    // time, like a smelter. What turns a ROW of them into a working link is
    // not an intention the game recorded — it is that their conveyor-out and
    // conveyor-in ports MEET. So the network is not stored, it is derived,
    // after every build and every demolition, from where things are.
    //
    // That is the same choice the deposits and the orbits already made, and
    // it buys the same thing: there is no second copy of the truth to fall
    // out of step. Demolish a segment in the middle of a run and the chain
    // simply is not there next frame, because the ports no longer meet.
    // ------------------------------------------------------------------------
    void StarWorksGame::rebuildConveyorNetwork()
    {

        // Everything standing, with its ports resolved into the body frame.
        std::vector<sw::factory::PortNode> nodes;
        std::vector<sw::ecs::Entity> bodies;
        m_world.forEach<sw::factory::BuildingComponent,
                        sw::phys::SurfaceAnchorComponent>(
            [&](sw::ecs::Entity entity, sw::factory::BuildingComponent& building,
                sw::phys::SurfaceAnchorComponent& anchor) {
                const auto* definition = sw::parts::findDefinition(building.definitionId);
                if (definition == nullptr)
                {
                    return;
                }
                sw::factory::PortNode node{};
                node.entity = entity;
                node.isBelt =
                    building.category == sw::factory::BuildingCategory::Conveyor;
                node.centre = anchor.localPosition;
                // EVERY mouth, in authored order — the order is the
                // contract: out mouth i ships the recipe's product i.
                for (const sw::parts::AttachNode* port : sw::parts::conveyorNodes(
                         *definition, sw::parts::NodeType::ConveyorOut))
                {
                    if (node.outCount >= sw::factory::kMaxMachinePorts) { break; }
                    node.outPorts[node.outCount++] =
                        anchor.localPosition +
                        sw::WorldVec3(anchor.localRotation * port->position);
                }
                for (const sw::parts::AttachNode* port : sw::parts::conveyorNodes(
                         *definition, sw::parts::NodeType::ConveyorIn))
                {
                    if (node.inCount >= sw::factory::kMaxMachinePorts) { break; }
                    node.inPorts[node.inCount++] =
                        anchor.localPosition +
                        sw::WorldVec3(anchor.localRotation * port->position);
                }
                nodes.push_back(node);
                bodies.push_back(anchor.body);
            });

        // Old conveyors and their links go first: the graph below is the only
        // author of both, so anything left over is a ghost of a demolished run.
        std::vector<sw::ecs::Entity> stale;
        m_world.forEach<ConveyorComponent>(
            [&](sw::ecs::Entity entity, ConveyorComponent&) { stale.push_back(entity); });
        for (const sw::ecs::Entity entity : stale)
        {
            m_world.destroyEntity(entity);
        }
        for (const sw::factory::PortNode& node : nodes)
        {
            if (m_world.hasComponent<sw::factory::ItemLinkComponent>(node.entity))
            {
                m_world.removeComponent<sw::factory::ItemLinkComponent>(node.entity);
            }
        }

        for (const sw::factory::Chain& chain :
             sw::factory::traceConveyorChains(nodes, sw::factory::kConveyorPortSnapM))
        {
            const sw::factory::PortNode& source = nodes[chain.source];
            const sw::factory::PortNode& destination = nodes[chain.destination];
            if (bodies[chain.source] != bodies[chain.destination])
            {
                continue; // two different worlds: not a belt, a coincidence
            }

            // The cargo path: out of the source, along every deck, into the
            // destination.
            std::vector<sw::WorldVec3> path;
            path.push_back(source.outPorts[chain.sourcePort]);
            for (const sw::u32 belt : chain.belts)
            {
                path.push_back(nodes[belt].centre);
            }
            path.push_back(destination.inPorts[chain.destinationPort]);

            // WHAT does it carry? EVERYTHING the source makes — a belt out
            // of an electrolyser carries the hydrogen and the oxygen, because
            // ONE MOUTH, EVERYTHING; SEVERAL MOUTHS, ONE PRODUCT EACH.
            //
            // A machine with a single out port has nowhere else to put its
            // products, so its belt carries all of them — that is what made
            // the fuel chain buildable at all. A machine with SEVERAL ports
            // is making a different statement: mouth i ships product i, so
            // hydrogen leaves by one belt and oxygen by the other, and the
            // player decides where each goes. Nothing to say means nothing
            // to move.
            sw::res::Resource carried[sw::factory::kMaxRecipeIngredients]{
                sw::res::Resource::Count, sw::res::Resource::Count,
                sw::res::Resource::Count, sw::res::Resource::Count};
            sw::usize carriedCount = 0;
            const bool splitByPort = source.outCount > 1;
            if (const auto* state =
                    m_world.tryGetComponent<sw::factory::RecipeStateComponent>(
                        source.entity))
            {
                if (const auto* recipe = sw::factory::findRecipe(state->recipeId))
                {
                    sw::u32 productIndex = 0;
                    for (const sw::factory::Ingredient& output : recipe->outputs)
                    {
                        if (output.resource == sw::res::Resource::Count ||
                            output.unitsPerSecond <= 0.0)
                        {
                            continue;
                        }
                        if (splitByPort)
                        {
                            // Mouth i ships product i, and a mouth past the
                            // last product ships nothing — an unused port is
                            // an empty belt, which is the honest picture.
                            if (productIndex == chain.sourcePort)
                            {
                                carried[carriedCount++] = output.resource;
                            }
                        }
                        else
                        {
                            carried[carriedCount++] = output.resource;
                        }
                        ++productIndex;
                    }
                }
            }
            if (carriedCount == 0 &&
                m_world.hasComponent<sw::factory::AssemblyComponent>(source.entity))
            {
                // AN ASSEMBLY HALL SHIPS ROCKETS. It has no recipe, and the
                // silo rule below would have it export the iron it is
                // standing on — the belt to the pad would run backwards,
                // carrying the metal away from the machine that needs it.
                // A hall's product is the one thing it makes.
                carried[carriedCount++] = sw::res::Resource::Vehicle;
            }
            if (carriedCount == 0)
            {
                // A silo ships what it is holding.
                if (const auto* inventory =
                        m_world.tryGetComponent<sw::factory::InventoryComponent>(
                            source.entity))
                {
                    for (const sw::factory::InventorySlot& slot : inventory->slots)
                    {
                        if (slot.resource != sw::res::Resource::Count && slot.units > 0.0)
                        {
                            carried[carriedCount++] = slot.resource;
                            break;
                        }
                    }
                }
            }
            if (carriedCount == 0 ||
                !m_world.hasComponent<sw::factory::InventoryComponent>(
                    destination.entity))
            {
                continue; // nothing to carry, or nowhere to put it
            }
            const sw::res::Resource resource = carried[0]; // what it looks like

            // THE LINK, on the destination — one channel per good. Several
            // belts may arrive at the same machine, so the component may
            // already be there: add to it rather than replacing it.
            if (!m_world.hasComponent<sw::factory::ItemLinkComponent>(destination.entity))
            {
                m_world.addComponent(destination.entity,
                                     sw::factory::ItemLinkComponent{});
            }
            {
                auto& link =
                    m_world.getComponent<sw::factory::ItemLinkComponent>(
                        destination.entity);
                for (sw::usize i = 0; i < carriedCount; ++i)
                {
                    sw::factory::linkAddChannel(link, source.entity, carried[i],
                                                kConveyorRateUnitsPerSecond);
                }
            }

            // ...and the cargo path, subsampled if the run is longer than the
            // component can hold: the crates are a depiction, and sixteen
            // waypoints depict a belt of any length perfectly well.
            ConveyorComponent conveyor{};
            conveyor.body = bodies[chain.source];
            conveyor.link = destination.entity;
            conveyor.source = source.entity;
            conveyor.cargoColor = resourceCargoColor(resource);
            conveyor.cargoMesh = (resource == sw::res::Resource::Vehicle)
                                     ? m_vehicleCargoMeshIndex
                                     : m_cargoMeshIndex;
            const sw::usize count =
                std::min<sw::usize>(path.size(), ConveyorComponent::kMaxPoints);
            conveyor.pointCount = static_cast<sw::u32>(count);
            for (sw::usize i = 0; i < count; ++i)
            {
                const sw::usize pick =
                    (count == 1) ? 0 : (i * (path.size() - 1)) / (count - 1);
                conveyor.points[i] = path[pick];
            }
            sw::f64 length = 0.0;
            for (sw::u32 i = 0; i + 1 < conveyor.pointCount; ++i)
            {
                length += glm::length(conveyor.points[i + 1] - conveyor.points[i]);
            }
            conveyor.lengthM = static_cast<sw::f32>(length);
            if (conveyor.lengthM < 0.5f)
            {
                continue;
            }

            const sw::ecs::Entity e = m_world.createEntity();
            m_world.addComponent(e, TransformComponent{});
            m_world.addComponent(e, PreviousTransformComponent{});
            sw::phys::SurfaceAnchorComponent anchor{};
            anchor.body = bodies[chain.source];

            anchor.localPosition = conveyor.points[0];
            m_world.addComponent(e, anchor);
            m_world.addComponent(e, conveyor);
        }
    }

    // ------------------------------------------------------------------------
    // F2 — THE GROUND BUILD CURSOR
    //
    // Arm a building in the F menu, walk to where you want it, look at the
    // ground. The ghost lands where your gaze meets the heightfield — the
    // real one, marched, not a flat plane at sea level — inside a reach you
    // have to walk to extend. The wheel spins it. Click builds. R razes what
    // you are looking at.
    //
    // The green/red is not a second opinion: it is exactly the verdict
    // `placeBuilding` will be handed, from exactly the .swpart fields, so
    // the ghost cannot promise something the commit refuses.
    // ------------------------------------------------------------------------
    std::vector<sw::build::Footprint> StarWorksGame::footprintsOn(
        sw::ecs::Entity body)
    {
        std::vector<sw::build::Footprint> footprints;
        m_world.forEach<sw::factory::BuildingComponent,
                        sw::phys::SurfaceAnchorComponent>(
            [&](sw::ecs::Entity, sw::factory::BuildingComponent& building,
                sw::phys::SurfaceAnchorComponent& anchor) {
                if (anchor.body != body)
                {
                    return;
                }
                const auto* definition = sw::parts::findDefinition(building.definitionId);
                if (definition == nullptr ||
                    building.category == sw::factory::BuildingCategory::Conveyor)
                {
                    return; // belts do not block: see validatePlacement
                }
                footprints.push_back(
                    {sw::Vec3(glm::normalize(anchor.localPosition)),
                     sw::build::footprintRadius(definition->building)});
            });
        return footprints;
    }

    sw::ecs::Entity StarWorksGame::siteNear(sw::ecs::Entity body,
                                            const sw::Vec3& direction)
    {
        const auto* gravity =
            m_world.tryGetComponent<sw::phys::GravitySourceComponent>(body);
        if (gravity == nullptr)
        {
            return {};
        }
        sw::ecs::Entity best{};
        sw::f64 bestDistance = 400.0; // a site is a PLACE: 400 m across, no more
        m_world.forEach<sw::factory::SiteComponent, sw::phys::SurfaceAnchorComponent>(
            [&](sw::ecs::Entity entity, sw::factory::SiteComponent& site,
                sw::phys::SurfaceAnchorComponent& anchor) {
                if (site.body != body)
                {
                    return;
                }
                const sw::f64 distance = sw::build::groundDistance(
                    direction, sw::Vec3(glm::normalize(anchor.localPosition)),
                    gravity->bodyRadius);
                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    best = entity;
                }
            });
        return best;
    }

    void StarWorksGame::updateBuildCursor()
    {
        m_buildCursor = {};
        m_beltPreview.clear();
        m_beltVerdict = sw::build::Verdict::NoGround;
        if (!m_evaMode || m_mapView || m_editorMode || m_buildMenu ||
            !m_configTarget.isNull() || m_capsuleEntity.isNull())
        {
            m_beltSource = {};
            m_cableSource = {};
            return;
        }

        // What are we standing on? The SOI primary, if it has ground.
        const sw::i32 primaryIndex = controlledPrimaryIndex();
        if (primaryIndex < 0)
        {
            return;
        }
        const sw::ecs::Entity body =
            m_celestialIndex.body(static_cast<sw::usize>(primaryIndex)).entity;
        const auto* terrain = m_world.tryGetComponent<sw::planet::TerrainComponent>(body);
        const auto* gravity =
            m_world.tryGetComponent<sw::phys::GravitySourceComponent>(body);
        const auto* bodyTransform = m_world.tryGetComponent<TransformComponent>(body);
        if (terrain == nullptr || gravity == nullptr || bodyTransform == nullptr)
        {
            return;
        }
        m_buildCursor.body = body;

        // Into the body's ROTATING frame, through the pose the body is being
        // DRAWN at. The camera sits in the rendered world; transforming it
        // with the raw tick pose would start the ray up to 595 m from where
        // the player actually is.
        sw::WorldVec3 bodyPosition{};
        glm::dquat bodyRotation{};
        bodyRenderPose(body, bodyPosition, bodyRotation);
        const glm::dquat toBody = glm::inverse(bodyRotation);
        const sw::WorldVec3 eyeLocal = toBody * (m_camera.position() - bodyPosition);
        const sw::WorldVec3 aimLocal = toBody * glm::dvec3(m_camera.forward());

        sw::WorldVec3 hitLocal{};
        if (!sw::build::raycastTerrain(*terrain, gravity->bodyRadius, eyeLocal, aimLocal,
                                       kBuildRangeM, hitLocal))
        {
            m_buildCursor.verdict = sw::build::Verdict::NoGround;
            return;
        }
        m_buildCursor.active = true;
        m_buildCursor.direction = sw::Vec3(glm::normalize(hitLocal));
        m_buildCursor.rangeM = glm::length(hitLocal - eyeLocal);
        m_buildCursor.yawRadians = m_buildYaw;

        // ---- what is under the cursor, for R ----------------------------
        {
            sw::f64 nearest = 1.0e9;
            m_world.forEach<sw::factory::BuildingComponent,
                            sw::phys::SurfaceAnchorComponent>(
                [&](sw::ecs::Entity entity, sw::factory::BuildingComponent& building,
                    sw::phys::SurfaceAnchorComponent& anchor) {
                    if (anchor.body != body)
                    {
                        return;
                    }
                    const auto* definition =
                        sw::parts::findDefinition(building.definitionId);
                    if (definition == nullptr)
                    {
                        return;
                    }
                    const sw::f64 distance = sw::build::groundDistance(
                        m_buildCursor.direction,
                        sw::Vec3(glm::normalize(anchor.localPosition)),
                        gravity->bodyRadius);
                    if (distance <
                            static_cast<sw::f64>(
                                sw::build::footprintRadius(definition->building)) &&
                        distance < nearest)
                    {
                        nearest = distance;
                        m_buildCursor.target = entity;
                    }
                });
        }

        const auto* held = sw::parts::findDefinition(m_heldBuilding);

        // ---- BELT MODE: pick an output, then an input --------------------
        // A conveyor is not placed tile by tile. The player's operation is
        // "feed this from that", so the tool takes the two machines and the
        // run between their mouths is the RESULT. It starts moving goods the
        // instant it lands, because the network is derived from where the
        // ports ended up and nothing else has to be told.
        const bool beltMode =
            held != nullptr &&
            held->building.category == sw::factory::BuildingCategory::Conveyor;
        if (beltMode)
        {
            if (!m_world.isAlive(m_beltSource))
            {
                m_beltSource = {};
            }
            // The aim point on the ground, in the body frame: which mouth
            // the player means is answered by which one they are nearest.
            const sw::WorldVec3 aimLocal =
                sw::WorldVec3(m_buildCursor.direction) *
                (gravity->bodyRadius +
                 sw::planet::terrainElevation(*terrain, m_buildCursor.direction));
            if (!m_beltSource.isNull() && !m_buildCursor.target.isNull())
            {
                bool anyIn = false;
                m_beltDestinationPort = chooseConveyorPort(
                    m_buildCursor.target, sw::parts::NodeType::ConveyorIn, aimLocal,
                    anyIn);
                m_beltVerdict =
                    anyIn ? planBelt(body, m_beltSource, m_beltSourcePort,
                                     m_buildCursor.target, m_beltDestinationPort,
                                     m_beltPreview)
                          : sw::build::Verdict::NoDefinition;
                if (!anyIn)
                {
                    m_beltPreview.clear();
                }
            }

            if (input().wasKeyPressed(sw::KeyCode::R) && !m_beltSource.isNull())
            {
                m_beltSource = {}; // R cancels a pending pick
                m_beltPreview.clear();
                return;
            }
            if (input().wasMouseButtonPressed(sw::MouseButton::Left) &&
                !m_buildCursor.target.isNull())
            {
                if (m_beltSource.isNull())
                {
                    bool anyOut = false;
                    const sw::u32 port = chooseConveyorPort(
                        m_buildCursor.target, sw::parts::NodeType::ConveyorOut, aimLocal,
                        anyOut);
                    if (anyOut)
                    {
                        m_beltSource = m_buildCursor.target;
                        m_beltSourcePort = port;
                    }
                    else
                    {
                        SW_LOG_INFO("Game",
                                    "That machine has no free output port");
                    }
                }
                else if (m_beltVerdict == sw::build::Verdict::Ok &&
                         !m_beltPreview.empty())
                {
                    for (const BeltTile& tile : m_beltPreview)
                    {
                        placeBuilding(sw::parts::kBuildingConveyor, body, tile.direction,
                                      tile.yawRadians, 0u,
                                      siteNear(body, tile.direction), {});
                    }
                    SW_LOG_INFO("Game", "BELT laid: {} segments, port {} -> port {}",
                                m_beltPreview.size(), m_beltSourcePort,
                                m_beltDestinationPort);
                    m_beltSource = {};
                    m_beltPreview.clear();
                    // ...and it is carrying goods from this frame on.
                    rebuildConveyorNetwork();
                }
            }
            return;
        }
        m_beltSource = {};

        // ---- CABLE MODE: pick two power nodes ----------------------------
        // The same two clicks as the belt, asking a different question: not
        // "feed this from that" but "put these on the same grid". A cable is
        // ONE entity rather than a row of tiles, so there is nothing to walk
        // along the ground — the span hangs between the two nodes.
        const bool cableMode =
            held != nullptr &&
            held->building.category == sw::factory::BuildingCategory::Cable;
        if (cableMode)
        {
            if (!m_world.isAlive(m_cableSource))
            {
                m_cableSource = {};
            }
            m_cableVerdict = sw::factory::CableVerdict::NoPowerNode;
            if (!m_cableSource.isNull() && !m_buildCursor.target.isNull())
            {
                sw::WorldVec3 from{};
                sw::WorldVec3 to{};
                m_cableVerdict = planCable(m_cableSource, m_buildCursor.target, from, to);
            }

            if (input().wasKeyPressed(sw::KeyCode::R))
            {
                if (!m_cableSource.isNull())
                {
                    m_cableSource = {}; // R cancels a pending pick
                    return;
                }
                // ...and with nothing pending, R CUTS. A wire has no
                // footprint to look at, so the thing you aim at is the
                // building it is tied to — which is also how you think
                // about it ("unplug the smelter").
                if (!m_buildCursor.target.isNull())
                {
                    std::vector<sw::ecs::Entity> cut;
                    const sw::ecs::Entity target = m_buildCursor.target;
                    m_world.forEach<sw::factory::PowerLinkComponent>(
                        [&](sw::ecs::Entity entity,
                            sw::factory::PowerLinkComponent& link) {
                            if (link.a == target || link.b == target)
                            {
                                cut.push_back(entity);
                            }
                        });
                    for (const sw::ecs::Entity entity : cut)
                    {
                        m_world.destroyEntity(entity);
                    }
                    if (!cut.empty())
                    {
                        SW_LOG_INFO("Game", "CUT {} cable(s)", cut.size());
                        rebuildPowerNetwork();
                    }
                }
                return;
            }
            if (input().wasMouseButtonPressed(sw::MouseButton::Left) &&
                !m_buildCursor.target.isNull())
            {
                if (m_cableSource.isNull())
                {
                    sw::WorldVec3 unused{};
                    if (powerNodeOf(m_buildCursor.target, unused))
                    {
                        m_cableSource = m_buildCursor.target;
                    }
                    else
                    {
                        SW_LOG_INFO("Game", "That building has no power connection");
                    }
                }
                else if (m_cableVerdict == sw::factory::CableVerdict::Ok)
                {
                    layCable(body, m_cableSource, m_buildCursor.target);
                    SW_LOG_INFO("Game", "CABLE laid");
                    m_cableSource = {};
                }
            }
            return;
        }
        m_cableSource = {};

        // ---- the verdict, for the armed building -------------------------
        if (held == nullptr)
        {
            m_buildCursor.verdict = sw::build::Verdict::NoDefinition;
        }
        else
        {
            static const sw::planet::DepositComponent kNoDeposits{};
            const auto* deposits =
                m_world.tryGetComponent<sw::planet::DepositComponent>(body);
            m_buildCursor.verdict = sw::build::validatePlacement(
                *terrain, (deposits != nullptr) ? *deposits : kNoDeposits,
                gravity->bodyRadius, *held, m_buildCursor.direction,
                footprintsOn(body));
        }

        // ---- input -------------------------------------------------------
        // The wheel has nothing else to do on foot (first person has no zoom),
        // so it spins the building.
        if (const sw::f32 scroll = input().scrollDeltaY(); scroll != 0.0f)
        {
            m_buildYaw += scroll * 0.19634954f; // 11.25 degrees a notch
            m_buildCursor.yawRadians = m_buildYaw;
        }

        if (input().wasKeyPressed(sw::KeyCode::R) &&
            !m_buildCursor.target.isNull())
        {
            const auto& building = m_world.getComponent<sw::factory::BuildingComponent>(
                m_buildCursor.target);
            const auto* definition = sw::parts::findDefinition(building.definitionId);
            SW_LOG_INFO("Game", "DEMOLISHED {}",
                        (definition != nullptr) ? definition->name : "building");
            m_world.destroyEntity(m_buildCursor.target);
            m_buildCursor.target = {};
            rebuildConveyorNetwork();
            // ...and the GRID, which may just have been cut in two. Any
            // cable left with a dead end is dropped in there, so a wire can
            // never outlive the thing it was tied to.
            rebuildPowerNetwork();
            return;
        }

        if (held != nullptr && m_buildCursor.verdict == sw::build::Verdict::Ok &&
            input().wasMouseButtonPressed(sw::MouseButton::Left))
        {
            // A HUB founds its own site; everything else joins the nearest.
            const sw::ecs::Entity entity = placeBuilding(
                m_heldBuilding, body, m_buildCursor.direction, m_buildYaw,
                defaultRecipeFor(held->building.category),
                siteNear(body, m_buildCursor.direction),
                (held->building.category == sw::factory::BuildingCategory::Beacon)
                    ? sw::Vec4{1.0f, 0.78f, 0.28f, 1.0f}
                    : sw::Vec4{});
            if (!entity.isNull())
            {
                if (held->building.category == sw::factory::BuildingCategory::Hub)
                {
                    sw::factory::SiteComponent site{};
                    std::snprintf(site.name, sizeof(site.name), "SITE %u", entity.index);
                    site.body = body;
                    m_world.addComponent(entity, site);
                    m_world.getComponent<sw::factory::BuildingComponent>(entity).site =
                        entity;
                }
                if (held->building.category == sw::factory::BuildingCategory::Beacon)
                {
                    sw::factory::BeaconComponent beacon{};
                    std::snprintf(beacon.label, sizeof(beacon.label), "BEACON %u",
                                  entity.index);
                    m_world.addComponent(entity, beacon);
                }
                SW_LOG_INFO("Game", "BUILT {} at {:.0f} m", held->name,
                            m_buildCursor.rangeM);
                rebuildConveyorNetwork();
                rebuildPowerNetwork(); // a new building is its own grid of one
            }
        }
    }

    // Choosing a recipe changes FOUR things at once, and all four have to
    // move together or the site lies about itself: what the machine runs,
    // what the ground under it is worth for that, what it draws from the
    // grid, and what its outgoing belt is carrying.
    void StarWorksGame::applyRecipeChoice(sw::ecs::Entity entity, sw::u32 recipeId)
    {
        auto* state = m_world.tryGetComponent<sw::factory::RecipeStateComponent>(entity);
        if (state == nullptr)
        {
            return;
        }
        state->recipeId = recipeId;
        state->state = sw::factory::RecipeStateComponent::kIdle;

        // RE-SAMPLE THE GROUND. `groundDensity` is the density of the one
        // resource the machine is digging for, so switching a mine from iron
        // to ice makes the stored number answer the wrong question — it
        // would run an ice harvester at the iron grade under its feet (or,
        // more often, park it at kStarved because there is no iron there).
        // The SITING stays permanent, which is the point of sampling once:
        // this re-reads the same field at the same place, only for the
        // resource now being asked for.
        if (auto* building = m_world.tryGetComponent<sw::factory::BuildingComponent>(entity))
        {
            if (const auto* anchor =
                    m_world.tryGetComponent<sw::phys::SurfaceAnchorComponent>(entity);
                anchor != nullptr && glm::length(anchor->localPosition) > 1.0)
            {
                building->groundDensity = depositDensityFor(
                    m_world.tryGetComponent<sw::planet::DepositComponent>(anchor->body),
                    sw::Vec3(glm::normalize(anchor->localPosition)), recipeId);
            }
        }

        if (auto* power = m_world.tryGetComponent<sw::factory::PowerComponent>(entity))
        {
            const auto* building =
                m_world.tryGetComponent<sw::factory::BuildingComponent>(entity);
            const auto* definition =
                (building != nullptr)
                    ? sw::parts::findDefinition(building->definitionId)
                    : nullptr;
            const sw::f64 idleKw =
                (definition != nullptr) ? std::max(0.0, -definition->building.powerKw)
                                        : 0.0;
            power->consumedKw = idleKw;
            if (const auto* recipe = sw::factory::findRecipe(recipeId))
            {
                power->consumedKw += recipe->powerKw;
            }
        }
        // The belt out of this machine carries whatever it now makes.
        rebuildConveyorNetwork();
    }
} // namespace game
