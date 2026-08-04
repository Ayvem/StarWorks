// ============================================================================
// GameDocking.cpp — F46: two craft become one.
//
// The engine owns the arithmetic — which faces can mate, whether they are
// touching, and how the momentum of two bodies becomes the momentum of one
// (sw::parts::dockingCapture / dockVessels / undockAt). What lives here is
// everything that is a GAME decision rather than a physical one:
//
//   * WHICH craft survives a merge. Physically neither does; for a player it
//     matters enormously, because the surviving root is the thing they are
//     flying. The one they are aboard always wins.
//   * WHAT MOVES ACROSS before the other root is destroyed. A vessel root
//     carries the controls, the autopilot, the air's answer and its map
//     marker, and none of those are the engine's to know about.
//   * WHAT THE PILOT IS TOLD while lining up. A dock that silently refuses is
//     indistinguishable from one that is not implemented, and the four limits
//     fail in four different ways.
// ============================================================================

#include "StarWorksGame.hpp"

#include "GameInternal.hpp"
#include "Systems.hpp"

#include <algorithm>
#include <format>
#include <vector>

namespace game
{
    namespace
    {
        /// How often the search runs. Every frame is wasted work — a capture
        /// needs half a metre a second, so nothing can cross the gate between
        /// two of these — and once a second is slow enough to miss nothing and
        /// cheap enough that the pair loop never shows up in a frame timing.
        constexpr sw::f64 kSearchIntervalSeconds = 0.1;

        /// How near two ports have to be before the HUD starts talking about
        /// them. Far enough out to be useful on final approach, near enough
        /// that it says nothing while you are flying past a station.
        constexpr sw::f32 kAdviceRangeM = 60.0f;
    } // namespace

    // ------------------------------------------------------------------------
    // WHAT THE SURVIVOR INHERITS
    //
    // Only what it does not already have. Both craft are normally built the
    // same way and carry the same set, in which case this does nothing at all;
    // it exists for the case that is not normal — docking something assembled
    // by the scene builder, or a wreck, or a station that was never meant to
    // be flown — where the survivor can genuinely be missing the components
    // that make a vessel controllable.
    // ------------------------------------------------------------------------
    void StarWorksGame::adoptVesselRoot(sw::ecs::Entity survivor, sw::ecs::Entity absorbed)
    {
        if (survivor.isNull() || absorbed.isNull())
        {
            return;
        }
        auto adopt = [&]<typename T>() {
            if (m_world.tryGetComponent<T>(survivor) != nullptr)
            {
                return;
            }
            if (const auto* source = m_world.tryGetComponent<T>(absorbed))
            {
                m_world.addComponent(survivor, *source);
            }
        };
        adopt.template operator()<ShipComponent>();
        adopt.template operator()<ShipControlsComponent>();
        adopt.template operator()<SasComponent>();
        adopt.template operator()<sw::aero::AeroStateComponent>();
        adopt.template operator()<BoundsComponent>();
        adopt.template operator()<MapMarkerComponent>();
    }

    // ------------------------------------------------------------------------
    // THE SEARCH
    // ------------------------------------------------------------------------
    void StarWorksGame::updateDocking()
    {
        m_dockStatus.clear();
        if (clock().totalSeconds() - m_lastDockCheckSeconds < kSearchIntervalSeconds)
        {
            return;
        }
        m_lastDockCheckSeconds = clock().totalSeconds();

        struct Port
        {
            sw::ecs::Entity part;
            sw::ecs::Entity vessel;
            sw::WorldVec3 position;
        };
        std::vector<Port> ports;
        m_world.forEach<sw::parts::PartComponent, TransformComponent>(
            [&](sw::ecs::Entity entity, sw::parts::PartComponent& part,
                TransformComponent& transform) {
                const auto* definition = sw::parts::findDefinition(part.definitionId);
                if (definition != nullptr &&
                    definition->type == sw::parts::PartType::DockingPort)
                {
                    ports.push_back({entity, part.vessel, transform.position});
                }
            });

        sw::parts::DockingCapture advice{};
        sw::f32 adviceRange = kAdviceRangeM;
        for (sw::usize a = 0; a < ports.size(); ++a)
        {
            for (sw::usize b = a + 1; b < ports.size(); ++b)
            {
                if (ports[a].vessel == ports[b].vessel)
                {
                    continue;
                }
                // A cheap reject first: the pair loop is over every port in
                // the world, and a capture is a third of a metre.
                const sw::f32 range =
                    static_cast<sw::f32>(glm::length(ports[a].position -
                                                     ports[b].position));
                if (range > kAdviceRangeM)
                {
                    continue;
                }
                const sw::parts::DockingCapture capture =
                    sw::parts::dockingCapture(m_world, ports[a].part, ports[b].part);
                if (!capture.valid)
                {
                    // Keep the nearest failing pair, but only if one of them is
                    // the craft being flown: advice about two other people's
                    // ships is noise.
                    const bool mine = ports[a].vessel == m_shipEntity ||
                                      ports[b].vessel == m_shipEntity;
                    if (mine && range < adviceRange)
                    {
                        adviceRange = range;
                        advice = capture;
                    }
                    continue;
                }

                // THE CRAFT YOU ARE ABOARD SURVIVES. Not the heavier one, not
                // the first one found: the surviving root is what the player is
                // flying, what the camera follows and what the autopilot holds,
                // and having that swapped out from under them at the moment of
                // capture would be the most confusing possible way to succeed.
                const bool shipIsA = ports[a].vessel == m_shipEntity;
                sw::parts::DockingCapture ordered = capture;
                if (!shipIsA && ports[b].vessel == m_shipEntity)
                {
                    ordered.partA = capture.partB;
                    ordered.partB = capture.partA;
                    ordered.nodeA = capture.nodeB;
                    ordered.nodeB = capture.nodeA;
                }
                const sw::ecs::Entity survivor =
                    m_world.getComponent<sw::parts::PartComponent>(ordered.partA).vessel;
                const sw::ecs::Entity absorbed =
                    m_world.getComponent<sw::parts::PartComponent>(ordered.partB).vessel;
                adoptVesselRoot(survivor, absorbed);
                if (sw::parts::dockVessels(m_world, ordered))
                {
                    SW_LOG_INFO("Game",
                                "DOCKED: gap {:.2f} m, facing {:.3f}, {:.2f} m/s",
                                static_cast<double>(capture.gapM),
                                static_cast<double>(capture.facing),
                                static_cast<double>(capture.closingMps));
                    m_dockStatus = "DOCKED";
                    // The port list holds stale vessel handles now, and one
                    // capture per pass is all a 0.1 s search can possibly miss.
                    return;
                }
            }
        }

        // ---- what the pilot is failing ---------------------------------------
        //
        // ONE LINE, AND IT NAMES THE LIMIT. Four conditions fail in four ways
        // and three of them look identical from the cockpit: nothing happens.
        // The order is the order they matter in on an approach — you get lined
        // up, then square, then slow, and only then does the gap close.
        if (!advice.partA.isNull())
        {
            const sw::parts::DockingLimits limits{};
            if (advice.facing < limits.facing)
            {
                m_dockStatus = std::format("DOCK: ROLL IN, PORTS NOT FACING  {:.2f}",
                                           advice.facing);
            }
            else if (advice.onAxis < limits.onAxis)
            {
                m_dockStatus = std::format("DOCK: OFF AXIS  {:.2f}", advice.onAxis);
            }
            else if (advice.closingMps > limits.closingMps)
            {
                m_dockStatus = std::format("DOCK: TOO FAST  {:.2f} M/S",
                                           advice.closingMps);
            }
            else if (advice.closingMps < -0.02f)
            {
                m_dockStatus = std::format("DOCK: DRIFTING APART  {:.2f} M/S",
                                           -advice.closingMps);
            }
            else
            {
                m_dockStatus = std::format("DOCK: LINED UP  {:.2f} M", advice.gapM);
            }
        }
    }

    // ------------------------------------------------------------------------
    // RELEASE
    // ------------------------------------------------------------------------
    void StarWorksGame::undockPart(sw::ecs::Entity portPart)
    {
        if (portPart.isNull() || !m_world.isAlive(portPart))
        {
            return;
        }
        const sw::ecs::Entity leaving = sw::parts::undockAt(m_world, portPart);
        if (leaving.isNull())
        {
            SW_LOG_INFO("Game", "UNDOCK: that port is not docked to anything");
            return;
        }
        // splitVessel hands over a transform, a body and a VesselComponent;
        // the two things that make a craft VISIBLE on the map are the game's,
        // exactly as they are when a stage separates.
        m_world.addComponent(leaving, BoundsComponent{0.1f});
        m_world.addComponent(leaving, MapMarkerComponent{{0.6f, 0.75f, 0.9f, 1.0f}});
        // A RELEASED CRAFT IS STILL A CRAFT. Without controls and an autopilot
        // it cannot be flown, cannot be boarded meaningfully, and would sit
        // there as debris that used to be a ship — which is what happens today
        // to a decoupled stage, correctly, and is wrong for a dock.
        if (m_world.tryGetComponent<ShipComponent>(leaving) == nullptr)
        {
            m_world.addComponent(leaving, ShipComponent{});
            m_world.addComponent(leaving, ShipControlsComponent{});
            m_world.addComponent(leaving, SasComponent{});
        }
        m_menuPart = {};
        m_dockStatus = "UNDOCKED";
        SW_LOG_INFO("Game", "UNDOCK: released, vessel {} away", leaving.index);
    }
} // namespace game
