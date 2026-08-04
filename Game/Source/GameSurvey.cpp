// ============================================================================
// GameSurvey.cpp — the orbital survey: arming the OS-1 and filling in the
// coverage under the ground track.
//
// The split this file lives on is the one Deposits.hpp sets out: ore is an
// analytic FIELD, a property of a place that is never stored, and the survey
// is the STATE that says whether anyone has looked. Whatever draws the first
// may only do it where the second permits, so the picture the player plans
// from and the number the miner is paid on are the same function — a survey
// cannot lie, because there is nothing for it to lie about.
//
// IT DRAWS ONE THING NOW: the tint on the ground you are standing on. The
// orbital map used to carry a diamond per surveyed cell as well, and F44
// removed them — two overlays of one dataset, one of which could show only
// the richest of three resources, is one overlay too many. The map is for
// flying and F4 is for geology.
// ============================================================================

#include "StarWorksGame.hpp"

#include "GameInternal.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <format>

namespace game
{
    namespace
    {
        /// How wide a swath one pass reveals, as an angle at the body's
        /// centre. Six degrees is a little over one latitude cell, so a
        /// polar orbit closes the map in a few dozen revolutions and an
        /// equatorial one never does — which is the correct incentive to
        /// think about the orbit you put the instrument in.
        constexpr sw::f32 kSwathRadians = 0.105f;
    } // namespace

    // ------------------------------------------------------------------------
    // ARMED, AND IN A STABLE ORBIT
    //
    // Two conditions, and the second is the interesting one. "Stable" here
    // means the orbit the flight state already computes for the warp ladder:
    // closed, not resting on anything, and with its LOW point above the air.
    // An instrument on a suborbital arc is a camera pointed at the ground it
    // is about to hit, and one dipping into the atmosphere every pass has a
    // mission measured in hours — neither is a survey.
    // ------------------------------------------------------------------------
    bool StarWorksGame::surveyArmed() const
    {
        if (m_shipEntity.isNull() || !m_world.isAlive(m_shipEntity))
        {
            return false;
        }
        bool armed = false;
        auto& world = const_cast<sw::ecs::World&>(m_world);
        world.forEach<sw::parts::PartComponent, sw::parts::PartAnimationComponent>(
            [&](sw::ecs::Entity, sw::parts::PartComponent& part,
                sw::parts::PartAnimationComponent& state) {
                if (armed || part.vessel != m_shipEntity ||
                    part.definitionId != sw::parts::kPartOrbitalSurveyor)
                {
                    return;
                }
                armed = state.count > 0 && state.target[0] > 0.5f;
            });
        return armed;
    }

    bool StarWorksGame::surveyOrbitStable() const
    {
        return !m_flight.grounded && m_flight.closedOrbit &&
               m_flight.periapsisAltitude > std::max(m_flight.atmosphereTop, 0.0);
    }

    void StarWorksGame::updateSurvey()
    {
        m_surveyStatus.clear();
        m_surveyFraction = 0.0f;
        if (!surveyArmed())
        {
            return;
        }
        if (!surveyOrbitStable())
        {
            // SAID, not silently ignored. An instrument that is switched on
            // and doing nothing is the single most confusing state this
            // feature can be in, and the player cannot see an orbit's
            // periapsis from the cockpit.
            m_surveyStatus = "SURVEY IDLE: NEEDS A STABLE ORBIT";
            return;
        }
        if (m_flight.primaryIndex < 0)
        {
            return;
        }
        const auto& primary =
            m_celestialIndex.body(static_cast<sw::usize>(m_flight.primaryIndex));
        if (m_world.tryGetComponent<sw::planet::DepositComponent>(primary.entity) ==
            nullptr)
        {
            m_surveyStatus = "SURVEY IDLE: NO GEOLOGY HERE";
            return; // a gas giant or a star has nothing to survey
        }
        // ONE FRAME IN A SPHERE OF INFLUENCE IS NOT A SURVEY.
        //
        // Measured, and it put a row on the geology screen nobody could
        // explain: at the moment warp went to x1000 the flight state named
        // MARS as the primary for a single tick, and a body that is the
        // primary for one tick got a survey component, a swath of coverage,
        // and a place in a list of "worlds a satellite has looked at" — for a
        // planet no craft has ever been near.
        //
        // So the first frame at a new primary only REMEMBERS it. Painting
        // needs a previous sample under the same body, which is what the arc
        // already needed anyway; a flicker never gets a second frame, and a
        // real arrival loses nothing but its first sixtieth of a second.
        if (m_surveyPrimary != primary.entity)
        {
            m_surveyPrimary = primary.entity;
            m_surveyPrevious = sw::Vec3{0.0f};
            return;
        }
        // ADDED WHERE IT IS FIRST NEEDED rather than at the nine places a
        // deposit is attached — and that includes the systems loaded lazily
        // on arrival, which no pass over the starting scene could reach.
        if (m_world.tryGetComponent<sw::planet::SurveyComponent>(primary.entity) == nullptr)
        {
            // SAID OUT LOUD, once per body. The geology screen lists every
            // world that carries one of these, so a body that acquires a
            // survey it never should have turns into a row the player cannot
            // explain. One line in the log names the moment it happened.
            SW_LOG_INFO("Game", "Survey started on {}", primary.name);
            m_world.addComponent(primary.entity, sw::planet::SurveyComponent{});
        }
        auto* survey = m_world.tryGetComponent<sw::planet::SurveyComponent>(primary.entity);
        const auto* bodyTransform =
            m_world.tryGetComponent<TransformComponent>(primary.entity);
        const auto* self = m_world.tryGetComponent<TransformComponent>(controlledEntity());
        if (survey == nullptr || bodyTransform == nullptr || self == nullptr)
        {
            return;
        }

        // THE POINT UNDER THE SPACECRAFT, in the body's ROTATING frame —
        // the frame the ore field is a function of, and the reason a
        // stationary satellite over a spinning planet still sweeps out a
        // band. Taken from the body's f64 spin state for the same reason
        // the terrain patch is: an f32 quaternion slides the direction by
        // enough to move the ground under your feet.
        const sw::WorldVec3 radial = self->position - bodyTransform->position;
        const sw::f64 distance = glm::length(radial);
        if (!(distance > 1.0))
        {
            return;
        }
        const auto* spin =
            m_world.tryGetComponent<sw::phys::GravitySourceComponent>(primary.entity);
        const glm::dquat inverseRotation =
            (spin != nullptr) ? glm::inverse(sw::phys::spinRotation(*spin))
                              : glm::inverse(glm::dquat(bodyTransform->rotation));
        const sw::Vec3 subSatellite =
            sw::Vec3(glm::normalize(inverseRotation * (radial / distance)));

        // The coverage grows because the CRAFT WENT SOMEWHERE, not because
        // frames happened: the arc between this sample and the last is
        // painted whole, so warp and frame rate both stop mattering. A single
        // point per frame read one per cent after nine revolutions at x100,
        // because the craft crosses eight degrees between frames and the
        // instrument sees six.
        if (glm::length(m_surveyPrevious) > 0.5f)
        {
            sw::planet::markSurveyArc(*survey, m_surveyPrevious, subSatellite,
                                      kSwathRadians);
        }
        else
        {
            sw::planet::markSurveySwath(*survey, subSatellite, kSwathRadians);
        }
        m_surveyPrevious = subSatellite;
        m_surveyFraction = sw::planet::surveyFraction(*survey);
        if (std::getenv("SW_SURVEYPROBE") != nullptr)
        {
            static sw::u32 tick = 0;
            if ((tick++ % 40u) == 0u)
            {
                SW_LOG_INFO("Game",
                            "survey: sub-satellite {:.3f} {:.3f} {:.3f}  {:.1f}%",
                            static_cast<double>(subSatellite.x),
                            static_cast<double>(subSatellite.y),
                            static_cast<double>(subSatellite.z),
                            static_cast<double>(m_surveyFraction * 100.0f));
            }
        }
        m_surveyStatus = std::format("SURVEY {} {:.0f}%", primary.name,
                                     m_surveyFraction * 100.0f);
    }

    // ------------------------------------------------------------------------
    // WHAT A SURVEYED PLACE IS WORTH LOOKING AT
    //
    // One colour per resource, and the alpha carries the grade. The baseline
    // tenth that every rock now carries must NOT paint the whole planet, or
    // the tint says "there is ore everywhere" — which is true and useless. So
    // the display threshold sits above the floor: what the ground shows is
    // where it is worth putting a drill.
    //
    // The only caller left is the terrain patch builder. The geology screen
    // reads the ramp directly, because a screen devoted to one resource has
    // no use for "the best of three" — which is exactly the reading that made
    // the old map overlay worth deleting.
    // ------------------------------------------------------------------------
    bool StarWorksGame::surveyTint(const sw::planet::DepositComponent& deposits,
                                   const sw::Vec3& bodyDirection, sw::Vec4& outColor)
    {
        sw::f32 density = 0.0f;
        const sw::res::Resource best =
            sw::planet::bestDeposit(deposits, bodyDirection, density);
        constexpr sw::f32 kWorthMoving = 0.28f;
        if (best == sw::res::Resource::Count || density < kWorthMoving)
        {
            return false;
        }
        // ONE PALETTE FOR THE WHOLE GAME. The ground under your boots and the
        // globe on F4 are two views of one number, and they were briefly two
        // views in two sets of colours — copper an orange here and malachite
        // there, which is the kind of disagreement that makes a player think
        // they are looking at different data.
        //
        // The ramp is sampled at a fixed step rather than at the density,
        // because here the GRADE is carried by the alpha: taking both from the
        // ramp would multiply the same fact into the picture twice and leave a
        // marginal deposit as a smear of black on the rock.
        const sw::Vec3 hue = sw::planet::oreRampColor(best, 0.62f);
        // Grade, mapped across the band that is actually on the map: at the
        // threshold the mark is faint, at a rich core it is solid.
        const sw::f32 grade =
            glm::clamp((density - kWorthMoving) / (1.0f - kWorthMoving), 0.0f, 1.0f);
        outColor = {hue.r, hue.g, hue.b, 0.35f + 0.65f * grade};
        return true;
    }

} // namespace game
