#pragma once

// ============================================================================
// Planet/Survey.hpp
// WHAT HAS BEEN LOOKED AT — the one thing about a body's geology that is
// state rather than a function.
//
// Deposits.hpp is an analytic field: ore is a property of a place, not an
// object that exists, and nothing about it is saved. That is exactly right
// for the ore and exactly wrong for the SURVEY, because a survey is a record
// of what a player did. So the split is: the field says what is in the
// ground, this says whether anyone has looked, and the overlay draws the
// first only where the second permits.
//
// The grid is equirectangular and coarse — sixty-four by thirty-two cells,
// one bit each, two hundred and fifty-six bytes for a whole planet. Coarse is
// correct: the cells gate WHETHER the field is shown, and the field itself is
// still sampled continuously underneath, so a cell boundary never quantises
// what the player reads. On Terra a cell is about 600 km across, which is
// roughly what a real orbital instrument resolves in one pass anyway.
//
// Trivially copyable and fixed size, like every component the snapshot
// memcpy's by column.
// ============================================================================

#include "Core/Types.hpp"
#include "Math/Math.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <type_traits>

namespace sw::planet
{
    /// Per-body survey coverage. Rides on the body entity next to its
    /// TerrainComponent and DepositComponent.
    struct SurveyComponent
    {
        static constexpr u32 kLongitudeCells = 64;
        static constexpr u32 kLatitudeCells = 32;
        static constexpr u32 kCells = kLongitudeCells * kLatitudeCells;
        static constexpr u32 kWords = kCells / 32;

        u32 cells[kWords]{};
    };
    static_assert(std::is_trivially_copyable_v<SurveyComponent>);
    static_assert(sizeof(SurveyComponent) == 256);

    /// The cell a direction in the body's rotating frame falls in.
    ///
    /// Equirectangular, so cells near the poles are narrow — which is the
    /// right bias for a polar orbit and harmless for the overlay, because
    /// what it indexes is a yes/no and not an area.
    [[nodiscard]] inline u32 surveyCell(const Vec3& unitDirectionBodyFrame)
    {
        const Vec3 dir = unitDirectionBodyFrame;
        constexpr f32 kPi = 3.14159265358979f;
        const f32 longitude = std::atan2(dir.z, dir.x);          // [-pi, pi]
        const f32 latitude = std::asin(glm::clamp(dir.y, -1.0f, 1.0f)); // [-pi/2, pi/2]
        const f32 u = (longitude + kPi) / (2.0f * kPi);
        const f32 v = (latitude + kPi * 0.5f) / kPi;
        const u32 x = std::min(static_cast<u32>(u * SurveyComponent::kLongitudeCells),
                               SurveyComponent::kLongitudeCells - 1);
        const u32 y = std::min(static_cast<u32>(v * SurveyComponent::kLatitudeCells),
                               SurveyComponent::kLatitudeCells - 1);
        return y * SurveyComponent::kLongitudeCells + x;
    }

    /// The centre of a cell, as a unit direction in the body's frame. The
    /// map overlay samples the ore field here.
    [[nodiscard]] inline Vec3 surveyCellDirection(u32 cell)
    {
        constexpr f32 kPi = 3.14159265358979f;
        const u32 x = cell % SurveyComponent::kLongitudeCells;
        const u32 y = (cell / SurveyComponent::kLongitudeCells) %
                      SurveyComponent::kLatitudeCells;
        const f32 u = (static_cast<f32>(x) + 0.5f) /
                      static_cast<f32>(SurveyComponent::kLongitudeCells);
        const f32 v = (static_cast<f32>(y) + 0.5f) /
                      static_cast<f32>(SurveyComponent::kLatitudeCells);
        const f32 longitude = u * 2.0f * kPi - kPi;
        const f32 latitude = v * kPi - kPi * 0.5f;
        const f32 ring = std::cos(latitude);
        return Vec3{ring * std::cos(longitude), std::sin(latitude),
                    ring * std::sin(longitude)};
    }

    [[nodiscard]] inline bool surveyed(const SurveyComponent& survey, u32 cell)
    {
        if (cell >= SurveyComponent::kCells)
        {
            return false;
        }
        return (survey.cells[cell / 32u] & (1u << (cell % 32u))) != 0u;
    }

    [[nodiscard]] inline bool surveyed(const SurveyComponent& survey,
                                       const Vec3& unitDirectionBodyFrame)
    {
        return surveyed(survey, surveyCell(unitDirectionBodyFrame));
    }

    inline void markSurveyed(SurveyComponent& survey, u32 cell)
    {
        if (cell < SurveyComponent::kCells)
        {
            survey.cells[cell / 32u] |= (1u << (cell % 32u));
        }
    }

    /// Marks everything within `swathRadians` of a direction — one pass of an
    /// instrument with a real swath rather than a single pixel under the
    /// spacecraft. Returns how many cells this call newly revealed, which is
    /// what a progress readout wants: cells already known cost nothing.
    inline u32 markSurveySwath(SurveyComponent& survey, const Vec3& unitDirectionBodyFrame,
                               f32 swathRadians)
    {
        const f32 cosine = std::cos(swathRadians);
        u32 revealed = 0;
        for (u32 cell = 0; cell < SurveyComponent::kCells; ++cell)
        {
            if (surveyed(survey, cell))
            {
                continue;
            }
            if (glm::dot(surveyCellDirection(cell), unitDirectionBodyFrame) >= cosine)
            {
                markSurveyed(survey, cell);
                ++revealed;
            }
        }
        return revealed;
    }

    /// Marks the swath along the ARC between two sub-satellite points.
    ///
    /// A single point per frame is what a scanner does at sixty hertz and a
    /// lie at any other rate: under warp the craft moves eight degrees
    /// between frames while the instrument sees six, so the track comes out
    /// dotted and a survey that should close in a dozen orbits never closes
    /// at all. Measured before this existed — nine revolutions at x100 read
    /// one per cent.
    ///
    /// Stepping along the arc at no more than most of a swath makes the
    /// coverage a function of WHERE THE CRAFT WENT rather than of how often
    /// anyone looked, which is the property that makes warp and frame rate
    /// both stop mattering. A jump too large to be flight — a teleport, a
    /// rails hand-off, entering a new sphere of influence — is not an arc and
    /// is not painted.
    inline u32 markSurveyArc(SurveyComponent& survey, const Vec3& from, const Vec3& to,
                             f32 swathRadians)
    {
        const f32 cosine = glm::clamp(glm::dot(from, to), -1.0f, 1.0f);
        const f32 sweep = std::acos(cosine);
        constexpr f32 kMaxArc = 1.0f; // ~57 deg: past this it is not flight
        if (!(sweep > 0.0f) || sweep > kMaxArc)
        {
            return markSurveySwath(survey, to, swathRadians);
        }
        const f32 step = std::max(swathRadians * 0.8f, 1.0e-3f);
        const u32 steps = static_cast<u32>(std::ceil(sweep / step));
        u32 revealed = 0;
        for (u32 i = 1; i <= steps; ++i)
        {
            const f32 t = static_cast<f32>(i) / static_cast<f32>(steps);
            const Vec3 point = glm::normalize(glm::mix(from, to, t));
            revealed += markSurveySwath(survey, point, swathRadians);
        }
        return revealed;
    }

    /// Fraction of the body surveyed, in [0,1]. Counted by CELL rather than
    /// by area: the poles are over-represented, and saying so here is better
    /// than a progress bar that stalls at 97% because two polar cells are
    /// each the size of a car park.
    [[nodiscard]] inline f32 surveyFraction(const SurveyComponent& survey)
    {
        u32 known = 0;
        for (const u32 word : survey.cells)
        {
            known += static_cast<u32>(std::popcount(word));
        }
        return static_cast<f32>(known) / static_cast<f32>(SurveyComponent::kCells);
    }
} // namespace sw::planet
