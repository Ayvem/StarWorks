// ============================================================================
// GameStars.cpp — the interstellar half of the game.
//
// Three things live here, and they are one idea seen from three sides.
//
// THE FLOATING ORIGIN. The world's origin is a star, and it moves. Everything
// in the ECS is stored relative to it, so however far from Sol the player
// travels the numbers a double has to hold stay the size of one solar system.
//
// SYSTEM STREAMING. Only one system's planets exist at a time. Twenty-three
// systems' worth of bodies would be a hundred and thirty entities in every
// per-tick loop in the game, a hundred and twenty-nine of them light-years
// away and contributing nothing but a distance computation — and the celestial
// index's topological sort is O(n^2). The stars are all always there, because
// you must be able to SEE where you are going; their planets arrive when you
// do.
//
// THE STARS THEMSELVES. Drawn as real bodies at real distances, which means
// their apparent brightness has to come out of the same arithmetic the Sun's
// does or the neighbourhood will not read as a neighbourhood: Sirius has to be
// the brightest thing in the sky from here, and Sol has to be an ordinary
// yellow star of magnitude 2.7 from Sirius.
// ============================================================================

#include "StarWorksGame.hpp"

#include "GameInternal.hpp"

#include <algorithm>
#include <cmath>

namespace game
{
    namespace
    {
        /// The vertex colour a world of this style starts from, before the
        /// per-fragment surface takes over. The same twelve values the solar
        /// system's own bodies use, keyed by style rather than by name.
        [[nodiscard]] sw::Vec4 exoplanetBaseColor(sw::i32 style)
        {
            switch (style)
            {
            case 2: return {0.62f, 0.32f, 0.18f, 1.0f};  // Mars: rust
            case 3: return {0.55f, 0.52f, 0.49f, 1.0f};  // Mercury: grey rock
            case 4: return {0.86f, 0.72f, 0.36f, 1.0f};  // Io: sulphur
            case 5: return {0.80f, 0.82f, 0.85f, 1.0f};  // Europa: cracked ice
            case 6: return {0.58f, 0.55f, 0.52f, 1.0f};  // Ganymede
            case 7: return {0.48f, 0.44f, 0.41f, 1.0f};  // Callisto
            case 9: return {0.92f, 0.95f, 1.00f, 1.0f};  // Enceladus: clean ice
            case 13: return {0.82f, 0.75f, 0.72f, 1.0f}; // Triton: nitrogen
            case 20: return {0.78f, 0.66f, 0.50f, 1.0f}; // Jupiter
            case 21: return {0.85f, 0.75f, 0.55f, 1.0f}; // Saturn
            case 22: return {0.60f, 0.82f, 0.85f, 1.0f}; // Uranus
            case 23: return {0.35f, 0.50f, 0.90f, 1.0f}; // Neptune
            default: return {0.60f, 0.58f, 0.55f, 1.0f};
            }
        }
    } // namespace
    // ------------------------------------------------------------------------
    // THE ORIGIN SHIFT
    // ------------------------------------------------------------------------
    void StarWorksGame::rebaseOrigin(sw::u32 systemIndex)
    {
        if (systemIndex == m_originSystem ||
            systemIndex >= sw::space::systems().size())
        {
            return;
        }
        // Positions are LOCAL, so moving the origin by +d moves every local
        // position by -d. Written as (old - new) so the sign is read off the
        // anchors rather than remembered.
        const sw::WorldVec3 delta = sw::space::systems()[m_originSystem].position -
                                    sw::space::systems()[systemIndex].position;
        const sw::u32 previous = m_originSystem;
        m_originSystem = systemIndex;

        // 1. EVERY ENTITY, and its previous transform with it. Missing the
        //    second one does not look like a missing shift: it looks like one
        //    frame of motion blur four light-years long, because the renderer
        //    interpolates between them.
        m_world.forEach<TransformComponent>(
            [&](sw::ecs::Entity, TransformComponent& transform) {
                transform.position += delta;
            });
        m_world.forEach<sw::PreviousTransformComponent>(
            [&](sw::ecs::Entity, sw::PreviousTransformComponent& previousTransform) {
                previousTransform.position += delta;
            });

        // 2. EVERY CAMERA, in the same frame. The renderer is camera-relative,
        //    so a shift both sides agree on is invisible and a shift only one
        //    side got is the whole sky jumping.
        m_camera.translate(delta);
        m_mapCamera.translate(delta);
        m_hangarCamera.translate(delta);
        m_menuCamera.translate(delta);
        // ...and the free camera's private copy, which would otherwise put the
        // view back where it was the next time a movement key was pressed.
        m_cameraController.translate(delta);

        // 3. THE PARTICLES. Reentry plasma and engine exhaust are integrated
        //    in absolute space and live across frames — the one non-ECS cache
        //    in the game that holds a world position. Un-shifted, a ship that
        //    crossed a system boundary under thrust would leave its flame
        //    behind at the old star.
        for (ReentryParticle& particle : m_particles)
        {
            particle.position += delta;
        }

        // 4. Everything else is either rebuilt from the ECS before it is next
        //    read (the celestial index, the physics scratch arrays, the bubble
        //    focus) or was never absolute to begin with (terrain patch origins
        //    are body-local, trajectory segments are primary-relative). The
        //    predictions are dropped anyway, because their cached primary
        //    INDICES are about to point at a different set of bodies.
        m_prediction.clear();
        m_nodePrediction.clear();
        m_burnCoast.clear();
        m_targetIndex = -1;
        m_mapFocusIndex = -1;

        SW_LOG_INFO("Game", "Origin moved from {} to {} ({:.3f} ly)",
                    sw::space::systems()[previous].name,
                    sw::space::systems()[systemIndex].name,
                    glm::length(delta) / sw::space::kLightYear);
    }

    // ------------------------------------------------------------------------
    // THE STARS
    // ------------------------------------------------------------------------
    void StarWorksGame::buildCatalogueStars()
    {
        const auto starTable = sw::space::stars();
        m_starEntities.assign(starTable.size(), sw::ecs::Entity{});
        m_systemLoaded.assign(sw::space::systems().size(), false);
        // Sol is built by hand and never streams: the colony is on it.
        m_systemLoaded[sw::space::kSolSystem] = true;

        // Sol was built by hand in buildScene() and is star zero. It keeps its
        // own entity — the spawn point, the menu backdrop and half the game's
        // named handles hang off it — and only gains the component that says
        // it is one of thirty-six rather than the only one.
        m_starEntities[0] = m_solEntity;
        m_world.addComponent(m_solEntity,
                             StarVisualComponent{0u, 1.0f, 1.0f,
                                                 blackbodyColor(5772.0f)});

        for (sw::u32 i = 1; i < starTable.size(); ++i)
        {
            const sw::space::StarRecord& star = starTable[i];
            const sw::ecs::Entity e = m_world.createEntity();

            TransformComponent transform{};
            transform.position =
                localPosition(sw::space::starPositionAt(i, 0.0));
            transform.uniformScale = static_cast<sw::f32>(star.radius);
            m_world.addComponent(e, transform);
            m_world.addComponent(e, PreviousTransformComponent{transform.position,
                                                                transform.rotation});
            m_world.addComponent(e, BoundsComponent{1.0f});

            // SURFACE brightness, not luminosity — see StarVisualComponent.
            // (T/Tsol)^4, because that is what a square metre of photosphere
            // radiates and a square metre of photosphere is what a pixel of it
            // shows. Sirius B is 1/30000 of Sol's output and fifty-six times
            // brighter per unit area.
            const sw::f32 ratio =
                static_cast<sw::f32>(star.temperature) / 5772.0f;
            const sw::f32 radiance = ratio * ratio * ratio * ratio;
            const sw::Vec3 hue =
                blackbodyColor(static_cast<sw::f32>(star.temperature));
            m_world.addComponent(
                e, StarVisualComponent{i, radiance,
                                       static_cast<sw::f32>(star.luminosity), hue});
            m_world.addComponent(
                e, makeSphereLodSet({hue.r, hue.g, hue.b, 1.0f}));

            sw::phys::GravitySourceComponent gravity{star.mu, star.radius};
            gravity.soiRadius = star.soiRadius;
            m_world.addComponent(e, gravity);

            // A COMPANION ORBITS ITS PRIMARY; a primary does not move at all.
            // "Les systemes solaires ne se deplacent pas" was the brief and it
            // is also the physics: proper motion is real but it is a
            // millionth of a light-year a year, and a sky that drifts is a
            // sky you cannot learn.
            const sw::space::SystemRecord& system =
                sw::space::systems()[star.systemIndex];
            if (star.sma > 0.0)
            {
                const sw::space::StarRecord& primary =
                    starTable[system.firstStar];
                const sw::phys::KeplerOrbit orbit = sw::phys::kepler::fromElements(
                    primary.mu, star.sma, star.eccentricity, star.inclination,
                    star.ascendingNode, 0.0, star.meanAnomaly, 0.0);
                m_world.addComponent(
                    e, sw::space::makeCelestialBody(star.name,
                                                    m_starEntities[system.firstStar],
                                                    &orbit));
            }
            else
            {
                m_world.addComponent(e, sw::space::makeCelestialBody(star.name));
            }
            m_world.addComponent(e, MapMarkerComponent{{hue.r, hue.g, hue.b, 1.0f}});
            m_starEntities[i] = e;
        }
        SW_LOG_INFO("Game", "Star catalogue: {} stars in {} systems within 12 ly",
                    starTable.size(), sw::space::systems().size());
    }

    // ------------------------------------------------------------------------
    // WHICH STAR IS THE SUN HERE
    // ------------------------------------------------------------------------
    sw::ecs::Entity StarWorksGame::dominantStar(const sw::WorldVec3& position) const
    {
        // BRIGHTEST AT THIS POINT, not nearest. From a world orbiting Alpha
        // Centauri B the nearest star is B and the brightest is usually A,
        // twenty AU away and three times the output; from Proxima the nearest
        // is Proxima and it is also the brightest by six orders of magnitude.
        // Nearest gets both of those right most of the time and one of them
        // wrong in exactly the system a player is most likely to visit.
        sw::ecs::Entity best{};
        sw::f64 bestIrradiance = -1.0;
        for (sw::ecs::Entity entity : m_starEntities)
        {
            if (entity.isNull())
            {
                continue;
            }
            const auto* transform =
                const_cast<sw::ecs::World&>(m_world).tryGetComponent<TransformComponent>(entity);
            const auto* visual =
                const_cast<sw::ecs::World&>(m_world).tryGetComponent<StarVisualComponent>(entity);
            if (transform == nullptr || visual == nullptr)
            {
                continue;
            }
            const sw::WorldVec3 delta = transform->position - position;
            const sw::f64 distanceSq = std::max(glm::dot(delta, delta), 1.0);
            const sw::f64 irradiance =
                static_cast<sw::f64>(visual->luminosity) / distanceSq;
            if (irradiance > bestIrradiance)
            {
                bestIrradiance = irradiance;
                best = entity;
            }
        }
        return best;
    }

    // ------------------------------------------------------------------------
    // EVERY SUN IN THIS SKY.
    //
    // "Quand nous sommes dans un systeme binaire seule 1 etoile sur les deux a
    // les effets style soleil alors que les deux devraient l'avoir."
    //
    // The renderer had exactly one sun — dominantStar, the brightest from
    // here — and everything else went down the billboard path, whose angular
    // size is capped at flux^0.2 so that a star four light-years away stays a
    // point. Alpha Centauri B is twenty-three astronomical units from A, half
    // Sol's output, magnitude about -19 from a world orbiting A. It got the
    // four-light-year treatment.
    //
    // THE TEST IS A RATIO, AND THE RATIO IS WHY IT WORKS. An absolute
    // brightness cut would POP: fly out of a system and the star would cross
    // the threshold and change from a glare to a billboard in one frame. Both
    // members of a pair dim together as you leave, so their ratio is very
    // nearly constant all the way out — they keep their glare together, and it
    // shrinks smoothly to the same two-pixel floor the primary's does.
    //
    // It also excludes what it should. From Proxima's own planet, Alpha Cen A
    // is thirteen thousand AU away and delivers 1.3e-8 of what Proxima itself
    // does — eight orders below the cut, so it stays the bright star it looks
    // like from there. A wide companion a thousand AU out lands at 1e-6 and
    // stays a billboard too, which is right: at that range a second Sol is
    // magnitude -11, a very bright point and not a second daylight.
    //
    // A hundredth of a percent is the cut. Alpha Cen B from A's habitable zone
    // comes out at 6.3e-4, comfortably inside; nothing that is not genuinely a
    // sun in this sky comes near it.
    // ------------------------------------------------------------------------
    void StarWorksGame::collectSunsHere(const sw::WorldVec3& position,
                                        std::vector<sw::ecs::Entity>& out) const
    {
        sw::ecs::World& world = const_cast<sw::ecs::World&>(m_world);
        auto irradianceAt = [&](sw::ecs::Entity candidate) -> sw::f64 {
            const auto* transform = world.tryGetComponent<TransformComponent>(candidate);
            const auto* visual = world.tryGetComponent<StarVisualComponent>(candidate);
            if (transform == nullptr || visual == nullptr)
            {
                return -1.0;
            }
            const sw::WorldVec3 delta = transform->position - position;
            return static_cast<sw::f64>(visual->luminosity) /
                   std::max(glm::dot(delta, delta), 1.0);
        };
        const sw::f64 dominant = irradianceAt(m_lightStar);
        if (!(dominant > 0.0))
        {
            if (!m_lightStar.isNull()) { out.push_back(m_lightStar); }
            return;
        }
        // The dominant star FIRST: its is the flare that gets drawn, and the
        // caller decides that by comparing against m_lightStar.
        out.push_back(m_lightStar);
        for (const sw::ecs::Entity candidate : m_starEntities)
        {
            if (candidate.isNull() || candidate == m_lightStar)
            {
                continue;
            }
            if (irradianceAt(candidate) >= dominant * sw::space::kSunIrradianceRatio)
            {
                out.push_back(candidate);
            }
        }
    }

    // ------------------------------------------------------------------------
    // THE OTHER STARS
    // ------------------------------------------------------------------------
    void StarWorksGame::collectDistantStars(const sw::Camera& activeCamera)
    {
        // The three-layer glare belongs to the local sun. Every other star is
        // ONE billboard, and its size and brightness come from the same two
        // formulas the nine-thousand-star dome uses, fed a real apparent
        // magnitude instead of a random draw.
        //
        // Sharing the formulas is the whole point. These thirty-six sit in the
        // same sky as the dome's nine thousand, and if they were lit by a
        // different rule the eye would find them instantly: Sirius has to be
        // brighter than everything around it because it IS, not because it is
        // special-cased. Flux is on the dome's scale, where 1.0 is magnitude
        // 6.5 — the naked-eye limit — so Sirius from Terra comes out at 1.5e3
        // and Proxima, four light-years nearer and magnitude 11, at 0.02.
        const sw::WorldVec3 cameraPosition = activeCamera.position();
        for (sw::ecs::Entity entity : m_starEntities)
        {
            // Skip every star drawn as a SUN this frame, not just the
            // brightest: a companion given both treatments would carry a
            // billboard buried inside its own glare.
            if (entity.isNull() ||
                std::find(m_sunsHere.begin(), m_sunsHere.end(), entity) !=
                    m_sunsHere.end())
            {
                continue;
            }
            const auto* transform = m_world.tryGetComponent<TransformComponent>(entity);
            const auto* visual = m_world.tryGetComponent<StarVisualComponent>(entity);
            if (transform == nullptr || visual == nullptr)
            {
                continue;
            }
            const sw::Vec3 offset = sw::Vec3(transform->position - cameraPosition);
            const sw::f32 distance = glm::length(offset);
            if (!(distance > 1.0f))
            {
                continue;
            }
            // Apparent magnitude: absolute magnitude from the luminosity
            // (Sol's is 4.83), then the distance modulus. One parsec is
            // 3.2616 light-years.
            const sw::f64 parsecs =
                static_cast<sw::f64>(distance) / (sw::space::kLightYear * 3.26156);
            if (!(parsecs > 1.0e-12))
            {
                continue;
            }
            // 4.74 is the IAU 2015 nominal BOLOMETRIC absolute magnitude of
            // the Sun, and bolometric is the right one here because the
            // catalogue's luminosities are bolometric. It does flatter the red
            // dwarfs — most of what an M5 emits is infrared nobody sees, so
            // Proxima comes out about half a magnitude brighter than its
            // visual 11.1 — and that is a known, bounded lie preferred to
            // carrying a bolometric correction per spectral class.
            const sw::f64 absolute =
                4.74 - 2.5 * std::log10(std::max<sw::f64>(visual->luminosity, 1.0e-12));
            const sw::f64 apparent = absolute + 5.0 * std::log10(parsecs / 10.0);
            const sw::f32 flux =
                static_cast<sw::f32>(std::pow(10.0, -0.4 * (apparent - 6.5)));
            if (flux < 0.02f)
            {
                continue; // fainter than the dome's own floor: nothing to draw
            }

            // The dome's two laws, unchanged. The size exponent is a fifth,
            // which is why a star a million times brighter is only sixteen
            // times wider — that is what an overexposed point does, and it is
            // also what stops Sirius from being a dinner plate.
            const sw::f32 angular = 0.00220f * std::pow(flux, 0.20f);
            const sw::f32 coverage = std::min(1.0f, 0.105f * std::pow(flux, 0.55f));
            // Radiance from coverage: the billboard's own alpha profile peaks
            // at 1, so the peak pixel is the graded colour and nothing else.
            // Four is the grade's white point, so a saturated star clips and a
            // sixth-magnitude one lands one grey level over the sky.
            const sw::f32 radiance = 4.0f * coverage;

            const sw::f32 radius = distance * angular;
            const sw::Vec3 z = -offset / distance;
            const sw::Vec3 reference =
                (std::abs(z.y) < 0.99f) ? sw::Vec3{0, 1, 0} : sw::Vec3{1, 0, 0};
            const sw::Vec3 x = glm::normalize(glm::cross(reference, z));
            const sw::Vec3 y = glm::cross(z, x);
            sw::DrawItem item{};
            item.mesh = &m_meshes[m_sunCoreMeshIndex];
            item.transform = sw::Mat4{sw::Vec4(x, 0.0f), sw::Vec4(y, 0.0f),
                                      sw::Vec4(z, 0.0f), sw::Vec4(offset, 1.0f)} *
                             glm::scale(sw::Mat4{1.0f}, sw::Vec3{radius});
            item.boundsCenter = offset;
            item.boundsRadius = radius;
            // 2.45: the soft-emissive branch, same as the sun's glare. Not the
            // plain emissive one — a billboard has to reach alpha exactly 1.0
            // at its rim and that is the wrong side of the emissive flag test,
            // which is how the star dome once came out as wireframe quads.
            item.tint = {visual->color.r * radiance, visual->color.g * radiance,
                         visual->color.b * radiance, 2.45f};
            item.transparent = true;
            // Behind everything else in the transparent pass: they really are.
            item.sortDistanceSquared = distance * distance * 1.05f;
            m_drawItems.push_back(item);
        }
    }

    // ------------------------------------------------------------------------
    // SYSTEM STREAMING
    // ------------------------------------------------------------------------
    void StarWorksGame::loadSystemPlanets(sw::u32 systemIndex)
    {
        if (systemIndex >= m_systemLoaded.size() || m_systemLoaded[systemIndex])
        {
            return;
        }
        m_systemLoaded[systemIndex] = true;
        const sw::space::SystemRecord& system = sw::space::systems()[systemIndex];
        sw::u32 built = 0;
        for (sw::u32 i = 0; i < sw::space::planets().size(); ++i)
        {
            const sw::space::PlanetRecord& planet = sw::space::planets()[i];
            const sw::space::StarRecord& host = sw::space::stars()[planet.starIndex];
            if (host.systemIndex != systemIndex)
            {
                continue;
            }
            const sw::ecs::Entity hostEntity = m_starEntities[planet.starIndex];
            const auto* hostTransform =
                m_world.tryGetComponent<TransformComponent>(hostEntity);
            if (hostTransform == nullptr)
            {
                continue;
            }
            const sw::phys::KeplerOrbit orbit = sw::phys::kepler::fromElements(
                host.mu, planet.sma, planet.eccentricity, planet.inclination,
                planet.ascendingNode, 0.0, planet.meanAnomaly, 0.0);

            const sw::ecs::Entity e = m_world.createEntity();
            TransformComponent transform{};
            sw::WorldVec3 relative{};
            sw::phys::kepler::evaluate(orbit, 0.0, relative);
            transform.position = hostTransform->position + relative;
            transform.uniformScale = static_cast<sw::f32>(planet.radius);
            m_world.addComponent(e, transform);
            m_world.addComponent(
                e, PreviousTransformComponent{transform.position, transform.rotation});
            m_world.addComponent(e, BoundsComponent{1.0f});

            const bool landable = !isGasStyle(planet.surfaceStyle);
            m_world.addComponent(
                e, makeSphereLodSet(exoplanetBaseColor(planet.surfaceStyle),
                                    planet.surfaceStyle,
                                    landable ? planet.radius : 0.0));
            // TIDALLY LOCKED, and for once that is not a simplification. Every
            // confirmed planet in this catalogue orbits inside a fifth of an
            // AU of a red dwarf bar four; the circularisation timescale there
            // is tens of millions of years against stellar ages of billions.
            // One turn per orbit is the honest default.
            const sw::f64 lockedRate =
                std::sqrt(host.mu / (planet.sma * planet.sma * planet.sma));
            m_world.addComponent(
                e, SpinComponent{{0.0f, 1.0f, 0.0f}, static_cast<sw::f32>(lockedRate)});

            sw::phys::GravitySourceComponent gravity{planet.mu, planet.radius};
            gravity.soiRadius = planet.soiRadius;
            gravity.angularVelocity = {0.0, lockedRate, 0.0};
            m_world.addComponent(e, gravity);
            if (planet.atmosphereTopAltitude > 0.0)
            {
                // Scale height a twelfth of the column, sea-level density from
                // Terra's scaled by surface gravity — a first-order guess and
                // no more, for worlds whose air nobody has measured.
                sw::phys::AtmosphereComponent air{};
                air.topAltitude = planet.atmosphereTopAltitude;
                air.scaleHeight = planet.atmosphereTopAltitude / 12.0;
                air.surfaceDensity =
                    1.225 * glm::clamp(planet.mu / (planet.radius * planet.radius) /
                                           9.80665, 0.3, 2.5);
                m_world.addComponent(e, air);
            }
            if (landable)
            {
                m_world.addComponent(e, sw::planet::terrainPreset(planet.surfaceStyle));
                m_world.addComponent(e, sw::planet::DepositComponent{});
            }
            m_world.addComponent(
                e, sw::space::makeCelestialBody(planet.name, hostEntity, &orbit));
            m_world.addComponent(
                e, MapMarkerComponent{exoplanetBaseColor(planet.surfaceStyle)});
            ++built;
        }
        if (built > 0)
        {
            SW_LOG_INFO("Game", "Entered {}: {} planet(s) built", system.name, built);
        }
    }

    void StarWorksGame::updateSystemStreaming()
    {
        if (m_starEntities.empty())
        {
            return;
        }
        const auto* transform =
            m_world.tryGetComponent<TransformComponent>(controlledEntity());
        if (transform == nullptr)
        {
            return;
        }
        const sw::WorldVec3 absolute = absolutePosition(transform->position);

        // THE ORIGIN FOLLOWS THE NEAREST ANCHOR, not the containing SOI, and
        // the difference is the whole cruise. Between two stars there is no
        // containing system for light-years at a stretch; holding the origin
        // at the star you left would let a craft reach 4.0e16 metres, where a
        // double's step is eight metres. Handing it over at the midpoint caps
        // the worst case at half the gap and costs one shift per crossing.
        const sw::u32 nearest = sw::space::nearestSystem(absolute);
        if (nearest != m_originSystem)
        {
            rebaseOrigin(nearest);
        }

        // ...and the PLANETS arrive when you are actually inside the star's
        // sphere of influence, which is the moment they start to matter.
        //
        // Nothing is ever unloaded. The mesh registry is append-only — a LOD
        // set is four GPU meshes and there is no path that frees one — so a
        // system that loaded, unloaded and loaded again would leak eight
        // meshes a visit. A visited system costs four entities in the per-tick
        // loops instead, which is the cheaper of the two by a wide margin, and
        // Sol could not be unloaded in any case: the colony is on it.
        const sw::i32 inside = sw::space::containingSystem(absolute);
        if (inside >= 0)
        {
            loadSystemPlanets(static_cast<sw::u32>(inside));
        }
    }

    // ------------------------------------------------------------------------
    // INTERSTELLAR GUIDANCE
    // ------------------------------------------------------------------------
    StarWorksGame::InterstellarGuidance StarWorksGame::interstellarGuidance() const
    {
        InterstellarGuidance guidance{};
        if (m_targetIndex < 0 ||
            static_cast<sw::usize>(m_targetIndex) >= m_celestialIndex.size())
        {
            return guidance;
        }
        sw::ecs::World& world = const_cast<sw::ecs::World&>(m_world);
        const auto& target =
            m_celestialIndex.body(static_cast<sw::usize>(m_targetIndex));
        const auto* targetStar = world.tryGetComponent<StarVisualComponent>(target.entity);
        if (targetStar == nullptr)
        {
            return guidance; // a planet is a destination for an orbit, not a heading
        }
        const sw::u32 system =
            sw::space::stars()[targetStar->catalogueIndex].systemIndex;
        if (system == m_originSystem)
        {
            return guidance; // already there: the map has better things to say
        }

        // ---- IS THIS ACTUALLY AN INTERSTELLAR TRAJECTORY? -------------------
        // The test is the two-body ENERGY against the local star and not the
        // shape of the drawn plan, because the drawn plan now stops at ten
        // billion kilometres and a bound orbit with an apoapsis beyond that
        // looks exactly like an escape on screen. Energy cannot be fooled that
        // way: it is positive if and only if the craft never comes back.
        const auto* sunTransform =
            world.tryGetComponent<TransformComponent>(m_lightStar);
        const auto* sunSource =
            world.tryGetComponent<sw::phys::GravitySourceComponent>(m_lightStar);
        const auto* craftTransform =
            world.tryGetComponent<TransformComponent>(controlledEntity());
        if (sunTransform == nullptr || sunSource == nullptr || craftTransform == nullptr)
        {
            return guidance;
        }
        sw::WorldVec3 sunVelocity{0.0};
        if (const sw::i32 sunIndex = m_celestialIndex.indexOf(m_lightStar); sunIndex >= 0)
        {
            sw::WorldVec3 ignored{};
            m_celestialIndex.stateAt(sunIndex, m_physicsLane->presentSeconds(), ignored,
                                     &sunVelocity);
        }
        const sw::WorldVec3 relative = craftTransform->position - sunTransform->position;
        const sw::WorldVec3 velocity = controlledVelocity() - sunVelocity;
        const sw::f64 radius = std::max(glm::length(relative), 1.0);
        const sw::f64 energy =
            0.5 * glm::dot(velocity, velocity) - sunSource->mu / radius;
        if (energy < 0.0)
        {
            return guidance; // still bound to this star
        }

        // ---- THE HEADING ERROR ----------------------------------------------
        const sw::WorldVec3 wanted = sw::space::systems()[system].position -
                                     absolutePosition(craftTransform->position);
        const sw::f64 range = glm::length(wanted);
        const sw::f64 speed = glm::length(velocity);
        if (!(range > 1.0) || !(speed > 1.0e-3))
        {
            return guidance;
        }
        const sw::WorldVec3 want = wanted / range;
        const sw::WorldVec3 have = velocity / speed;

        // AXIS-ANGLE, not three separate angles. Asking "how far off am I in
        // X" three times over gives three numbers that do not compose: correct
        // each in turn and you arrive somewhere else, because rotations do not
        // commute. The rotation that takes the current heading onto the wanted
        // one is a single axis and a single angle, and ITS components about the
        // three axes do compose — they are one turn written down three ways,
        // they vanish together, and each one's sign is the direction to turn.
        const sw::WorldVec3 axis = glm::cross(have, want);
        const sw::f64 sine = glm::length(axis);
        const sw::f64 cosine = glm::dot(have, want);
        const sw::f64 angle = std::atan2(sine, cosine);
        constexpr sw::f64 kToDegrees = 57.29577951308232;
        const sw::WorldVec3 rotation =
            (sine > 1.0e-12) ? (axis / sine) * angle * kToDegrees : sw::WorldVec3{0.0};

        guidance.valid = true;
        guidance.systemIndex = system;
        guidance.systemName = sw::space::systems()[system].name;
        guidance.distanceMeters = range;
        guidance.deviationDegrees = sw::Vec3(rotation);
        guidance.totalDegrees = static_cast<sw::f32>(angle * kToDegrees);
        guidance.closingSpeedMps = glm::dot(velocity, want);
        guidance.etaSeconds = (guidance.closingSpeedMps > 1.0)
                                  ? range / guidance.closingSpeedMps
                                  : -1.0;
        return guidance;
    }
} // namespace game
