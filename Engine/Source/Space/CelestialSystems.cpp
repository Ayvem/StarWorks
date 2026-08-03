#include "Space/CelestialSystems.hpp"

#include "ECS/World.hpp"
#include "Simulation/Simulation.hpp"

namespace sw::space
{
    void CelestialMotionSystem::update(ecs::World& world, f32 /*deltaSeconds*/)
    {
        m_index.rebuild(world);
        // AT FULL CLOCK PRECISION. A double holding 3e11 seconds has a step
        // of 61 microseconds, which at Terra's 29.8 km/s is 1.8 m of position
        // NOISE per tick — not drift, noise, different every tick. That is
        // what the ground vibrating after a trip to another star actually is.
        f64 whole = 0.0;
        f64 fraction = 0.0;
        m_timebase.presentSecondsSplit(whole, fraction);

        world.forEach<TransformComponent, PreviousTransformComponent,
                      phys::GravitySourceComponent, CelestialBodyComponent>(
            [this, whole, fraction](ecs::Entity entity, TransformComponent& transform,
                         PreviousTransformComponent& previous,
                         phys::GravitySourceComponent& source, CelestialBodyComponent&) {
                const i32 index = m_index.indexOf(entity);
                if (index < 0)
                {
                    return;
                }

                WorldVec3 position{};
                WorldVec3 velocity{};
                m_index.stateAtSplit(index, whole, fraction, position, &velocity);

                // Real interpolation: previous = last tick's position. The
                // rotation snapshot belongs to the spin system.
                previous.position = transform.position;
                transform.position = position;
                source.worldVelocity = velocity;
            });
    }
} // namespace sw::space
