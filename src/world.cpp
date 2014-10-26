#include "world.h"

using namespace marballs;

void World::startFrame()
{
    BodyRegistration *reg = firstBody;
    while (reg)
    {
        // Remove all forces from the accumulator.
        reg->body->clearAccumulators();
        reg->body->calculateDerivedData();
        // Get the next registration.
        reg = reg->next;
    }
}

void World::runPhysics(real duration)
{
    // First apply the force generators
    registry.updateForces(duration);

    // Then integrate the objects
    BodyRegistration *reg = firstBody;
    while (reg)
    {
        // Remove all forces from the accumulator
        reg->body->integrate(duration);
        // Get the next registration
        reg = reg->next;
    }
}
