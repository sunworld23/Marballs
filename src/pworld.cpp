/*************************************************************
 * pworld.cpp
 * ---------------
 * Source file to implement pworld.h
 *
 * Last Revision: Oct. 13 2014
 *
 * TO DO: - Continue following tutorial to fill this out.
 *************************************************************/

#include "pworld.h"

using namespace marballs;

void ParticleWorld::startFrame()
{
    ParticleRegistration *reg = firstParticle;

    while (reg)
    {
        // Removes forces from force accumulator
        reg->particle->ClearAccumulator();

        // Get next particle in list
        reg = reg->next;
    }
}

unsigned ParticleWorld::generateContacts()
{
    unsigned limit = maxContacts;
    ParticleContact *nextContact = contacts;

    ContactGenRegistration *reg = firstContactGen;
    while (reg)
    {
        unsigned used = reg->gen->addContact(nextContact, limit);
        limit -= used;
        nextContact += used;

        //Run out of contacts meaning we are missing contacts
        if (limit <= 0)
            break;

        reg = reg->next;
    }

    // Return the number of contacts used
    return maxContacts - limit;
}

void ParticleWorld::integrate(marb duration)
{
    ParticleRegistration *reg = firstParticle;
    while (reg)
    {
        // Remove all forces from the accumulator
        reg->particle->intgrate(duration);

        // Get the next registration
        reg = reg->next;
    }
}

void ParticleWorld::runPhysics(marb duration)
{
    // Apply the force generators
    registry.UpdateForces(duration);

    // Integrate objects
    integrate(duration);

    // Generate Contacts
    unsigned usedContacts = generateContacts();

    // Process Contacts
    if (calculateIterations) //not sure where this variable comes from yet
        resolver.SetIterations(usedContacts * 2);
    resolver.resolveContacts(contacts, usedContacts, duration);

}
