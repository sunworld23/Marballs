/*************************************************************
 * pworld.cpp
 * ---------------
 * Source file to implement pworld.h
 *
 * Last Revision: Oct. 20 2014
 *
 * TO DO: - Continue following tutorial to fill this out.
 *************************************************************/

#include <cstddef>
#include "pworld.h"

using namespace marballs;

// Explicit Constructor
ParticleWorld::ParticleWorld(unsigned maxContacts, unsigned iterations) : resolver(iterations), maxContacts(maxContacts) {
    contacts = new ParticleContact[maxContacts];
    calculateIterations = (iterations == 0);

}

// Destructor
ParticleWorld::~ParticleWorld() {
    delete[] contacts;
}

// StartFrame - Clears force accumulators for particles so new forces can be added.
void ParticleWorld::StartFrame() {
    for (Particles::iterator p = particles.begin(); p != particles.end(); p++) {
        // Remove all forces from the accumulator
        (*p)->ClearAccumulator();
    }
}

// GenerateContacts - Generates contacts everywhere there is a collision.
unsigned ParticleWorld::GenerateContacts() {
    unsigned limit = maxContacts;
    ParticleContact *nextContact = contacts;

    for (ContactGenerators::iterator g = contactGenerators.begin(); g != contactGenerators.end(); g++) {
        unsigned used =(*g)->AddContact(nextContact, limit);
        limit -= used;
        nextContact += used;

        // Running out of contacts means there are missing contacts.
        if (limit <= 0) break;
    }

    // Return the number of contacts used.
    return maxContacts - limit;
}

// Integrate - Integrates particles for the specified duration.
void ParticleWorld::Integrate(marb duration) {
    for (Particles::iterator p = particles.begin(); p != particles.end(); p++) {
        // Remove all forces from the accumulator
        (*p)->Integrate(duration);
    }
}

// RunPhysics - Runs contact physics for the specified duration.
void ParticleWorld::RunPhysics(marb duration) {
    registry.UpdateForces(duration); // First apply the force generators
    Integrate(duration); // Then integrate the objects
    unsigned usedContacts = GenerateContacts(); // Generate contacts

    // And process them
    if (usedContacts) {
        if (calculateIterations) resolver.SetIterations(usedContacts * 2);
        resolver.ResolveContacts(contacts, usedContacts, duration);
    }
}

// GETTER FUNCTIONS
ParticleWorld::Particles& ParticleWorld::GetParticles() { return particles; }
ParticleWorld::ContactGenerators& ParticleWorld::GetContactGenerators() { return contactGenerators; }
ParticleForceRegistry& ParticleWorld::GetForceRegistry() { return registry; }

// Init - Initializes list of ground contacts.
void GroundContacts::Init(marballs::ParticleWorld::Particles *particles) {
    GroundContacts::particles = particles;
}

// AddContact - Adds an additional contact.
unsigned GroundContacts::AddContact(marballs::ParticleContact *contact, unsigned limit) const {
    unsigned count = 0;
    for (marballs::ParticleWorld::Particles::iterator p = particles->begin(); p != particles->end(); p++) {
        marballs::marb y = (*p)->GetPosition().y;

        if (y < 0.0f) {
            contact->contactNormal = marballs::Vector3::UP;
            contact->particle[0] = *p;
            contact->particle[1] = NULL;
            contact->penetration = -y;
            contact->restitution = 0.2f;
            contact++;
            count++;
        }

        if (count >= limit) return count;
    }
    return count;
}
