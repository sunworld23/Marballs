/*************************************************************
 * pcontacts.cpp
 * -------------
 * Source file that will implement our particle contacts header file.
 *
 * Last Revision: October 15, 2014
 *
 * TO DO: - Debug
 *************************************************************/

#include "pcontacts.h"

using namespace marballs;

// Resolve - Resolves contacts.
void ParticleContact::Resolve(marb duration) {
    ResolveVelocity(duration);
    ResolveInterpenetration(duration);
}

marb ParticleContact::CalculateSeparatingVelocity() const {
    Vector3 relativeVelocity = particle[0]->GetVelocity();

    if(particle[1])
        relativeVelocity -= particle[1]->GetVelocity();

    return relativeVelocity.DotProduct(contactNormal); // POSSIBLE WRONG SUBSTITUTION
}

void ParticleContact::ResolveVelocity(marb duration) {
    //Find velocity in the direction of contact
    marb separatingVelocity = CalculateSeparatingVelocity();

    // Check if it needs to be resolved
    if (separatingVelocity > 0)
    {
        // The contact is separating or stationary. No impulse
        return;
    }

    // Calculate the new separating velocity
    marb newSepVelocity = -separatingVelocity * restitution;

    // Check the velocity build-up due to acceleration
    Vector3 accCausedVelocity = particle[0]->GetAcceleration();

    if(particle[1]) accCausedVelocity -= particle[0]->GetAcceleration();

    marb accCausedSepVelocity = accCausedVelocity.DotProduct(contactNormal) * duration; // POSSIBLE WRONG SUBSTITUTION

    if(accCausedSepVelocity < 0)
    {
        newSepVelocity += restitution * accCausedSepVelocity;

        // Make sure we don't remove more than we need
        if(newSepVelocity < 0) newSepVelocity = 0;
    }
    marb deltaVelocity = newSepVelocity - separatingVelocity;

    // Apply the change in velocity to each object in proportion to its inverse mass
    // Lower inverse mass gets less change in velocity
    marb totalInverseMass = particle[0]->GetInverseMass();

    if(particle[1]) totalInverseMass += particle[1]->GetInverseMass();

    // If all particles have infinite mass, then impulse has no effect
    if(totalInverseMass <= 0) return;

    // Calculate the impulse to apply
    marb impulse = deltaVelocity / totalInverseMass;

    // Find the amount of impulse per unit of inverse mass
    Vector3 impulsePerIMass = contactNormal * impulse;

    // Apply impulse in direction of the contact, proportional to the inverse mass
    particle[0]->SetVelocity(particle[0]->GetVelocity() +
                             impulsePerIMass * particle[0]->GetInverseMass());

    if(particle[1])
    {
        // Particle 1 goes in the opposite direction
        particle[1]->SetVelocity(particle[1]->GetVelocity() +
                                 impulsePerIMass * -particle[1]->GetInverseMass());
    }
}

void ParticleContact::ResolveInterpenetration(marb duration) {
    // No penetration, skip
    if(penetration <= 0) return;

    // The movement is based on inverse mass
    marb totalInverseMass = particle[0]->GetInverseMass();
    if(particle[1]) totalInverseMass += particle[1]->GetInverseMass();

    // If particle has infinite mass, do nothing
    if(totalInverseMass <= 0) return;

    // Find amount of penetration resolution per unit of inverse mass
    Vector3 movePerIMass = contactNormal *
                            (-penetration / totalInverseMass);

    // Apply penetration resolution
    particle[0]->SetPosition(particle[0]->GetPosition() +
                             movePerIMass * particle[0]->GetInverseMass());
    if(particle[1])
        particle[1]->SetPosition(particle[1]->GetPosition() +
                                 movePerIMass * particle[1]->GetInverseMass());
}

// Constructor
ParticleContactResolver::ParticleContactResolver(unsigned iterations)
:
iterations(iterations) {}

// SetIterations - Sets the number of iterations.
void ParticleContactResolver::SetIterations(unsigned iterations) {
    ParticleContactResolver::iterations = iterations;
}

// ResolveContacts - Resolves contacts.
void ParticleContactResolver::ResolveContacts(ParticleContact *contactArray, unsigned numContacts, marb duration) {
    iterationsUsed = 0;
    while(iterationsUsed < iterations)
    {
        // Find contact with largest closing velocity
        marb max = 0;
        unsigned maxIndex = numContacts;
        for(unsigned i = 0; i < numContacts; i++)
        {
            marb sepVel = contactArray[i].CalculateSeparatingVelocity();
            if(sepVel < max)
            {
                  max = sepVel;
                  maxIndex = i;
            }

            // Resolve contact
            contactArray[maxIndex].Resolve(duration);
            iterationsUsed++;
        }
    }
}


