/*************************************************************
 * pcontacts.cpp
 * -------------
 * Source file that will implement our particle contacts header file.
 *
 * Last Revision: October 20, 2014
 *
 * TO DO: - Debug
 *************************************************************/

#include "pcontacts.h"

using namespace marballs;

// Resolve - Calls functions to resolve contacts.
void ParticleContact::Resolve(marb duration) {
    ResolveVelocity(duration);
    ResolveInterpenetration(duration);
}

// CalculateSeparatingVelocity - Calculates velocity of particles post-separation.
marb ParticleContact::CalculateSeparatingVelocity() const {
    Vector3 relativeVelocity = particle[0]->GetVelocity();
    if (particle[1]) relativeVelocity -= particle[1]->GetVelocity();
    return relativeVelocity * contactNormal;
}

// ResolveVelocity - Further resolves velocity.
void ParticleContact::ResolveVelocity(marb duration) {
    // Find the velocity in the direction of the contact
    marb separatingVelocity = CalculateSeparatingVelocity();

    // Check if it even needs to be resolved
    if (separatingVelocity > 0) {
        return; // The contact is either separating, or stationary - there's no impulse required.
    }

    // Calculate the new separating velocity
    marb newSepVelocity = -separatingVelocity * restitution;

    // Check the velocity build-up due to acceleration only
    Vector3 accCausedVelocity = particle[0]->GetAcceleration();
    if (particle[1]) accCausedVelocity -= particle[1]->GetAcceleration();
    marb accCausedSepVelocity = accCausedVelocity * contactNormal * duration;

    // If we've got a closing velocity due to acceleration build-up,
    // remove it from the new separating velocity
    if (accCausedSepVelocity < 0) {
        newSepVelocity += restitution * accCausedSepVelocity;

        // Make sure we haven't removed more than was
        // there to remove.
        if (newSepVelocity < 0) newSepVelocity = 0;
    }

    marb deltaVelocity = newSepVelocity - separatingVelocity;

    // We apply the change in velocity to each object in proportion to
    // their inverse mass (i.e. those with lower inverse mass [higher
    // actual mass] Get less change in velocity)..
    marb totalInverseMass = particle[0]->GetInverseMass();
    if (particle[1]) totalInverseMass += particle[1]->GetInverseMass();

    // If all particles have infinite mass, then impulses have no effect
    if (totalInverseMass <= 0) return;

    // Calculate the impulse to apply
    marb impulse = deltaVelocity / totalInverseMass;

    // Find the amount of impulse per unit of inverse mass
    Vector3 impulsePerIMass = contactNormal * impulse;

    // Apply impulses: they are applied in the direction of the contact,
    // and are proportional to the inverse mass.
    particle[0]->SetVelocity(particle[0]->GetVelocity() + impulsePerIMass * particle[0]->GetInverseMass());

    if (particle[1]) {
        // Particle 1 goes in the opposite direction
        particle[1]->SetVelocity(particle[1]->GetVelocity() + impulsePerIMass * -particle[1]->GetInverseMass());
    }
}

// ResolveInterpenetration - Fixes overlapping particles.
void ParticleContact::ResolveInterpenetration(marb duration)
{
    // If we don't have any penetration, skip this step.
    if (penetration <= 0) return;

    // The movement of each object is based on their inverse mass, so total that.
    marb totalInverseMass = particle[0]->GetInverseMass();
    if (particle[1]) totalInverseMass += particle[1]->GetInverseMass();

    if (totalInverseMass <= 0) return; // If everything has infinite mass, do nothing.

    // Find the amount of penetration resolution per unit of inverse mass
    Vector3 movePerIMass = contactNormal * (penetration / totalInverseMass);

    // Calculate the the movement amounts
    particleMovement[0] = movePerIMass * particle[0]->GetInverseMass();
    if (particle[1]) {
        particleMovement[1] = movePerIMass * -particle[1]->GetInverseMass();
    } else {
        particleMovement[1].Clear();
    }

    // Apply the penetration resolution
    particle[0]->SetPosition(particle[0]->GetPosition() + particleMovement[0]);
    if (particle[1]) {
        particle[1]->SetPosition(particle[1]->GetPosition() + particleMovement[1]);
    }
}

// Constructor
ParticleContactResolver::ParticleContactResolver(unsigned iterations) : iterations(iterations) {}

// SetIterations - Sets the number of iterations to go through.
void ParticleContactResolver::SetIterations(unsigned iterations) {
    ParticleContactResolver::iterations = iterations;
}

// ResolveContacts - Resolves contacts.
void ParticleContactResolver::ResolveContacts(ParticleContact *contactArray, unsigned numContacts, marb duration) {
    unsigned i;

    iterationsUsed = 0;
    while(iterationsUsed < iterations) {
        // Find the contact with the largest closing velocity;
        marb max = MARB_MAX;
        unsigned maxIndex = numContacts;
        for (i = 0; i < numContacts; i++) {
            marb sepVel = contactArray[i].CalculateSeparatingVelocity();
            if (sepVel < max && (sepVel < 0 || contactArray[i].penetration > 0)) {
                max = sepVel;
                maxIndex = i;
            }
        }

        // Check if there's anything left worth resolving.
        if (maxIndex == numContacts) break;

        // Resolve this contact
        contactArray[maxIndex].Resolve(duration);

        // Update the interpenetrations for all particles
        Vector3 *move = contactArray[maxIndex].particleMovement;
        for (i = 0; i < numContacts; i++) {
            if (contactArray[i].particle[0] == contactArray[maxIndex].particle[0]) {
                contactArray[i].penetration -= move[0] * contactArray[i].contactNormal;

            } else if (contactArray[i].particle[0] == contactArray[maxIndex].particle[1]) {
                contactArray[i].penetration -= move[1] * contactArray[i].contactNormal;
            }

            if (contactArray[i].particle[1]) {
                if (contactArray[i].particle[1] == contactArray[maxIndex].particle[0]) {
                    contactArray[i].penetration += move[0] * contactArray[i].contactNormal;

                } else if (contactArray[i].particle[1] == contactArray[maxIndex].particle[1])  {
                    contactArray[i].penetration += move[1] * contactArray[i].contactNormal;
                }
            }
        }

        iterationsUsed++;
    }
}
