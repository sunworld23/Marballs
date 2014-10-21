/*************************************************************
 * plinks.cpp
 * -------------
 * Source file that will implement our particle link header file.
 *
 * Last Revision: October 20, 2014
 *
 * TO DO: - Debug
 *************************************************************/

#include "plinks.h"

using namespace marballs;

// PARTICLELINK CurrentLength - Returns length of link.
marb ParticleLink::CurrentLength() const {
    Vector3 relativePos = particle[0]->GetPosition() - particle[1]->GetPosition();
    return relativePos.Magnitude();
}

// PARTICLECABLE AddContact - Prevents a cable from overextending.
unsigned ParticleCable::AddContact(ParticleContact *contact, unsigned limit) const {
    // Find the length of the cable
    marb length = CurrentLength();

    // Check for overextension
    if(length < maxLength) return 0;

    // Otherwise return the contact
    contact->particle[0] = particle[0];
    contact->particle[1] = particle[1];

    // Calculate the normal
    Vector3 normal = particle[1]->GetPosition() - particle[0]->GetPosition();
    normal.Normalize();
    contact->contactNormal = normal;

    contact->penetration = length-maxLength;
    contact->restitution = restitution;

    return 1;
}

// PARTICLEROD AddContact - Prevents rod from extending or compressing.
unsigned ParticleRod::AddContact(ParticleContact *contact, unsigned limit) const {

    // Find the length of the rod
    marb currentLen = CurrentLength();

    // Check for overextension
    if(currentLen == length) return 0;

    // Otherwise return the contact
    contact->particle[0] = particle[0];
    contact->particle[1] = particle[1];

    // Calculate the normal
    Vector3 normal = particle[1]->GetPosition() - particle[0]->GetPosition();
    normal.Normalize();

    // The contact normal depends on whether we're extending or compressing
    if (currentLen > length) {
        contact->contactNormal = normal;
        contact->penetration = currentLen - length;
    } else {
        contact->contactNormal = normal * -1;
        contact->penetration = length - currentLen;
    }

    // Always use zero restitution (no bounciness)
    contact->restitution = 0;

    return 1;
}

// PARTICLECONSTRAINT CurrentLength - Returns length of constraint.
marb ParticleConstraint::CurrentLength() const
{
    Vector3 relativePos = particle->GetPosition() - anchor;
    return relativePos.Magnitude();
}

// PARTICLECABLECONSTRAINT AddContact - Adds a contact to resolve when particle strays too far.
unsigned ParticleCableConstraint::AddContact(ParticleContact *contact, unsigned limit) const {

    // Find the length of the cable
    marb length = CurrentLength();

    // Check if we're over-extended
    if (length < maxLength) { return 0; }

    // Otherwise return the contact
    contact->particle[0] = particle;
    contact->particle[1] = 0;

    // Calculate the normal
    Vector3 normal = anchor - particle->GetPosition();
    normal.Normalize();
    contact->contactNormal = normal;

    contact->penetration = length-maxLength;
    contact->restitution = restitution;

    return 1;
}

// PARTICLERODCONSTRAINT AddContact - Adds a contact to resolve when particle moves too far or close.
unsigned ParticleRodConstraint::AddContact(ParticleContact *contact, unsigned limit) const {

    // Find the length of the rod
    marb currentLen = CurrentLength();

    // Check if we're over-extended
    if (currentLen == length) { return 0; }

    // Otherwise return the contact
    contact->particle[0] = particle;
    contact->particle[1] = 0;

    // Calculate the normal
    Vector3 normal = anchor - particle->GetPosition();
    normal.Normalize();

    // The contact normal depends on whether we're extending or compressing
    if (currentLen > length) {
        contact->contactNormal = normal;
        contact->penetration = currentLen - length;
    } else {
        contact->contactNormal = normal * -1;
        contact->penetration = length - currentLen;
    }

    // Always use zero restitution (no bounciness)
    contact->restitution = 0;

    return 1;
}
