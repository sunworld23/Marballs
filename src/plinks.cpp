/*************************************************************
 * plinks.cpp
 * -------------
 * Source file that will implement our particle link header file.
 *
 * Last Revision: October 12, 2014
 *
 * TO DO: - Debug
 *************************************************************/

#include <pcontacts.h>

using namespace marballs;

real ParticleLink::currentLength() const
{
    Vector3 relativePos = particle[0]->GetPosition() -
                            particle[1]->GetPosition();
    return relativePos.Magnitude();
}

unsigned ParticleCable::fillContact(ParticleContact *contact, unsigned limit) const
{
    //Find length of cable
    marb length = currentLength();

    // Check for overextension
    if(length < maxLength) return 0;

    // Otherwise return contact
    contact->particle[0] = particle[0];
    contact->particle[1] = particle[1];

    // Calculate normal
    Vector3 normal = particle[1]->GetPosition() - particle[0]->GetPosition();
    normal.Normalize();

    contact->contactNormal = normal;

    contact->penetration = length - maxLength;
    contact->restitution = restitution;

    return 1;

}

unsigned ParticleRod::fillContact(ParticleContact *contact, unsigned limit) const
{
    // Length of rod
    marb currentLen = currentLength();

    // Check for overextension
    if(currentLen == length) return 0;

    // Otherwise return contact
    contact->particle[0] = particle[0];
    contact->particle[1] = particle[1];

    // Calculate normal
    Vector3 normal = particle[1]->GetPosition() - particle[0]->GetPosition();
    normal.Normalize();

    // Contact normal depends on extension or compression
    if(currentLen > length)
    {
        contact->contactNormal = normal;
        contact->penetration = currentLen - length;
    } else
    {
        contact->contactNormal = normal * -1;
        contact->penetration = length - currentLen;
    }

    // Always use zero restitution (no bounciness)
    contact->restitution = 0;

    return 1;
}


/*****************************
* GETTER AND SETTER FUNCTIONS
******************************/
// NOTE: Supposedly these setters should only be used as a last resort. Use other, less direct methods first.



