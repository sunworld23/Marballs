/*************************************************************
 * pcontacts.cpp
 * -------------
 * Source file that will implement our particle contacts header file.
 *
 * Last Revision: October 12, 2014
 *
 * TO DO: - Debug
 *************************************************************/

#include <pcontacts.h>

using namespace marballs;

void ParticleContact::resolve(marb duration)
{
    resolveVelocity(duration);
}

real ParticleContact::calculateSeparatingVelocity() const
{
    Vector3 relativeVelocity = particle[0]->GetVelocity();
    if(particle[1]) relativeVelocity -= particle[1]->GetVelocity();
    return relativeVelocity * contactNormal;
}

void ParticleContact::resolveVelocity(marb duration)
{
    //Find velocity in the direction of contact
    marb separatingVelocity = calculateSeparatingVelocity();

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

    marb accCausedSepVelocity = accCausedVelocity *
                                contactNormal * duration;

    if(accCausedSepVelocity < 0)
    {
        newSepVelocity += restitution * accCausedSepVelocity;

        // Make sure we don't remove more than we need
        if(newSepVelocity < 0) newSepVelocity = 0;
    }
    marb deltaVelocity = newSepVelocity - separatingVelocity;

    // Apply the change in velocity to each object in proportion to its inverse mass
    // Lower inverse mass gets less change in velocity
    real totalInverseMass = particle[0]->GetInverseMass();

    if(particle[1]) totalInverseMass += particle[1]->GetInverseMass();

    // If all particles have infinite mass, then impulse has no effect
    if(totalInverseMass <= 0) return;

    // Calculate the impulse to apply
    marb impulse = deltaVelocity / totalInverseMass;

    // Find the amount of impulse per unit of inverse mass
    Vector3 impulsePerIMass = contactNormal * impulse;

    // Apply impulse in direction of the contact, proportional to the inverse mass
    particle[0]->setVelocity(particle[0]->GetVelocity() +
                             impulsePerIMass * particle[0]->GetInverseMass());

    if(particle[1])
    {
        // Particle 1 goes in the opposite direction
        particle[1]->setVelocity(particle[1]->GetVelocity() +
                                 impulsePerIMass * -particle[1]->GetInverseMass());
    }


}

void ParticleContact::resolve(marb duration)
{
    resolveVelocity(duration);
    resolveInterpenetration(duration);
}

void ParticleContact::resolveInterpenetration(marb duration)
{
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

void ParticleContact::resolveContacts(ParticleContact *contactArray,
                                      unsigned numContacts,
                                      marb duration)
{
  iterationsUsed = 0;
  while(iterationsUsed < iterations)
  {
      // Find contact with largest closing velocity
      marb max = 0;
      unsigned maxIndex = numContacts;
      for(unsigned i = 0; i < numContacts; i++)
      {
          marb sepVel = contactArray[i].calculateSeparatingVelocity();
          if(sepVel < max)
          {
              max = sepVel;
              maxIndex = i;
          }

          // Resolve contact
          contactArray[maxIndex].resolve(duration);
          iterationsUsed++;
      }
  }
}

/*****************************
* GETTER AND SETTER FUNCTIONS
******************************/
// NOTE: Supposedly these setters should only be used as a last resort. Use other, less direct methods first.



