/*************************************************************
 * pcontacts.h
 * -------------
 * Header file that defines collision between two particles.
 *
 * Last Revision: October 12, 2014
 *
 * TO DO: - Debug
 *************************************************************/

/********************************************************
 * Contact represents two objects in contact.
 * In this case, particle contact represents two particles.
 * Resolving a contact removes interpenetration and applies sufficient
 * impulse to keep them apart.
*********************************************************/
#ifndef PCONTACTS_INCLUDED
#define PCONTACTS_INCLUDED

#include "marballs.h"

namespace marballs
{
    class ParticleContact
    {
        /*********************************
        *     Variable Declarations
        *********************************/
        public:
            Particle* particle[2];  // Holds the particles that are involved in the contact
                                    // Contact with scenery is makes the second particle NULL

            marb restitution;       // Holds the normal restitution coefficient at the contact

            Vector3 contactNormal;  // Holds the direction of the contact in world coordinates

            marb penetration;

            ParticleContactResolver(unsigned iterations);

            void SetIterations(unsigned iterations);

            void resolveContacts(ParticleContact *contactArray, unsigned numContacts, marb duration);

        /*********************************
        *     Function Declarations
        *********************************/
        protected:
            unsigned iterations;

            unsigned iterationsUsed;

            void resolve(marb duration); // Resolves this contact, for both velocity and interpenetration

            marb carculateSeparatingVelocity() const;   // Calculates the separating velocity at this contact


        private:
            void resolveVelocity(marbs duration);   // Handles the impulse caculations for this collision

            void resolveInterpenetration(marb duration); // Handles interpenetration resolution

            /**********************************************************************************
            * END OF SETTER AND GETTER FUNCTION (AVOID USING SETTERS IF POSSIBLE)
            ***********************************************************************************/

    }; // Particle Contact class end
} // marballs namespace end

 #endif // PCONTACTS_INCLUDED
