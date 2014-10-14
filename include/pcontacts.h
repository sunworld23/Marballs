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

            marb penetration;       // Depth of the penetration between objects

        /*********************************
        *     Function Declarations
        *********************************/
        protected:

            // resolve - Resolves this contact, for both velocity and interpenetration.
            void resolve(marb duration);

            // calculateSeparatingVelocity - Calculates the separating velocity at this contact
            marb calculateSeparatingVelocity() const;


        private:
            // resolveVeloctity - Handles the impulse caculations for this collision
            void resolveVelocity(marbs duration);

            // resolveInterpenetration - Handles interpenetration resolution
            void resolveInterpenetration(marb duration);

            /**********************************************************************************
            * END OF SETTER AND GETTER FUNCTION (AVOID USING SETTERS IF POSSIBLE)
            ***********************************************************************************/

    }; // ParticleContact class end

    class ParticleContactResolver
    {
        /*********************************
        *     Variable Declarations
        *********************************/
        protected:
            unsigned iterations; // Number of iterations allowed

            unsigned iterationsUsed; // Performance tracking, actual number of iterations used

        /*********************************
        *     Function Declarations
        *********************************/

        public:
            // Constructor - Creates a new contact resolver
            ParticleContactResolver(unsigned iterations);

            // SetIterations - sets the number of iterations allowed
            void SetIterations(unsigned iterations);

            // resolveContacts - Resolves particle contact for both penetration and velocity
            void resolveContacts(ParticleContact *contactArray, unsigned numContacts, marb duration);

    }; // ParticleContactResolver class end

    class ParticleContactGenerator
    {
        public:
            // addContact - fills the contact structure with the generated contact.
            // The contact pointer should always point to the first available contact.
            // Limit is the maximum number of contacts that can be written to.
            // Returns the number of contacts that were written
            virtual unsigned addContact(ParticleContact *contact, unsigned limit) const = 0;

    }; // ParticleContactGenerator class end
} // marballs namespace end

 #endif // PCONTACTS_INCLUDED
