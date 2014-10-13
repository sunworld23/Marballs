/*************************************************************
 * plinks.h
 * -------------
 * Header file that connects two particles together
 *
 * Last Revision: October 12, 2014
 *
 * TO DO: - Debug
 *************************************************************/

/********************************************************
 * A link connects two particles together, generating a contact
 * if they violate the constraints of their link
 * Base class for cables and rods
*********************************************************/
#ifndef PLINK_INCLUDED
#define PLINK_INCLUDED

#include "marballs.h"

namespace marballs
{
    class ParticleLink
    {
        /*********************************
        *     Variable Declarations
        *********************************/
        public:
            // Holds the pair of particles that are connected by link
            Particle* particle[2];

        /*********************************
        *     Function Declarations
        *********************************/
            // Fills given contact structure with the contact needed to keep link
            // from violating constraint
            virtual unsigned fillContact(ParticleContact *contact, unsigned limit) const = 0;

        protected:
            // Return length of cable
            marb currentLength() const;


        private:
            void resolveVelocity(marbs duration);   // Handles the impulse caculations for this collision

            void resolveInterpenetration(marb duration); // Handles interpenetration resolution

            /**********************************************************************************
            * END OF SETTER AND GETTER FUNCTION (AVOID USING SETTERS IF POSSIBLE)
            ***********************************************************************************/

    }; // Particle Contact class end

    class ParticleCable: public ParticleLink
    {
        public:
            marb maxLength;
            marb restitution;

            virtual unsigned fillContact(ParticleContact *contact, unsigned limit) const;

    };

    // Rods link a pair of particles
    class ParticleRod: public ParticleLink
    {
        public:
            // Length of rod
            marb length;

            marb currentLength() const;

            // Fills given contact structure with the contact needed to keep rod
            // from extending or compressing
            virtual unsigned fillContact(ParticleContact *contact, unsinged limit) const;


    };


} // marballs namespace end

 #endif // PLINK_INCLUDED
