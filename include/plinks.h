/*************************************************************
 * plinks.h
 * -------------
 * Header file that connects two particles together
 *
 * Last Revision: October 20, 2014
 *
 * TO DO: - Debug
 *************************************************************/

/********************************************************
 * A link connects two particles together, generating a contact
 * if they violate the constraints of their link
 * Base class for cables and rods
*********************************************************/
#ifndef PLINKS_INCLUDED
#define PLINKS_INCLUDED

#include "pcontacts.h"

namespace marballs {

    class ParticleLink : public ParticleContactGenerator {
        /*********************************
        *     Variable Declarations
        *********************************/
        public:
            // Holds the pair of particles that are connected by link
            Particle* particle[2];

        /*********************************
        *     Function Declarations
        *********************************/
            // AddContact - Fills given contact structure with the contact
            // needed to keep link from violating constraint
            virtual unsigned AddContact(ParticleContact *contact, unsigned limit) const = 0;

        protected:
            // CurrentLength - Return length of cable
            marb CurrentLength() const;


        private:
            // ResolveVelocity - Handles the impulse caculations for this collision
            void ResolveVelocity(marb duration);

            // ResolveInterpenetration - Handles interpenetration resolution
            void ResolveInterpenetration(marb duration);

            /**********************************************************************************
            * END OF SETTER AND GETTER FUNCTIONS
            ***********************************************************************************/

    }; // Particle Contact class end

    class ParticleCable : public ParticleLink {
        public:
            marb maxLength; // Holds max length cable can stretch
            marb restitution; // Holds Restitution or bounciness of cable

            // AddContact - Prevents cable from overextending via contacts.
            virtual unsigned AddContact(ParticleContact *contact, unsigned limit) const;

    };

    // Rods link a pair of particles
    class ParticleRod : public ParticleLink {
        public:
            marb length; // Length of rod.
        public:
            // AddContact - Prevents rod from extending or compressing via contacts.
            virtual unsigned AddContact(ParticleContact *contact, unsigned limit) const;

    };

    // CLASS ParticleConstraint - Like links, but connects particle to anchor.
    class ParticleConstraint : public ParticleContactGenerator {
    public:
        Particle* particle; // Holds particles connected by this constraint.
        Vector3 anchor; // The point to which the particle is anchored.

    protected:
        marb CurrentLength() const; // Returns current length of link.

    public:
        /** AUTHOR'S NOTE
        * Generates the contacts to keep this link from being
        * violated. This class can only ever generate a single
        * contact, so the pointer can be a pointer to a single
        * element, the limit parameter is assumed to be at least one
        * (zero isn't valid) and the return value is either 0, if the
        * cable wasn't over-extended, or one if a contact was needed.
        *
        * NB: This method is declared in the same way (as pure
        * virtual) in the parent class, but is replicated here for
        * documentation purposes.
        */
        virtual unsigned AddContact(ParticleContact *contact, unsigned limit) const = 0;
    };

    // CLASS ParticleCableConstraint - Links particles to anchor, generating contact if too far apart.
    class ParticleCableConstraint : public ParticleConstraint {
    public:
        marb maxLength; // Maximum length of the cable.
        marb restitution; // Holds bounciness of the cable.

    public:
        // AddContact - Prevents cable from overextending.
        virtual unsigned AddContact(ParticleContact *contact, unsigned limit) const;
    };

    // CLASS ParticleRodConstraint - Like a cable, but prevents particle from getting too close, too.
    class ParticleRodConstraint : public ParticleConstraint {
    public:
        marb length; // Length of the rod.

    public:
        // AddContact - Prevents rod from extending or compressing.
        virtual unsigned AddContact(ParticleContact *contact, unsigned limit) const;
    };

} // End of marballs namespace.

 #endif // PLINKS_INCLUDED
