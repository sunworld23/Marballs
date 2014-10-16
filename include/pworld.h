/*************************************************************
 * pworld.h
 * ---------------
 * Keeps track of all particles and updates them all,
 * a world for the particles to be kept track of. Particles
 * are all held within a linked list.
 *
 * Last Revision: Oct. 15 2014
 *
 * TO DO: - Continue following tutorial to fill this out.
 *************************************************************/
#ifndef PWORLD_INCLUDED
#define PWORLD_INCLUDED

#include "pfgen.h"
#include "plinks.h"

namespace marballs {
    class ParticleWorld {
        /*********************************
        *     Variable Declarations
        *********************************/

    public:
        typedef std::vector<Particle*> Particles;
        typedef std::vector<ParticleContactGenerator*> ContactGenerators;

    protected:
        Particles particles; // Holds the particles.
        bool calculateIterations; // Determines if iterations need to be counted, then multiply by 2 * usedContacts

        struct ParticleRegistration { // Holds a linked list of all particles created.;
            Particle *particle; // The values held by the particle at this position.
            ParticleRegistration *next; // Pointer to next particle in out list.
        };

        ParticleRegistration *firstParticle; // The first particle used in the linked list of particles.
        ParticleForceRegistry registry; // Holds the force generators for whole world
        ParticleContactResolver resolver; // Holds the resolver for contacts
        ContactGenerators contactGenerators; // Holds contact generators.

        struct ContactGenRegistration // Holds one registered contact generator
        {
            ParticleContactGenerator *gen;
            ContactGenRegistration *next;
        };

        ContactGenRegistration *firstContactGen; // Holds list of contact generators
        ParticleContact *contacts; // Holds list of contacts
        unsigned maxContacts; // Holds max number of contacts allowed

        /*********************************
        *     Function Declarations
        *********************************/

        public:

            // ParticleWorld - Constructor to create new particle simulator.
            // The simulator is used to handle a given number of contacts per frame.
            // Optionally the world can be given a number of contact-resolution iterations.
            // If iteration number not given for contact-resolution, double the number of contacts.
            // NOTE: typed unsigned is assumed to be int within C++
            ParticleWorld(unsigned maxContacts, unsigned iterations=0);

            // Deconstructor
            ~ParticleWorld();

            // StartFrame - Clears force accumulators for particles in world then new forces can be added.
            void StartFrame();

            // GenerateContacts - Calls each of the registed contact generators
            // to return the number of generated contacts.
            unsigned GenerateContacts();

            // Integrate - Moves all particles forward by a given duration
            void Integrate(marb duration);

            // RunPhysics - Processes all physics in the particle world
            void RunPhysics(marb duration);

            // GetParticles - Returns list of particles.
            Particles& GetParticles();

            // GetContactGenerators - Returns list of contact generators.
            ContactGenerators& GetContactGenerators();

            // GetForceRegistry - Returns the force registry.
            ParticleForceRegistry& GetForceRegistry();

    }; // end of ParticleWorld class

    // CLASS GroundContacts - Contact generator that collides a vector of particles against the ground.
    class GroundContacts : public marballs::ParticleContactGenerator
    {
        marballs::ParticleWorld::Particles *particles;

    public:
        void Init(marballs::ParticleWorld::Particles *particles);

        virtual unsigned AddContact(marballs::ParticleContact *contact, unsigned limit) const;
    };
}// end of marballs namespace

#endif // PWORLD_INCLUDED
