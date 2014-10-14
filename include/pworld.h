/*************************************************************
 * pworld.h
 * ---------------
 * Keeps track of all particles and updates them all,
 * a world for the particles to be kept track of. Particles
 * are all held within a linked list.
 *
 * Last Revision: Oct. 13 2014
 *
 * TO DO: - Continue following tutorial to fill this out.
 *************************************************************/
#ifndef PARTICLE_WORLD_INCLUDED
#define PARTICLE_WORLD_INCLUDED

#include "marballs.h"

namespace marballs
{
    class ParticleWorld
    {
        /*********************************
        *     Variable Declarations
        *********************************/

        struct ParticleRegistration // Holds a linked list of all particles created.
        {
            Particle *particle; // The values held by the particle at this position.
            ParticleRegistration *next; // Pointer to next particle in out list.
        };

        ParticleRegistration *firstParticle; // The first particle used in the linked list of particles.

        ParticleForceRegistry registry; // Holds the force generators for whole world

        ParticleContactResolver resolver; // Holds the resolver for contacts

        struct ContactGenRegistration // Holds one registered contact generator
        {
            ParticleContactGenerator *gen;
            ContactGenRegistration *next;
        };

        ContactGenRegistration *firstContactGen; // Holds list of contact generators

        ParticleContact *contacts; // Holds list of contacts

        unsigned maxContacts; // Holds max number of contacts allowed

        public:
        /*********************************
        *     Function Declarations
        *********************************/

            // ParticleWorld - Constructor to create new particle simulator.
            // The simulator is used to handle a given number of contacts per frame.
            // Optionally the world can be given a number of contact-resolution iterations.
            // If iteration number not given for contact-resolution, double the number of contacts.
            // NOTE: typed unsigned is assumed to be int within C++
            ParticleWorld(unsigned maxContacts, unsigned iterations = 0);

            // startFrame - Clears force accumulators for particles in world then new forces can be added.
            void startFrame();

            // generateContacts - Calls each of the registed contact generators
            // to return the number of generated contacts.
            unsigned generateContacts();

            // integrate - moves all particles forward by a given duration
            void integrate(marb duration);

            // runPhysics - Processes all physics in the particle world
            void runPhysics(marb duration);

    }; // end of ParticleWorld class
}// end of marballs namespace

#endif // PARTICLE_WORLD_INCLUDED
