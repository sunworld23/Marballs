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

        struct ParticleList // Holds a linked list of all particles created.
        {
            Particle *particle; // The values held by the particle at this position.
            ParticleList *next; // Pointer to next particle in out list.
        };

        ParticleList *firstParticle; // The first particle used in the linked list of particles.

        /*********************************
        *     Variable Declarations
        *********************************/
        public:
            // ParticleWorld - Constructor to create new particle simulator.
            // The simulator is used to handle a given number of contacts per frame.
            // Optionally the world can be given a number of contact-resolution iterations.
            // If iteration number not given for contact-resolution, double the number of contacts.
            // NOTE: typed unsigned is assumed to be int within C++
            ParticleWorld(unsigned maxContacts, unsigned iterations = 0);

            // startFrame - Clears force accumulators for particles in world then new forces can be added.
            void startFrame();



    }; // end of ParticleWorld class
}// end of marballs namespace

#endif // PARTICLE_WORLD_INCLUDED
