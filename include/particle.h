/*************************************************************
 * particle.h
 * -------------
 * Header file that defines the simplest physics simulated object.
 * Particle class is not to be confused with particle physics.
 *
 * Last Revision: Sept. 24, 2014
 *
 * TO DO: - Continue following tutorial to fill this out?
 *************************************************************/

/********************************************************
 * #include guards to protect against double inclusions
 * when using header files
*********************************************************/
#ifndef PARTICLE_INCLUDED
#define PARTICLE_INCLUDED

#include "marballs.h"

namespace marballs
{
    class Particle
    {
        /*********************************
        *     Variable Declarations
        *********************************/
        public:

            Vector3 position;     // Holds the linear position of the particle.
            Vector3 velocity;     // Holds the linear speed of the particle.
            Vector3 acceleration; // Holds the acceleration of the particle (usually by gravity).

            /***********************************************
            * Removes energy added by the instability of the
            * integrator. Used to simply simulate drag due to
            * forces such as friction or air resistance.
            ************************************************/
            marb damping;

        protected:
            /*****************************************************
            * Holds the inverse mass of the particle (1/m).
            * Holding inverse mass makes it easier to integrate as
            * well as avoids infinite mass (immovable) objects.
            * Protected to avoid changing it to a direct mass
            * if mass set directly, an object might act differently
            * than intended.
            ******************************************************/
            marb inverseMass;

        /*********************************
        *     Function Declarations
        *********************************/
        public:
            /****************************************************
            * TO DO: - add in the integrator header and implement
            *          the function in a .cpp file (page 50).
            *****************************************************/

    }; // Particle class end
} // marballs namespace end

 #endif // PARTICLE_INCLUDED
