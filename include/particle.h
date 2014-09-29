/*************************************************************
 * particle.h
 * -------------
 * Header file that defines the simplest physics simulated object.
 * Particle class is not to be confused with particle physics.
 *
 * Last Revision: Sept. 28, 2014
 *
 * TO DO: - Polish formatting. This file's mostly done.
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
            Vector3 velocity;     // Holds the linear velocity of the particle.
            Vector3 acceleration; // Holds the acceleration of the particle (usually by gravity).
            Vector3 forceAccum;	  // Holds accumulated force to be applied each iteration. 0'd at each step.

            marb damping; 		  // Lessens (dampens) energy added by integrator. Simplifies drag simulation.

        protected:
            marb inverseMass; // Holds the inverse mass of the particle (1/m), making it easier to Integrate and
							  // avoiding infinite mass (immovable) objects. Protected to avoid using it
							  // as/changing it to direct mass.

        /*********************************
        *     Function Declarations
        *********************************/
        public:
			// Integrate - Integrates particle forward in time by the specified amount.
            void Integrate(marb duration);

            /***************************************************************************************
            * USE SETTER FUNCTIONS AS LAST RESORT, THEY INVALIDATE OTHER INTERNAL DATA SUPPOSEDLY
            ****************************************************************************************/
            void SetMass(const marb mass);
            marb GetMass() const;
            void SetInverseMass(const marb inverseMass);
            marb GetInverseMass() const;
            void SetDamping(const marb damping);
            marb GetDamping() const;
            void SetPosition(const Vector3 &position);
            void SetPosition(const marb x, const marb y, const marb z);
            //void GetPosition(Vector3 *position) const; // Makes given vector hold particle's position.
            // Commented out because it seems useless when you can just use an assignment statement.
            Vector3 GetPosition() const;
            void SetVelocity(const Vector3 &velocity);
            void SetVelocity(const marb x, const marb y, const marb z);
            //void GetVelocity(Vector3 *velocity) const; // Same as the other commented out ones.
            Vector3 GetVelocity() const;
            Vector3 SetAcceleration(const Vector3 &acceleration);
            void SetAcceleration(const marb x, const marb y, const marb z);
            //void GetAcceleration(Vector3 *acceleration) const; // Makes given vector hold particle's acceleration.
            // Commented out because it seems frivolous, just make the Vector3 = the value you get in the following function.
            Vector3 GetAcceleration() const;

            /**********************************************************************************
            * END OF SETTER AND GETTER FUNCTION (AVOID USING IF POSSIBLE)
            ***********************************************************************************/

            bool HasFiniteMass() const;
            void ClearAccumulator();
            void AddForce(const Vector3 &force);
    }; // Particle class end
} // marballs namespace end

 #endif // PARTICLE_INCLUDED
