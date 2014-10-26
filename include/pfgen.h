/*************************************************************
 * pfgen.h
 * -------------
 * Header file that defines particle force generators which
 * add forces to one or more particles.
 *
 * Last Revision: Oct. 25, 2014
 *
 * TO DO: - Continue tutorial.
 *		  - Depending on future tasks, this may be handy for
 *          adding magnetism and wind.
 *************************************************************/

#ifndef PFGEN_INCLUDED
#define PFGEN_INCLUDED

#include "engine_core.h"
#include "particle.h"
#include <vector>

namespace marballs {
	// CLASS ParticleForceGenerator - Base class for, well, particle force generators.
	class ParticleForceGenerator {
		public:
			// VIRTUAL UpdateForce - Calculates and updates applied force on particle.
			virtual void UpdateForce(Particle *particle, marb duration) = 0;
	};

	// CLASS ParticleForceRegistry - Holds all generators and the particles they apply to.
	class ParticleForceRegistry {
		protected:
			struct ParticleForceRegistration { // Keeps track of one force generator and the particle it applies to.
				Particle *particle;
				ParticleForceGenerator *fg;
			};

			typedef std::vector<ParticleForceRegistration> Registry;
			Registry registrations; // List of registered force-particle pairs.

			public:
				// Add - Registers the given force generator to apply it to the given particle.
				void Add(Particle* particle, ParticleForceGenerator *fg);
				// Remove - Removes the given pair from the registry.
				void Remove(Particle* particle, ParticleForceGenerator *fg);
				// Clear - Clears all registered force-particle connections from registry.
				void Clear();
				// UpdateForces - Makes force generators apply the appropriate forces to their particles.
				void UpdateForces(marb duration);
	};

	// CLASS ParticleGravity - A force generator used for gravity. One instance can be paired with several particles.
	// NOTE: Tutorial author opts not to use this, though for creative level design we might.
	class ParticleGravity : public ParticleForceGenerator {

		Vector3 gravity; // Holds gravity acceleration

		public:
			// Constructor - Creates generator with given gravity value.
			ParticleGravity(const Vector3 &gravity);
			virtual void UpdateForce(Particle* particle, marb duration);
	};

	// CLASS ParticleDrag - A force generator used for drag. One instance can be paired with several particles.
	// NOTE: Tutorial author opts not to use this, but we might.
	class ParticleDrag : ParticleForceGenerator {
		marb k1;	// Holds velocity drag coefficient.
		marb k2;	// Holds the SQUARED velocity drag coefficient.

		public:
			// Constructor - Creates generator with the given coefficients.
			ParticleDrag(marb k1, marb k2);
			// OVERLOADED UpdateForce - Applies drag to given particle.
			virtual void UpdateForce(Particle* particle, marb duration);

	};

    // CLASS ParticleSpring - A force generator that applies a spring force.
    class ParticleSpring : public ParticleForceGenerator {
        Particle *other; //Holds the particle at the other end of the spring.
        marb springConstant; //Holds the spring constant.
        marb restLength; //Holds the resting length of the spring.

    public:
        // Constructor - Creates a spring with given properties.
        ParticleSpring(Particle *other, marb springConstant, marb restLength);

        // OVERLOADED UpdateForce - Applies spring force to given particle.
        virtual void UpdateForce(Particle *particle, marb duration);
    };

    // CLASS ParticleAnchoredSpring - A force generator that applies a spring force
    // where one end of the spring is attached to a fixed point.
    class ParticleAnchoredSpring : public ParticleForceGenerator{
        //The location of attached end of the spring.
        Vector3 *anchor;

        //Holds spring constant.
        marb springConstant;

        //Holds the rest length of the spring.
        marb restLength;

    public:
        // Constructor - creates an anchored spring with the given parameters.
        ParticleAnchoredSpring(Vector3 *anchor, marb springConstant, marb restLength);

        // OVERLOADED UpdateForce - Applies spring force to given particle.
        virtual void UpdateForce(Particle *partical, marb duration);
    };

    // CLASS Particle Bungee - Force generator that only applies spring force when extended.
    class ParticleBungee : public ParticleForceGenerator
    {
        // The particle at the other end of the spring.
        Particle *other;

        // Holds the sprint constant.
        marb springConstant;

        // Holds the length of the bungee at the point it begins to generator a force.
        marb restLength;

    public:

        // Constructor - creates a bungee with the given parameters.
        ParticleBungee(Particle *other, marb springConstant, marb restLength);

        // Applies the spring force to the given particle.
        virtual void UpdateForce(Particle *particle, marb duration);
    };

    // CLASS ParticleBuoyancy - A force generator that applies a buoyancy force for a plane of liquid parrallel to XZ plane.
    class ParticleBuoyancy : public ParticleForceGenerator
    {
        marb maxDepth;// The maximum submersion depth of the object before it generates its maximum boyancy force.

        marb volume;// Volume of the object

        marb waterHeight;// The height of the water plane above y=0. The plane will be parrallel to the XZ plane.

        marb liquidDensity;// The density of the liquid. Pure water has a density of 1000kg per cubic meter.

    public:

        // Constructor - Creates a new buoyancy force with the given parameters.
        ParticleBuoyancy(marb maxDepth, marb volume, marb waterHeight, marb liquidDensity = 1000.0f);

        // Applies the buoyancy force to the given particle.
        virtual void UpdateForce(Particle *particle, marb duration);
    };

    // CLASS ParticleFakeSpring - A force generator that fakes a stiff spring force, and where one end is attached to a fixed point.
    class ParticleFakeSpring : public ParticleForceGenerator
    {
        Vector3 *anchor;// The location of the anchored end of the spring.
        marb springConstant;// Holds the spring constant
        marb damping;// Holds the damping on the oscillation of the spring.

    public:
        //Creates a new spring with the given parameters.
        ParticleFakeSpring(Vector3 *anchor, marb springConstant, marb damping);

        // Applies the spring force to the given particle.
        virtual void UpdateForce(Particle *particle, marb duration);
    };
	
	//CLASS Gravity - A force generator used for gravity with rigid bodies
	class Gravity : public ForceGenerator
	{
		
		//Acceleration due to gravity
		Vector3 gravity;
		
		public:
			//Create generator with specified acceleration
			Gravity(const Vector3 &gravity);
			
			//Applies gravity to specified rigid body
			virtual void updateForce(RigidBody *body, marb duration);
	};
	
	//CLASS Spring - A force generator used to apply Spring force
	class Spring : public ForceGenerator
	{
		//Point of connection of spring
		Vector3 connectionPoint;
		
		//Point of connection between spring and the other object
		Vector3 otherConnectionPoint;
		
		//Particle at the other end of the spring
		RigidBody *pther;
		
		//Spring constant
		marb sprintConstant;
		
		//Resting length of the spring
		marb restLength;
		
		public:
			//Creates a new spring with specified values
			Spring(const Vector3 &localConnectionPt, RigidBody *other,
					const Vector3 &otherConnectionPt, marb springConstant, marb springLength);
					
			//Applies spring force to specified particle
			virtual void updateForce(RigidBody *body, marb duration);
	};
}


#endif // PFGEN_INCLUDED
