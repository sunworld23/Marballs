/*************************************************************
 * pfgen.h
 * -------------
 * Header file that defines particle force generators which
 * add forces to one or more particles.
 *
 * Last Revision: Sept. 29, 2014
 *
 * TO DO: - Continue tutorial.
 *		  - Depending on future tasks, this may be handy for
 *          adding magnetism and wind.
 *************************************************************/

#ifndef MARBALLS_PFGEN_H
#define MARBALLS_PFGEN_H

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
}


#endif // MARBALLS_PFGEN_H
