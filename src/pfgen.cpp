/*************************************************************
 * pfgen.cpp
 * -------------
 * File that defines what a particle force generator actually does.
 *
 * Last Revision: Sept. 29, 2014
 *
 * TO DO: - Continue tutorial.
 *************************************************************/

#include "pfgen.h"

using namespace marballs;

// UpdateForces - Calls all registered force generators to update their forces.
void ParticleForceRegistry::UpdateForces(marb duration) {
	Registry::iterator i = registrations.begin();
	for (; i != registrations.end(); i++)
		i->fg->UpdateForce(i->particle, duration);
}

// GRAVITY GENERATOR - OVERLOADED UpdateForce - Applies force of gravity.
void ParticleGravity::UpdateForce(Particle* particle, marb duration) {
	if (!particle->HasFiniteMass()) return; // Check to make sure particle doesn't have infinite mass
	
	particle->AddForce(gravity * particle->GetMass()); // Apply force to particle
}

// DRAG GENERATOR - OVERLOADED UpdateForce - Applies drag.
void ParticleDrag::UpdateForce(Particle* particle, marb duration) {
	Vector3 force;
	force = particle->GetVelocity();
	
	// Calculate total drag coefficient
	marb dragCoeff = force.Magnitude();
	dragCoeff = k1 * dragCoeff + k2 * dragCoeff * dragCoeff;
	
	// Calculate and apply final force
	force.Normalize();
	force *= -dragCoeff;
	particle->addForce(force);
}


