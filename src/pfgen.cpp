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
	particle->AddForce(force);
}

// SPRING GENERATOR - OVERLOADED UpdateForce - Applies spring.
void ParticleSpring::UpdateForce(Particle* particle, marb duration) {
    // Calculates the vector of the spring.
    Vector3 force;
    //particle->GetPosition(&force); //This function seems to do the same as setPosition
    particle->SetPosition(force); // This is in the place of GetPosition(&force) which is commented out, keep an eye
    force -= other->GetPosition();

    // Calculates the magnitude of the force.
    marb magnitude = force.Magnitude();
    magnitude = marb_abs(magnitude - restLength);
    magnitude *= springConstant;

    // Calculates the final force and applies it.
    force.Normalize();
    force *= -magnitude;
    particle->AddForce(force);
}

//ANCHORED SPRING GENERATOR - OVERLOADED UpdateForce - Applies anchored spring.
void ParticleAnchoredSpring::UpdateForce(Particle* particle, marb duration){
    // Calculate the vector of the spring.
    Vector3 force;
    //particle->GetPosition(&force); //This function seems to do the same as setPosition
    particle->SetPosition(force); // This is in the place of GetPosition(&force) which is commented out, keep an eye
    force -= *anchor;

    // Calculate the magnitude of the force.
    marb magnitude = force.magnitude();
    magnitude = marb_abs(magnitude - restLength);
    magnitude *= springConstant;

    // Calculate the final force and apply it.
    force.Normalize();
    force *= -magnitude;
    particle->AddForce(force);
}

//BUNGEE GENERATOR - OVERLOADED UpdateForce - Applies bungee forces.
void ParticleBungee::UpdateForce(Particle* particle, marb duration)
{
    // Calculate the vector of the spring.
    Vector3 force;
    //particle->GetPosition(&force); //This function seems to do the same as setPosition
    particle->SetPosition(force); // This is in the place of GetPosition(&force) which is commented out, keep an eye
    force -= other->GetPosition();

    // Check if the bungee is compressed.
    marb magnitude = force.Magnitude();
    if (magnitude <= restLength) return;

    // Calculate the magnitude of the force.
    magnitude = springConstant * (restLength - magnitude);

    // Calculate the final force and apply it.
    force.Normalize();
    force *= -magnitude;
    particle->AddForce(force);
}

//BUOYANCY GENERATOR - OVERLOADED UpdateForce - Applies buoyancy properties.
void ParticleBuoyancy::UpdateForce(Particle* particle, marb duration)
{
    // Calculate the submersion depth.
    marb depth = particle->GetPosition().y;

    // Check if we’re out of the water.
    if (depth >= waterHeight + maxDepth) return;
    Vector3 force(0,0,0);

    // Check if we’re at maximum depth.
    if (depth <= waterHeight - maxDepth)
    {
        force.y = liquidDensity * volume;
        particle->AddForce(force);
        return;
    }

    // Otherwise we are partly submerged.
    force.y = liquidDensity * volume *
    (depth - maxDepth - waterHeight) / 2 * maxDepth;
    particle->AddForce(force);
}

//FAKE SPRING GENERATOR - OVERLOADED UpdateForce - Applies fake spring forces.
void ParticleFakeSpring::UpdateForce(Particle* particle, marb duration)
{
    // Check that we do not have infinite mass.
    if (!particle->HasFiniteMass()) return;

    // Calculate the relative position of the particle to the anchor.
    Vector3 position;
    //particle->GetPosition(&position); //This function seems to do the same as setPosition
    particle->SetPosition(position) // This is in the place of GetPosition(&position) which is commented out, keep an eye
    position -= *anchor;

    // Calculate the constants and check whether they are in bounds.
    marb gamma = 0.5f * marb_sqrt(4 * springConstant - damping*damping);
    if (gamma == 0.0f) return;

    Vector3 c = position * (damping / (2.0f * gamma)) + particle->GetVelocity() * (1.0f / gamma);

    // Calculate the target position.
    Vector3 target = position * marb_cos(gamma * duration) + c * marb_sin(gamma * duration);
    target *= marb_exp(-0.5f * duration * damping);

    // Calculate the resulting acceleration and therefore the force
    Vector3 accel = (target - position) * (1.0f / duration*duration) - particle->GetVelocity() * duration;
    particle->AddForce(accel * particle->GetMass());
}
