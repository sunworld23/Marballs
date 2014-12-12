/*************************************************************
 * particle.cpp
 * -------------
 * Source file that will implement our particle header file.
 * Particle class is not to be confused with particle physics.
 *
 * Last Revision: Oct. 20, 2014
 *
 * TO DO: - Polish formatting, this file's close to done.
 *************************************************************/

#include <assert.h>
#include "marballs.h"

using namespace marballs;

// Integrate - Integrates particle forward in time by specified amount.
void Particle::Integrate(marb duration)
{
    assert(duration > 0.0);

    position.AddScaledVector(velocity, duration); // Updates linear position.

    // Work out the acceleration from the force.
    Vector3 resultingAcc = acceleration;
    resultingAcc.AddScaledVector(forceAccum, inverseMass);

    velocity.AddScaledVector(resultingAcc, duration); // Update linear velocity based on the calculated acceleration.

    velocity *= marb_pow(damping, duration); // Inflicts drag on the object by applying damping

    ClearAccumulator(); // Clear the forces acting on the particle
}


bool Particle::HasFiniteMass() const { return inverseMass >= 0.0f; }

void Particle::ClearAccumulator() { forceAccum.Clear(); }

void Particle::AddForce(const Vector3 &force) { forceAccum += force; }

/*****************************
* GETTER AND SETTER FUNCTIONS
******************************/
// NOTE: Supposedly these setters should only be used as a last resort. Use other, less direct methods first.

void Particle::SetMass(const marb mass) {
    assert(mass != 0);
    Particle::inverseMass = ((marb)1.0)/mass;
}

marb Particle::GetMass() const {
    if (inverseMass == 0) {
        return MARB_MAX;
    } else {
        return ((marb)1.0)/inverseMass;
    }
}

void Particle::SetInverseMass(const marb inverseMass) { Particle::inverseMass = inverseMass; }

marb Particle::GetInverseMass() const { return inverseMass; }

void Particle::SetDamping(const marb damping) { Particle::damping = damping; }

marb Particle::GetDamping() const { return damping; }

void Particle::SetPosition(const Vector3 &position) { Particle::position = position; }

void Particle::SetPosition(const marb x, const marb y, const marb z) {
    position.x = x;
    position.y = y;
    position.z = z;
}

// GetPosition - Sets a given vector to this particle's position.
// NOTE: Commented out because it seems useless. Just use an assignment statement.
//void Particle::GetPosition(Vector3 *position) const { }

Vector3 Particle::GetPosition() const { return position; }

void Particle::SetVelocity(const Vector3 &velocity) { Particle::velocity = velocity; }

void Particle::SetVelocity(const marb x, const marb y, const marb z) {
    velocity.x = x;
    velocity.y = y;
    velocity.z = z;
}

// GetVelocity - Sets a given vector to this particle's velocity.
// NOTE: Commented out because it seems useless. Just use an assignment statement.
//void Particle::GetVelocity(Vector3 *velocity) const { }

Vector3 Particle::GetVelocity() const { return velocity; }

Vector3 Particle::SetAcceleration(const Vector3 &acceleration) { Particle::acceleration = acceleration; }

void Particle::SetAcceleration(const marb x, const marb y, const marb z) {
    acceleration.x = x;
    acceleration.y = y;
    acceleration.z = z;
}

// GetAcceleration - Sets a given vector to this particle's acceleration.
// NOTE: Commented out because it seems useless. Just use an assignment statement.
//void Particle::GetAcceleration(Vector3 *acceleration) const { }

Vector3 Particle::GetAcceleration() const { return acceleration; }
