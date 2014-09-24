/*************************************************************
 * particle.cpp
 * -------------
 * Source file that will implement our particle header file.
 * Particle class is not to be confused with particle physics.
 *
 * Last Revision: Sept. 24, 2014
 *
 * TO DO: - Follow rest of tutorial. (Left off on page 54).
 *************************************************************/

#include <assert.h>
#include "marballs.h"

using namespace marballs;

void Particle::integrate(marb duration)
{
    assert(duration > 0.0);
    
    //Updates linear position.
    position.addScaledVector(velocity, duration);
    
    //Work out the acceleration from the force.
    Vector3 resultingAcc = acceleration;
    resultingAcc.addScaledVector(forceAccum, inverseMass);
    
    //Update linear velocity from the acceleration.
    velocity.addScaledVector(resultingAcc, duration);
    
    //Inflicts drag on the object
    velocity *= real_pow(damping, duration);
}
