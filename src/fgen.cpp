#include "fgen.h"

using namespace marballs;

void ForceRegistry::UpdateForces(marb duration)
{
    Registry::iterator i = registrations.begin();
    for (; i != registrations.end(); i++)
    {
        i->fg->UpdateForce(i->body, duration);
    }
}

void ForceRegistry::Add(RigidBody *body, ForceGenerator *fg)
{
    ForceRegistry::ForceRegistration registration;
    registration.body = body;
    registration.fg = fg;
    registrations.push_back(registration);
}

Buoyancy::Buoyancy(const Vector3 &cOfB, marb maxDepth, marb volume,
                   marb waterHeight, marb liquidDensity /* = 1000.0f */)
{
    centreOfBuoyancy = cOfB;
    Buoyancy::liquidDensity = liquidDensity;
    Buoyancy::maxDepth = maxDepth;
    Buoyancy::volume = volume;
    Buoyancy::waterHeight = waterHeight;
}

void Buoyancy::UpdateForce(RigidBody *body, marb duration)
{
    // Calculate the submersion depth
    Vector3 pointInWorld = body->GetPointInWorldSpace(centreOfBuoyancy);
    marb depth = pointInWorld.y;

    // Check if we're out of the water
    if (depth >= waterHeight + maxDepth) return;
    Vector3 force(0,0,0);

    // Check if we're at maximum depth
    if (depth <= waterHeight - maxDepth)
    {
        force.y = liquidDensity * volume;
        body->AddForceAtBodyPoint(force, centreOfBuoyancy);
        return;
    }

    // Otherwise we are partly submerged
    force.y = liquidDensity * volume *
        (depth - maxDepth - waterHeight) / 2 * maxDepth;
    body->AddForceAtBodyPoint(force, centreOfBuoyancy);
}

Gravity::Gravity(const Vector3& gravity): gravity(gravity)
{
}

void Gravity::UpdateForce(RigidBody* body, marb duration)
{
    // Check that we do not have infinite mass
    if (!body->HasFiniteMass()) return;

    // Apply the mass-scaled force to the body
    body->AddForce(gravity * body->GetMass());
}

Spring::Spring(const Vector3 &localConnectionPt,
               RigidBody *other,
               const Vector3 &otherConnectionPt,
               marb springConstant,
               marb restLength)
: connectionPoint(localConnectionPt),
  otherConnectionPoint(otherConnectionPt),
  other(other),
  springConstant(springConstant),
  restLength(restLength)
{
}

void Spring::UpdateForce(RigidBody* body, marb duration)
{
    // Calculate the two ends in world space
    Vector3 lws = body->GetPointInWorldSpace(connectionPoint);
    Vector3 ows = other->GetPointInWorldSpace(otherConnectionPoint);

    // Calculate the vector of the spring
    Vector3 force = lws - ows;

    // Calculate the magnitude of the force
    marb magnitude = force.Magnitude();
    magnitude = marb_abs(magnitude - restLength);
    magnitude *= springConstant;

    // Calculate the final force and apply it
    force.Normalize();
    force *= -magnitude;
    body->AddForceAtPoint(force, lws);
}

Aero::Aero(const Matrix3 &tensor, const Vector3 &position, const Vector3 *windspeed)
{
    Aero::tensor = tensor;
    Aero::position = position;
    Aero::windspeed = windspeed;
}

void Aero::UpdateForce(RigidBody *body, marb duration)
{
    Aero::UpdateForceFromTensor(body, duration, tensor);
}

void Aero::UpdateForceFromTensor(RigidBody *body, marb duration,
                                 const Matrix3 &tensor)
{
    // Calculate total velocity (windspeed and body's velocity).
    Vector3 velocity = body->GetVelocity();
    velocity += *windspeed;

    // Calculate the velocity in body coordinates
    Vector3 bodyVel = body->GetTransform().TransformInverseDirection(velocity);

    // Calculate the force in body coordinates
    Vector3 bodyForce = tensor.Transform(bodyVel);
    Vector3 force = body->GetTransform().TransformDirection(bodyForce);

    // Apply the force
    body->AddForceAtBodyPoint(force, position);
}

AeroControl::AeroControl(const Matrix3 &base, const Matrix3 &min, const Matrix3 &max,
                              const Vector3 &position, const Vector3 *windspeed)
:
Aero(base, position, windspeed)
{
    AeroControl::minTensor = min;
    AeroControl::maxTensor = max;
    controlSetting = 0.0f;
}

Matrix3 AeroControl::GetTensor()
{
    if (controlSetting <= -1.0f) return minTensor;
    else if (controlSetting >= 1.0f) return maxTensor;
    else if (controlSetting < 0)
    {
        return Matrix3::LinearInterpolate(minTensor, tensor, controlSetting+1.0f);
    }
    else if (controlSetting > 0)
    {
        return Matrix3::LinearInterpolate(tensor, maxTensor, controlSetting);
    }
    else return tensor;
}

void AeroControl::SetControl(marb value)
{
    controlSetting = value;
}

void AeroControl::UpdateForce(RigidBody *body, marb duration)
{
    Matrix3 tensor = GetTensor();
    Aero::UpdateForceFromTensor(body, duration, tensor);
}

void Explosion::UpdateForce(RigidBody* body, marb duration)
{
}
