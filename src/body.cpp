/***************************************************************
 * body.cpp
 * -------------
 * Source file that implements the RigidBody class.
 *
 * Last Revision: Nov. 8, 2014
 *
 * TO DO: - Fix formatting and comments.
 *				* Capitalize function names if underscore is unneeded.
 *				* Add comments to getters and setters.
 ***************************************************************/

#include "body.h"
#include <memory.h>
#include <assert.h>

using namespace marballs;

/*
 * --------------------------------------------------------------------------
 * INTERNAL OR HELPER FUNCTIONS:
 * --------------------------------------------------------------------------
 */

/**
 * Internal function that checks the validity of an inverse inertia tensor.
 */
static inline void _checkInverseInertiaTensor(const Matrix3 &iitWorld) {
    // TODO: Perform a validity check in an assert.
}

/**
 * Internal function to do an intertia tensor transform by a quaternion.
 * Note that the implementation of this function was created by an
 * automated code-generator and optimizer.
 */
static inline void _transformInertiaTensor(Matrix3 &iitWorld,
                                           const Quaternion &q,
                                           const Matrix3 &iitBody,
                                           const Matrix4 &rotmat)
{
    marb t4 = rotmat.data[0]*iitBody.data[0]+
        rotmat.data[1]*iitBody.data[3]+
        rotmat.data[2]*iitBody.data[6];
    marb t9 = rotmat.data[0]*iitBody.data[1]+
        rotmat.data[1]*iitBody.data[4]+
        rotmat.data[2]*iitBody.data[7];
    marb t14 = rotmat.data[0]*iitBody.data[2]+
        rotmat.data[1]*iitBody.data[5]+
        rotmat.data[2]*iitBody.data[8];
    marb t28 = rotmat.data[4]*iitBody.data[0]+
        rotmat.data[5]*iitBody.data[3]+
        rotmat.data[6]*iitBody.data[6];
    marb t33 = rotmat.data[4]*iitBody.data[1]+
        rotmat.data[5]*iitBody.data[4]+
        rotmat.data[6]*iitBody.data[7];
    marb t38 = rotmat.data[4]*iitBody.data[2]+
        rotmat.data[5]*iitBody.data[5]+
        rotmat.data[6]*iitBody.data[8];
    marb t52 = rotmat.data[8]*iitBody.data[0]+
        rotmat.data[9]*iitBody.data[3]+
        rotmat.data[10]*iitBody.data[6];
    marb t57 = rotmat.data[8]*iitBody.data[1]+
        rotmat.data[9]*iitBody.data[4]+
        rotmat.data[10]*iitBody.data[7];
    marb t62 = rotmat.data[8]*iitBody.data[2]+
        rotmat.data[9]*iitBody.data[5]+
        rotmat.data[10]*iitBody.data[8];

    iitWorld.data[0] = t4*rotmat.data[0]+
        t9*rotmat.data[1]+
        t14*rotmat.data[2];
    iitWorld.data[1] = t4*rotmat.data[4]+
        t9*rotmat.data[5]+
        t14*rotmat.data[6];
    iitWorld.data[2] = t4*rotmat.data[8]+
        t9*rotmat.data[9]+
        t14*rotmat.data[10];
    iitWorld.data[3] = t28*rotmat.data[0]+
        t33*rotmat.data[1]+
        t38*rotmat.data[2];
    iitWorld.data[4] = t28*rotmat.data[4]+
        t33*rotmat.data[5]+
        t38*rotmat.data[6];
    iitWorld.data[5] = t28*rotmat.data[8]+
        t33*rotmat.data[9]+
        t38*rotmat.data[10];
    iitWorld.data[6] = t52*rotmat.data[0]+
        t57*rotmat.data[1]+
        t62*rotmat.data[2];
    iitWorld.data[7] = t52*rotmat.data[4]+
        t57*rotmat.data[5]+
        t62*rotmat.data[6];
    iitWorld.data[8] = t52*rotmat.data[8]+
        t57*rotmat.data[9]+
        t62*rotmat.data[10];
}

/**
 * Inline function that creates a transform matrix from a
 * position and orientation.
 */
static inline void _calculateTransformMatrix(Matrix4 &transformMatrix,
                                             const Vector3 &position,
                                             const Quaternion &orientation)
{
    transformMatrix.data[0] = 1-2*orientation.j*orientation.j-
        2*orientation.k*orientation.k;
    transformMatrix.data[1] = 2*orientation.i*orientation.j -
        2*orientation.r*orientation.k;
    transformMatrix.data[2] = 2*orientation.i*orientation.k +
        2*orientation.r*orientation.j;
    transformMatrix.data[3] = position.x;

    transformMatrix.data[4] = 2*orientation.i*orientation.j +
        2*orientation.r*orientation.k;
    transformMatrix.data[5] = 1-2*orientation.i*orientation.i-
        2*orientation.k*orientation.k;
    transformMatrix.data[6] = 2*orientation.j*orientation.k -
        2*orientation.r*orientation.i;
    transformMatrix.data[7] = position.y;

    transformMatrix.data[8] = 2*orientation.i*orientation.k -
        2*orientation.r*orientation.j;
    transformMatrix.data[9] = 2*orientation.j*orientation.k +
        2*orientation.r*orientation.i;
    transformMatrix.data[10] = 1-2*orientation.i*orientation.i-
        2*orientation.j*orientation.j;
    transformMatrix.data[11] = position.z;
}

/*
 * --------------------------------------------------------------------------
 * FUNCTIONS DECLARED IN HEADER:
 * --------------------------------------------------------------------------
 */
void RigidBody::CalculateDerivedData() {
    orientation.Normalize();

    // Calculate the transform matrix for the body.
    _calculateTransformMatrix(transformMatrix, position, orientation);

    // Calculate the inertiaTensor in world space.
    _transformInertiaTensor(inverseInertiaTensorWorld,
        orientation,
        inverseInertiaTensor,
        transformMatrix);

}

void RigidBody::Integrate(marb duration) {
    if (!isAwake) return;

    // Calculate linear acceleration from force inputs.
    lastFrameAcceleration = acceleration;
    lastFrameAcceleration.AddScaledVector(forceAccum, inverseMass);

    // Calculate angular acceleration from torque inputs.
    //CHECK
    Vector3 angularAcceleration = inverseInertiaTensorWorld.Transform(torqueAccum);

    // Adjust velocities
    // Update linear velocity from both acceleration and impulse.
    velocity.AddScaledVector(lastFrameAcceleration, duration);

    // Update angular velocity from both acceleration and impulse.
    rotation.AddScaledVector(angularAcceleration, duration);

    // Impose drag.
    velocity *= marb_pow(linearDamping, duration);
    rotation *= marb_pow(angularDamping, duration);

    // Adjust positions
    // Update linear position.
    position.AddScaledVector(velocity, duration);

    // Update angular position.
    orientation.AddScaledVector(rotation, duration);

    // Normalise the orientation, and update the matrices with the new
    // position and orientation
    CalculateDerivedData();

    // Clear accumulators.
    ClearAccumulators();

    // Update the kinetic energy store, and possibly put the body to
    // sleep.
    if (canSleep) {
        marb currentMotion = velocity.ScalarProduct(velocity) +
            rotation.ScalarProduct(rotation);

        marb bias = marb_pow(0.5, duration);
        motion = bias*motion + (1-bias)*currentMotion;

        if (motion < sleepEpsilon) SetAwake(false);
        else if (motion > 10 * sleepEpsilon) motion = 10 * sleepEpsilon;
    }
}

// Region: Getters and Setters {

void RigidBody::SetMass(const marb mass) {
    assert(mass != 0);
    RigidBody::inverseMass = ((marb)1.0)/mass;
}

marb RigidBody::GetMass() const {
    if (inverseMass == 0) {
        return MARB_MAX;
    } else {
        return ((marb)1.0)/inverseMass;
    }
}

void RigidBody::SetInverseMass(const marb inverseMass) {
    RigidBody::inverseMass = inverseMass;
}

marb RigidBody::GetInverseMass() const {
    return inverseMass;
}

bool RigidBody::HasFiniteMass() const {
    return inverseMass >= 0.0f;
}

void RigidBody::SetInertiaTensor(const Matrix3 &inertiaTensor) {
    inverseInertiaTensor.SetInverse(inertiaTensor);
    _checkInverseInertiaTensor(inverseInertiaTensor);
}

void RigidBody::GetInertiaTensor(Matrix3 *inertiaTensor) const {
    inertiaTensor->SetInverse(inverseInertiaTensor);
}

Matrix3 RigidBody::GetInertiaTensor() const {
    Matrix3 it;
    GetInertiaTensor(&it);
    return it;
}

void RigidBody::GetInertiaTensorWorld(Matrix3 *inertiaTensor) const {
    inertiaTensor->SetInverse(inverseInertiaTensorWorld);
}

Matrix3 RigidBody::GetInertiaTensorWorld() const {
    Matrix3 it;
    GetInertiaTensorWorld(&it);
    return it;
}

void RigidBody::SetInverseInertiaTensor(const Matrix3 &inverseInertiaTensor) {
    _checkInverseInertiaTensor(inverseInertiaTensor);
    RigidBody::inverseInertiaTensor = inverseInertiaTensor;
}

void RigidBody::GetInverseInertiaTensor(Matrix3 *inverseInertiaTensor) const {
    *inverseInertiaTensor = RigidBody::inverseInertiaTensor;
}

Matrix3 RigidBody::GetInverseInertiaTensor() const {
    return inverseInertiaTensor;
}

void RigidBody::GetInverseInertiaTensorWorld(Matrix3 *inverseInertiaTensor) const {
    *inverseInertiaTensor = inverseInertiaTensorWorld;
}

Matrix3 RigidBody::GetInverseInertiaTensorWorld() const {
    return inverseInertiaTensorWorld;
}

void RigidBody::SetDamping(const marb linearDamping, const marb angularDamping) {
    RigidBody::linearDamping = linearDamping;
    RigidBody::angularDamping = angularDamping;
}

void RigidBody::SetLinearDamping(const marb linearDamping) {
    RigidBody::linearDamping = linearDamping;
}

marb RigidBody::GetLinearDamping() const {
    return linearDamping;
}

void RigidBody::SetAngularDamping(const marb angularDamping) {
    RigidBody::angularDamping = angularDamping;
}

marb RigidBody::GetAngularDamping() const {
    return angularDamping;
}

void RigidBody::SetPosition(const Vector3 &position) {
    RigidBody::position = position;
}

void RigidBody::SetPosition(const marb x, const marb y, const marb z) {
    position.x = x;
    position.y = y;
    position.z = z;
}

void RigidBody::GetPosition(Vector3 *position) const {
    *position = RigidBody::position;
}

Vector3 RigidBody::GetPosition() const {
    return position;
}

void RigidBody::SetOrientation(const Quaternion &orientation) {
    RigidBody::orientation = orientation;
    RigidBody::orientation.Normalize();
}

void RigidBody::SetOrientation(const marb r, const marb i, const marb j, const marb k) {
    orientation.r = r;
    orientation.i = i;
    orientation.j = j;
    orientation.k = k;
    orientation.Normalize();
}

void RigidBody::GetOrientation(Quaternion *orientation) const {
    *orientation = RigidBody::orientation;
}

Quaternion RigidBody::GetOrientation() const {
    return orientation;
}

void RigidBody::GetOrientation(Matrix3 *matrix) const {
    GetOrientation(matrix->data);
}

void RigidBody::GetOrientation(marb matrix[9]) const {
    matrix[0] = transformMatrix.data[0];
    matrix[1] = transformMatrix.data[1];
    matrix[2] = transformMatrix.data[2];

    matrix[3] = transformMatrix.data[4];
    matrix[4] = transformMatrix.data[5];
    matrix[5] = transformMatrix.data[6];

    matrix[6] = transformMatrix.data[8];
    matrix[7] = transformMatrix.data[9];
    matrix[8] = transformMatrix.data[10];
}

void RigidBody::GetTransform(Matrix4 *transform) const {
    memcpy(transform, &transformMatrix.data, sizeof(Matrix4));
}

void RigidBody::GetTransform(marb matrix[16]) const {
    memcpy(matrix, transformMatrix.data, sizeof(marb)*12);
    matrix[12] = matrix[13] = matrix[14] = 0;
    matrix[15] = 1;
}

void RigidBody::GetGLTransform(float matrix[16]) const {
    matrix[0] = (float)transformMatrix.data[0];
    matrix[1] = (float)transformMatrix.data[4];
    matrix[2] = (float)transformMatrix.data[8];
    matrix[3] = 0;

    matrix[4] = (float)transformMatrix.data[1];
    matrix[5] = (float)transformMatrix.data[5];
    matrix[6] = (float)transformMatrix.data[9];
    matrix[7] = 0;

    matrix[8] = (float)transformMatrix.data[2];
    matrix[9] = (float)transformMatrix.data[6];
    matrix[10] = (float)transformMatrix.data[10];
    matrix[11] = 0;

    matrix[12] = (float)transformMatrix.data[3];
    matrix[13] = (float)transformMatrix.data[7];
    matrix[14] = (float)transformMatrix.data[11];
    matrix[15] = 1;
}

Matrix4 RigidBody::GetTransform() const {
    return transformMatrix;
}


Vector3 RigidBody::GetPointInLocalSpace(const Vector3 &point) const {
    return transformMatrix.TransformInverse(point);
}

Vector3 RigidBody::GetPointInWorldSpace(const Vector3 &point) const {
    return transformMatrix.Transform(point);
}

Vector3 RigidBody::GetDirectionInLocalSpace(const Vector3 &direction) const {
    return transformMatrix.TransformInverseDirection(direction);
}

Vector3 RigidBody::GetDirectionInWorldSpace(const Vector3 &direction) const {
    return transformMatrix.TransformDirection(direction);
}


void RigidBody::SetVelocity(const Vector3 &velocity) {
    RigidBody::velocity = velocity;
}

void RigidBody::SetVelocity(const marb x, const marb y, const marb z) {
    velocity.x = x;
    velocity.y = y;
    velocity.z = z;
}

void RigidBody::GetVelocity(Vector3 *velocity) const {
    *velocity = RigidBody::velocity;
}

Vector3 RigidBody::GetVelocity() const {
    return velocity;
}

void RigidBody::AddVelocity(const Vector3 &deltaVelocity) {
    velocity += deltaVelocity;
}

void RigidBody::SetRotation(const Vector3 &rotation) {
    RigidBody::rotation = rotation;
}

void RigidBody::SetRotation(const marb x, const marb y, const marb z) {
    rotation.x = x;
    rotation.y = y;
    rotation.z = z;
}

void RigidBody::GetRotation(Vector3 *rotation) const {
    *rotation = RigidBody::rotation;
}

Vector3 RigidBody::GetRotation() const {
    return rotation;
}

void RigidBody::AddRotation(const Vector3 &deltaRotation) {
    rotation += deltaRotation;
}

void RigidBody::SetAwake(const bool awake) {

    if (awake) {
        isAwake= true;

        // Add a bit of motion to avoid it falling asleep immediately.
        motion = sleepEpsilon*2.0f;
    } else {
        isAwake = false;
        velocity.Clear();
        rotation.Clear();
    }
}

void RigidBody::SetCanSleep(const bool canSleep) {
    RigidBody::canSleep = canSleep;

    if (!canSleep && !isAwake) SetAwake();
}


void RigidBody::GetLastFrameAcceleration(Vector3 *acceleration) const {
    *acceleration = lastFrameAcceleration;
}

Vector3 RigidBody::GetLastFrameAcceleration() const {
    return lastFrameAcceleration;
}

void RigidBody::ClearAccumulators() {
    forceAccum.Clear();
    torqueAccum.Clear();
}

void RigidBody::AddForce(const Vector3 &force) {
    forceAccum += force;
    isAwake = true;
}

void RigidBody::AddForceAtBodyPoint(const Vector3 &force, const Vector3 &point) {
    // Convert to coordinates relative to center of mass.
    Vector3 pt = GetPointInWorldSpace(point);
    AddForceAtPoint(force, pt);

}

void RigidBody::AddForceAtPoint(const Vector3 &force, const Vector3 &point) {
    // Convert to coordinates relative to center of mass.
    Vector3 pt = point;
    pt -= position;

    forceAccum += force;
    torqueAccum += pt % force;

    isAwake = true;
}

void RigidBody::AddTorque(const Vector3 &torque) {
    torqueAccum += torque;
    isAwake = true;
}

void RigidBody::SetAcceleration(const Vector3 &acceleration) {
    RigidBody::acceleration = acceleration;
}

void RigidBody::SetAcceleration(const marb x, const marb y, const marb z) {
    acceleration.x = x;
    acceleration.y = y;
    acceleration.z = z;
}

void RigidBody::GetAcceleration(Vector3 *acceleration) const {
    *acceleration = RigidBody::acceleration;
}

Vector3 RigidBody::GetAcceleration() const {
    return acceleration;
}

// } Eng Region: Getters and Setters
