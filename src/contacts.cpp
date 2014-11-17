/*************************************************************
 * contacts.cpp
 * -------------
 * Source file that implements Contact class functions.
 *
 * Last Revision: Nov. 16, 2014
 *
 * TO DO: - Debug.
 *
 *************************************************************/

#include "marballs.h"

/*
 * Constructs an arbitrary orthonormal basis for the contact.  This is
 * stored as a 3x3 matrix, where each vector is a column (in other
 * words the matrix transforms contact space into world space). The x
 * direction is generated from the contact normal, and the y and z
 * directionss are set so they are at right angles to it.
 */
inline
void Contact::CalculateContactBasis() {

    Vector3 contactTangent[2];

    // Check whether the Z-axis is nearer to the X or Y axis
    if (marb_abs(contactNormal.x) > marb_abs(contactNormal.y)) {
        // Scaling factor to ensure the results are normalised
        const marb s = (marb)1.0f/marb_sqrt(contactNormal.z*contactNormal.z +
                                            contactNormal.x*contactNormal.x);

        // The new X-axis is at right angles to the world Y-axis
        contactTangent[0].x = contactNormal.z*s;
        contactTangent[0].y = 0;
        contactTangent[0].z = -contactNormal.x*s;

        // The new Y-axis is at right angles to the new X- and Z- axes
        contactTangent[1].x = contactNormal.y*contactTangent[0].x;
        contactTangent[1].y = contactNormal.z*contactTangent[0].x -
        contactNormal.x*contactTangent[0].z;
        contactTangent[1].z = -contactNormal.y*contactTangent[0].x;

    } else {

        // Scaling factor to ensure the results are normalised
        const marb s = (marb)1.0/marb_sqrt(contactNormal.z*contactNormal.z +
                                           contactNormal.y*contactNormal.y);

        // The new X-axis is at right angles to the world X-axis
        contactTangent[0].x = 0;
        contactTangent[0].y = -contactNormal.z*s;
        contactTangent[0].z = contactNormal.y*s;

        // The new Y-axis is at right angles to the new X- and Z- axes
        contactTangent[1].x = contactNormal.y*contactTangent[0].z -
        contactNormal.z*contactTangent[0].y;
        contactTangent[1].y = -contactNormal.x*contactTangent[0].z;
        contactTangent[1].z = contactNormal.x*contactTangent[0].y;
    }

    // Make a matrix from the three vectors.
    contactToWorld.setComponents(contactNormal, contactTangent[0], contactTangent[1]);
}

// Build a vector that shows the change in velocity in
// world space for a unit impulse in the direction of the contact
// normal.
Vector3 Contact::CalculateFrictionlessImpulse(Matrix3 * inverseInertiaTensor) {

    Vector3 impulseContact;

    // Build a vector that shows the change in velocity in
    // world space for a unit impulse in the direction of the contact
    // normal.
    Vector3 deltaVelWorld = relativeContactPosition[0] % contactNormal;
    deltaVelWorld = inverseInertiaTensor[0].transform(deltaVelWorld);
    deltaVelWorld = deltaVelWorld % relativeContactPosition[0];

    // Work out the change in velocity in contact coordiantes.
    marb deltaVelocity = deltaVelWorld * contactNormal;

    // Add the linear component of velocity change
    deltaVelocity += body[0]->getInverseMass();

    // Check if we need to the second body's data
    if (body[1]) {
        // Go through the same transformation sequence again
        Vector3 deltaVelWorld = relativeContactPosition[1] % contactNormal;
        deltaVelWorld = inverseInertiaTensor[1].transform(deltaVelWorld);
        deltaVelWorld = deltaVelWorld % relativeContactPosition[1];

        // Add the change in velocity due to rotation
        deltaVelocity += deltaVelWorld * contactNormal;

        // Add the change in velocity due to linear motion
        deltaVelocity += body[1]->getInverseMass();
    }

    // Calculate the required size of the impulse
    impulseContact.x = desiredDeltaVelocity / deltaVelocity;
    impulseContact.y = 0;
    impulseContact.z = 0;
    return impulseContact;
}

void Contact::ApplyVelocityChange(Vector3 velocityChange[2], Vector3 rotationChange[2]) {
    // Get hold of the inverse mass and inverse inertia tensor, both in
    // world coordinates.
    Matrix3 inverseInertiaTensor[2];
    body[0]->getInverseInertiaTensorWorld(&inverseInertiaTensor[0]);
    if (body[1])
        body[1]->getInverseInertiaTensorWorld(&inverseInertiaTensor[1]);

    // We will calculate the impulse for each contact axis
    Vector3 impulseContact;

    if (friction == (marb)0.0) {
        // Use the short format for frictionless contacts
        impulseContact = calculateFrictionlessImpulse(inverseInertiaTensor);
    } else {
        // Otherwise we may have impulses that aren't in the direction of the
        // contact, so we need the more complex version.
        impulseContact = calculateFrictionImpulse(inverseInertiaTensor);
    }

    // Convert impulse to world coordinates
    Vector3 impulse = contactToWorld.transform(impulseContact);

    // Split in the impulse into linear and rotational components
    Vector3 impulsiveTorque = relativeContactPosition[0] % impulse;
    rotationChange[0] = inverseInertiaTensor[0].transform(impulsiveTorque);
    velocityChange[0].clear();
    velocityChange[0].addScaledVector(impulse, body[0]->getInverseMass());

    // Apply the changes
    body[0]->addVelocity(velocityChange[0]);
    body[0]->addRotation(rotationChange[0]);

    if (body[1]) {
        // Work out body one's linear and angular changes
        Vector3 impulsiveTorque = impulse % relativeContactPosition[1];
        rotationChange[1] = inverseInertiaTensor[1].transform(impulsiveTorque);
        velocityChange[1].clear();
        velocityChange[1].addScaledVector(impulse, -body[1]->getInverseMass());

        // And apply them.
        body[1]->addVelocity(velocityChange[1]);
        body[1]->addRotation(rotationChange[1]);
    }
}


void Contact::ApplyPositionChange(Vector3 linearChange[2], Vector3 angularChange[2], marb penetration) {
    const marb angularLimit = (marb)0.2f;
    marb angularMove[2];
    marb linearMove[2];

    marb totalInertia = 0;
    marb linearInertia[2];
    marb angularInertia[2];

    // We need to work out the inertia of each object in the direction
    // of the contact normal, due to angular inertia only.
    for (unsigned i = 0; i < 2; i++) if (body[i]) {
        Matrix3 inverseInertiaTensor;
        body[i]->getInverseInertiaTensorWorld(&inverseInertiaTensor);

        // Use the same procedure as for calculating frictionless
        // velocity change to work out the angular inertia.
        Vector3 angularInertiaWorld =
            relativeContactPosition[i] % contactNormal;
        angularInertiaWorld =
            inverseInertiaTensor.transform(angularInertiaWorld);
        angularInertiaWorld =
            angularInertiaWorld % relativeContactPosition[i];
        angularInertia[i] =
            angularInertiaWorld * contactNormal;

        // The linear component is simply the inverse mass
        linearInertia[i] = body[i]->getInverseMass();

        // Keep track of the total inertia from all components
        totalInertia += linearInertia[i] + angularInertia[i];

        // We break the loop here so that the totalInertia value is
        // completely calculated (by both iterations) before
        // continuing.
    }

    // Loop through again calculating and applying the changes
    for (unsigned i = 0; i < 2; i++) if (body[i]) {
        // The linear and angular movements required are in proportion to
        // the two inverse inertias.
        marb sign = (i == 0)?1:-1;
        angularMove[i] =
            sign * penetration * (angularInertia[i] / totalInertia);
        linearMove[i] =
            sign * penetration * (linearInertia[i] / totalInertia);

        // To avoid angular projections that are too great (when mass is large
        // but inertia tensor is small) limit the angular move.
        Vector3 projection = relativeContactPosition[i];
        projection.addScaledVector(
            contactNormal,
            -relativeContactPosition[i].scalarProduct(contactNormal)
            );

        // Use the small angle approximation for the sine of the angle (i.e.
        // the magnitude would be sine(angularLimit) * projection.magnitude
        // but we approximate sine(angularLimit) to angularLimit).
        marb maxMagnitude = angularLimit * projection.magnitude();

        if (angularMove[i] < -maxMagnitude) {
            marb totalMove = angularMove[i] + linearMove[i];
            angularMove[i] = -maxMagnitude;
            linearMove[i] = totalMove - angularMove[i];

        } else if (angularMove[i] > maxMagnitude) {
            marb totalMove = angularMove[i] + linearMove[i];
            angularMove[i] = maxMagnitude;
            linearMove[i] = totalMove - angularMove[i];
        }

        // We have the linear amount of movement required by turning
        // the rigid body (in angularMove[i]). We now need to
        // calculate the desired rotation to achieve that.
        if (angularMove[i] == 0) {
            // Easy case - no angular movement means no rotation.
            angularChange[i].clear();

        } else {
            // Work out the direction we'd like to rotate in.
            Vector3 targetAngularDirection =
                relativeContactPosition[i].vectorProduct(contactNormal);

            Matrix3 inverseInertiaTensor;
            body[i]->getInverseInertiaTensorWorld(&inverseInertiaTensor);

            // Work out the direction we'd need to rotate to achieve that
            angularChange[i] =
                inverseInertiaTensor.transform(targetAngularDirection) *
                (angularMove[i] / angularInertia[i]);
        }

        // Velocity change is easier - it is just the linear movement
        // along the contact normal.
        linearChange[i] = contactNormal * linearMove[i];

        // Now we can start to apply the values we've calculated.
        // Apply the linear movement
        Vector3 pos;
        body[i]->getPosition(&pos);
        pos.addScaledVector(contactNormal, linearMove[i]);
        body[i]->setPosition(pos);

        // And the change in orientation
        Quaternion q;
        body[i]->getOrientation(&q);
        q.addScaledVector(angularChange[i], ((marb)1.0));
        body[i]->setOrientation(q);

        // We need to calculate the derived data for any body that is
        // asleep, so that the changes are reflected in the object's
        // data. Otherwise the resolution will not change the position
        // of the object, and the next collision detection round will
        // have the same penetration.
        if (!body[i]->getAwake()) body[i]->calculateDerivedData();
    }
}

inline
Vector3 Contact::CalculateFrictionImpulse(Matrix3 * inverseInertiaTensor) {
    Vector3 impulseContact;
    marb inverseMass = body[0]->getInverseMass();

    // The equivalent of a cross product in matrices is multiplication
    // by a skew symmetric matrix - we build the matrix for converting
    // between linear and angular quantities.
    Matrix3 impulseToTorque;
    impulseToTorque.setSkewSymmetric(relativeContactPosition[0]);

    // Build the matrix to convert contact impulse to change in velocity
    // in world coordinates.
    Matrix3 deltaVelWorld = impulseToTorque;
    deltaVelWorld *= inverseInertiaTensor[0];
    deltaVelWorld *= impulseToTorque;
    deltaVelWorld *= -1;

    // Check if we need to add body two's data
    if (body[1]) {
        // Set the cross product matrix
        impulseToTorque.setSkewSymmetric(relativeContactPosition[1]);

        // Calculate the velocity change matrix
        Matrix3 deltaVelWorld2 = impulseToTorque;
        deltaVelWorld2 *= inverseInertiaTensor[1];
        deltaVelWorld2 *= impulseToTorque;
        deltaVelWorld2 *= -1;

        // Add to the total delta velocity.
        deltaVelWorld += deltaVelWorld2;

        // Add to the inverse mass
        inverseMass += body[1]->getInverseMass();
    }

    // Do a change of basis to convert into contact coordinates.
    Matrix3 deltaVelocity = contactToWorld.transpose();
    deltaVelocity *= deltaVelWorld;
    deltaVelocity *= contactToWorld;

    // Add in the linear velocity change
    deltaVelocity.data[0] += inverseMass;
    deltaVelocity.data[4] += inverseMass;
    deltaVelocity.data[8] += inverseMass;

    // Invert to get the impulse needed per unit velocity
    Matrix3 impulseMatrix = deltaVelocity.inverse();

    // Find the target velocities to kill
    Vector3 velKill(desiredDeltaVelocity,
        -contactVelocity.y,
        -contactVelocity.z);

    // Find the impulse to kill target velocities
    impulseContact = impulseMatrix.transform(velKill);

    // Check for exceeding friction
    marb planarImpulse = marb_sqrt(impulseContact.y*impulseContact.y + impulseContact.z*impulseContact.z);

    if (planarImpulse > impulseContact.x * friction) {
        // We need to use dynamic friction
        impulseContact.y /= planarImpulse;
        impulseContact.z /= planarImpulse;

        impulseContact.x = deltaVelocity.data[0] + deltaVelocity.data[1]*friction*impulseContact.y
												 + deltaVelocity.data[2]*friction*impulseContact.z;
        impulseContact.x = desiredDeltaVelocity / impulseContact.x;
        impulseContact.y *= friction * impulseContact.x;
        impulseContact.z *= friction * impulseContact.x;
    }
    return impulseContact;
}

void ContactResolver::ResolveContacts(Contact *contacts, unsigned numContacts, marb duration) {
    // Make sure we have something to do.
    if (numContacts == 0) return;
    // Prepare the contacts for processing
    PrepareContacts(contacts, numContacts, duration);
    // Resolve the interpenetration problems with the contacts.
    AdjustPositions(contacts, numContacts, duration);
    // Resolve the velocity problems with the contacts.
    AdjustVelocities(contacts, numContacts, duration);
}

void Contact::CalculateInternals(marb duration) {
    // Check if the first object is NULL, and swap if it is.
    if (!body[0]) SwapBodies();
    assert(body[0]);
    // Calculate a set of axes at the contact point.
    CalculateContactBasis();
    // Store the relative position of the contact relative to each body.
    relativeContactPosition[0] = contactPoint - body[0]->GetPosition();
    if (body[1]) {
        relativeContactPosition[1] = contactPoint - body[1]->GetPosition();
    }
    // Find the relative velocity of the bodies at the contact point.
    contactVelocity = calculateLocalVelocity(0, duration);
    if (body[1]) {
        contactVelocity -= CalculateLocalVelocity(1, duration);
    }
    // Calculate the desired change in velocity for resolution.
    CalculateDesiredDeltaVelocity(duration);
}

/**
 * Swaps the bodies in the current contact, so body 0 is at body 1 and
 * vice versa. This also changes the direction of the contact normal, but
 * doesn’t update any calculated internal data. If you are calling this
 * method manually, then call calculateInternals afterward to make sure
 * the internal data is up to date.
 */
void Contact::SwapBodies() {
    contactNormal *= -1;
    RigidBody *temp = body[0];
    body[0] = body[1];
    body[1] = temp;
}

Vector3 Contact::CalculateLocalVelocity(unsigned bodyIndex, marb duration) {
    RigidBody *thisBody = body[bodyIndex];
    // Work out the velocity of the contact point.
    Vector3 velocity = thisBody->getRotation() % relativeContactPosition[bodyIndex];
    velocity += thisBody->getVelocity();
    // Turn the velocity into contact coordinates
    Vector3 contactVelocity = contactToWorld.TransformTranspose(velocity);
    // And return it.
    return contactVelocity;
}

void ContactResolver::PrepareContacts(Contact* contacts, unsigned numContacts, marb duration){
    // Generate contact velocity and axis information.
    Contact* lastContact = contacts + numContacts;
    for(Contact* contact=contacts; contact < lastContact; contact++) {
        // Calculate the internal contact data (inertia, basis, etc).
        contact->calculateInternals(duration);
    }
}

void ContactResolver::AdjustPositions(Contact *c, unsigned numContacts, marb duration) {
    unsigned i,index;
    Vector3 velocityChange[2], rotationChange[2];
    marb rotationAmount[2];
    marb max;
    Vector3 cp;
    // Iteratively resolve interpenetration in order of severity.
    positionIterationsUsed = 0;
    while(positionIterationsUsed < positionIterations) {
        // Find biggest penetration.
        max = positionEpsilon;
        index = numContacts;
        for(i = 0; i < numContacts; i++) {
            if(c[i].penetration > max) {
                max=c[i].penetration;
                index=i;
            }
        }
        if (index == numContacts) break;
        // Match the awake state at the contact.
        //c[index].matchAwakeState();
        // Resolve the penetration.
        c[index].ApplyPositionChange(velocityChange, rotationChange, rotationAmount, max);//-positionEpsilon);

        // Again this action may have changed the penetration of other bodies, so we update contacts.
        for(i = 0; i < numContacts; i++) {
            if(c[i].body[0]) {
                if(c[i].body[0]==c[index].body[0]) {
                    cp = rotationChange[0].VectorProduct(c[i].relativeContactPosition[0]);
                    cp += velocityChange[0];
                    c[i].penetration -= rotationAmount[0]*cp.ScalarProduct(c[i].contactNormal);

                } else if(c[i].body[0]==c[index].body[1]) {
                    cp = rotationChange[1].VectorProduct(c[i].relativeContactPosition[0]);
                    cp += velocityChange[1];
                    c[i].penetration -= rotationAmount[1]*cp.ScalarProduct(c[i].contactNormal);
                }
            }
            if(c[i].body[1]) {
                if(c[i].body[1]==c[index].body[0]) {
                    cp = rotationChange[0].VectorProduct(c[i].relativeContactPosition[1]);
                    cp += velocityChange[0];
                    c[i].penetration += rotationAmount[0]*cp.ScalarProduct(c[i].contactNormal);
                } else if(c[i].body[1]==c[index].body[1]) {
                    cp = rotationChange[1].VectorProduct(c[i].relativeContactPosition[1]);
                    cp += velocityChange[1];
                    c[i].penetration += rotationAmount[1]*cp.ScalarProduct(c[i].contactNormal);
                }
            }
        }
        positionIterationsUsed++;
    }
}

void Contact::CalculateDesiredDeltaVelocity(marb duration) {
    const static marb velocityLimit = (marb)0.25f;

    // Calculate the acceleration induced velocity accumulated this frame
    marb velocityFromAcc = 0;

    if (body[0]->GetAwake()) {
        velocityFromAcc += body[0]->getLastFrameAcceleration() * duration * contactNormal;
    }

    if (body[1] && body[1]->GetAwake()) {
        velocityFromAcc -= body[1]->getLastFrameAcceleration() * duration * contactNormal;
    }

    // If the velocity is very slow, limit the restitution
    marb thisRestitution = restitution;
    if (marb_abs(contactVelocity.x) < velocityLimit) {
        thisRestitution = (marb)0.0f;
    }

    // Combine the bounce velocity with the removed
    // acceleration velocity.
    desiredDeltaVelocity = -contactVelocity.x - thisRestitution * (contactVelocity.x - velocityFromAcc);
}
