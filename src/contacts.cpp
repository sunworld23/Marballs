#include "marballs.h"

/*
 * Constructs an arbitrary orthonormal basis for the contact.  This is
 * stored as a 3x3 matrix, where each vector is a column (in other
 * words the matrix transforms contact space into world space). The x
 * direction is generated from the contact normal, and the y and z
 * directionss are set so they are at right angles to it.
 */
inline
void Contact::CalculateContactBasis(){
    Vector3 contactTangent[2];
    
    // Check whether the Z-axis is nearer to the X or Y axis
    if (real_abs(contactNormal.x) > real_abs(contactNormal.y))
    {
        // Scaling factor to ensure the results are normalised
        const real s = (real)1.0f/real_sqrt(contactNormal.z*contactNormal.z +
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
    }
    else
    {
        // Scaling factor to ensure the results are normalised
        const real s = (real)1.0/real_sqrt(contactNormal.z*contactNormal.z +
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
Vector3 Contact::calculateFrictionlessImpulse(Matrix3 * inverseInertiaTensor)
{
    Vector3 impulseContact;
    
    // Build a vector that shows the change in velocity in
    // world space for a unit impulse in the direction of the contact
    // normal.
    Vector3 deltaVelWorld = relativeContactPosition[0] % contactNormal;
    deltaVelWorld = inverseInertiaTensor[0].transform(deltaVelWorld);
    deltaVelWorld = deltaVelWorld % relativeContactPosition[0];
    
    // Work out the change in velocity in contact coordiantes.
    real deltaVelocity = deltaVelWorld * contactNormal;
    
    // Add the linear component of velocity change
    deltaVelocity += body[0]->getInverseMass();
    
    // Check if we need to the second body's data
    if (body[1])
    {
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

// Convert impulse to world coordinates
Vector3 impulse = contactToWorld.transform(impulseContact);

// We need to work out the inertia of each object in the direction
// of the contact normal, due to angular inertia only.
for (unsigned i = 0; i < 2; i++) if (body[i])
{
    Matrix3 inverseInertiaTensor;
    body[i]->GetInverseInertiaTensorWorld(&inverseInertiaTensor);
    
    // Use the same procedure as for calculating frictionless
    // velocity change to work out the angular inertia.
    Vector3 angularInertiaWorld = relativeContactPosition[i] % contactNormal;
    angularInertiaWorld = inverseInertiaTensor.transform(angularInertiaWorld);
    angularInertiaWorld = angularInertiaWorld % relativeContactPosition[i];
    angularInertia[i] = angularInertiaWorld * contactNormal;
    
    // The linear component is simply the inverse mass
    linearInertia[i] = body[i]->getInverseMass();
    
    // Keep track of the total inertia from all components
    totalInertia += linearInertia[i] + angularInertia[i];
    
    // We break the loop here so that the totalInertia value is
    // completely calculated (by both iterations) before
    // continuing.
}

void ContactResolver::ResolveContacts(Contact *contacts, unsigned numContacts, real duration)
{
    // Make sure we have something to do.
    if (numContacts == 0) return;
    // Prepare the contacts for processing
    PrepareContacts(contacts, numContacts, duration);
    // Resolve the interpenetration problems with the contacts.
    AdjustPositions(contacts, numContacts, duration);
    // Resolve the velocity problems with the contacts.
    AdjustVelocities(contacts, numContacts, duration);
}

void Contact::CalculateInternals(real duration)
{
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
void Contact::SwapBodies()
{
    contactNormal *= -1;
    RigidBody *temp = body[0];
    body[0] = body[1];
    body[1] = temp;
}

Vector3 Contact::CalculateLocalVelocity(unsigned bodyIndex, real duration)
{
    RigidBody *thisBody = body[bodyIndex];
    // Work out the velocity of the contact point.
    Vector3 velocity = thisBody->getRotation() % relativeContactPosition[bodyIndex];
    velocity += thisBody->getVelocity();
    // Turn the velocity into contact coordinates
    Vector3 contactVelocity = contactToWorld.TransformTranspose(velocity);
    // And return it.
    return contactVelocity;
}

void ContactResolver::PrepareContacts(Contact* contacts, unsigned numContacts, real duration){
    // Generate contact velocity and axis information.
    Contact* lastContact = contacts + numContacts;
    for(Contact* contact=contacts; contact < lastContact; contact++)
    {
        // Calculate the internal contact data (inertia, basis, etc).
        contact->calculateInternals(duration);
    }
}

void ContactResolver::AdjustPositions(Contact *c, unsigned numContacts, real duration)
{
    unsigned i,index;
    Vector3 velocityChange[2], rotationChange[2];
    real rotationAmount[2];
    real max;
    Vector3 cp;
    // Iteratively resolve interpenetration in order of severity.
    positionIterationsUsed = 0;
    while(positionIterationsUsed < positionIterations)
    {
        // Find biggest penetration.
        max = positionEpsilon;
        index = numContacts;
        for(i=0;i<numContacts;i++) {
            if(c[i].penetration > max)
            {
                max=c[i].penetration;
                index=i;
            }
        }
        if (index == numContacts) break;
        // Match the awake state at the contact.
        //c[index].matchAwakeState();
        // Resolve the penetration.
        c[index].ApplyPositionChange(velocityChange, rotationChange, rotationAmount, max);//-positionEpsilon);
        // Again this action may have changed the penetration of other
        // bodies, so we update contacts.
        for(i=0; i<numContacts; i++)
        {
            if(c[i].body[0])
            {
                if(c[i].body[0]==c[index].body[0])
                {
                    cp = rotationChange[0].VectorProduct(c[i].relativeContactPosition[0]);
                    cp += velocityChange[0];
                    c[i].penetration -= rotationAmount[0]*cp.ScalarProduct(c[i].contactNormal);
                }
                else if(c[i].body[0]==c[index].body[1])
                {
                    cp = rotationChange[1].VectorProduct(c[i].relativeContactPosition[0]);
                    cp += velocityChange[1];
                    c[i].penetration -= rotationAmount[1]*cp.ScalarProduct(c[i].contactNormal);
                }
            }
            if(c[i].body[1])
            {
                if(c[i].body[1]==c[index].body[0])
                {
                    cp = rotationChange[0].VectorProduct(c[i].relativeContactPosition[1]);
                    cp += velocityChange[0];
                    c[i].penetration += rotationAmount[0]*cp.ScalarProduct(c[i].contactNormal);
                }
                else if(c[i].body[1]==c[index].body[1])
                {
                    cp = rotationChange[1].VectorProduct(c[i].relativeContactPosition[1]);
                    cp += velocityChange[1];
                    c[i].penetration += rotationAmount[1]*cp.ScalarProduct(c[i].contactNormal);
                }
            }
        }
        positionIterationsUsed++;
    }
}

void Contact::CalculateDesiredDeltaVelocity(real duration)
{
    const static real velocityLimit = (real)0.25f;
    
    // Calculate the acceleration induced velocity accumulated this frame
    real velocityFromAcc = 0;
    
    if (body[0]->GetAwake())
    {
        velocityFromAcc += body[0]->getLastFrameAcceleration() * duration * contactNormal;
    }
    
    if (body[1] && body[1]->GetAwake())
    {
        velocityFromAcc -= body[1]->getLastFrameAcceleration() * duration * contactNormal;
    }
    
    // If the velocity is very slow, limit the restitution
    real thisRestitution = restitution;
    if (real_abs(contactVelocity.x) < velocityLimit)
    {
        thisRestitution = (real)0.0f;
    }
    
    // Combine the bounce velocity with the removed
    // acceleration velocity.
    desiredDeltaVelocity = -contactVelocity.x -thisRestitution * (contactVelocity.x - velocityFromAcc);
}