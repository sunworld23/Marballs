/*************************************************************
 * collide_fine.cpp
 * -------------
 * Implementation of fine collision functions.
 *
 * Last Revision: Nov. 16, 2014
 *
 * TO DO: - Debug (functions need to be declared in header!)
 *        (members need to be accessible by inheritors!)
 *
 *************************************************************/

#include "collide_fine.h"

using namespace marballs;

unsigned CollisionDetector::SphereAndSphere(const Sphere &one, const Sphere &two, CollisionData *data) {

    // Make sure we have contacts.
    if (data->contactsLeft <= 0) return 0;

    // Get the sphere positions.
    Vector3 positionOne = one.GetAxis(3);
    Vector3 positionTwo = two.GetAxis(3);

    // Find the vector between the objects.
    Vector3 midline = positionOne - positionTwo;
    marb size = midline.Magnitude();
    // See if it is large enough.
    if (size <= 0.0f || size >= one.radius+two.radius)
        return 0;

    // We manually create the normal
    Vector3 normal = midline * (((marb)1.0)/size);
    Contact* contact = data->contacts;
    contact->contactNormal = normal;
    contact->contactPoint = positionOne + midline * (marb)0.5;
    contact->penetration = (one.radius+two.radius - size);

    // Write the appropriate data.
    contact->body[0] = one.body;
    contact->body[1] = two.body;
    contact->restitution = data->restitution;
    contact->friction = data->friction;
    return 1;
}


unsigned CollisionDetector::sphereAndHalfSpace(const Sphere &sphere, const Plane &plane, CollisionData *data) {
    // Make sure we have contacts.
    if (data->contactsLeft <= 0) return 0;

    // Cache the sphere position.
    Vector3 position = sphere.GetAxis(3);

    // Find the distance from the plane.
    marb ballDistance =
    plane.direction * position sphere.radius - plane.offset;
    if (ballDistance >= 0) return 0;

    // Create the contact - it has a normal in the plane direction.
    Contact* contact = data->contacts;
    contact->contactNormal = plane.direction;
    contact->penetration = -ballDistance;
    contact->contactPoint =
    position - plane.direction * (ballDistance + sphere.radius);

    // Write the appropriate data.
    contact->body[0] = sphere.body;
    contact->body[1] = NULL;
    contact->restitution = data->restitution;
    contact->friction = data->friction;
    return 1;
}


unsigned CollisionDetector::sphereAndTruePlane(const Sphere &sphere, const Plane &plane, CollisionData *data)
{
    // Make sure we have contacts.
    if (data->contactsLeft <= 0) return 0;
    // Cache the sphere position.
    Vector3 position = sphere.GetAxis(3);
    // Find the distance from the plane.
    marb centerDistance = plane.direction * position - plane.offset;
    // Check if we’re within radius.
    if (centerDistance*centerDistance > sphere.radius*sphere.radius)
    {
    return 0;
    }
    // Check which side of the plane we’re on.
    Vector3 normal = plane.direction;
    marb penetration = -centerDistance;
    if (centerDistance < 0)
    {
    normal *= -1;
    penetration = -penetration;
    }

    penetration += sphere.radius;
    // Create the contact - it has a normal in the plane direction.
    Contact* contact = data->contacts;
    contact->contactNormal = normal;
    contact->penetration = penetration;
    contact->contactPoint = position - plane.direction * centerDistance;
    // Write the appropriate data.
    contact->body[0] = sphere.body;
    contact->body[1] = NULL;
    contact->restitution = data->restitution;
    contact->friction = data->friction;
    return 1;
}


unsigned CollisionDetector::boxAndSphere(const Box &box, const Sphere &sphere, CollisionData *data)
{
    // Transform the center of the sphere into box coordinates.
    Vector3 center = sphere.GetAxis(3);
    Vector3 relCenter = box.transform.TransformInverse(center);
    // Early-out check to see if we can exclude the contact.
    if (marb_abs(relCenter.x) - sphere.radius > box.halfSize.x ||
    marb_abs(relCenter.y) - sphere.radius > box.halfSize.y ||
    marb_abs(relCenter.z) - sphere.radius > box.halfSize.z)
        return 0;

    Vector3 closestPt(0,0,0);
    marb dist;

    // Clamp each coordinate to the box.
    dist = relCenter.x;
    if (dist > box.halfSize.x) dist = box.halfSize.x;
    if (dist < -box.halfSize.x) dist = -box.halfSize.x;
    closestPt.x = dist;
    dist = relCenter.y;
    if (dist > box.halfSize.y) dist = box.halfSize.y;
    if (dist < -box.halfSize.y) dist = -box.halfSize.y;
    closestPt.y = dist;
    dist = relCenter.z;
    if (dist > box.halfSize.z) dist = box.halfSize.z;
    if (dist < -box.halfSize.z) dist = -box.halfSize.z;
    closestPt.z = dist;

    // Check we’re in contact.
    dist = (closestPt - relCenter).SquareMagnitude();
    if (dist > sphere.radius * sphere.radius) return 0;

    // Compile the contact.
    Vector3 closestPtWorld = box.transform.transform(closestPt);
    Contact* contact = data->contacts;
    contact->contactNormal = (center - closestPtWorld);
    contact->contactNormal.normalize();
    contact->contactPoint = closestPtWorld;
    contact->penetration = sphere.radius - marb_sqrt(dist);

    // Write the appropriate data.
    contact->body[0] = box.body;
    contact->body[1] = sphere.body;
    contact->restitution = data->restitution;
    contact->friction = data->friction;
    return 1;
}


unsigned CollisionDetector::boxAndPoint(const Box &box, const Vector3 &point, CollisionData *data)
{
    // Transform the point into box coordinates.
    Vector3 relPt = box.transform.TransformInverse(point);
    Vector3 normal;

    // Check each axis, looking for the axis on which the
    // penetration is least deep.
    marb min_depth = box.halfSize.x - marb_abs(relPt.x);
    if (min_depth < 0) return 0;
    normal = box.GetAxis(0) * ((relPt.x < 0)?-1:1);
    marb depth = box.halfSize.y - marb_abs(relPt.y);
    if (depth < 0) return 0;
    else if (depth < min_depth)
    {
        min_depth = depth;
        normal = box.GetAxis(1) * ((relPt.y < 0)?-1:1);
    }
    depth = box.halfSize.z - marb_abs(relPt.z);
    if (depth < 0) return 0;
    else if (depth < min_depth)
    {
        min_depth = depth;
        normal = box.GetAxis(2) * ((relPt.z < 0)?-1:1);
    }

    // Compile the contact.
    Contact* contact = data->contacts;
    contact->contactNormal = normal;
    contact->contactPoint = point;
    contact->penetration = min_depth;

    // Write the appropriate data.
    contact->body[0] = box.body;

    // Note that we don’t know what rigid body the point
    // belongs to, so we just use NULL. Where this is called
    // this value can be left, or filled in.
    contact->body[1] = NULL;
    contact->restitution = data->restitution;
    contact->friction = data->friction;
    return 1;
}


