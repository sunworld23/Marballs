/***************************************************************
 * collide_fine.h
 * -------------
 * Header file for fine collision classes.
 *
 * Last Revision: Nov. 9, 2014
 *
 * TO DO: - Compare against author's code to determine accuracy.
 ***************************************************************/

#ifndef COLLIDE_FINE_INCLUDED
#define COLLIDE_FINE_INCLUDED

#include "body.h"
#include "contacts.h"

namespace marballs {

    /**
    * A helper structure that contains information for the detector to use
    * in building its contact data.
    */
    struct CollisionData {
        // Holds the contact array to write into.
        Contact *contacts;

        // Holds the maximum number of contacts in the array.
        unsigned contactsLeft;

        marb friction;   // Holds the friction value of collision.

        marb restitution; // Holds the restitution value of collision.

    };

    class Primitive {
    public:
        RigidBody *body;
        Matrix4 offset;

        Vector3 GetAxis(unsigned index) const
        {
            return transform.GetAxisVector(index);
        }

        Matrix4 transform;

    };

    void DetectContacts(const Primitive &firstPrimitive,
        const Primitive &secondPrimitive,
        CollisionData *data);

    class Sphere : public Primitive {
    public:
        marb radius;
        Vector3 position; // Added because it was missing compared to the above Sphere class.
    };

    class Plane : public Primitive {
    public:
        Vector3 direction; // Plane normal
        marb offset;
    };

    class Box : public Primitive {
    public:
        Vector3 halfSize;
    };

    class CollisionDetector {
    /** Function Declaration **/
    public:
        unsigned SphereAndSphere(const Sphere &one, const Sphere &two, CollisionData *data);
        unsigned SphereAndHalfSpace(const Sphere &sphere, const Plane &plane, CollisionData *data);
        unsigned SphereAndTruePlane(const Sphere &sphere, const Plane &plane, CollisionData *data);

        unsigned BoxAndSphere(const Box &box, const Sphere &sphere, CollisionData *data);
        unsigned BoxAndPoint(const Box &box, const Vector3 &point, CollisionData *data);
    };



}


#endif // COLLIDE_FINE_INCLUDED
