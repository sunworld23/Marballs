#ifndef COLLIDE_FINE_INCLUDED
#define COLLIDE_FINE_INCLUDED

#include "body.h"

namespace marballs
{

    /**
    * A helper structure that contains information for the detector to use
    * in building its contact data.
    */
    struct CollisionData
    {
        // Holds the contact array to write into.
        Contact *contacts;

        // Holds the maximum number of contacts in the array.
        unsigned contactsLeft;
    };

    class Primitive
    {
    public:
        RigidBody *body;
        Matrix4 offset;
    };

    void DetectContacts(const Primitive &firstPrimitive,
        const Primitive &secondPrimitive,
        CollisionData *data);

    class Sphere
    {
    public:
        Vector3 position;
        marb radius;
    };

    class Sphere : public Primitive
    {
    public:
        marb radius;
    };

    class Plane : public Primitive
    {
    public:
        Vector3 normal;
        marb offset;
    };




}


#endif COLLIDE_FINE_INCLUDED
