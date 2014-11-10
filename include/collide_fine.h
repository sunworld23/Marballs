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
    };

    class Primitive {
    public:
        RigidBody *body;
        Matrix4 offset;
    };

    void DetectContacts(const Primitive &firstPrimitive,
        const Primitive &secondPrimitive,
        CollisionData *data);

    /*class Sphere { // Defined twice. If it was supposed to be like this, Code Blocks didn't like it.
    public:
        Vector3 position;
        marb radius;
    };*/

    class Sphere : public Primitive {
    public:
        marb radius;
        Vector3 position; // Added because it was missing compared to the above Sphere class.
    };

    class Plane : public Primitive {
    public:
        Vector3 normal;
        marb offset;
    };




}


#endif COLLIDE_FINE_INCLUDED
