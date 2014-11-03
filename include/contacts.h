/**
* A contact represents two bodies in contact. Resolving a
* contact removes their interpenetration, and applies sufficient
* impulse to keep them apart. Colliding bodies may also rebound.
* Contacts can be used to represent positional joints, by making
* the contact constraint keep the bodies in their correct
* orientation.
*/

#ifndef CONTACT_INCLUDED
#define CONTACT_INCLUDED

#include "engine_core.h"

namespace marballs
{

    class Contact
    {
        // Holds the position of the contact
        Vector3 contactPoint;

        // Holds the direction of the contact
        Vector3 contactNormal;

        /*
        * Holds the depth of penetration at the contact point. If both
        * bodies are specified then the contact point should be midway
        * between the inter-penetrating points.
        */
        marb penetration;
    };


}



#endif // CONTACT_INCLUDED
