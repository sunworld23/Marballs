/*******************************************************************
 * contacts.h
 * -------------
 * Header file for contact classes.
 *
 * Last Revision: Nov. 16, 2014
 *
 * TO DO: - Format comments properly.
 *        - Debug (ensure that everything is in here that should be!)
 ********************************************************************/

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

#include "body.h"

namespace marballs {

    class ContactResolver; // Forward declaration so compiler knows it's there.

    class Contact {

        Vector3 contactPoint; // Holds the position of the contact
        Vector3 contactNormal; // Holds the direction of the contact

        marb penetration; // Holds depth of penetration at contact point.
						  // If both bodies specified, point should be halfway between penetrating points.

        // The contact resolver needs access into the contacts to set and effect the contact
        friend ContactResolver;

        protected:

            Matrix3 contactToWorld; // A transform matrix that converts coordinates in the contact's
                                    // frame of reference to world coordinates.

            Vector3 contactVelocity; // Holds closing velocity at point of contact
                                    // NOTE - this is set when CalculateInternals function is ran.

            marb desiredDeltaVelocity; // Holds required change in velocity for this contact to be resolved.

            Vector3 relativeContactPosition[2]; // holds the world space position of the contact point relative to
                                                // the center of each body.
                                                // NOTE - this is set when CalculateInternals function is ran.
    };
    // NOTE:
    // The contact resolution routine. One resolver instance
    // can be shared for the whole simulation, as long as you need
    // roughly the same parameters each time (which is normal).

    class ContactResolver {
        public:
         void ResolveContacts(Contact *contactArray, unsigned numContacts, marb duration);

        protected:
            // PrepareContacts - Sets up contacts so they are ready for processing.
            // Also checks to make sure internal data is configured correctly and
            // that it is the correct set of bodies alive.
            void PrepareContacts(Contact *contactArray, unsigned numContacts, marb duration);
    };

} // end marballs namespace



#endif // CONTACT_INCLUDED
