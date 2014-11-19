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

#ifndef CONTACTS_INCLUDED
#define CONTACTS_INCLUDED

#include "body.h"

namespace marballs {

    class ContactResolver; // Forward declaration so compiler knows it's there.

    class Contact {
        // The contact resolver needs access into the contacts to set and effect the contact
        friend ContactResolver;
        public:
        RigidBody* body[2]; // Holds bodies involved in the contact

        marb friction; // Lateral friction coefficient
        marb restitution; // Normal restitution coefficient

        Vector3 contactPoint; // Holds the position of the contact
        Vector3 contactNormal; // Holds the direction of the contact

        marb penetration; // Holds depth of penetration at contact point.
						  // If both bodies specified, point should be halfway between penetrating points.

        protected:

            Matrix3 contactToWorld; // A transform matrix that converts coordinates in the contact's
                                    // frame of reference to world coordinates.

            Vector3 contactVelocity; // Holds closing velocity at point of contact
                                    // NOTE - this is set when CalculateInternals function is ran.

            marb desiredDeltaVelocity; // Holds required change in velocity for this contact to be resolved.

            Vector3 relativeContactPosition[2]; // holds the world space position of the contact point relative to
                                                // the center of each body.
                                                // NOTE - this is set when CalculateInternals function is ran.

            // CalculateFrictionlessImpulse - to be added
            Vector3 CalculateFrictionlessImpulse(Matrix3 * inverseInertiaTensor);

            // ApplyVelocityChange - to be added
            void ApplyVelocityChange(Vector3 velocityChange[2], Vector3 rotationChange[2]);

            // ApplyPositionChange - to be added
            void ApplyPositionChange(Vector3 linearChange[2], Vector3 angularChange[2], marb penetration);

            // CalculateFrictionImpulse- to be added
            inline Vector3 CalculateFrictionImpulse(Matrix3 * inverseInertiaTensor);

            // CalculateInternals - to be added
            void CalculateInternals(marb duration);

            // SwapBodies - to be added
            void SwapBodies();

            // CalculateLocalVelocity - to be added
            Vector3 CalculateLocalVelocity(unsigned bodyIndex, marb duration);

            // CalculateDesiredDeltaVelocity - to be added
            void CalculateDesiredDeltaVelocity(marb duration);

            // CalculateContactBasis
            void CalculateContactBasis();
    };
    // NOTE:
    // The contact resolution routine. One resolver instance
    // can be shared for the whole simulation, as long as you need
    // roughly the same parameters each time (which is normal).

    class ContactResolver {
        public:
            unsigned positionIterationsUsed; // Number of position iterations used last call

            void ResolveContacts(Contact *contactArray, unsigned numContacts, marb duration);

        protected:
            unsigned positionIterations; // TBD
            marb positionEpsilon; // TBD

            // PrepareContacts - Sets up contacts so they are ready for processing.
            // Also checks to make sure internal data is configured correctly and
            // that it is the correct set of bodies alive.
            void PrepareContacts(Contact *contactArray, unsigned numContacts, marb duration);

            // AdjustPositions - to be added
            void AdjustPositions(Contact *c, unsigned numContacts, marb duration);

            // AdjustVelocities - to be added
            void AdjustVelocities(Contact *c, unsigned numContacts, marb duration);
    };

} // end marballs namespace



#endif // CONTACT_INCLUDED
