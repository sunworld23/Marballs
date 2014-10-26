#ifndef WORLD_INCLUDED
#define WORLD_INCLUDED

/**
* The world represents an independent simulation of physics. It
* keeps track of a set of rigid bodies.
*/

namespace marballs
{
    class World
    {
        /**
        * Holds a single rigid body in a linked list of bodies.
        */
        struct BodyRegistration
        {
            RigidBody *body;
            BodyRegistration * next;
        };

        /**
        * Holds the first bodies.
        */
        BodyRegistration *firstBody;

        /**
        * Initializes the world for a simulation frame. This clears
        * the force and torque accumulators for bodies in the
        * world. After calling this, the bodies can have their forces
        * and torques for this frame added.
        */
        void startFrame();

        /**
        * Processes all the physics for the world.
        */
        void runPhysics(real duration);


    };
}


#endif // WORLD_INCLUDED
