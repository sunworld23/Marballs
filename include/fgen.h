/*************************************************************
 * fgen.h
 * -------------
 * Header file that defines force generators which
 * add forces to one or more rigidbodies.
 *
 * Last Revision: Nov. 16, 2014
 *
 * NOTE: This is the one RigidBodies use!
 *
 * TO DO: - Debug.
 *
 *************************************************************/

#ifndef FGEN_INCLUDED
#define FGEN_INCLUDED

#include "body.h"
#include "pfgen.h"
#include <vector>

namespace marballs {

    // A force generator can be asked to add a force to one or more bodies
    class ForceGenerator {
        public:

        // UpdateForce - Overload this in implementations of the interface
        // to calculate and update the force applied to the given rigid body
        virtual void UpdateForce(RigidBody *body, marb duration) = 0;
    };

    // A force generator that applies to a gravitational force.
    // One instance can be used for multiple rigid bodies.
    class Gravity : public ForceGenerator {

        Vector3 gravity;  // Holds the acceleration due to gravity

        public:

            // Gravity - Creates the generator with the given acceleration.
            Gravity(const Vector3 &gravity);

            // UpdateForce - Applies the gravitational force to the given rigid body.
            virtual void UpdateForce(RigidBody *body, marb duration);
    };


    // A force generator that applies a Spring force.
    class Spring : public ForceGenerator {

        Vector3 connectionPoint; // The point of connection of the spring (in local coordinates)
        Vector3 otherConnectionPoint; // The point of connection of the spring to the other object,
                                      // in that object's local coordinates.
        RigidBody *other; // The particle at the other end of the spring
        marb springConstant; // Holds the spring constant
        marb restLength; // Holds the rest length of the spring

    public:

        // Spring - Constructor, Creates a new spring with the given parameters.
        Spring(const Vector3 &localConnectionPt,
               RigidBody *other,
               const Vector3 &otherConnectionPt,
               marb springConstant,
               marb restLength);

        // UpdateForce - Applies the spring force to the given rigid body.
        virtual void UpdateForce(RigidBody *body, marb duration);
    };

    // A force generator showing a three component explosion effect.
    // This force generator is intended to represent a single
    // explosion effect for multiple rigid bodies. The force generator
    // can also act as a particle force generator.
    class Explosion : public ForceGenerator,
                      public ParticleForceGenerator {

        marb timePassed; // Tracks how long the explosion has been in operation
                         // NOTE - used for time-sensitive effects

    public:
        // Properties of the explosion, these are public because
        // there are so many and providing a suitable constructor
        // would be cumbersome:

        Vector3 detonation; // The location of the detonation of the weapon
        marb implosionMaxRadius; // The radius up to which objects implode in the first stage of the explosion
        marb implosionMinRadius; // The radius in which objects don't feel the implosion
                                 // force. Objects near the detonation aren't sucked in
                                 // by the air implosion
        marb implosionDuration; // The length of time that objects spend imploding before concussion phase
        marb implosionForce; // The maximal force that implosions can imply, should be small
                             // so that objects don't pass the detonation point
        marb shockwaveSpeed; // The speed that the shock wave is traveling, this is related to thickness
                             // NOTE - thickness >= speed * minimum frame duration
        marb shockwaveThickness; // The thickness the shock wave applies its force (Faster requires thicker)
        marb peakConcussionForce; // Force applied at the very center of the concussion wave.
                                   // objects moving away get proportionally less force, objects
                                   // moving in get proportionally more force
        marb concussionDuration; // The length of time that concussion wave is active.
                                  // As wave length nears this, force applied is reduced
        marb peakConvectionForce; // Peak force for stationary objects in center of convection chimney
        marb chimneyRadius; // The radius of the chimney cylinder in the XZ plane
        marb chimneyHeight; // Maximum height of the chimney
        marb convectionDuration; // Length of time the convection chimney is active

    public:

        // Explosion - Constructor, Creates a new explosion with sensible default values.
        Explosion();

        // UpdateForce - Calculates and applies the force that the explosion
        // has on the given rigid body.
        virtual void UpdateForce(RigidBody * body, marb duration);

        // UpdateForce - Calculates and applies the force that the explosion
        // has on the given particle.
        virtual void UpdateForce(Particle *particle, marb duration) = 0;

    };

    // A force generator that applies an aerodynamic force.
    class Aero : public ForceGenerator {

        protected:
            Matrix3 tensor; // Holds the aerodynamic tensor for the surface in body space
            Vector3 position; // Holds the relative position of the aerodynamic surface in body coords
            const Vector3* windspeed; // Holds a pointer to a vector containing windspeed of environment

        public:

            // Aero - Constructor, Creates a new aerodynamic force generator with the
            // given properties.
            Aero(const Matrix3 &tensor, const Vector3 &position, const Vector3 *windspeed);

            // UpdateForce - Applies the force to the given rigid body.
            virtual void UpdateForce(RigidBody *body, marb duration);

        protected:

            // UpdateForceFromTensor - Uses an explicit tensor matrix to update the force on
            // the given rigid body. This is exactly the same as for updateForce
            // only it takes an explicit tensor.
            void UpdateForceFromTensor(RigidBody *body, marb duration,
                                       const Matrix3 &tensor);
    };

    // A force generator with a control aerodynamic surface. This
    // requires three inertia tensors, for the two extremes and
    // 'resting' position of the control surface.  The latter tensor is
    // the one inherited from the base class, the two extremes are
    // defined in this class.
    class AeroControl : public Aero {

        protected:
            Matrix3 maxTensor; // The aerodynamic tensor for the surface when control is at max value
            Matrix3 minTensor; // The aerodynamic tensor for the surface when control is at min value
            marb controlSetting; // The current position of the control for this surface. This
                                 // should range between -1 (in which case the minTensor value
                                 // is used), through 0 (where the base-class tensor value is
                                 // used) to +1 (where the maxTensor value is used).

        private:

            // GetTensor - Calculates the final aerodynamic tensor for the current
            // control setting.
            Matrix3 GetTensor();

        public:

            // AeroControl - Constructor, Creates a new aerodynamic control surface with the given
            // properties.
            AeroControl(const Matrix3 &base,
                        const Matrix3 &min, const Matrix3 &max,
                        const Vector3 &position, const Vector3 *windspeed);


            // SetControl - Sets the control position of this control. This * should
            // range between -1 (in which case the minTensor value is *
            // used), through 0 (where the base-class tensor value is used) *
            // to +1 (where the maxTensor value is used). Values outside that
            // * range give undefined results.
            void SetControl(marb value);

            // Applies the force to the given rigid body.
            virtual void UpdateForce(RigidBody *body, marb duration);
    };


    // A force generator with an aerodynamic surface that can be
    // re-oriented relative to its rigid body.
    class AngledAero : public Aero {

        Quaternion orientation; // Holds the orientation of the aerodynamic surface
                                // relative to the rigid body to which it is attached

    public:

        // AngledAero - Constructor, Creates a new aerodynamic surface with the given properties.
        AngledAero(const Matrix3 &tensor, const Vector3 &position, const Vector3 *windspeed);

        // SetOrientation - Sets the relative orientation of the aerodynamic surface,
        // relative to the rigid body it is attached to. Note that
        // this doesn't affect the point of connection of the surface
        // to the body.
        void SetOrientation(const Quaternion &quat);

        // Applies the force to the given rigid body.
        virtual void UpdateForce(RigidBody *body, marb duration);
    };


    // A force generator to apply a buoyant force to a rigid body.
    class Buoyancy : public ForceGenerator {

            marb maxDepth; // The maximum submersion depth of the object before it
                           // generates its maximum buoyancy force.
            marb volume; // Volume of the object
            marb waterHeight; // Height of the water plane above y = 0.
                              // NOTE - plane will be parallel to the XZ plane.
            marb liquidDensity; // The density of the liquid. Pure water has a density of
                                // 1000kg per cubic meter.
            Vector3 centreOfBuoyancy; // The center of buoyancy of the rigid body, in body coordinates

        public:

            // Buoyancy - Constructor, Creates a new buoyancy force with the given parameters.
            Buoyancy(const Vector3 &cOfB, marb maxDepth, marb volume, marb waterHeight, marb liquidDensity = 1000.0f);

            // Applies the force to the given rigid body.
            virtual void UpdateForce(RigidBody *body, marb duration);
    };

    // Holds all the force generators and the bodies they apply to.
    class ForceRegistry {

        protected:

            struct ForceRegistration { // Keeps track of one force generator and the body it applies to;
                RigidBody *body;
                ForceGenerator *fg;
            };

            // Holds the list of registrations.
            typedef std::vector<ForceRegistration> Registry;
            Registry registrations;

        public:

            // Add - Registers the given force generator to apply to the
            // given body.
            void Add(RigidBody* body, ForceGenerator *fg);

            // Remove - Removes the given registered pair from the registry.
            // If the pair is not registered, this method will have
            // no effect.
            void Remove(RigidBody* body, ForceGenerator *fg);

            // Clear - Clears all registrations from the registry. This will
            // not delete the bodies or the force generators
            // themselves, just the records of their connection.
            void Clear();

            // UpdateForces - Calls all the force generators to update the forces of
            // their corresponding bodies.
            void UpdateForces(marb duration);
    };
}

#endif // FGEN_INCLUDED
