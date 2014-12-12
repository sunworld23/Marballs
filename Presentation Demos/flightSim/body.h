/*************************************************************
 * body.h
 * -------------
 * Header file that defines the body for rigid bodies.
 *
 * Last Revision: Oct. 25, 2014
 *
 * TO DO: - Continue following book to fill this out.
 *		  - Additional commenting/polish format
 *************************************************************/

#ifndef MARBALLS_BODY_H
#define MARBALLS_BODY_H

#include "engine_core.h"

namespace marballs {

	class RigidBody{

		//Variable declarations
		public:
		marb inverseMass;		//Holds inverse mass of the rigid body
		Vector3 position;		//Holds linear position of the rigid body
		Quaternion orientation;	//Holds angular orientation of the rigid body
		Vector3 velocity;		//Holds the velocity of the rigid body
		Vector3 rotation;		//Holds the rotation of the rigid body
		Matrix4 transformMatrix;//Holds matrix to convert body space to world space and vice versa
		marb angularDamping; 	//Holds damping applied to angular motion
		marb linearDamping;

        marb motion;
        Vector3 forceAccum;
        Vector3 torqueAccum;
		Matrix3 inverseInertiaTensor;
		Matrix3 inverseInertiaTensorWorld;
        Vector3 acceleration;
		Vector3 lastFrameAcceleration;

		bool isAwake;
		bool canSleep;

		//Function declarations
		public:
        void Integrate(marb duration);
		void CalculateDerivedData();
        void AddForce(const Vector3 &force);
        void AddForceAtPoint(const Vector3 &force, const Vector3 &point);
        void AddForceAtBodyPoint(const Vector3 &force, const Vector3 &point);
        void AddTorque(const Vector3 &torque);

        // Get and set the mass of a rigid body
        void SetMass(const marb mass);
        marb GetMass() const;

        // Get and set the inverse mass
        void SetInverseMass(const marb inverseMass);
        marb GetInverseMass() const;

        // Get and set inertia tensor of rigid body
        void SetInertiaTensor(const Matrix3 &inertiaTensor);
        void GetInertiaTensor(Matrix3 *inertiaTensor) const;
        Matrix3 GetInertiaTensor() const;

        // Get and set inverse inertia tensor
        void SetInverseInertiaTensor(const Matrix3 &inverseInertiaTensor);
        void GetInverseInertiaTensor(Matrix3 *inverseInertiaTensor) const;

        void SetDamping(const marb linearDamping, const marb angularDamping);
        void SetLinearDamping(const marb linearDamping);
        marb GetLinearDamping() const;

        void SetAngularDamping(const marb angularDamping);
        marb GetAngularDamping() const;

        void GetInertiaTensorWorld(Matrix3 *inertiaTensor) const;
        Matrix3 GetInertiaTensorWorld() const;

        Matrix3 GetInverseInertiaTensor() const;
        void GetInverseInertiaTensorWorld(Matrix3 *inverseInertiaTensor) const;
        Matrix3 GetInverseInertiaTensorWorld() const;

        // Sets the position of the rigid body
        void SetPosition(const Vector3 &position);
        void SetPosition(const marb x, const marb y, const marb z);
        void GetPosition(Vector3 *position) const;
        Vector3 GetPosition() const;

        // Sets the orientation of the rigid body.
        void SetOrientation(const Quaternion &orientation);
        void SetOrientation(const marb r, const marb i,
            const marb j, const marb k);
        void GetOrientation(Quaternion *orientation) const;
        Quaternion GetOrientation() const;

        void GetOrientation(Matrix3 *matrix) const;
        void GetOrientation(marb matrix[9]) const;
        void GetTransform(Matrix4 *transform) const;
        void GetTransform(marb matrix[16]) const;
        void GetGLTransform(float matrix[16]) const;
        Matrix4 GetTransform() const;

        Vector3 GetPointInLocalSpace(const Vector3 &point) const;
        Vector3 GetPointInWorldSpace(const Vector3 &point) const;
        Vector3 GetDirectionInLocalSpace(const Vector3 &direction) const;
        Vector3 GetDirectionInWorldSpace(const Vector3 &direction) const;

        // Sets the velocity of the rigid body.
        void SetVelocity(const Vector3 &velocity);
        void SetVelocity(const marb x, const marb y, const marb z);
        void GetVelocity(Vector3 *velocity) const;
        Vector3 GetVelocity() const;
        void AddVelocity(const Vector3 &deltaVelocity);

        // Sets the rotation of the rigid body
        void SetRotation(const Vector3 &rotation);
        void SetRotation(const marb x, const marb y, const marb z);
        void GetRotation(Vector3 *rotation) const;
        Vector3 GetRotation() const;
        void AddRotation(const Vector3 &deltaRotation);

        /**
         * Returns true if the body is awake and responding to
         * integration.
         *
         * @return The awake state of the body.
         */
        bool GetAwake() const
        {
            return isAwake;
        }
        void SetAwake(const bool awake=true);

        /**
         * Returns true if the body is allowed to go to sleep at
         * any time.
         */
        bool GetCanSleep() const
        {
            return canSleep;
        }
        void SetCanSleep(const bool canSleep=true);

        /**
         * @name Retrieval Functions for Dynamic Quantities
         *
         * These functions provide access to the acceleration
         * properties of the body. The acceleration is generated by
         * the simulation from the forces and torques applied to the
         * rigid body. Acceleration cannot be directly influenced, it
         * is set during integration, and represent the acceleration
         * experienced by the body of the previous simulation step.
         */
        void GetLastFrameAcceleration(Vector3 *linearAcceleration) const;
        Vector3 GetLastFrameAcceleration() const;

        /**
         * Force, Torque and Acceleration Set-up Functions
         *
         * These functions set up forces and torques to apply to the
         * rigid body.
         */

        // Clears the forces and torques in the accumulators
        void ClearAccumulators();

        // Get and sets the constant acceleration of the rigid body
        void SetAcceleration(const Vector3 &acceleration);
        void SetAcceleration(const marb x, const marb y, const marb z);
        void GetAcceleration(Vector3 *acceleration) const;
        Vector3 GetAcceleration() const;

        bool HasFiniteMass() const;
	};

}

#endif
