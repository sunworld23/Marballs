/*************************************************************
 * body.h
 * -------------
 * Header file that defines the body for rigid bodies.
 *
 * Last Revision: Nov. 16, 2014
 *
 * TO DO: - Continue following book to fill this out.
 *		  - Add/reformat additional comments?
 *************************************************************/

#ifndef MARBALLS_BODY_INCLUDED
#define MARBALLS_BODY_INCLUDED

#include "engine_core.h"

namespace marballs {

	class RigidBody {
		// Variable declarations
		public:
		marb inverseMass;			// Holds inverse mass of the rigid body
		Vector3 position;			// Holds linear position of the rigid body
		Quaternion orientation;		// Holds angular orientation of the rigid body
		Vector3 velocity;			// Holds the velocity of the rigid body
		Vector3 rotation;			// Holds the rotation of the rigid body
		Matrix4 transformMatrix;	// Holds matrix to convert body space to world space and vice versa
		marb angularDamping; 		// Holds damping applied to angular motion
		marb linearDamping;

									// Need comments for the below variables too!
        marb motion;
        Vector3 forceAccum;
        Vector3 torqueAccum;
		Matrix3 inverseInertiaTensor;
		Matrix3 inverseInertiaTensorWorld;
        Vector3 acceleration;
		Vector3 lastFrameAcceleration;

		bool isAwake;	// Whether the body is awake and mobile or not.
		bool canSleep;	// Whether the body can be put to sleep or not.

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
        void SetOrientation(const marb r, const marb i, const marb j, const marb k);
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

        // GetAwake - returns whether or not the body is awake and responding to integration
        bool GetAwake() const { return isAwake;}
        void SetAwake(const bool awake=true);

        // GetCanSleep - return true if body is allowed to sleep
        bool GetCanSleep() const { return canSleep; }
        void SetCanSleep(const bool canSleep=true);

        // Both functions provide access to acceleration properties of the body.
        // Acceleration is generated by simulation of forces and torques applied
        // to the rigid body. Acceleration cannot be directly influenced and is
        // only set during the integration.
        void GetLastFrameAcceleration(Vector3 *linearAcceleration) const;
        Vector3 GetLastFrameAcceleration() const;

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
