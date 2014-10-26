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
		
		//Function declarations
		public:
		void calculateDerivedData();
		Matrix3 inverseInertiaTensor;
		void addForce(const Vector3 &force);
		void addForceAtBodyPoint(const Vector3 &force, const Vector3 &point);
	};

}