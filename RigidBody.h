/*
 *  PhysicsObject.h
 *  tnm085
 *
 *  Created by Marcus Stenbeck on 2011-01-31.
 *  Copyright 2011 Macatak Media. All rights reserved.
 *
 */

#ifndef PHYSICSOBJECT_H
#define PHYSICSOBJECT_H

#include <vector>
#include "particle.h"

namespace fury
{
	class RigidBody
	{		
	public:

		Ogre::Vector3 position;
		Ogre::Quaternion orientation;
		Ogre::Vector3 velocity;
		Ogre::Vector3 acceleration;		
		Ogre::Vector3 lastFrameAcceleration;
		Ogre::Quaternion rotation; //angular velocity!
//		Ogre::Matrix4 transformMatrix;
		Ogre::Matrix3 inverseInertiaTensor;
		Ogre::Matrix3 inverseInertiaTensorWorld;
		
		real damping;
		real angularDamping;
		real inverseMass;
		
		//??
		Ogre::Vector3 forceAccum;
		Ogre::Vector3 torqueAccum;		
		
		
		void calculateDerivedData();
		void setInertiaTensor(const Ogre::Matrix3& inertiaTensor);
		void addForce(const Ogre::Vector3& force);
		void addForceAtBodyPoint(const Ogre::Vector3 &force, const Ogre::Vector3 &point);
		void addForceAtPoint(const Ogre::Vector3 &force, const Ogre::Vector3 &point);
		void integrate(real duration);
		void clearAccumulators();
		bool hasFiniteMass();
		real getMass();
		void setMass(real mass);
	};
}

#endif