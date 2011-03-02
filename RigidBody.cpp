/*
 *  PhysicsObject.cpp
 *  tnm085
 *
 *  Created by Marcus Stenbeck on 2011-01-31.
 *  Copyright 2011 Macatak Media. All rights reserved.
 *
 */

#include "RigidBody.h"
#include <iostream>

using namespace fury;

RigidBody::RigidBody()
{
	forceAccum = Ogre::Vector3(0.0, 0.0, 0.0);
	torqueAccum = Ogre::Vector3(0.0, 0.0, 0.0);
	position = Ogre::Vector3(0.0, 0.0, 0.0);
	angularVelocity = Ogre::Vector3(0.0, 0.0, 0.0);
	angularMomentum = Ogre::Vector3(0.0,0.0,0.0);
	orientation = Ogre::Quaternion(1.0, 0.0, 0.0, 0.0);
	setMass(1.0);
	velocity = Ogre::Vector3(0.0, 0.0, 0.0);
	damping = 1.0;
	angularDamping = 1.0;
	acceleration = Ogre::Vector3(0.0, 0.0, 0.0);
	
	calculateDerivedData();
}

void RigidBody::calculateDerivedData()
{
	orientation.normalise();
	
	Ogre::Matrix3 orientationMatrix;
	orientation.ToRotationMatrix(orientationMatrix);
	inverseInertiaTensorWorld = orientationMatrix * inverseInertiaTensor * orientationMatrix.Transpose();
	
	angularVelocity = (inverseInertiaTensorWorld * angularMomentum);
};

void RigidBody::setInertiaTensor(const Ogre::Matrix3& inertiaTensor)
{
	inverseInertiaTensor = inertiaTensor.Inverse();
};

void RigidBody::addForce(const Ogre::Vector3& force)
{
	forceAccum += force;
};

void RigidBody::addForceAtBodyPoint(const Ogre::Vector3 &force, const Ogre::Vector3 &point)
{
	
};

void RigidBody::addForceAtPoint(const Ogre::Vector3 &force, const Ogre::Vector3 &point)
{
	Ogre::Vector3 pt = point;
	
	// point relative to center of mass
	pt -= position;
	
	forceAccum += force;
	torqueAccum += pt.crossProduct(force);
	
	std::cout << "torqueAccum: ";
	std::cout << torqueAccum;
};

void RigidBody::clearAccumulators()
{
	forceAccum = Ogre::Vector3(0.0, 0.0, 0.0);
	torqueAccum = Ogre::Vector3(0.0, 0.0, 0.0);
};

void RigidBody::integrate(real duration)
{	
	calculateDerivedData();
	
	assert(duration >= 0.0);
	
	//Work out the acceleration from the force
	Ogre::Vector3 resultingAcc = acceleration;
	resultingAcc += forceAccum*inverseMass;
	angularMomentum += torqueAccum*duration;
	
	//Update linear velocity from the acceleration
	velocity += resultingAcc*duration;	
	
	//Drag 
	velocity *= Ogre::Math::Pow(damping, duration);
	angularVelocity = angularVelocity * Ogre::Math::Pow(angularDamping, duration);
	
	// Update linear position
	position += velocity*duration;
	
	
	// Update angular position
	Ogre::Quaternion qtemp = Ogre::Quaternion(0.0, angularVelocity.x * duration, angularVelocity.y * duration, angularVelocity.z * duration);
	qtemp = qtemp * orientation;
	orientation.w += qtemp.w * 0.5;
	orientation.x += qtemp.x * 0.5;
	orientation.y += qtemp.y * 0.5;
	orientation.z += qtemp.z * 0.5;
	
	//Clear accumulator
	clearAccumulators();
};

bool RigidBody::hasFiniteMass()
{
	if ( inverseMass > 0.0 ) return true;

	return false;
};

real RigidBody::getMass()
{
	return 1.0/inverseMass;
};

void RigidBody::setMass(real mass)
{
	assert(mass > 0.0);
	
	inverseMass = 1/mass;
};


void RigidBody::setPosition(const Ogre::Vector3 &pos)
{
	position = pos;
}

real RigidBody::getInverseMass() const
{
    return inverseMass;
};

void RigidBody::getInverseInertiaTensorWorld(Ogre::Matrix3 *inverseInertiaTensor) const
{
    *inverseInertiaTensor = inverseInertiaTensorWorld;
};