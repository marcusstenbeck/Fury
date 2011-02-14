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

void RigidBody::calculateDerivedData()
{
	orientation.normalise();
	
//	_calculateTransformMatrix(transformMatrix, position, orientation);
	
//	_transformInertiaTensor(inverseInertiaTensorWorld, orientation, inverseInertiaTensor, transformMatrix);
	
	Ogre::Matrix3 orientationMatrix;
	orientation.ToRotationMatrix(orientationMatrix);
	inverseInertiaTensorWorld = orientationMatrix * inverseInertiaTensor * orientationMatrix.Transpose();
};

/** * Inline function that creates a transform matrix from a position * and orientation. */
/*static inline void _calculateTransformMatrix(Ogre::Matrix4 &transformMatrix, const Ogre::Vector3 &position, const Ogre::Quaternion &orientation)
{
	transformMatrix[0] = 1-2*orientation.y*orientation.y- 2*orientation.z*orientation.z;
	transformMatrix[1] = 2*orientation.x*orientation.y - 2*orientation.w*orientation.z;
	transformMatrix[2] = 2*orientation.x*orientation.z + 2*orientation.w*orientation.y;
	transformMatrix[3] = position.x;
	transformMatrix[4] = 2*orientation.x*orientation.y + 2*orientation.w*orientation.z;
	transformMatrix[5] = 1-2*orientation.x*orientation.x- 2*orientation.z*orientation.z;
	transformMatrix[6] = 2*orientation.y*orientation.z - 2*orientation.w*orientation.x;
	transformMatrix[7] = position.y;
	transformMatrix[8] = 2*orientation.x*orientation.z - 2*orientation.w*orientation.y;
	transformMatrix[9] = 2*orientation.y*orientation.z + 2*orientation.w*orientation.x;
	transformMatrix[10] = 1-2*orientation.x*orientation.x- 2*orientation.y*orientation.y;
	transformMatrix[11] = position.z;
}

static inline void _transformInertiaTensor(Ogre::Matrix3 &iitWorld, const Ogre::Quaternion &q, const Ogre::Matrix3 &iitBody, const Ogre::Matrix4 &rotmat)
{
	real t4 = rotmat[0]*iitBody[0]+ rotmat[1]*iitBody[3]+ rotmat[2]*iitBody[6];
	real t9 = rotmat[0]*iitBody[1]+ rotmat[1]*iitBody[4]+ rotmat[2]*iitBody[7];
	real t14 = rotmat[0]*iitBody[2]+ rotmat[1]*iitBody[5]+ rotmat[2]*iitBody[8];
	real t28 = rotmat[4]*iitBody[0]+ rotmat[5]*iitBody[3]+ rotmat[6]*iitBody[6];
	real t33 = rotmat[4]*iitBody[1]+ rotmat[5]*iitBody[4]+ rotmat[6]*iitBody[7];
	real t38 = rotmat[4]*iitBody[2]+ rotmat[5]*iitBody[5]+ rotmat[6]*iitBody[8];
	real t52 = rotmat[8]*iitBody[0]+ rotmat[9]*iitBody[3]+ rotmat[10]*iitBody[6];
	real t57 = rotmat[8]*iitBody[1]+ rotmat[9]*iitBody[4]+ rotmat[10]*iitBody[7];
	real t62 = rotmat[8]*iitBody[2]+ rotmat[9]*iitBody[5]+ rotmat[10]*iitBody[8];
	
	iitWorld[0] = t4*rotmat[0]+ t9*rotmat[1]+ t14*rotmat[2];
	iitWorld[1] = t4*rotmat[4]+ t9*rotmat[5]+ t14*rotmat[6];
	iitWorld[2] = t4*rotmat[8]+ t9*rotmat[9]+ t14*rotmat[10];
	iitWorld[3] = t28*rotmat[0]+ t33*rotmat[1]+ t38*rotmat[2];
	iitWorld[4] = t28*rotmat[4]+ t33*rotmat[5]+ t38*rotmat[6];
	iitWorld[5] = t28*rotmat[8]+ t33*rotmat[9]+ t38*rotmat[10];
	iitWorld[6] = t52*rotmat[0]+ t57*rotmat[1]+ t62*rotmat[2];
	iitWorld[7] = t52*rotmat[4]+ t57*rotmat[5]+ t62*rotmat[6];
	iitWorld[8] = t52*rotmat[8]+ t57*rotmat[9]+ t62*rotmat[10];
};
*/

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
	assert(duration >= 0.0);
	
	//Work out the acceleration from the force
	Ogre::Vector3 resultingAcc = acceleration;
	resultingAcc += forceAccum*inverseMass;
	angularMomentum += torqueAccum*duration;
	
	angularVelocity = (inverseInertiaTensorWorld * angularMomentum);
	
	std::cout << "angularVelocity: ";
	std::cout << angularVelocity << std::endl;
	
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
	
	calculateDerivedData();
	
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

real RigidBody::getInverseMass() const
{
    return inverseMass;
};

void RigidBody::getInverseInertiaTensorWorld(Ogre::Matrix3 *inverseInertiaTensor) const
{
    *inverseInertiaTensor = inverseInertiaTensorWorld;
};