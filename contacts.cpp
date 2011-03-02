/*
 *  contacts.cpp
 *  tnm085
 *
 *  Created by Oscar Persson on 2011-02-07.
 *  Copyright 2011 Linköpings Univeristet. All rights reserved.
 *
 */

#include "contacts.h"

using namespace fury;

void Contact::setBodyData(RigidBody* one, RigidBody *two,
                          real friction, real restitution)
{
    Contact::body[0] = one;
    Contact::body[1] = two;
    Contact::friction = friction;
    Contact::restitution = restitution;
}

/**
 * Constructs an arbitrary orthonormal basis for the contact.
 * This is stored as a 3x3 matrix, where each vector is a column
 * (in other words the matrix transforms contact space into world
 * space). The x direction is generated from the contact normal,
 * and the y and z directions are set so they are at right angles to
 * it.
 */
void Contact::calculateContactBasis()
{
	Ogre::Vector3 contactTangent[2];
	// Kollar ifall Z-axeln ligger närmst X-axeln eller Y-axeln.
	if(abs(contactNormal.x) > abs(contactNormal.y))
	{
		// Skalning, för att försäkra oss om att resultaten är normaliserade.
		const real s = (real)1.0f/
		sqrt(contactNormal.z*contactNormal.z +
			 contactNormal.x*contactNormal.x);
		// Den nya X-axeln ska ha räta vinklar mot världens Y-axel.
		contactTangent[0].x = contactNormal.z*s;
		contactTangent[0].y = 0;
		contactTangent[0].z = -contactNormal.x*s;
		// Den nya Y-axeln ska ha räta vinklar mot de nya X och Z-axlarna.
		contactTangent[1].x = contactNormal.y*contactTangent[0].x;
		contactTangent[1].y = contactNormal.z*contactTangent[0].x - contactNormal.x*contactTangent[0].z;
		contactTangent[1].z = -contactNormal.y*contactTangent[0].x;
	}
	else
	{
		// Skalning, för att försäkra oss om att resultaten är normaliserade.
		const real s = (real)1.0/
		sqrt(contactNormal.z*contactNormal.z +
			 contactNormal.y*contactNormal.y);
		// Den nya X-axeln ska ha räta vinklar mot världens X-axel.
		contactTangent[0].x = 0;
		contactTangent[0].y = -contactNormal.z*s;
		contactTangent[0].z = contactNormal.y*s;
		// Den nya Y-axeln ska ha räta vinklar mot de nya X och Z-axlarna.
		contactTangent[1].x = contactNormal.y*contactTangent[0].z - contactNormal.z*contactTangent[0].y;
		contactTangent[1].y = -contactNormal.x*contactTangent[0].z;
		contactTangent[1].z = contactNormal.x*contactTangent[0].y;
	}
	// Skapa en matris av de 3 vektorerna.
	contactToWorld.FromAxes(contactNormal, contactTangent[0], contactTangent[1]);
	
	
	
}


Ogre::Vector3 Contact::calculateFrictionlessImpulse(Ogre::Matrix3 * inverseInertiaTensor)
{
	Ogre::Vector3 impulseContact;
	
	// Skapar en vektor som visar förändringen i hastighet i "world space" 
	// för en enhetsimpuls i kontaktnormalens riktning.
	Ogre::Vector3 deltaVelWorld = relativeContactPosition[0].crossProduct(contactNormal);
	deltaVelWorld = inverseInertiaTensor[0]*deltaVelWorld;
	deltaVelWorld = deltaVelWorld.crossProduct(relativeContactPosition[0]);
	
	// Ta reda på hastighetsförändringen i kontaktkoordinater.
	real deltaVelocity = deltaVelWorld.dotProduct(contactNormal);
	
	// Lägg till den linjära delen av hastighetsförändringen.
	deltaVelocity += body[0]->getInverseMass();
	
	// Kolla om vi behöver ta hänsyn till det andra objektets data.
	if (body[1])
	{
		// Hitta "inertia tensor" för detta objekt.
		body[1]->getInverseInertiaTensorWorld(&inverseInertiaTensor[1]);
		// Gå igenom samma transformationssekvens igen.
		Ogre::Vector3 deltaVelWorld = relativeContactPosition[1].crossProduct(contactNormal);
		deltaVelWorld = inverseInertiaTensor[1]*deltaVelWorld;
		deltaVelWorld = deltaVelWorld.crossProduct(relativeContactPosition[1]);
		// Lägg till hastighetsförändringen orsakad utav rotationen.
		deltaVelocity = deltaVelocity + deltaVelWorld.dotProduct(contactNormal);
		// Lägg till hastighetsförändringen orsakad av linjära rörelser.
		deltaVelocity += body[1]->getInverseMass();
	}
	
	// Beräkna storleken av kraftimpulsen som behövs för att stoppa kollisionen.
    impulseContact.x = desiredDeltaVelocity / deltaVelocity;
    impulseContact.y = 0;
    impulseContact.z = 0;
    return impulseContact;
}


void Contact::applyVelocityChange(Ogre::Vector3 velocityChange[2], Ogre::Vector3 rotationChange[2])
{
    // Get hold of the inverse mass and inverse inertia tensor, both in
    // world coordinates.
	Ogre::Matrix3 inverseInertiaTensor[2];
    body[0]->getInverseInertiaTensorWorld(&inverseInertiaTensor[0]);
    if (body[1])
        body[1]->getInverseInertiaTensorWorld(&inverseInertiaTensor[1]);
	
    // Beräkna impulsen för varje kontaktaxel.
	Ogre::Vector3 impulseContact;
	
	// Om friktionen är 0 använd funktionen som behandlar
	// friktionsfri impluls. 
    if (friction == (real)0.0)
    {
        impulseContact = calculateFrictionlessImpulse(inverseInertiaTensor);
    }
	
	// Använd annars den mer komplexa beräkningen med friktion. 
    else
    {
        //impulseContact = calculateFrictionImpulse(inverseInertiaTensor);
    }
	
    // Konvertera impulsen till världskoordinater.
	Ogre::Vector3 impulse = contactToWorld*impulseContact;
	
    // Dela upp impulsen i linjära och rotationskomponenter.
	Ogre::Vector3 impulsiveTorque = relativeContactPosition[0].crossProduct(impulse);
    rotationChange[0] = inverseInertiaTensor[0]*impulsiveTorque;
    velocityChange[0] = Ogre::Vector3(0,0,0);
    //velocityChange[0].addScaledVector(impulse, body[0]->getInverseMass());
	velocityChange[1] += impulse*(-body[0]->getInverseMass());
	
    // Sätt in förändringar.
    body[0]->addVelocity(velocityChange[0]);
    body[0]->addRotation(rotationChange[0]);
	
	// Om det finns en andra kropp.
    if (body[1])
    {
        // Dela upp impulsen i linjära och rotationskomponenter.
		Ogre::Vector3 impulsiveTorque = impulse.crossProduct(relativeContactPosition[1]);
        rotationChange[1] = inverseInertiaTensor[1]*impulsiveTorque;
        velocityChange[1] = Ogre::Vector3(0,0,0);
        //velocityChange[1].addScaledVector(impulse, -body[1]->getInverseMass());
		velocityChange[1] += impulse*(-body[1]->getInverseMass());
		
        // Sätt in förändringar.
        body[1]->addVelocity(velocityChange[1]);
        body[1]->addRotation(rotationChange[1]);
    }
<<<<<<< HEAD
}


/*Ogre::Vector3 Contact::calculateFrictionImpulse(Ogre::Matrix3 * inverseInertiaTensor)
{
	Ogre::Vector3 impulseContact;
    real inverseMass = body[0]->getInverseMass();
	
    // The equivalent of a cross product in matrices is multiplication
    // by a skew symmetric matrix - we build the matrix for converting
    // between linear and angular quantities.
	Ogre::Matrix3 impulseToTorque;
    impulseToTorque.setSkewSymmetric(relativeContactPosition[0]);
	
    // Build the matrix to convert contact impulse to change in velocity
    // in world coordinates.
	Ogre::Matrix3 deltaVelWorld = impulseToTorque;
    deltaVelWorld *= inverseInertiaTensor[0];
    deltaVelWorld *= impulseToTorque;
    deltaVelWorld *= -1;
	
    // Check if we need to add body two's data
    if (body[1])
    {
        // Set the cross product matrix
        impulseToTorque.setSkewSymmetric(relativeContactPosition[1]);
		
        // Calculate the velocity change matrix
		Ogre::Matrix3 deltaVelWorld2 = impulseToTorque;
        deltaVelWorld2 *= inverseInertiaTensor[1];
        deltaVelWorld2 *= impulseToTorque;
        deltaVelWorld2 *= -1;
		
        // Add to the total delta velocity.
        deltaVelWorld += deltaVelWorld2;
		
        // Add to the inverse mass
        inverseMass += body[1]->getInverseMass();
    }
	
    // Do a change of basis to convert into contact coordinates.
	Ogre::Matrix3 deltaVelocity = contactToWorld.transpose();
    deltaVelocity *= deltaVelWorld;
    deltaVelocity *= contactToWorld;
	
    // Add in the linear velocity change
    deltaVelocity.data[0] += inverseMass;
    deltaVelocity.data[4] += inverseMass;
    deltaVelocity.data[8] += inverseMass;
	
    // Invert to get the impulse needed per unit velocity
	Ogre::Matrix3 impulseMatrix = deltaVelocity.inverse();
	
    // Find the target velocities to kill
	Ogre::Vector3 velKill(desiredDeltaVelocity,
					-contactVelocity.y,
					-contactVelocity.z);
	
    // Find the impulse to kill target velocities
    impulseContact = impulseMatrix.transform(velKill);
	
    // Check for exceeding friction
    real planarImpulse = real_sqrt(
								   impulseContact.y*impulseContact.y +
								   impulseContact.z*impulseContact.z
								   );
    if (planarImpulse > impulseContact.x * friction)
    {
        // We need to use dynamic friction
        impulseContact.y /= planarImpulse;
        impulseContact.z /= planarImpulse;
		
        impulseContact.x = deltaVelocity.data[0] +
		deltaVelocity.data[1]*friction*impulseContact.y +
		deltaVelocity.data[2]*friction*impulseContact.z;
        impulseContact.x = desiredDeltaVelocity / impulseContact.x;
        impulseContact.y *= friction * impulseContact.x;
        impulseContact.z *= friction * impulseContact.x;
    }
    return impulseContact;
}

void Contact::setSkewSymmetric(const Ogre::Vector3 v)
{
	this[0] = data[4] = data[8] = 0;
	data[1] = -v.z;
	data[2] = v.y;
	data[3] = v.z;
	data[5] = -v.x;
	data[6] = -v.y;
	data[7] = v.x;
}*/


/*void Contact::applyImpulse(const Ogre::Vector3 &impulse, RigidBody *body, Ogre::Vector3 *velocityChange, Ogre::Vector3 *rotationChange)
{
	body->velocity += velocityChange;
	body->angularVelocity += rotationChange;
}*/
=======
}


/*Ogre::Vector3 Contact::calculateFrictionImpulse(Ogre::Matrix3 * inverseInertiaTensor)
 {
 Ogre::Vector3 impulseContact;
 real inverseMass = body[0]->getInverseMass();
 
 // The equivalent of a cross product in matrices is multiplication
 // by a skew symmetric matrix - we build the matrix for converting
 // between linear and angular quantities.
 Ogre::Matrix3 impulseToTorque;
 impulseToTorque.setSkewSymmetric(relativeContactPosition[0]);
 
 // Build the matrix to convert contact impulse to change in velocity
 // in world coordinates.
 Ogre::Matrix3 deltaVelWorld = impulseToTorque;
 deltaVelWorld *= inverseInertiaTensor[0];
 deltaVelWorld *= impulseToTorque;
 deltaVelWorld *= -1;
 
 // Check if we need to add body two's data
 if (body[1])
 {
 // Set the cross product matrix
 impulseToTorque.setSkewSymmetric(relativeContactPosition[1]);
 
 // Calculate the velocity change matrix
 Ogre::Matrix3 deltaVelWorld2 = impulseToTorque;
 deltaVelWorld2 *= inverseInertiaTensor[1];
 deltaVelWorld2 *= impulseToTorque;
 deltaVelWorld2 *= -1;
 
 // Add to the total delta velocity.
 deltaVelWorld += deltaVelWorld2;
 
 // Add to the inverse mass
 inverseMass += body[1]->getInverseMass();
 }
 
 // Do a change of basis to convert into contact coordinates.
 Ogre::Matrix3 deltaVelocity = contactToWorld.transpose();
 deltaVelocity *= deltaVelWorld;
 deltaVelocity *= contactToWorld;
 
 // Add in the linear velocity change
 deltaVelocity.data[0] += inverseMass;
 deltaVelocity.data[4] += inverseMass;
 deltaVelocity.data[8] += inverseMass;
 
 // Invert to get the impulse needed per unit velocity
 Ogre::Matrix3 impulseMatrix = deltaVelocity.inverse();
 
 // Find the target velocities to kill
 Ogre::Vector3 velKill(desiredDeltaVelocity,
 -contactVelocity.y,
 -contactVelocity.z);
 
 // Find the impulse to kill target velocities
 impulseContact = impulseMatrix.transform(velKill);
 
 // Check for exceeding friction
 real planarImpulse = real_sqrt(
 impulseContact.y*impulseContact.y +
 impulseContact.z*impulseContact.z
 );
 if (planarImpulse > impulseContact.x * friction)
 {
 // We need to use dynamic friction
 impulseContact.y /= planarImpulse;
 impulseContact.z /= planarImpulse;
 
 impulseContact.x = deltaVelocity.data[0] +
 deltaVelocity.data[1]*friction*impulseContact.y +
 deltaVelocity.data[2]*friction*impulseContact.z;
 impulseContact.x = desiredDeltaVelocity / impulseContact.x;
 impulseContact.y *= friction * impulseContact.x;
 impulseContact.z *= friction * impulseContact.x;
 }
 return impulseContact;
 }
 
 void Contact::setSkewSymmetric(const Ogre::Vector3 v)
 {
 this[0] = data[4] = data[8] = 0;
 data[1] = -v.z;
 data[2] = v.y;
 data[3] = v.z;
 data[5] = -v.x;
 data[6] = -v.y;
 data[7] = v.x;
 }*/


/*void Contact::applyImpulse(const Ogre::Vector3 &impulse, RigidBody *body, Ogre::Vector3 *velocityChange, Ogre::Vector3 *rotationChange)
 {
 body->velocity += velocityChange;
 body->angularVelocity += rotationChange;
 }*/
>>>>>>> 49cf9b07c32ac543fcf2288d541def1a512e7eea
