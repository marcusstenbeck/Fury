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
	
	/*
	// Skapar en vektor som visar förändringen i hastighet i "world space" 
	// för en enhetsimpuls i kontaktnormalens riktning.
	Ogre::Vector3 deltaVelWorld = relativeContactPosition[0].crossProduct(contactNormal);
	deltaVelWorld = inverseInertiaTensor[0].transform(deltaVelWorld);
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
		deltaVelWorld = inverseInertiaTensor[1].transform(deltaVelWorld);
		deltaVelWorld = deltaVelWorld.crossProduct(relativeContactPosition[1]);
		// Lägg till hastighetsförändringen orsakad utav rotationen.
		deltaVelocity = deltaVelocity + deltaVelWorld.dotProduct(contactNormal);
		// Lägg till hastighetsförändringen orsakad av linjära rörelser.
		deltaVelocity += body[1]->getInverseMass();
	}
	 
	 */
}
