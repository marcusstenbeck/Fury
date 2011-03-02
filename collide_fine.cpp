/*
 *  collide_fine.cpp
 *  tnm085
 *
 *  Created by Oscar Persson on 2011-02-08.
 *  Copyright 2011 Linköpings Univeristet. All rights reserved.
 *
 */

#include "collide_fine.h"


using namespace fury;

// **
//
// Box och plan.
//
// **
unsigned CollisionDetector::boxAndHalfSpace(const Box &box, const Plane &plane, CollisionData *data)
{
	// Varje kombination av + och - för varje halfSize.
	static real mults[8][3] = {{1,1,1},{-1,1,1},{1,-1,1},{-1,-1,1},
		{1,1,-1},{-1,1,-1},{1,-1,-1},{-1,-1,-1}};

	Contact* contact = data->contacts;
	unsigned contactsUsed = 0;
	for (unsigned i = 0; i < 8; i++) 
	{
	
		// Beräkna position för varje vertex.
		Ogre::Vector3 vertexPos(mults[i][0], mults[i][1], mults[i][2]);
		//vertexPos.componentProductUpdate(box.halfSize);
		vertexPos *= box.halfSize; 
		vertexPos = box.transform.transformAffine(vertexPos);
		
		// Räknar ut avståndet till planet.
		real vertexDistance = vertexPos.absDotProduct(plane.normal);
		
		if (vertexDistance <= (plane.offset + data->tolerance))
		{
			// Skapa kontaktdatan.
			// Kontaktpunkten är halvvägs mellan punkten och planet.
			contact->contactPoint = plane.normal;
			contact->contactPoint *= (vertexDistance - plane.offset);
			contact->contactPoint = vertexPos;
			contact->contactNormal = plane.normal;
			contact->penetration = plane.offset - vertexDistance;
			
			// Skriv in data.
			contact->setBodyData(box.body, NULL, data->friction, data->restitution);
			
			// Gå vidare till nästa kontakt.
            contact++;
            contactsUsed++;
            if (contactsUsed == data->contactsLeft) return contactsUsed;
		}
	}
	
	data->addContacts(contactsUsed);
    return contactsUsed;
}

// **
//
// Box och point.
//
// **
unsigned CollisionDetector::boxAndPoint(const Box &box,	const Ogre::Vector3 &point, CollisionData *data)
{
	//Transformera point till box-koordinater.
	Ogre::Vector3 relPt = box.transform.transformAffine(point);
	
	return 0;
}


static inline real transformToAxis(const Box &box, const Ogre::Vector3 &axis)
{
	//box.halfSize.x * real_abs(axis * box.getAxis(0)) + box.halfSize.y * real_abs(axis * box.getAxis(1)) + box.halfSize.z * real_abs(axis * box.getAxis(2));	
	return	box.halfSize.x * axis.absDotProduct(box.body->orientation.xAxis()) 
	+ box.halfSize.y * axis.absDotProduct(box.body->orientation.yAxis()) 
	+ box.halfSize.z * axis.absDotProduct(box.body->orientation.zAxis()); 
}

static inline bool overlapOnAxis(const Box &one, const Box &two,const Ogre::Vector3 &axis, const Ogre::Vector3 &toCenter)
{
	// Project the half-size of one onto axis.
	real oneProject = transformToAxis(one, axis);
	real twoProject = transformToAxis(two, axis);
	
	// Project this onto the axis.
	real distance = toCenter.absDotProduct(axis);
	// Check for overlap.
	return (distance < oneProject + twoProject);
}

#define TEST_OVERLAP(axis) overlapOnAxis(one, two, (axis), toCentre)
bool IntersectionTest::boxAndBox(const Box &one, const Box &two)
{
	// Find the vector between the two centre.
	Ogre::Vector3 toCentre = two.body->position - one.body->position;
	
    return (
			// Check on box one's axes first
			TEST_OVERLAP(one.body->orientation.xAxis()) &&
			TEST_OVERLAP(one.body->orientation.yAxis()) &&
			TEST_OVERLAP(one.body->orientation.zAxis()) &&
			
			// And on two's
			TEST_OVERLAP(two.body->orientation.xAxis()) &&
			TEST_OVERLAP(two.body->orientation.yAxis()) &&
			TEST_OVERLAP(two.body->orientation.zAxis()) &&
			
			// Now on the cross products
			TEST_OVERLAP(one.body->orientation.xAxis().crossProduct(two.body->orientation.xAxis())) &&
			TEST_OVERLAP(one.body->orientation.xAxis().crossProduct(two.body->orientation.yAxis())) &&
			TEST_OVERLAP(one.body->orientation.xAxis().crossProduct(two.body->orientation.zAxis())) &&
			TEST_OVERLAP(one.body->orientation.yAxis().crossProduct(two.body->orientation.xAxis())) &&
			TEST_OVERLAP(one.body->orientation.yAxis().crossProduct(two.body->orientation.yAxis())) &&
			TEST_OVERLAP(one.body->orientation.yAxis().crossProduct(two.body->orientation.zAxis())) &&
			TEST_OVERLAP(one.body->orientation.zAxis().crossProduct(two.body->orientation.xAxis())) &&
			TEST_OVERLAP(one.body->orientation.zAxis().crossProduct(two.body->orientation.yAxis())) &&
			TEST_OVERLAP(one.body->orientation.zAxis().crossProduct(two.body->orientation.zAxis()))
			//0
			);
}
#undef TEST_OVERLAP



/*unsigned CollisionDetector::boxAndPoint(const Box &box,	const Vector3 &point, CollisionData *data)
{
	
}*/
