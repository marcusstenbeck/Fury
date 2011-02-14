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

unsigned boxAndHalfSpace(const Box &box, const Plane &plane, CollisionData *data)
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
