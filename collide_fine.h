/*
 *  collide_fine.h
 *  tnm085
 *
 *  Created by Oscar Persson on 2011-02-08.
 *  Copyright 2011 Linköpings Univeristet. All rights reserved.
 *
 */

#ifndef COLLIDE_FINE_H
#define COLLIDE_FINE_H

#include "contacts.h"
// #include "core.h"

namespace fury 
{
	class IntersectionTest;
	class CollisionDetector;
	
	
	struct CollisionData
	{
		// Håller arrayen med kontakter som vi ska skriva till.
		Contact *contacts;
		
		// Håller antalet platser kvar i contacts.
		unsigned contactsLeft;
		
		// Toleransen för kollision.
		real tolerance;
		
		// Håller antalet kontakter så långt som man kommit.
        unsigned contactCount;
		
        // Friktionsvärde.
        real friction;
		
        // Håller "studs".
        real restitution;
		
		void addContacts(unsigned count)
        {
            // Uppdatera antal kontakter kvar resp. räknaren.
            contactsLeft -= count;
            contactCount += count;
			
            // Förflytta till nästa plats i arrayen.
            contacts += count;
        }
		
	};
	
	class Primitive
	{
	public:
		friend class IntersectionTest;
        friend class CollisionDetector;
		
		RigidBody *body;
	};
	
	class Plane : public Primitive
	{
	public:
		Ogre::Vector3 normal;
		real offset;
	};
	
	class Box : public Primitive
	{
	public:
		Ogre::Vector3 halfSize;
	};
	
	class IntersectionTest
	{
	public:
		bool boxAndBox(const Box &one, const Box &two);
	};
	
	class CollisionDetector
	{
	public:
		static unsigned boxAndHalfSpace(const Box &box, const Plane &plane, CollisionData *data);
		
		static unsigned boxAndPoint(const Box &box,	const Ogre::Vector3 &point, CollisionData *data);
	};
	

	void detectContacts(const Primitive &firstPrimitive, const Primitive &secondPrimitive, CollisionData *data);	
	
	
	
	
		
	void detectContacts(const Primitive &firstPrimitive, const Primitive &secondPrimitive, CollisionData *data);	
	
	//static inline real transformToAxis(const Box &box, const Ogre::Vector3 &axis);
	
	//bool overlapOnAxis(const Box &one, const Box &two, const Ogre::Vector3 &axis,  const Ogre::Vector3 &toCenter);
	
} // namespace fury

#endif