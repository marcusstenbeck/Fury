/*
 *  contacts.h
 *  tnm085
 *
 *  Created by Oscar Persson on 2011-02-07.
 *  Copyright 2011 Linköpings Univeristet. All rights reserved.
 *
 */

#ifndef CONTACTS_H
#define CONTACTS_H

#include <OgreVector3.h>
#include "RigidBody.h"

using namespace fury;

class Contact
{
public:
	// Håller positionen till kontakten i world-koordinater.
	Ogre::Vector3 contactPoint;
	
	// Håller riktiningen av kontakten i world-koordinater.
	Ogre::Vector3 contactNormal;
	
	// Håller kontaktdjupet.
	real penetration;
	
	// Håller de två kropparna som är i kontakt.
	// Den andra kan vara NULL, om det gäller kontakt med scenen.
	RigidBody* body[2];
	
	// Håller friktionskoeffisienten vid kontakt.
	real friction;
	
	// Håller "studs".
	real restitution;
	
	
	// Sätter in värden.
	void setBodyData(RigidBody* one, RigidBody *two,
					 real friction, real restitution);
	
protected:
	
	void calculateContactBasis();
	
	Ogre::Matrix3 contactToWorld;
	
	Ogre::Vector3 relativeContactPosition[2];
};


#endif