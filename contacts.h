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
	
	//Ogre::Matrix3 inverseInertiaTensor;
	
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
	
	/**
	 * Beräknar en ortogonal bas för kontaktpunkten baserad på den primära 
	 * friktionsriktningen (för "anisotropic" friktion) eller slumpmässig
	 * orientering (för "isotropic" friktion).
	 */
	void calculateContactBasis();
	
	/**
	 * Applicerar en impuls till en given kropp och returnerar förändringen i hastighet.
	 */
	void applyImpulse(const Ogre::Vector3 &impulse, RigidBody *body,
					  Ogre::Vector3 *velocityChange, Ogre::Vector3 *rotationChange);
	
	void applyVelocityChange(Ogre::Vector3 velocityChange[2], Ogre::Vector3 rotationChange[2]);
	
	/**
	 * Beräknar impulsen som krävs för att upphöra kontakten,
	 givet att kontakten inte har någon friktion.
	 */
	Ogre::Vector3 Contact::calculateFrictionlessImpulse(Ogre::Matrix3 * inverseInertiaTensor);
	
	/**
	 * Beräknar impulsen som krävs för att upphöra kontakten,
	 givet friktion.
	 */
	Ogre::Vector3 Contact::calculateFrictionImpulse(Ogre::Matrix3 * inverseInertiaTensor);
	
	/**
	 * En transformmatris som konverterar koordinaterna i kontaktens synsett
	 * till "world" koordinater. Kolumerna av matrisen är ortoganala vektorer.
	 */
	Ogre::Matrix3 contactToWorld;
	
	/**
	 * Håller "world space" positionen av kontakten punkten relativt
	 * till kroppens centrum. Denna sätts när calculateInternals körs.
	 */
	Ogre::Vector3 relativeContactPosition[2];
	
	/**
	 * Håller minsta hastighet för att lösa kontakten.
	 */
	real desiredDeltaVelocity;
	
	void setSkewSymmetric(const Ogre::Vector3 v);
};


#endif