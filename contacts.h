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

/*
 * Forward declaration, see full declaration below for complete
 * documentation.
 */
class ContactResolver;

class Contact
{
	friend class ContactResolver;
	
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
	
	void calculateInternals(real duration);
	
protected:
	
	void Contact::swapBodies();
	
	void Contact::applyPositionChange(Ogre::Vector3 linearChange[2],
									  Ogre::Vector3 angularChange[2],
									  real penetration);
	
	/**
	 * Calculates and sets the internal value for the desired delta
	 * velocity.
	 */
	void calculateDesiredDeltaVelocity(real duration);
	
	/**
	 * Calculates and returns the velocity of the contact
	 * point on the given body.
	 */
	Ogre::Vector3 calculateLocalVelocity(unsigned bodyIndex, real duration);
	
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
	
	/**
	 * Holds the closing velocity at the point of contact. This is set
	 * when the calculateInternals function is run.
	 */
	Ogre::Vector3 contactVelocity;
	
	void setSkewSymmetric(const Ogre::Vector3 v);
};


class ContactResolver
{
public:
	// HƒR KOMMER BARAFFFUNKTIONERNA
	
	static Ogre::Vector3 ContactResolver::pt_velocity(RigidBody * body, Ogre::Vector3 p)
	{
		return body->velocity + body->angularVelocity.crossProduct(p - body->position);
	}
	
	static bool ContactResolver::colliding(Contact * c)
	{
		Ogre::Vector3 padot = pt_velocity(c->body[0], c->contactPoint);
		Ogre::Vector3 pbdot(0,0,0);
		
		if ( c->body[1] )
		{	
			pbdot = pt_velocity(c->body[1], c->contactPoint);
		}
		
		//Relative velocity
		real vrel = c->contactNormal.dotProduct(padot-pbdot);
		
		std::cout << "RELATIVE VELOCITY : " << vrel << std::endl;
		
		real t = 0.01;
		
		if (vrel > t ) /*moving away*/
		{
			std::cout << "moving away: do nothing" << std::endl;
			return false;
		}
		if (vrel > -t)
		{	// resting contact
			
			std::cout << "resting contact: do nothing" << std::endl;
			return false;
		}
		else //vrel < - t 
		{	
			return true;
		}
	}
	
	static void ContactResolver::resolveContact(Contact * c, real restitution)
	{
		Ogre::Vector3	padot = pt_velocity(c->body[0], c->contactPoint),
						n = c->contactNormal,
						ra = c->contactPoint - c->body[0]->position;
		
		std::cout << "padot= " << padot << std::endl;
		
		
		real term2 = 0, term4 = 0;
		Ogre::Vector3 pbdot(0,0,0);
		Ogre::Vector3 rb(0,0,0);
		
		if ( c->body[1] )
		{
			pbdot = pt_velocity(c->body[1], c->contactPoint);
			rb = c->contactPoint - c->body[1]->position;
			
			term2 = c->body[1]->inverseMass;
			term4 = n.dotProduct( (c->body[1]->inverseInertiaTensorWorld * rb.crossProduct(n)).crossProduct(rb) ); //galna parenteser. De borde vara r‰tt.
		}
		std::cout << "NORMAL = " << n << std::endl;
		real vrel = n.dotProduct(padot - pbdot);
		std::cout << "vrel :" << vrel << std::endl;
		real numerator = -(1+restitution) * vrel;
		
		real term1 = c->body[0]->inverseMass;

		Ogre::Vector3 temp = ra.crossProduct(n);
		
		temp = c->body[0]->inverseInertiaTensorWorld * temp;
		
		temp = temp.crossProduct(ra);
		
		real term3 = n.dotProduct(temp);
		
		//real term3 = n.dotProduct( (c->body[0]->inverseInertiaTensorWorld * (ra.crossProduct(n))).crossProduct(ra) ); //galna parenteser. De borde vara r‰tt.
		
		real j = numerator/ (term1 + term2 + term3 + term4);
		
		Ogre::Vector3 force = j*n;
		
		std::cout << "impulse force: " << force << std::endl << "at body point :" << ra << std::endl;
		
		/* Apply the impulse to the bodies */
		c->body[0]->linearMomentum += force;
		c->body[0]->angularMomentum += ra.crossProduct(force);
		c->body[0]->angularMomentum = -c->body[0]->angularMomentum;


		
		
		c->body[0]->velocity = c->body[0]->linearMomentum*c->body[0]->getInverseMass();
		c->body[0]->angularVelocity = c->body[0]->inverseInertiaTensorWorld * c->body[0]->angularMomentum;
		
		
		std::cout << "angularMomentum : " << c->body[0]->angularMomentum << std::endl;
		
		if ( c->body[1] )
		{
			
		}
	}

};

#endif