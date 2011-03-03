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
protected:
	/**
	 * Holds the number of iterations to perform when resolving
	 * velocity.
	 */
	unsigned velocityIterations;
	
	/**
	 * Holds the number of iterations to perform when resolving
	 * position.
	 */
	unsigned positionIterations;
	
	/**
	 * To avoid instability velocities smaller
	 * than this value are considered to be zero. Too small and the
	 * simulation may be unstable, too large and the bodies may
	 * interpenetrate visually. A good starting point is the default
	 * of 0.01.
	 */
	real velocityEpsilon;
	
	/**
	 * To avoid instability penetrations
	 * smaller than this value are considered to be not interpenetrating.
	 * Too small and the simulation may be unstable, too large and the
	 * bodies may interpenetrate visually. A good starting point is
	 * the default of0.01.
	 */
	real positionEpsilon;
	
public:
	/**
	 * Stores the number of velocity iterations used in the
	 * last call to resolve contacts.
	 */
	unsigned velocityIterationsUsed;
	
	/**
	 * Stores the number of position iterations used in the
	 * last call to resolve contacts.
	 */
	unsigned positionIterationsUsed;
	
private:
	/**
	 * Keeps track of whether the internal settings are valid.
	 */
	bool validSettings;
	
public:
	/**
	 * Creates a new contact resolver with the given number of iterations
	 * per resolution call, and optional epsilon values.
	 */
	ContactResolver(unsigned iterations,
					real velocityEpsilon=(real)0.01,
					real positionEpsilon=(real)0.01);
	
	/**
	 * Creates a new contact resolver with the given number of iterations
	 * for each kind of resolution, and optional epsilon values.
	 */
	ContactResolver(unsigned velocityIterations,
					unsigned positionIterations,
					real velocityEpsilon=(real)0.01,
					real positionEpsilon=(real)0.01);
	
	/**
	 * Returns true if the resolver has valid settings and is ready to go.
	 */
	bool isValid()
	{
		return (velocityIterations > 0) &&
		(positionIterations > 0) &&
		(positionEpsilon >= 0.0f) &&
		(positionEpsilon >= 0.0f);
	}
	
	/**
	 * Sets the number of iterations for each resolution stage.
	 */
	void setIterations(unsigned velocityIterations,
					   unsigned positionIterations);
	
	/**
	 * Sets the number of iterations for both resolution stages.
	 */
	void setIterations(unsigned iterations);
	
	/**
	 * Sets the tolerance value for both velocity and position.
	 */
	void setEpsilon(real velocityEpsilon,
					real positionEpsilon);
	
	/**
	 * Resolves a set of contacts for both penetration and velocity.
	 *
	 * Contacts that cannot interact with
	 * each other should be passed to separate calls to resolveContacts,
	 * as the resolution algorithm takes much longer for lots of
	 * contacts than it does for the same number of contacts in small
	 * sets.
	 *
	 * @param contactArray Pointer to an array of contact objects.
	 *
	 * @param numContacts The number of contacts in the array to resolve.
	 *
	 * @param numIterations The number of iterations through the
	 * resolution algorithm. This should be at least the number of
	 * contacts (otherwise some constraints will not be resolved -
	 * although sometimes this is not noticable). If the iterations are
	 * not needed they will not be used, so adding more iterations may
	 * not make any difference. In some cases you would need millions
	 * of iterations. Think about the number of iterations as a bound:
	 * if you specify a large number, sometimes the algorithm WILL use
	 * it, and you may drop lots of frames.
	 *
	 * @param duration The duration of the previous integration step.
	 * This is used to compensate for forces applied.
	 */
	void resolveContacts(Contact *contactArray,
						 unsigned numContacts,
						 real duration);
	
protected:
	/**
	 * Sets up contacts ready for processing. This makes sure their
	 * internal data is configured correctly and the correct set of bodies
	 * is made alive.
	 */
	void prepareContacts(Contact *contactArray, unsigned numContacts,
						 real duration);
	
	/**
	 * Resolves the velocity issues with the given array of constraints,
	 * using the given number of iterations.
	 */
	void adjustVelocities(Contact *contactArray,
						  unsigned numContacts,
						  real duration);
	
	/**
	 * Resolves the positional issues with the given array of constraints,
	 * using the given number of iterations.
	 */
	void adjustPositions(Contact *contacts,
						 unsigned numContacts,
						 real duration);
};

#endif