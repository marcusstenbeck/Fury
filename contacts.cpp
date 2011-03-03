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

/*
 * Swaps the bodies in the current contact, so body 0 is at body 1 and
 * vice versa. This also changes the direction of the contact normal,
 * but doesn't update any calculated internal data. If you are calling
 * this method manually, then call calculateInternals afterwards to
 * make sure the internal data is up to date.
 */
void Contact::swapBodies()
{
    contactNormal *= -1;
	
    RigidBody *temp = body[0];
    body[0] = body[1];
    body[1] = temp;
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
	std::cout << "deltaVelWorld: " << deltaVelWorld << std::endl;
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

Ogre::Vector3 Contact::calculateFrictionImpulse(Ogre::Matrix3 * inverseInertiaTensor)
{
	Ogre::Vector3 impulseContact;
    real inverseMass = body[0]->getInverseMass();
	
    // The equivalent of a cross product in matrices is multiplication
    // by a skew symmetric matrix - we build the matrix for converting
    // between linear and angular quantities.
	Ogre::Matrix3 impulseToTorque;
	impulseToTorque[0][0] = impulseToTorque[1][1] = impulseToTorque[2][2] = 0;
	impulseToTorque[0][1] = -relativeContactPosition[0].z;
	impulseToTorque[0][2] = relativeContactPosition[0].y;
	impulseToTorque[1][0] = relativeContactPosition[0].z;
	impulseToTorque[1][2] = -relativeContactPosition[0].x;
	impulseToTorque[2][0] = -relativeContactPosition[0].y;
	impulseToTorque[2][1] = relativeContactPosition[0].x;
	
    // Build the matrix to convert contact impulse to change in velocity
    // in world coordinates.
	Ogre::Matrix3 deltaVelWorld = impulseToTorque;
    deltaVelWorld = deltaVelWorld * inverseInertiaTensor[0];
    deltaVelWorld = deltaVelWorld * impulseToTorque;
    deltaVelWorld = deltaVelWorld * -1;
	
    // Check if we need to add body two's data
    if (body[1])
    {
        // Set the cross product matrix
		impulseToTorque[0][0] = impulseToTorque[1][1] = impulseToTorque[2][2] = 0;
		impulseToTorque[0][1] = -relativeContactPosition[1].z;
		impulseToTorque[0][2] = relativeContactPosition[1].y;
		impulseToTorque[1][0] = relativeContactPosition[1].z;
		impulseToTorque[1][2] = -relativeContactPosition[1].x;
		impulseToTorque[2][0] = -relativeContactPosition[1].y;
		impulseToTorque[2][1] = relativeContactPosition[1].x;
		
        // Calculate the velocity change matrix
		Ogre::Matrix3 deltaVelWorld2 = impulseToTorque;
        deltaVelWorld = deltaVelWorld2 * inverseInertiaTensor[1];
        deltaVelWorld = deltaVelWorld2 * impulseToTorque;
        deltaVelWorld = deltaVelWorld2 * -1;
		
        // Add to the total delta velocity.
        deltaVelWorld = deltaVelWorld + deltaVelWorld2;
		
        // Add to the inverse mass
        inverseMass += body[1]->getInverseMass();
    }
	
    // Do a change of basis to convert into contact coordinates.
	Ogre::Matrix3 deltaVelocity = contactToWorld.Transpose();
    deltaVelocity = deltaVelocity * deltaVelWorld;
    deltaVelocity = deltaVelocity * contactToWorld;
	
    // Add in the linear velocity change
    deltaVelocity[0][0] += inverseMass;
    deltaVelocity[1][1] += inverseMass;
    deltaVelocity[2][2] += inverseMass;
	
    // Invert to get the impulse needed per unit velocity
	Ogre::Matrix3 impulseMatrix = deltaVelocity.Inverse();
	
    // Find the target velocities to kill
	Ogre::Vector3 velKill(desiredDeltaVelocity,
					-contactVelocity.y,
					-contactVelocity.z);
	
    // Find the impulse to kill target velocities
    impulseContact = impulseMatrix * velKill;
	
    // Check for exceeding friction
    real planarImpulse = Ogre::Math::Sqrt(
								   impulseContact.y*impulseContact.y +
								   impulseContact.z*impulseContact.z
								   );
    if (planarImpulse > impulseContact.x * friction)
    {
        // We need to use dynamic friction
        impulseContact.y /= planarImpulse;
        impulseContact.z /= planarImpulse;
		
        impulseContact.x = deltaVelocity[0][0] +
		deltaVelocity[0][1]*friction*impulseContact.y +
		deltaVelocity[0][2]*friction*impulseContact.z;
        impulseContact.x = desiredDeltaVelocity / impulseContact.x;
        impulseContact.y *= friction * impulseContact.x;
        impulseContact.z *= friction * impulseContact.x;
    }
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
        impulseContact = calculateFrictionImpulse(inverseInertiaTensor);
    }
	
    // Konvertera impulsen till världskoordinater.
	Ogre::Vector3 impulse = contactToWorld * impulseContact;
	
    // Dela upp impulsen i linjära och rotationskomponenter.
	Ogre::Vector3 impulsiveTorque = relativeContactPosition[0].crossProduct(impulse);
	
    rotationChange[0] = inverseInertiaTensor[0]*impulsiveTorque;
    velocityChange[0] = Ogre::Vector3(0,0,0);
	velocityChange[0] += impulse*(body[0]->getInverseMass());
	
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
		velocityChange[1] += impulse*(-body[1]->getInverseMass());
		
        // Sätt in förändringar.
        body[1]->addVelocity(velocityChange[1]);
        body[1]->addRotation(rotationChange[1]);
    }
}

void Contact::calculateInternals(real duration)
{
	if(!body[0]) swapBodies();
	assert(body[0]);
	
	calculateContactBasis();
	
	relativeContactPosition[0] = contactPoint - body[0]->position;
	if(body[1])
		relativeContactPosition[1] = contactPoint - body[1]->position;
	
	contactVelocity = calculateLocalVelocity(0, duration);
	if(body[1])
		contactVelocity -= calculateLocalVelocity(1, duration);
	
	calculateDesiredDeltaVelocity(duration);
};

Ogre::Vector3 Contact::calculateLocalVelocity(unsigned bodyIndex, real duration)
{
    RigidBody *thisBody = body[bodyIndex];
	
    // Work out the velocity of the contact point.
	Ogre::Vector3 velocity =
	thisBody->angularVelocity.crossProduct(relativeContactPosition[bodyIndex]);
    velocity += thisBody->velocity;
	
    // Turn the velocity into contact-coordinates.
	Ogre::Vector3 contactVelocity = contactToWorld.Transpose() * velocity;
	
    // Calculate the ammount of velocity that is due to forces without
    // reactions.
	Ogre::Vector3 accVelocity = thisBody->lastFrameAcceleration * duration;
	
    // Calculate the velocity in contact-coordinates.
    accVelocity = contactToWorld.Transpose() * accVelocity;
	
    // We ignore any component of acceleration in the contact normal
    // direction, we are only interested in planar acceleration
    accVelocity.x = 0;
	
    // Add the planar velocities - if there's enough friction they will
    // be removed during velocity resolution
    contactVelocity += accVelocity;
	
    // And return it
    return contactVelocity;
}

void Contact::calculateDesiredDeltaVelocity(real duration)
{
    const static real velocityLimit = (real)0.25f;
	
    // Calculate the acceleration induced velocity accumulated this frame
    real velocityFromAcc = 0;
	
	body[0]->lastFrameAcceleration * duration * contactNormal;
	
	if(body[1])
	{
		velocityFromAcc = velocityFromAcc - (body[1]->lastFrameAcceleration.dotProduct(duration * contactNormal));

	}
		
    // If the velocity is very slow, limit the restitution
    real thisRestitution = restitution;
    if (Ogre::Math::Abs(contactVelocity.x) < velocityLimit)
    {
        thisRestitution = (real)0.0f;
    }
	
    // Combine the bounce velocity with the removed
    // acceleration velocity.
    desiredDeltaVelocity =
	-contactVelocity.x
	-thisRestitution * (contactVelocity.x - velocityFromAcc);
}


void Contact::applyPositionChange(Ogre::Vector3 linearChange[2],
                                  Ogre::Vector3 angularChange[2],
                                  real penetration)
{
    const real angularLimit = (real)0.2f;
    real angularMove[2];
    real linearMove[2];
	
    real totalInertia = 0;
    real linearInertia[2];
    real angularInertia[2];
	
    // We need to work out the inertia of each object in the direction
    // of the contact normal, due to angular inertia only.
    for (unsigned i = 0; i < 2; i++) if (body[i])
    {
		Ogre::Matrix3 inverseInertiaTensor;
        body[i]->getInverseInertiaTensorWorld(&inverseInertiaTensor);
		
        // Use the same procedure as for calculating frictionless
        // velocity change to work out the angular inertia.
		Ogre::Vector3 angularInertiaWorld = relativeContactPosition[i].crossProduct(contactNormal);
        angularInertiaWorld = inverseInertiaTensor * angularInertiaWorld;
        angularInertiaWorld = angularInertiaWorld.crossProduct(relativeContactPosition[i]);
        angularInertia[i] = angularInertiaWorld.dotProduct(contactNormal);
		
        // The linear component is simply the inverse mass
        linearInertia[i] = body[i]->getInverseMass();
		
        // Keep track of the total inertia from all components
        totalInertia += linearInertia[i] + angularInertia[i];
		
        // We break the loop here so that the totalInertia value is
        // completely calculated (by both iterations) before
        // continuing.
    }
	
    // Loop through again calculating and applying the changes
    for (unsigned i = 0; i < 2; i++) if (body[i])
    {
        // The linear and angular movements required are in proportion to
        // the two inverse inertias.
        real sign = (i == 0)?1:-1;
        angularMove[i] =
		sign * penetration * (angularInertia[i] / totalInertia);
        linearMove[i] =
		sign * penetration * (linearInertia[i] / totalInertia);
		
        // To avoid angular projections that are too great (when mass is large
        // but inertia tensor is small) limit the angular move.
		Ogre::Vector3 projection = relativeContactPosition[i];
        projection += contactNormal * (-relativeContactPosition[i].dotProduct(contactNormal));
		
        // Use the small angle approximation for the sine of the angle (i.e.
        // the magnitude would be sine(angularLimit) * projection.magnitude
        // but we approximate sine(angularLimit) to angularLimit).
        real maxMagnitude = angularLimit * projection.length();
		
        if (angularMove[i] < -maxMagnitude)
        {
            real totalMove = angularMove[i] + linearMove[i];
            angularMove[i] = -maxMagnitude;
            linearMove[i] = totalMove - angularMove[i];
        }
        else if (angularMove[i] > maxMagnitude)
        {
            real totalMove = angularMove[i] + linearMove[i];
            angularMove[i] = maxMagnitude;
            linearMove[i] = totalMove - angularMove[i];
        }
		
        // We have the linear amount of movement required by turning
        // the rigid body (in angularMove[i]). We now need to
        // calculate the desired rotation to achieve that.
        if (angularMove[i] == 0)
        {
            // Easy case - no angular movement means no rotation.
            angularChange[i] = Ogre::Vector3(0.0, 0.0, 0.0);
        }
        else
        {
            // Work out the direction we'd like to rotate in.
			Ogre::Vector3 targetAngularDirection =
			relativeContactPosition[i].crossProduct(contactNormal);
			
			Ogre::Matrix3 inverseInertiaTensor;
            body[i]->getInverseInertiaTensorWorld(&inverseInertiaTensor);
			
            // Work out the direction we'd need to rotate to achieve that
            angularChange[i] =
			(inverseInertiaTensor * targetAngularDirection) *
			(angularMove[i] / angularInertia[i]);
        }
		
        // Velocity change is easier - it is just the linear movement
        // along the contact normal.
        linearChange[i] = contactNormal * linearMove[i];
		
        // Now we can start to apply the values we've calculated.
        // Apply the linear movement
		Ogre::Vector3 pos = body[i]->position;
        pos += contactNormal * linearMove[i];
        body[i]->position = pos;
		
        // And the change in orientation
		
		Ogre::Quaternion q = body[i]->orientation;
		Ogre::Quaternion qtemp = Ogre::Quaternion(0.0, angularChange[i].x, angularChange[i].y, angularChange[i].z);
		qtemp = qtemp * q;
		q.w += qtemp.w * 0.5;
		q.x += qtemp.x * 0.5;
		q.y += qtemp.y * 0.5;
		q.z += qtemp.z * 0.5;
		
		//q.normalise();
		
        body[i]->orientation = q;
		
		
        // We need to calculate the derived data for any body that is
        // asleep, so that the changes are reflected in the object's
        // data. Otherwise the resolution will not change the position
        // of the object, and the next collision detection round will
        // have the same penetration.
        //if (!body[i]->getAwake()) body[i]->calculateDerivedData();
    }
}




// Contact resolver implementation

ContactResolver::ContactResolver(unsigned iterations,
                                 real velocityEpsilon,
                                 real positionEpsilon)
{
    setIterations(iterations, iterations);
    setEpsilon(velocityEpsilon, positionEpsilon);
}

ContactResolver::ContactResolver(unsigned velocityIterations,
                                 unsigned positionIterations,
                                 real velocityEpsilon,
                                 real positionEpsilon)
{
    setIterations(velocityIterations);
    setEpsilon(velocityEpsilon, positionEpsilon);
}

void ContactResolver::setIterations(unsigned iterations)
{
    setIterations(iterations, iterations);
}

void ContactResolver::setIterations(unsigned velocityIterations,
                                    unsigned positionIterations)
{
    ContactResolver::velocityIterations = velocityIterations;
    ContactResolver::positionIterations = positionIterations;
}

void ContactResolver::setEpsilon(real velocityEpsilon,
                                 real positionEpsilon)
{
    ContactResolver::velocityEpsilon = velocityEpsilon;
    ContactResolver::positionEpsilon = positionEpsilon;
}

void ContactResolver::resolveContacts(Contact *contacts,
                                      unsigned numContacts,
                                      real duration)
{
    // Make sure we have something to do.
    if (numContacts == 0) return;
    if (!isValid()) return;
	
    // Prepare the contacts for processing
    prepareContacts(contacts, numContacts, duration);
	
    // Resolve the interpenetration problems with the contacts.
    adjustPositions(contacts, numContacts, duration);
	
    // Resolve the velocity problems with the contacts.
    adjustVelocities(contacts, numContacts, duration);
}

void ContactResolver::prepareContacts(Contact* contacts,
                                      unsigned numContacts,
                                      real duration)
{
    // Generate contact velocity and axis information.
    Contact* lastContact = contacts + numContacts;
    for (Contact* contact=contacts; contact < lastContact; contact++)
    {
        // Calculate the internal contact data (inertia, basis, etc).
        contact->calculateInternals(duration);
    }
}

void ContactResolver::adjustVelocities(Contact *c,
                                       unsigned numContacts,
                                       real duration)
{
	Ogre::Vector3 velocityChange[2], rotationChange[2];
	Ogre::Vector3 deltaVel;
	
    // iteratively handle impacts in order of severity.
    velocityIterationsUsed = 0;
    while (velocityIterationsUsed < velocityIterations)
    {
        // Find contact with maximum magnitude of probable velocity change.
        real max = velocityEpsilon;
        unsigned index = numContacts;
        for (unsigned i = 0; i < numContacts; i++)
        {
            if (c[i].desiredDeltaVelocity > max)
            {
                max = c[i].desiredDeltaVelocity;
                index = i;
            }
        }
        if (index == numContacts) break;
		
        // Match the awake state at the contact
        //c[index].matchAwakeState();
		
        // Do the resolution on the contact that came out top.
        c[index].applyVelocityChange(velocityChange, rotationChange);
		
        // With the change in velocity of the two bodies, the update of
        // contact velocities means that some of the relative closing
        // velocities need recomputing.
        for (unsigned i = 0; i < numContacts; i++)
        {
            // Check each body in the contact
            for (unsigned b = 0; b < 2; b++) if (c[i].body[b])
            {
                // Check for a match with each body in the newly
                // resolved contact
                for (unsigned d = 0; d < 2; d++)
                {
                    if (c[i].body[b] == c[index].body[d])
                    {
                        deltaVel = velocityChange[d] +
						rotationChange[d].crossProduct(c[i].relativeContactPosition[b]);
						
                        // The sign of the change is negative if we're dealing
                        // with the second body in a contact.
                        c[i].contactVelocity += (c[i].contactToWorld.Transpose() * deltaVel) * (b?-1:1);
                        c[i].calculateDesiredDeltaVelocity(duration);
                    }
                }
            }
        }
        velocityIterationsUsed++;
    }
}

void ContactResolver::adjustPositions(Contact *c,
                                      unsigned numContacts,
                                      real duration)
{
    unsigned i,index;
	Ogre::Vector3 linearChange[2], angularChange[2];
    real max;
	Ogre::Vector3 deltaPosition;
	
    // iteratively resolve interpenetrations in order of severity.
    positionIterationsUsed = 0;
    while (positionIterationsUsed < positionIterations)
    {
        // Find biggest penetration
        max = positionEpsilon;
        index = numContacts;
        for (i=0; i<numContacts; i++)
        {
            if (c[i].penetration > max)
            {
                max = c[i].penetration;
                index = i;
            }
        }
        if (index == numContacts) break;
		
        // Match the awake state at the contact
        //c[index].matchAwakeState();
		
        // Resolve the penetration.
        c[index].applyPositionChange(
									 linearChange,
									 angularChange,
									 max);
		
        // Again this action may have changed the penetration of other
        // bodies, so we update contacts.
        for (i = 0; i < numContacts; i++)
        {
            // Check each body in the contact
            for (unsigned b = 0; b < 2; b++) if (c[i].body[b])
            {
                // Check for a match with each body in the newly
                // resolved contact
                for (unsigned d = 0; d < 2; d++)
                {
                    if (c[i].body[b] == c[index].body[d])
                    {
						
						std::cout << "angularChange : " << angularChange[d] << std::endl;
                        deltaPosition = linearChange[d] + angularChange[d].crossProduct(c[i].relativeContactPosition[b]);
						
                        // The sign of the change is positive if we're
                        // dealing with the second body in a contact
                        // and negative otherwise (because we're
                        // subtracting the resolution)..
                        c[i].penetration += deltaPosition.dotProduct(c[i].contactNormal) * (b?1:-1);
                    }
                }
            }
        }
        positionIterationsUsed++;
    }
}