/*
 *  particle.cpp
 *  tnm085
 *
 *  Created by Oscar Persson on 2011-01-26.
 *  Copyright 2011 Linköpings Univeristet. All rights reserved.
 *
 */

#include "particle.h"
#include <assert.h>
using namespace fury;

void Particle::integrate(real duration)
{
	
	// Om det inte har gått någon tid så abortas funktion och ett 
	// felmeddelande visas.
	assert(duration > 0.0);
	
	// Uppdatera den linjära positionen.
	position += velocity * duration;
	
	// Beräkna accerelationen från kraften.
	Ogre::Vector3 resultingAcc = acceleration;
	resultingAcc += forceAccum * inverseMass;
	
	// Uppdatera den linjära hastigheten med hjälp av accelerationen
	velocity += resultingAcc * duration;
	
	// Lägg till en bromsande kraft
	velocity *= pow(damping, duration);
	
	clearAccumulator();
};


void Particle::clearAccumulator()
{
	
	forceAccum *= 0;
	
};

void Particle::addForce(const Ogre::Vector3 &force)
{
	
	forceAccum += force;
	
};

bool Particle::hasFiniteMass()
{
	if (inverseMass == 0) {
		return false;
	}
	
	return true;
};

real Particle::getMass()
{
	return 1/inverseMass;
};



/*	
Ogre::Vector3 Particle::getVelocity(Ogre::Vector3 &force)
{
	force = 
};
*/
