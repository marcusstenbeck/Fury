/*
 *  pfgen.cpp
 *  tnm085
 *
 *  Created by Oscar Persson on 2011-01-26.
 *  Copyright 2011 Linköpings Univeristet. All rights reserved.
 *
 */

#include "pfgen.h"
using namespace fury;

void ParticleForceRegistry::updateForces(real duration)
{
	Registry::iterator i = registrations.begin();
	for(; i != registrations.end(); i++)
	{
		i->fg->updateForce(i->particle, duration);
	}
};

void ParticleGravity::updateForce(Particle* particle, real duration)
{
	if(!particle->hasFiniteMass()) return;
	
	particle->addForce(gravity * particle->getMass());
};

void ParticleDrag::updateForce(Particle* particle, real duration)
{
	Ogre::Vector3 force;
	//particle->getVelocity(&force);
	
	real dragCoeff = force.length();
	dragCoeff = k1 * dragCoeff + k2 * dragCoeff * dragCoeff;
	
	force.normalise();
	force *= -dragCoeff;
	particle->addForce(force);
};