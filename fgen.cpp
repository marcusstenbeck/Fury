/*
 *  fgen.cpp
 *  tnm085
 *
 *  Created by Marcus Stenbeck on 2011-02-01.
 *  Copyright 2011 Macatak Media. All rights reserved.
 *
 */

#include "fgen.h"

using namespace Ogre;
using namespace fury;

// TESTA
void ForceRegistry::add(RigidBody *rb, ForceGenerator *fg)
{
	ForceRegistration fr;
	
	fr.rb = rb;
	fr.fg = fg;
	
	registrations.push_back(fr);
};

// DEFINIERA
void ForceRegistry::remove(RigidBody *rb, ForceGenerator *fg)
{
	for (Registry::iterator i = registrations.begin(); i != registrations.end(); ++i)
	{
		if(i->fg == fg && i->rb == rb)
		{
			registrations.erase(i);
			return;
		}
	}
};

// DEFINIERA
void ForceRegistry::clear()
{
	registrations.clear();
};

// TESTA
void ForceRegistry::updateForces(real duration)
{
	for (Registry::iterator i = registrations.begin(); i != registrations.end(); ++i)
	{
		i->fg->updateForce(i->rb, duration);
	}
};

//CLASS GRAVITY START! OMG

Gravity::Gravity(const Ogre::Vector3 &g)
{
	gravity = g;
}

void Gravity::updateForce(RigidBody *rb, real duration)
{
	if (!rb->hasFiniteMass()) return;
	
	rb->addForce(gravity*rb->getMass());
};