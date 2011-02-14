/*
 *  particle.h
 *  tnm085
 *
 *  Created by Oscar Persson on 2011-01-26.
 *  Copyright 2011 Linköpings Univeristet. All rights reserved.
 *
 */

#ifndef PARTICLE_H
#define PARTICLE_H

#include "OgreFramework.h"
#include "precision.h"

namespace fury
{
	class Particle
	{
		
	public:
		Ogre::Vector3 position, velocity, acceleration, forceAccum;
		
		//Val av inverse massa eftersom lättare att beräkna och stabilare.
		real damping, inverseMass;
		
		//Funktioner
		void integrate(real duration);
		void clearAccumulator();
		void addForce(const Ogre::Vector3 &force);
		
		bool hasFiniteMass();
		real getMass();
		void setMass(real mass);
		
	private:
		bool highPrecision;
		
	};
}
#endif