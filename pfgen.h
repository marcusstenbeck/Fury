/*
 *  pfgen.h
 *  tnm085
 *
 *  Created by Oscar Persson on 2011-01-26.
 *  Copyright 2011 Linköpings Univeristet. All rights reserved.
 *
 */

#ifndef PFGEN_H
#define PFGEN_H

#include <vector>
#include "precision.h"
#include "particle.h"

namespace fury
{

	class ParticleForceGenerator
	{
		
	public:
		// Uppdatera kraften på en given partikel
		virtual void updateForce(Particle *particle, real duration) = 0;
		
	};
	
	class ParticleForceRegistry
	{
		
	protected:
		// Håller koll på en force generator och partikeln den hör till.
		struct ParticleForceRegistration
		{
			
			Particle *particle;
			ParticleForceGenerator *fg;
			
		};
		
		//Vectorn innehåller regristrations
		typedef std::vector<ParticleForceRegistration> Registry;
		Registry registrations;
		
	public:
		
		// Lägger till den givna force generator till givna partikeln
		void add (Particle *particle, ParticleForceGenerator *fg);
		
		// Tar bort det givna paret. Om det inte finns har denna funktion ingen verkan.
		void remove (Particle *particle, ParticleForceGenerator *fg);
		
		// Rensar registret
		void clear();
		
		//Anropar kraftgeneratorer och uppdaterar krafterna för dess partiklar.
		void updateForces(real duration);
		
	};
	
	class ParticleGravity : public ParticleForceGenerator
	{
		Ogre::Vector3 gravity;
		
	public:
		ParticleGravity(const Ogre::Vector3 &gravity);
		
		virtual void updateForce(Particle *particle, real duration);
	};
	
	class ParticleDrag : public ParticleForceGenerator
	{
		real k1, k2;
		
	public:
		ParticleDrag(real k1, real k2);
		
		virtual void updateForce(Particle *particle, real duration);
	};
}
#endif