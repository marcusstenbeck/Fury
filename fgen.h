/*
 *  fgen.h
 *  tnm085
 *
 *  Created by Marcus Stenbeck on 2011-02-01.
 *  Copyright 2011 Macatak Media. All rights reserved.
 *
 */

#ifndef FGEN_H
#define FGEN_H

#include "RigidBody.h"
#include <vector>
#include "precision.h"

namespace fury
{
	class ForceGenerator
	{
	public:
	
		virtual void updateForce(RigidBody* rb, real duration) = 0;
	
	};
	
	class ForceRegistry
	{
	protected:
		struct ForceRegistration
		{
			RigidBody *rb;
			ForceGenerator *fg;
		};
		
		typedef std::vector<ForceRegistration> Registry;
		
		Registry registrations;
		
	public:
		void add(RigidBody *rb, ForceGenerator *fg);
		void remove(RigidBody *rb, ForceGenerator *fg);
		void clear();
		void updateForces(real duration);
	};
	
	class Gravity : public ForceGenerator
	{
		Ogre::Vector3 gravity;
		
	public: 
		Gravity(const Ogre::Vector3 &g);
		
		virtual void updateForce(RigidBody *rb, real duration);
	};
		
	
}


#endif