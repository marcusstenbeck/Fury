/*
 *  GameObject.cpp
 *  tnm085
 *
 *  Created by Marcus Stenbeck on 2011-01-31.
 *  Copyright 2011 Macatak Media. All rights reserved.
 *
 */

#include "GameObject.h"
#include "OgreFramework.h"

using namespace Ogre;
using namespace fury;

void GameObjectRegistry::add(RigidBody *rb, Ogre::SceneNode *sn, std::string gName)
{
	GameObjectRegistration gor;
	
	gor.sn = sn;
	gor.rb = rb;
	gor.gName = gName;
	
	registrations.push_back(gor);
	
	// Om det inte redan finns en toppnod
	// i boundingvolumeheirarkin så skapar vi toppnoden
	
	
};

void GameObjectRegistry::remove(RigidBody *rb, Ogre::SceneNode *sn)
{
	for (Registry::iterator i = registrations.begin(); i != registrations.end(); ++i)
	{
		if(i->sn == sn && i->rb == rb)
		{
			registrations.erase(i);
			return;
		}
	}
};

void GameObjectRegistry::clear()
{
	registrations.clear();
};

void GameObjectRegistry::updateSceneNodes(real duration)
{
	for (Registry::iterator i = registrations.begin(); i != registrations.end(); ++i)
	{
		i->rb->integrate(duration);
		i->sn->_setDerivedPosition(i->rb->position);
		i->sn->setOrientation(i->rb->orientation);
	}
};

struct GameObjectRegistry::GameObjectRegistration* GameObjectRegistry::getGameObjectRegistration(std::string s)
{
	for (Registry::iterator i = registrations.begin(); i != registrations.end(); ++i)
	{
		if (i->gName == s) 
		{
			return &*i;
		}
	}
	
	return NULL;
};

void GameObjectRegistry::runCollisions()
{
	std::vector<PotentialContact> pcr;
	
	for (Registry::iterator i = registrations.begin(); i != registrations.end(); ++i)
	{
		// testa en registration mot alla andra registrations
		BoundingSphere* thisBs = new BoundingSphere(i->rb->position, .7);
		
		for (Registry::iterator j = registrations.begin(); j != registrations.end(); ++j)
		{
			if (j != i)
			{
				BoundingSphere* thatBs = new BoundingSphere(j->rb->position, .7);
				
				if(thisBs->overlaps(thatBs))
				{
					PotentialContact pc;
					pc.body[0] = i->rb;
					pc.body[1] = j->rb;
					
					pcr.push_back(pc);
					std::cout << "Overlap!" << std::endl;
				}
				
				delete thatBs;
			}
		}
		
		delete thisBs;
	}
};