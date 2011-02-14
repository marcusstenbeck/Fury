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

void GameObjectRegistry::add(RigidBody *rb, Ogre::SceneNode *sn)
{
	GameObjectRegistration gor;
	
	gor.sn = sn;
	gor.rb = rb;
	
	registrations.push_back(gor);
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