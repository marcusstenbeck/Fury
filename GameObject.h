/*
 *  GameObject.h
 *  tnm085
 *
 *  Created by Marcus Stenbeck on 2011-01-31.
 *  Copyright 2011 Macatak Media. All rights reserved.
 *
 */
#ifndef GAMEOBJECT_H
#define GAMEOBJECT_H

#include "OgreFramework.h"
#include "RigidBody.h"

namespace fury
{
	class GameObjectRegistry
	{
	protected:
		struct GameObjectRegistration
		{
			RigidBody* rb;
			Ogre::SceneNode* sn;
		};
		
		typedef std::vector<GameObjectRegistration> Registry;
		Registry registrations;
		
	public:
		
		void add(RigidBody *rb, Ogre::SceneNode *sn);
		void remove(RigidBody *rb, Ogre::SceneNode *sn);
		void clear();
		
		void updateSceneNodes(real duration);
	};

}
#endif