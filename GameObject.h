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
#include "collide_coarse.h"
#include <string>

namespace fury
{
	class GameObjectRegistry
	{
	protected:
		struct GameObjectRegistration
		{
			std::string gName;
			RigidBody* rb;
			Ogre::SceneNode* sn;
		};
		
		typedef std::vector<GameObjectRegistration> Registry;
		Registry registrations;
		
	public:
		
		void add(RigidBody *rb, Ogre::SceneNode *sn, std::string gName);
		void remove(RigidBody *rb, Ogre::SceneNode *sn);
		void clear();
		
		void updateSceneNodes(real duration);
		
		void runCollisions();
		
		struct GameObjectRegistration* getGameObjectRegistration(std::string s);
	};

}
#endif