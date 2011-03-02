/*
 *  Core.h
 *  tnm085
 *
 *  Created by Marcus Stenbeck on 2011-01-31.
 *  Copyright 2011 Macatak Media. All rights reserved.
 *
 */

#ifndef CORE_H
#define CORE_H

#include "fgen.h"
#include "GameObject.h"
#include "OgreFramework.h"
#include <vector>
#include "collide_coarse.h"

namespace fury
{
	class Core : public Ogre::Singleton<Core>
	{
	public:
		ForceRegistry fr;
		GameObjectRegistry gor;
		
		static Core& getSingleton();
		static Core* getSingletonPtr();
		
	};

}
#endif