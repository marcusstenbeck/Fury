/*
 *  Core.cpp
 *  tnm085
 *
 *  Created by Marcus Stenbeck on 2011-01-31.
 *  Copyright 2011 Macatak Media. All rights reserved.
 *
 */

#include "Core.h"

using namespace fury;

template<> Core* Ogre::Singleton<Core>::ms_Singleton = 0;
Core* Core::getSingletonPtr(void)
{
    return ms_Singleton;
}
Core& Core::getSingleton(void)
{  
    assert( ms_Singleton );  return ( *ms_Singleton );  
}
