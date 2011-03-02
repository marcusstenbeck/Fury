/*
 *  collide_coarse.h
 *  tnm085
 *
 *  Created by Oscar Persson on 2011-02-01.
 *  Copyright 2011 Linköpings Univeristet. All rights reserved.
 *
 */

#ifndef COLLIDE_COARSE_H
#define COLLIDE_COARSE_H

#include <OgreVector3.h>
#include "precision.h"
#include "RigidBody.h"

namespace fury
{

	/**
     * Represents a bounding sphere that can be tested for overlap.
     */
    struct BoundingSphere
    {
		Ogre::Vector3 center;
        real radius;
		
    public:
        /**
         * Creates a new bounding sphere at the given center and radius.
         */
        BoundingSphere(const Ogre::Vector3 &center, real radius);
		
        /**
         * Creates a bounding sphere to enclose the two given bounding
         * spheres.
         */
        BoundingSphere(const BoundingSphere &one, const BoundingSphere &two);
		
        /**
         * Checks if the bounding sphere overlaps with the other given
         * bounding sphere.
         */
        bool overlaps(const BoundingSphere *other) const;
		
        /**
         * Reports how much this bounding sphere would have to grow
         * by to incorporate the given bounding sphere. Note that this
         * calculation returns a value not in any particular units (i.e.
         * its not a volume growth). In fact the best implementation
         * takes into account the growth in surface area (after the
         * Goldsmith-Salmon algorithm for tree construction).
         */
        real getGrowth(const BoundingSphere &other) const;
		
        /**
         * Returns the volume of this bounding volume. This is used
         * to calculate how to recurse into the bounding volume tree.
         * For a bounding sphere it is a simple calculation.
         */
        real getSize() const
        {
            return ((real)1.333333) * Ogre::Math::PI * radius * radius * radius;
        }
    };
	
    /**
     * Stores a potential contact to check later.
     */
    struct PotentialContact
    {
        /**
         * Holds the bodies that might be in contact.
         */
		RigidBody* body[2];
    };

} // end namespace fury


#endif