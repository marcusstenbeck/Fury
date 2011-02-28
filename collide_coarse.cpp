/*
 *  collide_coarse.cpp
 *  tnm085
 *
 *  Created by Oscar Persson on 2011-02-01.
 *  Copyright 2011 Linköpings Univeristet. All rights reserved.
 *
 */

#include "collide_coarse.h"
#include "precision.h"

using namespace fury;

BoundingSphere::BoundingSphere(const Ogre::Vector3 &center, real radius)
{
    BoundingSphere::center = center;
    BoundingSphere::radius = radius;
}

bool BoundingSphere::overlaps(const BoundingSphere *other) const
{
	real distanceSquared = (center - other->center).squaredLength();
	return distanceSquared < (radius + other->radius)* (radius + other->radius);
}

BoundingSphere::BoundingSphere(const BoundingSphere &one,
                               const BoundingSphere &two)
{
	Ogre::Vector3 centerOffset = two.center - one.center;
    real distance = centerOffset.squaredLength();
    real radiusDiff = two.radius - one.radius;
	
    // Check if the larger sphere encloses the small one
    if (radiusDiff*radiusDiff >= distance)
    {
        if (one.radius > two.radius)
        {
            center = one.center;
            radius = one.radius;
        }
        else
        {
            center = two.center;
            radius = two.radius;
        }
    }
	
    // Otherwise we need to work with partially
    // overlapping spheres
    else
    {
        distance = Ogre::Math::Sqrt(distance);
        radius = (distance + one.radius + two.radius) * ((real)0.5);
		
        // The new centre is based on one's centre, moved towards
        // two's centre by an ammount proportional to the spheres'
        // radii.
        center = one.center;
        if (distance > 0)
        {
            center += centerOffset * ((radius - one.radius)/distance);
        }
    }	
}


real BoundingSphere::getGrowth(const BoundingSphere &other) const
{
    BoundingSphere newSphere(*this, other);
	
    // We return a value proportional to the change in surface
    // area of the sphere.
    return newSphere.radius*newSphere.radius - radius*radius;
}