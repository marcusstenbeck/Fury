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

bool BoundingSphere::overlaps(const BoundingSphere *other) const
{
	real distanceSquared = (center - other->center).squaredLength();
	return distanceSquared < (radius + other->radius)* (radius + other->radius);
}