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

using namespace fury;

struct PotentialContact
{
	// Lagrar kropparna som möjligtvis är i kontakt.
	RigidBody* body[2];
};


template<class BoundingVolumeClass>
class BVHNode
{
public:
	// Lagrar child nodes till denna node.
	BVHNode * children[2];
	
	// Lagrar volymen som omsluter alla barn till denna noden.
	BoundingVolumeClass volume;
	
	RigidBody * body;
	
	// Lagrar noden precis ovan.
	BVHNode * parent;
	
	// Skapar en ny nod i hierarkin med givna parametrar.
	BVHNode(BVHNode *parent, const BoundingVolumeClass &volume,
            RigidBody* body=NULL)
	: parent(parent), volume(volume), body(body)
	{
		children[0] = children[1] = NULL;
	}
	
	// Kollar om noden är "längst ner" i trädet.
	bool isLeaf() const
	{
		return (body != NULL);
	}
	
	
	// Kollar möjliga kontakter från denna nod och neråt i trädet 
	// och sparar dem i en array.
	unsigned getPotentialContacts(PotentialContact * contacts, unsigned limit) const;
	
	bool overlaps(const BVHNode<BoundingVolumeClass> * other) const;
	
	unsigned getPotentialContactsWith(const BVHNode<BoundingVolumeClass> * other, PotentialContact* contacts, unsigned limit) const;
	
	void insert(RigidBody* body, const BoundingVolumeClass &volume);
	
	~BVHNode();
	
protected:
	// För icke-leaf nodes
	// Metoden bäräkna om "bounding volume" av dess barn.
	void recalculateBoundingVolume(bool recurse = true);
	
};


template<class BoundingVolumeClass>
void BVHNode<BoundingVolumeClass>::insert(RigidBody* newBody, const BoundingVolumeClass &newVolume)
{
	if(isLeaf())
	{
		children[0] = new BVHNode<BoundingVolumeClass>(this, volume, body);
		children[1] = new BVHNode<BoundingVolumeClass>(this, volume, body);
		
		this->body = NULL;
		
		recalculateBoundingVolume();
	}
	else
	{
		if (children[0]->volume.getGrowth(newVolume) < children[1]->volume.getGrowth(newVolume))
		{
			children[0]->insert(newBody, newVolume);
		}
		else
		{
			children[1]->insert(newBody, newVolume);
		}
	}
}

template<class BoundingVolumeClass>
bool BVHNode<BoundingVolumeClass>::overlaps(const BVHNode<BoundingVolumeClass> * other) const
{
	return volume->overlaps(other->volume);
}

template<class BoundingVolumeClass>
unsigned BVHNode<BoundingVolumeClass>::getPotentialContacts(PotentialContact* contacts, unsigned limit) const
{
	//Om vi ej har utrymme för kontakter eller om vi är längst ner i hierarkin.
	if (isLeaf() || limit == 0) return 0;
	
	//Får de möjliga kontakterna mellan ett av barnen och ett annat barn.
	return children[0]->getPotentialContactsWith(children[1], contacts, limit);
}

template<class BoundingVolumeClass>
unsigned BVHNode<BoundingVolumeClass>::getPotentialContactsWith(const BVHNode<BoundingVolumeClass> * other, PotentialContact* contacts, unsigned limit) const
{
	// Om vi inte kolliderar eller om vi inte har plats att spara kontakter.
	if(!overlaps(other) || limit == 0) return 0;
	
	// Om båda objekten är längst ner i hierarkin så har vi en möjlig kontakt.
	if(isLeaf() && other->isLeaf())
	{
		contacts->body[0] = body;
		contacts->body[1] = other->body;
		return 1;
	}
	
	// Bestämmer vilken nod man ska fortsätta till i trädet.
	// Om ingen av dem är ett löv så väljer man den med störst volym.
	if(other->isLeaf() || (!isLeaf() && volume->getSize() >= other->volume->getSize()))
	{
		// RECURSE INTO OURSELF
		unsigned count = children[0]->getPotentialContactsWith(other, contacts, limit);
		
		// Kollar ifall vi har tillräckligt med ledig plats för att kolla även den andra kontaktytan.
		if(limit > count)
		{
			return count + children[1]->getPotentialContactsWith(other, contacts+count, limit-count);
		}
		else
		{
			return count;
		}
	}
	
	else
	{
		// Recurse into the other node
		unsigned count = getPotentialContactsWith(other->children[0], contacts, limit);
		
		// Kollar ifall vi har tillräckligt med ledig plats för att kolla även den andra kontaktytan.
		if(limit > count)
		{
			return count + getPotentialContactsWith(other->children[1], contacts + count, limit - count);
		}
		else
		{
			return count;
		}
	}
}

template<class BoundingVolumeClass>
BVHNode<BoundingVolumeClass>::~BVHNode<BoundingVolumeClass>()
{
	// Om vi har en parent, annars ignorera.
	if (parent)
	{
		// Hitta syskon.
		BVHNode<BoundingVolumeClass> *sibling;
		if (parent->children[0] == this) sibling = parent->children[1];
		else sibling = parent->children[0];
		
		// Skriv över syskonets information till dess parent.
		parent->volume = sibling->volume;
		parent->body = sibling->body;
		parent->children[0] = sibling->children[0];
		parent->children[1] = sibling->children[1];
		
		// Ta bort syskonet.
		sibling->parent = NULL;
		sibling->body = NULL;
		sibling->children[0] = NULL;
		sibling->children[1] = NULL;
		delete sibling;
		
		// Uppdatera volymen (boundingVolume) för parent.
		parent->recalculateBoundingVolume();
	}
	
	// Ta bort children.
	if (children[0]) {
		children[0]->parent = NULL;
		delete children[0];
	}
	if (children[1]) {
		children[1]->parent = NULL;
		delete children[0];
	}
}









struct BoundingSphere
{
	Ogre::Vector3 center;
	real radius;
	
public:
	// Skapar en ny BoundingSphere med given radie och centrum.
	BoundingSphere(const Ogre::Vector3 &center, real radius);
	
	// Skapar en omslutande boundingSphere till två andra spheres.
	BoundingSphere(const BoundingSphere &one, const BoundingSphere &two);
	
	// Kollar om boundingSphere överlappar med nån annan boundingSphere.
	bool overlaps(const BoundingSphere *other) const;
};






#endif