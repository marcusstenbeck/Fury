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
#include "contacts.h"
#include "collide_fine.h"
#include "RigidBody.h"

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

void GameObjectRegistry::runCollisions(real duration)
{
	// Leta efter grova kollisioner med andra RigidBody's
	
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
					if(pcr.size() == 0)
					{
						PotentialContact pc;
						pc.body[0] = i->rb;
						pc.body[1] = j->rb;
						
						pcr.push_back(pc);
					}
					else
					{
						bool collisionExists = false;
						
						for (std::vector<PotentialContact>::iterator m = pcr.begin(); m != pcr.end(); ++m)
						{
							if( (m->body[0] == i->rb && m->body[1] == j->rb) || (m->body[0] == j->rb && m->body[1] == i->rb) )
							{
								collisionExists = true;
							}
						}
						
						if (!collisionExists)
						{
							PotentialContact pc;
							pc.body[0] = i->rb;
							pc.body[1] = j->rb;
							
							pcr.push_back(pc);
						}
					}
				}
				
				delete thatBs;
			}
		}
		
		delete thisBs;
		
		//std::cout << "Possible object-to-object collisions: " << pcr.size() << std::endl;
		
		// Nu har vi fått en lista på potentiella kontakter.
		// Kolla dessa med fine collision.
		
		CollisionData cd;
		Contact contacts[512];
		
		cd.contactsArray = contacts;
		cd.contact = contacts;
		cd.contactCount = 0;
		cd.contactsLeft = 512;
		cd.tolerance = 0.0001;
		cd.restitution = .4;
		cd.friction = 0.9;
		
		Plane flr;
		flr.normal = Ogre::Vector3(0.0, 1.0, 0.0);
		flr.offset = 0.0;
		
		// kolla kollision med golvet
		for (Registry::iterator i = registrations.begin(); i != registrations.end(); ++i)
		{
			Box b;
			b.body = i->rb;
			b.halfSize = Ogre::Vector3(.5, .5, .5);
			
			CollisionDetector::boxAndHalfSpace(b, flr, &cd);
		}
		
		
		// Nu har vi möjligtvis kollisioner
		
		if(cd.contactsLeft < 512)
			std::cout << "Collisions: " << cd.contactCount << std::endl;
		
		
		/*
		for (Contact* contact=contacts; contact < (contacts + cd.contactCount); contact++)
		{
			std::cout << "cp: " << contact->body[0]->position << std::endl;
		}
		*/
		
		//ContactResolver cr(512);
		
		//cr.resolveContacts(cd.contactsArray, 512 - cd.contactsLeft, duration);
		
		/*
		 ** Behˆver dra ner flerpunktskontakt.
		 ** "If there are initially several collisions to be resoloved 
		 ** (such as a cube dropped flat onto a plane , with all four vertices colliding at once)
		 ** the order of the contact list may have an effect on the simulation." -BARAFF, GOD OF GRAPHICS
		 */
		
		Contact* c  = cd.contactsArray;
		int numContacts = 512 - cd.contactsLeft;
		int tick = 1;
		
		for (int i= (numContacts-1) ; i>0; i--)
		{
			if ( c[i].body[0] == c[i-1].body[0]  &&  c[i].contactNormal == c[i-1].contactNormal) //Om det ‰r samma RigidBody och samma contactNormal , sammans‰tt dem.
			{
				std::cout << "Contact fusion! " << std::endl;
				
				//std::cout << c[i].contactPoint << " + " << c[i-1].contactPoint << std::endl;
				
				c[i-1].contactPoint = (c[i].contactPoint + c[i-1].contactPoint);
				
				
				//TO - DO : Kolla penetration, vilken som har mest. Uppdatera den nya kontakten. (Kanske inte behˆvs).
				if ( c[i-1].penetration < c[i].penetration) c[i-1].penetration = c[i].penetration;
				
				cd.contactCount--;
				tick++;
				
			}
			
			if ( c[i-2].body[0] != c[i-1].body[0] )
			{
				//std::cout << "tick: " << tick << std::endl;
				//Normalize contact point.
				c[i-1].contactPoint = c[i-1].contactPoint/tick;
				
				//std::cout << c[i-1].contactPoint << std::endl;
				tick = 1;
			}
		}
		
		
		
		/*
		 ** Fixa en l‰ttare interpenetrationslˆsare
		 ** Flyttning l‰ngs contactNormal verkar tillr‰ckligt enkel.
		 */
		
		c = cd.contactsArray;
		
		for (int k = 0 ; k< cd.contactCount; k++ )
		{
			//Kommer inte pÂ hur man r‰knar ut detta sÂ den kˆrs godtyckligt l‰ngs contactNormal. 
			//HÂrdkodar in att y ‰r l‰ngs normalen i detta fall.
			
			//Antar att man ska r‰kna nÂn skal‰rprodukt. elller?
			
			//c[i].body[0]->position.y -= c[i].penetration; //penetration ‰r negativ dÂ den ‰r under planet? 
			
			// Detta kanske st‰mmer.
			
			std::cout << "penetration : " <<  c->penetration << std::endl;
			c->body[0]->position += c->penetration*c->contactNormal; // elementvis multiplikation
			
			std::cout << "position efter penetrationfix : " << c->body[0]->position << std::endl;
			
			c->contactPoint += c->penetration*c->contactNormal;
			
			//Os‰ker h‰r pÂ hur parametrarna ska vara. Men bara och fixa om sÂ det fungerar....
			//if (ContactResolver::colliding(c))
				ContactResolver::resolveContact(c,c->restitution);
			
			
			
			
			
			
			c++;
		}
	}
};