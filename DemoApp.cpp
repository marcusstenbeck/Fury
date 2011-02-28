//|||||||||||||||||||||||||||||||||||||||||||||||

#include "DemoApp.h"

#include <OgreLight.h>
#include <OgreWindowEventUtilities.h>

//|||||||||||||||||||||||||||||||||||||||||||||||

DemoApp::DemoApp()
{
	m_pCubeNode			= 0;
	m_pCubeEntity		= 0;
}

//|||||||||||||||||||||||||||||||||||||||||||||||

DemoApp::~DemoApp()
{
	delete OgreFramework::getSingletonPtr();
	delete fury::Core::getSingletonPtr();
}

//|||||||||||||||||||||||||||||||||||||||||||||||

void DemoApp::startDemo()
{
	new OgreFramework();
    if(!OgreFramework::getSingletonPtr()->initOgre("TNM085 a.1 (OfDoom)", this, 0))
        return;
	
	m_bShutdown = false;

	OgreFramework::getSingletonPtr()->m_pLog->logMessage("Demo initialized!");
	
	new fury::Core();

	setupDemoScene();
	runDemo();
}

//|||||||||||||||||||||||||||||||||||||||||||||||

void DemoApp::setupDemoScene()
{
	OgreFramework::getSingletonPtr()->m_pSceneMgr->setSkyBox(true, "Examples/SpaceSkyBox");

	OgreFramework::getSingletonPtr()->m_pSceneMgr->createLight("Light")->setPosition(75,75,75);
	OgreFramework::getSingletonPtr()->m_pSceneMgr->getLight("Light")->setDiffuseColour(Ogre::ColourValue(1.0, 0.7, 0.7));

	m_pCubeEntity = OgreFramework::getSingletonPtr()->m_pSceneMgr->createEntity("Cube", Ogre::SceneManager::PT_CUBE);
	m_pCubeNode = OgreFramework::getSingletonPtr()->m_pSceneMgr->getRootSceneNode()->createChildSceneNode("CubeNode");
	m_pCubeNode->attachObject(m_pCubeEntity);
	m_pCubeNode->setScale(0.01, 0.01, 0.01);

	
	fury::Core::getSingletonPtr()->gor.add(new fury::RigidBody(), m_pCubeNode, std::string("ass"));
	
	
	rb = fury::Core::getSingletonPtr()->gor.getGameObjectRegistration(std::string("ass"))->rb;
	rb->setPosition(Ogre::Vector3(0.0, 5.0, 0.0));
	rb->setInertiaTensor((rb->getMass()/12) * Ogre::Matrix3(0.5, 0.0, 0.0,
															  0.0, 0.5, 0.0,
															  0.0, 0.0, 0.5
															  ));
	rb->addForceAtPoint(Ogre::Vector3(0.0, 100.0, 0.0), rb->position + Ogre::Vector3(0.0, 0.0, 0.0));
	
	
	fury::Core::getSingletonPtr()->fr.add(rb, new fury::Gravity(Ogre::Vector3(0.0, -9.82, 0.0)));
	
	m_pCubeEntity = OgreFramework::getSingletonPtr()->m_pSceneMgr->createEntity("Cube2", Ogre::SceneManager::PT_CUBE);
	m_pCubeNode = OgreFramework::getSingletonPtr()->m_pSceneMgr->getRootSceneNode()->createChildSceneNode("CubeNode2");
	m_pCubeNode->attachObject(m_pCubeEntity);
	m_pCubeNode->setScale(0.01, 0.01, 0.01);
	
	fury::Core::getSingletonPtr()->gor.add(new fury::RigidBody(), m_pCubeNode, std::string("kass"));
	rb = fury::Core::getSingletonPtr()->gor.getGameObjectRegistration(std::string("kass"))->rb;
	rb->setPosition(Ogre::Vector3(1.0, 0.0, 0.0));
	rb->setInertiaTensor((rb->getMass()/12) * Ogre::Matrix3(0.5, 0.0, 0.0,
															0.0, 0.5, 0.0,
															0.0, 0.0, 0.5
															));
	rb->addForceAtPoint(Ogre::Vector3(0.0, 100.0, 0.0), rb->position + Ogre::Vector3(.01, 0.0, 0.0));
	fury::Core::getSingletonPtr()->fr.add(rb, new fury::Gravity(Ogre::Vector3(0.0, -4.0, 0.0)));
}

//|||||||||||||||||||||||||||||||||||||||||||||||

void DemoApp::runDemo()
{
	OgreFramework::getSingletonPtr()->m_pLog->logMessage("Start main loop...");
	
	double timeSinceLastFrame = 0;
	double startTime = 0;

	OgreFramework::getSingletonPtr()->m_pRenderWnd->resetStatistics();
	
	while(!m_bShutdown && !OgreFramework::getSingletonPtr()->isOgreToBeShutDown()) 
	{
		if(OgreFramework::getSingletonPtr()->m_pRenderWnd->isClosed())m_bShutdown = true;

#if OGRE_PLATFORM != OGRE_PLATFORM_IPHONE
		Ogre::WindowEventUtilities::messagePump();
#endif	
		if(OgreFramework::getSingletonPtr()->m_pRenderWnd->isActive())
		{
			startTime = OgreFramework::getSingletonPtr()->m_pTimer->getMillisecondsCPU();
			
			OgreFramework::getSingletonPtr()->m_pKeyboard->capture();
			OgreFramework::getSingletonPtr()->m_pMouse->capture();
			OgreFramework::getSingletonPtr()->updateOgre(timeSinceLastFrame);
			
			if(timeSinceLastFrame > 0)
			{
				fury::Core::getSingletonPtr()->gor.runCollisions();
				fury::Core::getSingletonPtr()->fr.updateForces(timeSinceLastFrame/1000);
				fury::Core::getSingletonPtr()->gor.updateSceneNodes(timeSinceLastFrame/1000);
			}
			
			OgreFramework::getSingletonPtr()->m_pRoot->renderOneFrame();
		
			timeSinceLastFrame = OgreFramework::getSingletonPtr()->m_pTimer->getMillisecondsCPU() - startTime;
		}
		else
		{
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
           Sleep(1000);
#elif OGRE_PLATFORM == OGRE_PLATFORM_APPLE
           sleep(1000);
#endif
		}
	}

	OgreFramework::getSingletonPtr()->m_pLog->logMessage("Main loop quit");
	OgreFramework::getSingletonPtr()->m_pLog->logMessage("Shutdown OGRE...");
}

//|||||||||||||||||||||||||||||||||||||||||||||||

bool DemoApp::keyPressed(const OIS::KeyEvent &keyEventRef)
{
#if OGRE_PLATFORM != OGRE_PLATFORM_IPHONE
	OgreFramework::getSingletonPtr()->keyPressed(keyEventRef);
	
	if(OgreFramework::getSingletonPtr()->m_pKeyboard->isKeyDown(OIS::KC_F))
	{
		//rb->addForceAtPoint(Ogre::Vector3(0.0, 1000.0, 0.0), Ogre::Vector3(0.0, 1.0, 0.0).randomDeviant(Ogre::Radian(Ogre::Math::PI)));
		rb->addForceAtPoint(Ogre::Vector3(0.0, 150.0, 0.0), rb->position + Ogre::Vector3(.05, 0.0, 0.0));
	}
	if(OgreFramework::getSingletonPtr()->m_pKeyboard->isKeyDown(OIS::KC_G))
	{
		rb->addForceAtPoint(Ogre::Vector3(0.0, 150.0, 0.0), rb->position + Ogre::Vector3(-.05, 0.0, 0.0));
	}
	if(OgreFramework::getSingletonPtr()->m_pKeyboard->isKeyDown(OIS::KC_H))
	{
		rb->addForceAtPoint(Ogre::Vector3(50.0, 100.0, 0.0), rb->position + Ogre::Vector3(0.0, 0.0, .05));
	}
#endif
	return true;
}

//|||||||||||||||||||||||||||||||||||||||||||||||

bool DemoApp::keyReleased(const OIS::KeyEvent &keyEventRef)
{
	OgreFramework::getSingletonPtr()->keyReleased(keyEventRef);
	
	return true;
}

//|||||||||||||||||||||||||||||||||||||||||||||||
