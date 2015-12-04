/*
CLASS TO MANAGE THE WORLD
Calls draw function for all models
Manages and updates dynamicsWorld which manages the physics simulation
*/

#include <GL\glew.h>

#include "World.h"
#include <glm/gtc/type_ptr.hpp>
#include "btBulletDynamicsCommon.h"

using namespace std;

const float GRAVITY = -9.81f;

World::World()
{
	// Setup Camera

	// The geometry should be loaded from a scene file

	//Set up physics parameters
	//Specify what Broadphase algorithm we want to use
	broadphase = new btDbvtBroadphase();

	//Set up collision configuration and dispatcher to manage collision detections
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfiguration);

	//The actual physics solver that allows objects to interact properly, taking all factors into account
	solver = new btSequentialImpulseConstraintSolver;

	//Set up dynamics world
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
	dynamicsWorld->setGravity(btVector3(0.0, GRAVITY, 0));

	//Add debugger
	debugDrawer = new GLBulletDebugDrawer();
	debugDrawer->setDebugMode(btIDebugDraw::DBG_DrawAabb | btIDebugDraw::DBG_DrawWireframe);
	dynamicsWorld->setDebugDrawer(debugDrawer);


}

void World::init()
{
	snowManager.init();
}

World::~World()
{
	// Physical Models
	//Remove rigidbodies from dynamicsWorld's list of rigid bodies, then delete the rigidbody

	//delete rigidbodies

	//delete other things

	//Delete physics parameters
	delete dynamicsWorld;
	delete solver;
	delete dispatcher;
	delete broadphase;
}

btRigidBody* World::createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape)
{
	//no need to calculate inertia for static objects (mass == 0)
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		shape->calculateLocalInertia(mass, localInertia);

	//motionstate provides interpolation capabilities

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

	btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);

	btRigidBody* body = new btRigidBody(cInfo);

	body->setUserIndex(-1);
	dynamicsWorld->addRigidBody(body);
	return body;
}

void World::testSnow()
{

	if (dynamicsWorld->getDebugDrawer())
		dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	///create a few basic rigid bodies
	btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(15.), btScalar(15.), btScalar(15.)));

	//groundShape->initializePolyhedralFeatures();
	//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);

	collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -15, 0));


	btScalar mass(0.);
	createRigidBody(mass, groundTransform, groundShape);

	///create a box to test against
	btBoxShape* boxShape = new btBoxShape(btVector3(btScalar(.5), btScalar(.5), btScalar(.5)));

	//groundShape->initializePolyhedralFeatures();
	//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);

	collisionShapes.push_back(boxShape);

	btTransform boxTrans;
	boxTrans.setIdentity();
	boxTrans.setOrigin(btVector3(2, 55, 2));

	mass = btScalar(10.0f);

	btRigidBody* boxBody = createRigidBody(mass, boxTrans, boxShape);
	rigidBodies.push_back(boxBody);



	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btSphereShape* colShape = snowManager.snowShape;

		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(0.2f);


		for (int k = 0; k<10; k++)
		{
			for (int i = 0; i<10; i++)
			{
				for (int j = 0; j<10; j++)
				{
					startTransform.setOrigin(btVector3(
						btScalar(0.8*i),
						btScalar(0 + 0.8*k),
						btScalar(0.8*j)));


					btRigidBody* body = createRigidBody(mass, startTransform, colShape);

					body->setAngularFactor(btScalar(0.2f));
					body->setDamping(btScalar(0.5f), btScalar(0.95));
					//body->setGravity(btVector3(.0f, -2.0f, 0.0f));
					rigidBodies.push_back(body);
					RigidSnowflake newSnowflake(body, 0.8f, 0.8f);
					newSnowflake.setWindForce(btVector3(20.0f, 15.0f, 20.0f));
					//newSnowflake.SetApplyWind(true);
					snowManager.snowflakes.push_back(newSnowflake);
					body = NULL;


				}
			}
		}
	}



	getBulletDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawAabb);
}

void World::Update(float dt)
{
	snowManager.Update(dt);
	//Step through physics simulation
	dynamicsWorld->stepSimulation(dt);

	// Update models
	/*
	for (vector<Model*>::iterator it = mModel.begin(); it < mModel.end(); ++it)
	{
	(*it)->Update(dt);
	}
	*/
}


void World::Draw()
{
	GLdouble modelview[16];
	GLdouble projection[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);


	glm::mat4 viewMatrix = glm::make_mat4(modelview);

	glm::mat4 projectMatrix = glm::make_mat4(projection);

	snowManager.Draw(viewMatrix, projectMatrix);

	//Draw Bullet debug information
	dynamicsWorld->debugDrawWorld();

}