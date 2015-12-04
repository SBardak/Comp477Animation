#ifndef WORLD_H
#define WORLD_H

#include "SnowManager.h"
#include "bullet-util\BulletDebugDrawer.h"

class World
{
public:
	World();
	~World();

	btRigidBody* createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape);
	void init();
	void testSnow();

	void Update(float dt);
	void Draw();

	GLBulletDebugDrawer* getBulletDebugDrawer() { return debugDrawer; }

	SnowManager snowManager;

private:

	btAlignedObjectArray<btRigidBody*> rigidBodies;

	btAlignedObjectArray<btCollisionShape*> collisionShapes;

	//The Broadphase algorithm to use
	btBroadphaseInterface* broadphase;

	//The collision dispatcher to manage collision detections
	btCollisionDispatcher* dispatcher;

	//The actual physics solver that allows objects to interact properly, taking all factors into account
	btSequentialImpulseConstraintSolver* solver;

	//Set up dynamics world
	btDiscreteDynamicsWorld* dynamicsWorld;

	//DebugDrawer
	GLBulletDebugDrawer* debugDrawer;
};
#endif