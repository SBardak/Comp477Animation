#include "BulletUtility.h"

btRigidBody* createRigidBody(btDynamicsWorld* world, float mass, const btTransform& startTransform, btCollisionShape* shape)
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
	world->addRigidBody(body);
	return body;
}

btRigidBody* createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape)
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
	return body;
}