#ifndef BULLET_UTILITY_H
#define BULLET_UTILITY_H

#include <btBulletDynamicsCommon.h>

// Creates and adds to world and returns rigidbody
btRigidBody* createRigidBody(btDynamicsWorld* world, float mass, const btTransform& startTransform, btCollisionShape* shape);

// Creates and returns rigidbody
btRigidBody* createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape);

#endif