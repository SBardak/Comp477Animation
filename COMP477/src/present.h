/*
* Present
*/
#ifndef PRESENT_H
#define PRESENT_H

#pragma managed(push, off)
#include <btBulletDynamicsCommon.h>
#include <BulletCollision\Gimpact\btGImpactShape.h>
#include <BulletCollision\CollisionShapes\btCompoundShape.h>
#include "./glm.h"
#pragma managed(pop)

#include <string>

class Present
{
private:
	float width = 2.6f, height = 1.8f, depth = 2.4f,
		x, y, z, mass = 1, scale = 0.5;

	/* Models */
	GLMmodel * pmodel;

	/* Bullet */
	btDynamicsWorld *world;
	//btRigidBody *sphereBody2;

	/* Loading */
	void loadMesh();

	void setMaterialColor(int mat, float r, float g, float b);
public:
	btRigidBody *body;
	Present(float x, float y, float z, float mass, float scale);
	~Present();
	void Delete();

	void Present::init(btDynamicsWorld *world);
	void glDraw();

	void setBoxColor(float r, float g, float b);
	void setBowColor(float r, float g, float b);
};


#endif
