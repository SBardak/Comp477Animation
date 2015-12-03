/*
* Quaternion
*/
#ifndef SNOWGLOBE_H
#define SNOWGLOBE_H

#pragma managed(push, off)
#include <btBulletDynamicsCommon.h>
#include <BulletCollision\Gimpact\btGImpactShape.h>
#include "BulletCollision\CollisionShapes\btCompoundShape.h"
#pragma managed(pop)

#include "./glm.h"
#include <string>

#define BIT(x) (1<<(x))
enum collisiontypes {
	COL_NOTHING = 0, //<Collide with nothing
	COL_GLOBE = BIT(1), //<Collide globe
	COL_PLANE = BIT(2), //<Collide plane
};

class SnowGlobe
{
private:
	bool showSphere = false;
	bool showGlobe = true;
	bool showPlane = false;
	int mass = 0;
	int collisionSphereScale = 980;
	float sphereOrigin = 5.3;
	float planeOrigin = 1.8;

	/* Models */
	GLMmodel * pmodelCollisionSphere;
	GLMmodel * pmodelGlobe;

	/* Bullet */
	btDynamicsWorld *world;
	btRigidBody *planeBody;
	btRigidBody *sphereBody;

	/* Loading */
	void SnowGlobe::loadCollisionSphere();
	void SnowGlobe::loadGlobeMesh();
	void SnowGlobe::loadMesh(GLMmodel *pmodel, std::string mesh);

	void prepareCollisionPlane();
	void removeBody(btRigidBody *body);
public:
	
	SnowGlobe();
	~SnowGlobe();
	
	void SnowGlobe::init(btDynamicsWorld *world);
	void glDraw();

	void toggleCollisionSphere() { showSphere = !showSphere; }
	void toggleGlobe() { showGlobe = !showGlobe; }
	void toggleCollisionPlane() { showPlane = !showPlane; }

	void move(float x, float y, float z);

	//btRigidBody* getRigidBody()
};


#endif
