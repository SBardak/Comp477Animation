#include "present.h"
//#include <BulletCollision\Gimpact\btCompoundFromGimpact.h>

#define BIT(x) (1<<(x))
enum collisiontypes {
	COL_NOTHING = 0, //<Collide with nothing
	COL_GLOBE = BIT(1), //<Collide globe
	COL_PLANE = BIT(2), //<Collide plane
};

#include <iostream>

void Present::loadMesh()
{
	pmodel = NULL;
	if (!pmodel) {	/* load up the model */

		char meshFile[] = "model/present.obj";
		pmodel = glmReadOBJ(meshFile);
		if (!pmodel) {
			return;
		}
		//glmUnitize(pmodel);
		glmFacetNormals(pmodel);
		glmVertexNormals(pmodel, 0);
		glmFacetNormals(pmodel);
	}

}

Present::Present(float x, float y, float z, float mass, float scale) :
x(x), y(y), z(z), mass(mass), scale(scale)
{

}

void Present::init(btDynamicsWorld *world)
{
	/* Start by loading model */
	loadMesh();

	/* Physics */
	/* ================================================================== */
	this->world = world;
	btTransform t;
	t.setIdentity();
	t.setOrigin(btVector3(x, y, z));
	//2, 2, 3, 0, 5, 0, 20);
	btBoxShape* box = new btBoxShape(btVector3(width * scale / 2.0, height * scale / 2.0, depth * scale / 2.0));
	btVector3 inertia(0, 0, 0);
	if (mass != 0.0)
		box->calculateLocalInertia(mass, inertia);

	btMotionState* motion = new btDefaultMotionState(t);
	btRigidBody::btRigidBodyConstructionInfo info(mass, motion, box, inertia);
	body = new btRigidBody(info);
	int all = collisiontypes::COL_GLOBE | collisiontypes::COL_PLANE;

	world->addRigidBody(body, all, all);
}

void Present::glDraw()
{
	int mode;
	mode = GLM_MATERIAL | GLM_SMOOTH;

	btTransform t;
	body->getMotionState()->getWorldTransform(t);
	float mat[16];
	t.getOpenGLMatrix(mat);

	glColor3f(0.5, 1, 0.5);
	glPushMatrix();
		glMultMatrixf(mat);     //translation,rotation
		glScalef(scale, scale, scale);
		glmDraw(pmodel, mode);
	glPopMatrix();
	glEnable(GL_COLOR_MATERIAL);
}

void Present::Delete()
{
	world->removeCollisionObject(body);
	btMotionState* motionState = body->getMotionState();
	btCollisionShape* shape = body->getCollisionShape();
	delete body;
	delete shape;
	delete motionState;

	delete pmodel;
}

Present::~Present()
{
}

void Present::setMaterialColor(int mat, float r, float g, float b)
{
	if (mat >= pmodel->nummaterials)
		return;
	pmodel->materials[mat].diffuse[0] = r;
	pmodel->materials[mat].diffuse[1] = g;
	pmodel->materials[mat].diffuse[2] = b;
}
void Present::setBoxColor(float r, float g, float b)
{
	setMaterialColor(2, r, g, b);
}
void Present::setBowColor(float r, float g, float b)
{
	setMaterialColor(1, r, g, b);
}