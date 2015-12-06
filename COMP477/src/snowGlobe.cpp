#include "snowGlobe.h"
#include <BulletCollision\Gimpact\btCompoundFromGimpact.h>

#include <iostream>

void SnowGlobe::loadCollisionSphere()
{
	pmodelCollisionSphere = NULL;
	if (!pmodelCollisionSphere) {	/* load up the model */

		char meshFile[] = "model/balls.obj";
		pmodelCollisionSphere = glmReadOBJ(meshFile);
		if (!pmodelCollisionSphere) {
			return;
		}
		//glmUnitize(pmodel);
		glmFacetNormals(pmodelCollisionSphere);
		glmVertexNormals(pmodelCollisionSphere, 0);
		glmFacetNormals(pmodelCollisionSphere);
	}
	//loadMesh(pmodelCollisionSphere, "model/balls.obj");
}

void SnowGlobe::loadGlobeMesh()
{
	pmodelGlobe = NULL;
	if (!pmodelGlobe) {	/* load up the model */

		char meshFile[] = "model/snowglobe.obj";
		pmodelGlobe = glmReadOBJ(meshFile);
		if (!pmodelGlobe) {
			return;
		}
		//glmUnitize(pmodel);
		glmFacetNormals(pmodelGlobe);
		glmVertexNormals(pmodelGlobe, 0);
		glmFacetNormals(pmodelGlobe);
	}
	//loadMesh(pmodelGlobe, "model/snowglobe.obj");
}

void SnowGlobe::loadTree()
{
	pmodelTree = NULL;
	if (!pmodelTree) {	/* load up the model */

		char meshFile[] = "model/lowpolytree.obj";
		pmodelTree = glmReadOBJ(meshFile);
		if (!pmodelTree) {
			return;
		}
		glmFacetNormals(pmodelTree);
		glmVertexNormals(pmodelTree, 0);
		glmFacetNormals(pmodelTree);
	}
}

void SnowGlobe::loadMesh(GLMmodel *pmodel, std::string mesh)
{
	pmodel = NULL;
	if (!pmodel) {	/* load up the model */

		char *test = &mesh[0u];
		pmodel = glmReadOBJ(test);
		if (!pmodel) {
			return;
		}
		//glmUnitize(pmodel);
		glmFacetNormals(pmodel);
		glmVertexNormals(pmodel, 0);
		glmFacetNormals(pmodel);
	}
}

SnowGlobe::SnowGlobe()
{

}

void SnowGlobe::init(btDynamicsWorld *world)
{
	this->world = world;
	
	/* Start by loading models */
	loadCollisionSphere();
	loadGlobeMesh();
	loadTree();

	/* Globe physics */
	/* ================================================================== */
	/* Use the sphere's triangles to create a mesh shape */
	int *tes = (int*)malloc(pmodelCollisionSphere->numtriangles * 3 * sizeof(int));
	for (int i = 0; i < pmodelCollisionSphere->numtriangles; ++i)
	{
		int start = i * 3;
		tes[start + 0] = pmodelCollisionSphere->triangles[i].vindices[0];
		tes[start + 1] = pmodelCollisionSphere->triangles[i].vindices[1];
		tes[start + 2] = pmodelCollisionSphere->triangles[i].vindices[2];
	}
	btTriangleIndexVertexArray *vertexArray = new btTriangleIndexVertexArray(
		(int)pmodelCollisionSphere->numtriangles,
		tes,
		3 * sizeof(int),
		(int)pmodelCollisionSphere->numvertices,
		pmodelCollisionSphere->vertices,
		3 * sizeof(GLfloat)
		);

	/* Inner sphere */
	btGImpactMeshShape *globeShape = new btGImpactMeshShape(vertexArray);
	int scale = collisionSphereScale * m_scale;
	globeShape->setLocalScaling(btVector3(scale, scale, scale));
	globeShape->updateBound();

	btTransform t;
	t.setIdentity();
	t.setOrigin(btVector3(0, sphereOrigin, 0));
	btMotionState* motion = new btDefaultMotionState(t);

	/* Create a compound shape to cotain all subshapes */
	/* Sphere / Box base / Tree */
	btCompoundShape* compound = new btCompoundShape();
	btTransform localTrans;
	localTrans.setIdentity();
	//localTrans effectively shifts the center of mass with respect to the chassis
	localTrans.setOrigin(btVector3(0, 0, 0));
	compound->addChildShape(localTrans, globeShape);

	btRigidBody::btRigidBodyConstructionInfo info(mass, motion, compound);

	/* Modify its values */
	info.m_friction = 0.5;
	info.m_angularDamping = 0.2;
	info.m_restitution = 0.5;
	sphereBody = new btRigidBody(info);

	/* Add the body to the world */
	world->addRigidBody(sphereBody, collisiontypes::COL_GLOBE, collisiontypes::COL_GLOBE);
	/* Remove gravity */
	sphereBody->setGravity(btVector3(0, 0, 0));
	/* Set factors to zero so only user interaction can move it */
	btVector3 z(0, 0, 0);
	sphereBody->setAngularFactor(0);
	sphereBody->setLinearFactor(z);

	sphereBody->setCcdMotionThreshold(0.50);
	sphereBody->setCcdSweptSphereRadius(10);

	/* Add the globe's base */
	t;
	t.setIdentity();
	t.setOrigin(btVector3(0, planeOrigin - (3.7f * (m_scale - 1)), 0));
	btBoxShape* box = new btBoxShape(btVector3(10 / 2.0 * m_scale, 0.5f, 10 / 2.0 * m_scale));
	localTrans.setIdentity();
	//localTrans effectively shifts the center of mass with respect to the chassis
	localTrans.setOrigin(btVector3(0, -6.5, 0));
	compound->addChildShape(localTrans, box);

	/* Add the globe's tree */
	btConeShape* tree = new btConeShape(2.4, 8.5);
	localTrans.setIdentity();
	//localTrans effectively shifts the center of mass with respect to the chassis
	localTrans.setOrigin(btVector3(0,-0.5, 0));
	compound->addChildShape(localTrans, tree);

	/* Add the globe's tree base */
	btCylinderShape* treeBase = new btCylinderShape(btVector3(0.7, 0.7, 0.7));
	localTrans.setIdentity();
	//localTrans effectively shifts the center of mass with respect to the chassis
	localTrans.setOrigin(btVector3(0, -5.5, 0));
	compound->addChildShape(localTrans, treeBase);

}

void SnowGlobe::setUpdatedOrigin(btVector3 &newOrigin){
	origin = newOrigin;
}

void SnowGlobe::glDraw()
{
	int mode;
	glColor3f(0.5, 0.5, 0.5);
	mode = GLM_MATERIAL | GLM_SMOOTH;

	if (showGlobe || showSphere)
	{
		float ccd1 = sphereBody->getCcdMotionThreshold(),
			ccd2 = sphereBody->getCcdSweptSphereRadius();

		sphereBody->setCcdMotionThreshold(ccd1);
		sphereBody->setCcdSweptSphereRadius(ccd2);


		btTransform t;
		sphereBody->getMotionState()->getWorldTransform(t);
		setUpdatedOrigin(t.getOrigin());
		float mat[16];
		t.getOpenGLMatrix(mat);

		/* Render sphere */
		if (showSphere)
		{
			int scale = collisionSphereScale * m_scale;
			glColor3f(0.5, 1, 0.5);
			glPushMatrix();
			glMultMatrixf(mat);     //translation,rotation
			glScalef(scale, scale, scale);
			glmDraw(pmodelCollisionSphere, mode);
			glPopMatrix();
			glEnable(GL_COLOR_MATERIAL);
		}

		/* Render globe & tree*/
		if (showGlobe)
		{
			int scale = 15 * m_scale;
			glPushMatrix();
			glMultMatrixf(mat);     //translation,rotation
			glTranslatef(0, -sphereOrigin * m_scale, 0);
			glScalef(scale, scale, scale);
			glmDraw(pmodelGlobe, mode);
			glTranslatef(0, 0.22, 0);
			glScalef(0.085, 0.085, 0.085);
			glmDraw(pmodelTree, mode);
			glPopMatrix();
			glEnable(GL_COLOR_MATERIAL);
		}
	}

	mode = GLM_COLOR | GLM_SMOOTH;

	/* Render plane */
	if (showPlane)
	{
		btVector3 extent = ((btBoxShape*)((btCompoundShape*)sphereBody->getCollisionShape())->getChildShape(1))->getHalfExtentsWithoutMargin();
		glColor3f(1, 0, 0);
		btTransform t;
		sphereBody->getMotionState()->getWorldTransform(t);
		float mat[16];
		t.getOpenGLMatrix(mat);
		glPushMatrix();
		glMultMatrixf(mat);     //translation,rotation

		t = ((btCompoundShape*)sphereBody->getCollisionShape())->getChildTransform(1);
		t.getOpenGLMatrix(mat);
		glMultMatrixf(mat);     //translation,rotation


		glBegin(GL_QUADS);
		glVertex3f(-extent.x(), extent.y(), -extent.z());
		glVertex3f(-extent.x(), -extent.y(), -extent.z());
		glVertex3f(-extent.x(), -extent.y(), extent.z());
		glVertex3f(-extent.x(), extent.y(), extent.z());
		glEnd();
		glBegin(GL_QUADS);
		glVertex3f(extent.x(), extent.y(), -extent.z());
		glVertex3f(extent.x(), -extent.y(), -extent.z());
		glVertex3f(extent.x(), -extent.y(), extent.z());
		glVertex3f(extent.x(), extent.y(), extent.z());
		glEnd();
		glBegin(GL_QUADS);
		glVertex3f(-extent.x(), extent.y(), extent.z());
		glVertex3f(-extent.x(), -extent.y(), extent.z());
		glVertex3f(extent.x(), -extent.y(), extent.z());
		glVertex3f(extent.x(), extent.y(), extent.z());
		glEnd();
		glBegin(GL_QUADS);
		glVertex3f(-extent.x(), extent.y(), -extent.z());
		glVertex3f(-extent.x(), -extent.y(), -extent.z());
		glVertex3f(extent.x(), -extent.y(), -extent.z());
		glVertex3f(extent.x(), extent.y(), -extent.z());
		glEnd();
		glBegin(GL_QUADS);
		glVertex3f(-extent.x(), extent.y(), -extent.z());
		glVertex3f(-extent.x(), extent.y(), extent.z());
		glVertex3f(extent.x(), extent.y(), extent.z());
		glVertex3f(extent.x(), extent.y(), -extent.z());
		glEnd();
		glBegin(GL_QUADS);
		glVertex3f(-extent.x(), -extent.y(), -extent.z());
		glVertex3f(-extent.x(), -extent.y(), extent.z());
		glVertex3f(extent.x(), -extent.y(), extent.z());
		glVertex3f(extent.x(), -extent.y(), -extent.z());
		glEnd();
		glPopMatrix();
	}
}

void SnowGlobe::move(float x, float y, float z)
{
	btVector3 direction(x, y, z);

	sphereBody->setActivationState(ACTIVE_TAG);
	sphereBody->activate();
	sphereBody->setLinearVelocity(direction);

}

void SnowGlobe::rotate(float x, float y, float z)
{
	btVector3 direction(y, -x, z);
	sphereBody->setAngularVelocity(direction);
}

SnowGlobe::~SnowGlobe()
{
	removeBody(sphereBody);

	delete pmodelCollisionSphere;
	delete pmodelGlobe;
}
void SnowGlobe::removeBody(btRigidBody *body)
{
	world->removeCollisionObject(body);
	btMotionState* motionState = body->getMotionState();
	btCollisionShape* shape = body->getCollisionShape();
	delete body;
	delete shape;
	delete motionState;
}