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

//void SnowGlobe::prepareCollisionPlane()
//{
//	/* Generate a plane for base */
//	btTransform t;
//	t.setIdentity();
//	t.setOrigin(btVector3(0, planeOrigin, 0));
//	btStaticPlaneShape* plane = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
//	btVector3 scale = plane->getLocalScaling();
//	btMotionState* motion = new btDefaultMotionState(t);
//	btRigidBody::btRigidBodyConstructionInfo info(mass, motion, plane);
//	planeBody = new btRigidBody(info);
//	world->addRigidBody(planeBody, collisiontypes::COL_PLANE, collisiontypes::COL_PLANE);
//}

void SnowGlobe::prepareCollisionPlane()
{
	/* Generate a box for base */
	btTransform t;
	t.setIdentity();
	t.setOrigin(btVector3(0, planeOrigin - (3.7f * (m_scale - 1)), 0));
	btBoxShape* box = new btBoxShape(btVector3(10 / 2.0 * m_scale, 0.5f, 10 / 2.0 * m_scale));
	btVector3 inertia(0, 0, 0);
	if (mass != 0.0)
		box->calculateLocalInertia(mass, inertia);

	btMotionState* motion = new btDefaultMotionState(t);
	btRigidBody::btRigidBodyConstructionInfo info(mass, motion, box, inertia);
	planeBody = new btRigidBody(info);
	world->addRigidBody(planeBody, collisiontypes::COL_PLANE, collisiontypes::COL_PLANE);

	planeBody->setGravity(btVector3(0, 0, 0));
	btVector3 z(0, 0, 0);
	planeBody->setAngularFactor(0);
	planeBody->setLinearFactor(z);
}

SnowGlobe::SnowGlobe()
{
	
}

void SnowGlobe::init(btDynamicsWorld *world)
{
	/* Start by loading models */
	loadCollisionSphere();
	loadGlobeMesh();

	/* Globe physics */
	/* ================================================================== */
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

	btCompoundShape* a = btCreateCompoundFromGimpactShape(globeShape, 5);
	//
	btTransform t;
	t.setIdentity();
	t.setOrigin(btVector3(0, sphereOrigin, 0));
	btMotionState* motion = new btDefaultMotionState(t);
	//btRigidBody::btRigidBodyConstructionInfo info(mass, motion, a);
	btRigidBody::btRigidBodyConstructionInfo info(mass, motion, globeShape);

	info.m_friction = 0.5;
	info.m_angularDamping = 0.2;
	info.m_restitution = 0.5;
	sphereBody = new btRigidBody(info);

	world->addRigidBody(sphereBody, collisiontypes::COL_GLOBE, collisiontypes::COL_GLOBE);
	//sphereBody->setCollisionFlags(sphereBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	//sphereBody->setActivationState(DISABLE_DEACTIVATION);
	sphereBody->setGravity(btVector3(0, 0, 0));
	btVector3 z(0, 0, 0);
	sphereBody->setAngularFactor(0);
	sphereBody->setLinearFactor(z);

	sphereBody->setCcdMotionThreshold(0.50);
	sphereBody->setCcdSweptSphereRadius(10);

	/* Outer sphere */
	/*btGImpactMeshShape *globeShape2 = new btGImpactMeshShape(vertexArray);
	scale = collisionOuterSphereScale  * m_scale;
	globeShape2->setLocalScaling(btVector3(scale, scale, scale));
	globeShape2->updateBound();
	//
	t;
	t.setIdentity();
	t.setOrigin(btVector3(0, sphereOrigin, 0));
	btMotionState* motion2 = new btDefaultMotionState(t);
	//btRigidBody::btRigidBodyConstructionInfo info(mass, motion, a);
	btRigidBody::btRigidBodyConstructionInfo info2(mass, motion2, globeShape2);

	info2.m_friction = 0.5;
	info2.m_angularDamping = 0.2;
	info2.m_restitution = 0;
	sphereBody2 = new btRigidBody(info2);

	world->addRigidBody(sphereBody2, collisiontypes::COL_GLOBE, collisiontypes::COL_GLOBE);
	//sphereBody->setCollisionFlags(sphereBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	//sphereBody->setActivationState(DISABLE_DEACTIVATION);
	sphereBody2->setGravity(btVector3(0, 0, 0));
	sphereBody2->setAngularFactor(0);
	sphereBody2->setLinearFactor(z);

	sphereBody2->setCcdMotionThreshold(0.50);
	sphereBody2->setCcdSweptSphereRadius(10);
	*/
	/* ================================================================== */

	this->world = world;
	prepareCollisionPlane();
}

void SnowGlobe::glDraw()
{
	int mode;
	glColor3f(0.5, 0.5, 0.5);
	//mode = GLM_NONE;
	mode = GLM_MATERIAL | GLM_SMOOTH;// | GLM_TEXTURE;// | GL_SMOOTH;
	//mode = GLM_MATERIAL;// | GLM_COLOR;
	
	if (showGlobe || showSphere)
	{
		float ccd1 = sphereBody->getCcdMotionThreshold(),
			ccd2 = sphereBody->getCcdSweptSphereRadius();

		sphereBody->setCcdMotionThreshold(ccd1);
		sphereBody->setCcdSweptSphereRadius(ccd2);


		btTransform t;
		sphereBody->getMotionState()->getWorldTransform(t);
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
			//glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

			/*
			sphereBody2->getMotionState()->getWorldTransform(t);
			t.getOpenGLMatrix(mat);
			scale = collisionOuterSphereScale * m_scale;
			//glColor3f(0.5, 1, 0.5);
			glPushMatrix();
			glMultMatrixf(mat);     //translation,rotation
			glScalef(scale, scale, scale);
			glmDraw(pmodelCollisionSphere, mode);
			glPopMatrix();
			glEnable(GL_COLOR_MATERIAL);
			*/
		}

		/* Render globe */
		if (showGlobe)
		{
			int scale = 15 * m_scale;
			glPushMatrix();
				glTranslatef(0, -sphereOrigin * m_scale, 0);
				glMultMatrixf(mat);     //translation,rotation
				glScalef(scale, scale, scale);
				glmDraw(pmodelGlobe, mode);
			glPopMatrix();
			glEnable(GL_COLOR_MATERIAL);
		}
	}
	
	mode = GLM_COLOR | GLM_SMOOTH;

	/* Render plane */
	//if (showPlane && !(planeBody->getCollisionShape()->getShapeType() != STATIC_PLANE_PROXYTYPE))
	if (showPlane && !(planeBody->getCollisionShape()->getShapeType() != BOX_SHAPE_PROXYTYPE))
	{
		glColor3f(1, 0, 0);
		btVector3 extent = ((btBoxShape*)planeBody->getCollisionShape())->getHalfExtentsWithoutMargin();
		btTransform t;
		planeBody->getMotionState()->getWorldTransform(t);
		float mat[16];
		t.getOpenGLMatrix(mat);
		glPushMatrix();
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



		//glColor3f(1, 0, 0);
		//btTransform t;
		//planeBody->getMotionState()->getWorldTransform(t);
		//float mat[16];
		//t.getOpenGLMatrix(mat);
		//glPushMatrix();
		//glMultMatrixf(mat);     //translation,rotation
		//glBegin(GL_QUADS);
		//	glVertex3f(-10, 0, 10);
		//	glVertex3f(-10, 0, -10);
		//	glVertex3f(10, 0, -10);
		//	glVertex3f(10, 0, 10);
		//glEnd();
		//glPopMatrix();
	}
}

void SnowGlobe::move(float x, float y, float z)
{
	btVector3 direction(x, y, z);
	
	//if (direction.length2() > 500)
	//	direction *= 500 / direction.length2();

	btTransform t;
	//planeBody->translate(direction);
	planeBody->setActivationState(ACTIVE_TAG);
	planeBody->activate();
	planeBody->setLinearVelocity(direction);
	//
	//planeBody->getMotionState()->getWorldTransform(t);
	//t.setOrigin(t.getOrigin() + direction); // add offset here
	//planeBody->setWorldTransform(t);
	//planeBody->getMotionState()->setWorldTransform(t);

	//planeBody->applyForce(direction, t.getOrigin());

	//sphereBody->translate(direction);
	sphereBody->setActivationState(ACTIVE_TAG);
	sphereBody->activate();
	sphereBody->setLinearVelocity(direction);

	/*sphereBody2->setActivationState(ACTIVE_TAG);
	sphereBody2->activate();
	sphereBody2->setLinearVelocity(direction);*/

	//sphereBody->getMotionState()->getWorldTransform(t);
	//t.setOrigin(t.getOrigin() + direction); // add offset here
	//sphereBody->setWorldTransform(t);
	//sphereBody->getMotionState()->setWorldTransform(t);
	
	//sphereBody->applyForce(direction, t.getOrigin());
}

SnowGlobe::~SnowGlobe()
{
	removeBody(planeBody);
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