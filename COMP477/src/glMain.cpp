/*
*	Concordia University Fall 2015 
*   COMP 477 Animation for Computer Games 
*   Team Snowglobe
*
*   Anthony Moniz 26558259
*   Sebouh Bardakjian 29640185
*   Francis Hamelin-Owens 27067275
*
*/

#include "SnowManager.h"

#pragma managed(push, off)
#include <btBulletDynamicsCommon.h>
#pragma managed(pop)

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#ifdef _WIN32
#include "GL/glut.h"
#else
#include <GL/freeglut.h>
#endif
#endif

#include "bullet-util\BulletDebugDrawer.h"
#include <iostream>
#include <cmath>
#include <cstring>
#include "snowGlobe.h"
#include "present.h"
#include "TimeManager.h"
#include <glm/gtc/type_ptr.hpp>

using namespace std;


//Create Globe
SnowGlobe globe;

/* Used to exit */
int windowID;

bool alt = false;
bool ctrl = false;

//Switches
int meshModel=0;
bool drawSkeleton=true;

//Window parameters
int width = 1024;
int height = 768;
///* Ortho (if used) */
double _left = 0.0;		/* ortho view volume params */
double _right = 0.0;
double _bottom = 0.0;
double _top = 0.0;
double _zNear = 0.1;
double _zFar = 1000.0;
double fovy = 45.0;
double prev_z = 0;

//Model matrices
double _matrix[16];
double _matrixI[16];

/* Mouse Interface  */
int _mouseX = 0;		/* mouse control variables */
int _mouseY = 0;
bool _mouseLeft = false;
bool _mouseMiddle = false;
bool _mouseRight = false;

double _dragPosX = 0.0;
double _dragPosY = 0.0;
double _dragPosZ = 0.0;

//Frame timer stuff
//Frame timer
TimeManager frameTimer;
double frameTime_prev;
double frameTime_curr;

// Snow stuff
SnowManager snowmanager;
const int dontDraw = 5;
// Bullet Stuff
GLUquadricObj* quad;
btDynamicsWorld* world;
btDispatcher* dispatcher;
btCollisionConfiguration* collisionConfig;
btBroadphaseInterface* broadphase;
btConstraintSolver* solver;

GLBulletDebugDrawer* debugDrawer;

//Bullet managing stuff
std::vector<btRigidBody*> bodies;

//Presents
std::vector<Present> presents;

// Initial object spawn position
glm::vec3 spawnLoc = glm::vec3(0, 5, 0.2);

glm::vec3 getVector(GLdouble x1, GLdouble y1, GLdouble z1, GLdouble x2, GLdouble y2, GLdouble z2)
{
	GLdouble pos3D_x, pos3D_y, pos3D_z;

	// arrays to hold matrix information

	GLdouble model_view[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, model_view);

	GLdouble projection[16];
	glGetDoublev(GL_PROJECTION_MATRIX, projection);

	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);

	GLdouble
		xt[4] = { x1, x2 },
		yt[4] = { y1, y2 },
		zt[4] = { z1, z2 }
	;
	glm::vec3 v[2];

	for (int i = 0; i < 2; ++i)
	{
		x1 = xt[i]; y1 = yt[i]; z1 = zt[0];
		// get 3D coordinates based on window coordinates
		gluUnProject(x1, y1, z1,
			model_view, projection, viewport,
			&pos3D_x, &pos3D_y, &pos3D_z);

		v[i] = glm::vec3(pos3D_x, pos3D_y, pos3D_z);
	}

	return v[1] - v[0];
}

btRigidBody* addSnowflake(float x, float y, float z)
{
	btRigidBody* body = snowmanager.createSnowflake(x, y, z);
	int all = collisiontypes::COL_GLOBE | collisiontypes::COL_PLANE;
	world->addRigidBody(body, all, all);

	btVector3 h(0.3, 0.3, 0.3);
	body->setLinearFactor(h);
	body->setCcdMotionThreshold(0.5);
	body->setCcdSweptSphereRadius(0.5);
	body->setUserIndex(dontDraw);
	bodies.push_back(body);
	return body;
}

double vlen(double x, double y, double z)
{
	return sqrt(x * x + y * y + z * z);
}

void invertMatrix(const GLdouble * m, GLdouble * out)
{

	/* NB. OpenGL Matrices are COLUMN major. */
#define MAT(m,r,c) (m)[(c)*4+(r)]

	/* Here's some shorthand converting standard (row,column) to index. */
#define m11 MAT(m,0,0)
#define m12 MAT(m,0,1)
#define m13 MAT(m,0,2)
#define m14 MAT(m,0,3)
#define m21 MAT(m,1,0)
#define m22 MAT(m,1,1)
#define m23 MAT(m,1,2)
#define m24 MAT(m,1,3)
#define m31 MAT(m,2,0)
#define m32 MAT(m,2,1)
#define m33 MAT(m,2,2)
#define m34 MAT(m,2,3)
#define m41 MAT(m,3,0)
#define m42 MAT(m,3,1)
#define m43 MAT(m,3,2)
#define m44 MAT(m,3,3)

	GLdouble det;
	GLdouble d12, d13, d23, d24, d34, d41;
	GLdouble tmp[16];		/* Allow out == in. */

	/* Inverse = adjoint / det. (See linear algebra texts.) */

	/* pre-compute 2x2 dets for last two rows when computing */
	/* cofactors of first two rows. */
	d12 = (m31 * m42 - m41 * m32);
	d13 = (m31 * m43 - m41 * m33);
	d23 = (m32 * m43 - m42 * m33);
	d24 = (m32 * m44 - m42 * m34);
	d34 = (m33 * m44 - m43 * m34);
	d41 = (m34 * m41 - m44 * m31);

	tmp[0] = (m22 * d34 - m23 * d24 + m24 * d23);
	tmp[1] = -(m21 * d34 + m23 * d41 + m24 * d13);
	tmp[2] = (m21 * d24 + m22 * d41 + m24 * d12);
	tmp[3] = -(m21 * d23 - m22 * d13 + m23 * d12);

	/* Compute determinant as early as possible using these cofactors. */
	det = m11 * tmp[0] + m12 * tmp[1] + m13 * tmp[2] + m14 * tmp[3];

	/* Run singularity test. */
	if (det == 0.0) {
		/* printf("invert_matrix: Warning: Singular matrix.\n"); */
		/* 	  memcpy(out,_identity,16*sizeof(double)); */
	}
	else {
		GLdouble invDet = 1.0 / det;
		/* Compute rest of inverse. */
		tmp[0] *= invDet;
		tmp[1] *= invDet;
		tmp[2] *= invDet;
		tmp[3] *= invDet;

		tmp[4] = -(m12 * d34 - m13 * d24 + m14 * d23) * invDet;
		tmp[5] = (m11 * d34 + m13 * d41 + m14 * d13) * invDet;
		tmp[6] = -(m11 * d24 + m12 * d41 + m14 * d12) * invDet;
		tmp[7] = (m11 * d23 - m12 * d13 + m13 * d12) * invDet;

		/* Pre-compute 2x2 dets for first two rows when computing */
		/* cofactors of last two rows. */
		d12 = m11 * m22 - m21 * m12;
		d13 = m11 * m23 - m21 * m13;
		d23 = m12 * m23 - m22 * m13;
		d24 = m12 * m24 - m22 * m14;
		d34 = m13 * m24 - m23 * m14;
		d41 = m14 * m21 - m24 * m11;

		tmp[8] = (m42 * d34 - m43 * d24 + m44 * d23) * invDet;
		tmp[9] = -(m41 * d34 + m43 * d41 + m44 * d13) * invDet;
		tmp[10] = (m41 * d24 + m42 * d41 + m44 * d12) * invDet;
		tmp[11] = -(m41 * d23 - m42 * d13 + m43 * d12) * invDet;
		tmp[12] = -(m32 * d34 - m33 * d24 + m34 * d23) * invDet;
		tmp[13] = (m31 * d34 + m33 * d41 + m34 * d13) * invDet;
		tmp[14] = -(m31 * d24 + m32 * d41 + m34 * d12) * invDet;
		tmp[15] = (m31 * d23 - m32 * d13 + m33 * d12) * invDet;

		memcpy(out, tmp, 16 * sizeof(GLdouble));
	}

#undef m11
#undef m12
#undef m13
#undef m14
#undef m21
#undef m22
#undef m23
#undef m24
#undef m31
#undef m32
#undef m33
#undef m34
#undef m41
#undef m42
#undef m43
#undef m44
#undef MAT
}


void getMatrix()
{
	glGetDoublev(GL_MODELVIEW_MATRIX, _matrix);
	invertMatrix(_matrix, _matrixI);
}

void pos(double *px, double *py, double *pz, const int x, const int y,
	const int *viewport)
{
	/*
	Use the ortho projection and viewport information
	to map from mouse co-ordinates back into world
	co-ordinates
	*/

	*px = (double)(x - viewport[0]) / (double)(viewport[2]);
	*py = (double)(y - viewport[1]) / (double)(viewport[3]);

	*px = _left + (*px) * (_right - _left);
	*py = _top + (*py) * (_bottom - _top);
	*pz = _zNear;
}

btScalar mMaxSpeed = 15;
void myTickCallback(btDynamicsWorld *world, btScalar timeStep, btRigidBody *body) {
	// Limit velocities to max speed (max for better physics in the globe)
	btVector3 velocity = body->getLinearVelocity();
	btScalar speed = velocity.length();
	if (speed > mMaxSpeed) {
		velocity *= mMaxSpeed / speed;
		body->setLinearVelocity(velocity);
	}
}

void physicsUpdate()
{
	frameTimer.update();
	frameTime_curr = frameTimer.getCurrentTime();

	double difference = frameTime_curr - frameTime_prev;

	//printf("frametime: %f \n", difference);
	//updateDynamicsWorld
	float one = difference;// 1.0f / 60;
	float	two = 30;
	float	three = 1.0f / 600;

	snowmanager.Update(one);
	world->stepSimulation(one, two, three);

	frameTime_prev = frameTime_curr;

	globe.killvel();

	int numManifolds = world->getDispatcher()->getNumManifolds();

	for each (Present bod in presents)
	{
		myTickCallback(world, three, bod.body);
	}

	for (int i = 0; i < numManifolds; i++)
	{
		btPersistentManifold* contactManifold = world->getDispatcher()->getManifoldByIndexInternal(i);
		//btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
		//btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());

		int numContacts = contactManifold->getNumContacts();
		for (int j = 0; j < numContacts; j++)
		{
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			if (pt.getDistance() < 0.f)
			{
				if (pt.m_appliedImpulse > 10)
					std::cout << pt.m_appliedImpulse << std::endl;
				const btVector3& ptA = pt.getPositionWorldOnA();
				const btVector3& ptB = pt.getPositionWorldOnB();
				const btVector3& normalOnB = pt.m_normalWorldOnB;
			}
		}
	}
}

void display()
{	
	//physicsUpdate();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	glLoadIdentity();
	glMultMatrixd(_matrix);

	glColor3f(0.5, 0.5, 0.5);
	glPushMatrix();													//draw terrain
	glColor3f(0.3, 0.3, 0.3);

	/* Draw presents */
	for (int i = 0; i < presents.size(); i++)
		presents[i].glDraw();

	// Draw snowflakes
	GLdouble modelview[16];
	GLdouble projection[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);


	glm::mat4 viewMatrix = glm::make_mat4(modelview);

	glm::mat4 projectMatrix = glm::make_mat4(projection);

	snowmanager.Draw(viewMatrix, projectMatrix);

	globe.glDraw();

	world->debugDrawWorld();

	glPopMatrix();
	glutSwapBuffers();
}

void changeSize(int w, int h)
{
	glViewport(0, 0, w, h);

	_top = 1.0;
	_bottom = -1.0;
	_left = -(double)w / (double)h;
	_right = -_left;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	/* glOrtho(_left,_right,_bottom,_top,_zNear,_zFar);  Ortho */
	gluPerspective(fovy, (double)w / (double)h, _zNear, _zFar);	/* PErspective for stereo */

	glMatrixMode(GL_MODELVIEW);
}

void initTime()
{
	frameTimer.update();
	frameTime_curr = frameTimer.getCurrentTime();
	frameTime_prev = frameTime_curr;
}

void timerFunction(int value)
{
	glutTimerFunc(10, timerFunction, 1);
	physicsUpdate();
	glutPostRedisplay();
}

void mouseEvent(int button, int state, int x, int y)
{
	int viewport[4];

	_mouseX = x;
	_mouseY = y;

	if (state == GLUT_UP)
		switch (button) {
		case GLUT_LEFT_BUTTON:
			_mouseLeft = false;
			break;
		case GLUT_MIDDLE_BUTTON:
			_mouseMiddle = false;
			break;
		case GLUT_RIGHT_BUTTON:
			_mouseRight = false;
			break;
	}
	else
		switch (button) {
		case GLUT_LEFT_BUTTON:
			_mouseLeft = true;
			break;
		case GLUT_MIDDLE_BUTTON:
			_mouseMiddle = true;
			break;
		case GLUT_RIGHT_BUTTON:
			_mouseRight = true;
			break;
		case 4:         //Zoomout
			glLoadIdentity();
			glTranslatef(0, 0, -1.);
			glMultMatrixd(_matrix);
			getMatrix();
			glutPostRedisplay();
			break;
		case 3:         //Zoomin
			glLoadIdentity();
			glTranslatef(0, 0, 1);
			glMultMatrixd(_matrix);
			getMatrix();
			glutPostRedisplay();
			break;
		default:
			break;
			//std::cout<<button<<std::endl;
	}

	glGetIntegerv(GL_VIEWPORT, viewport);
	pos(&_dragPosX, &_dragPosY, &_dragPosZ, x, y, viewport);
}

void updateSpawnLocation(){
	spawnLoc.x = (double)globe.origin.getX();
	spawnLoc.y = (double)globe.origin.getY();
	spawnLoc.z = (double)globe.origin.getZ();
}	

void mouseMoveEvent(int x, int y)
{
	//std::cout << "move" << std::endl;
	bool changed = false;

	const int dx = x - _mouseX;
	const int dy = y - _mouseY;

	if (!alt)
	{
		int viewport[4];
		glGetIntegerv(GL_VIEWPORT, viewport);

		glm::vec3 v = getVector(_mouseX, _mouseY, 0, x, y, 0);
		v *= 5000;
		if (!ctrl)
			globe.move(v.x, -v.y, v.z);
		else
			globe.rotate(v.x, -v.y, v.z);

		_mouseX = x;
		_mouseY = y;
	}
	else
	{
		int viewport[4];
		glGetIntegerv(GL_VIEWPORT, viewport);

		if (dx == 0 && dy == 0)
			return;

		if (_mouseMiddle || (_mouseLeft && _mouseRight)) {
			///* double s = exp((double)dy*0.01); */
			///* glScalef(s,s,s); */
			///* if(abs(prev_z) <= 1.0) */

			//glLoadIdentity();
			//glTranslatef(0, 0, dy * 0.1);
			//glMultMatrixd(_matrix);

			//changed = true;
			double px, py, pz;

			pos(&px, &py, &pz, x, y, viewport);

			glLoadIdentity();
			glTranslatef(px - _dragPosX, py - _dragPosY, pz - _dragPosZ);
			glMultMatrixd(_matrix);

			_dragPosX = px;
			_dragPosY = py;
			_dragPosZ = pz;

			changed = true;
		}
		else if (_mouseLeft) {
			double ax, ay, az;
			double bx, by, bz;
			double angle;

			ax = 0;// dy;
			ay = dx;
			az = 0.0;
			angle = vlen(ax, ay, az) / (double)(viewport[2] + 1) * 180.0;

			/* Use inverse matrix to determine local axis of rotation */

			bx = _matrixI[0] * ax + _matrixI[4] * ay + _matrixI[8] * az;
			by = _matrixI[1] * ax + _matrixI[5] * ay + _matrixI[9] * az;
			bz = _matrixI[2] * ax + _matrixI[6] * ay + _matrixI[10] * az;

			//glRotatef(angle, bx, by, bz);
			if (dx > 0)
				glRotatef(angle, 0, 1, 0);
			else
				glRotatef(angle, 0, -1, 0);


			changed = true;
		}
		else if (_mouseRight) {
			double ax, ay, az;
			double bx, by, bz;
			double angle;

			ax = dy;
			ay = 0.0;
			az = 0.0;
			angle = vlen(ax, ay, az) / (double)(viewport[2] + 1) * 180.0;

			/* Use inverse matrix to determine local axis of rotation */

			bx = _matrixI[0] * ax + _matrixI[4] * ay + _matrixI[8] * az;
			by = _matrixI[1] * ax + _matrixI[5] * ay + _matrixI[9] * az;
			bz = _matrixI[2] * ax + _matrixI[6] * ay + _matrixI[10] * az;

			//glRotatef(angle, bx, by, bz);
			if (dy > 0)
				glRotatef(angle, 1, 0, 0);
			else
				glRotatef(angle, -1, 0, 0);


			changed = true;
		}

		_mouseX = x;
		_mouseY = y;

		if (changed) {
			getMatrix();
			glutPostRedisplay();
		}
	}
}

void handleKeyPress(unsigned char key, int x, int y)
{
	std::cout << key << std::endl;
	switch (key)
	{
		/* Frame movement */
	case 's':
		for (int i = 0; i < 10; i++)
		{
			addSnowflake(spawnLoc.x, spawnLoc.y, spawnLoc.z);
		}		
		break;
	case 'w':
		snowmanager.SetApplyWind(!snowmanager.IsWindOn());
		break;
	case 'h':
		globe.toggleCollisionSphere();
		break;
	case 'g':
		globe.toggleGlobe();
		break;
	case 'f':
		globe.toggleCollisionPlane();
		break;
	//case 't':
	//	planeBody->getMotionState()->getWorldTransform(t);
	//	t.setRotation(btQuaternion(0.38268343236, 0, 0, 0.92387));
	//	planeBody->setWorldTransform(t);
	//	planeBody->getMotionState()->setWorldTransform(t);
	//	break;
	case '+':
		globe.move(0, 1, 0);
		updateSpawnLocation();
		break;
	case '-':
		globe.move(0, -1, 0);
		updateSpawnLocation();
		break;
	//case '-':
	//	planeBody->getMotionState()->getWorldTransform(t);
	//	t.setOrigin(t.getOrigin() - btVector3(0, 1, 0)); // add offset here
	//	planeBody->setWorldTransform(t);
	//	planeBody->getMotionState()->setWorldTransform(t);
	//	planeBody->applyForce(btVector3(0, -1, 0), t.getOrigin());
	//	break;
	case 'v':
		debugDrawer->setDebugMode(btIDebugDraw::DBG_NoDebug); break;
	case 'b':
		debugDrawer->setDebugMode(btIDebugDraw::DBG_DrawAabb); break;
	case 'n':
		debugDrawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe); break;
	case 'm':
		debugDrawer->setDebugMode(btIDebugDraw::DBG_DrawAabb | btIDebugDraw::DBG_DrawWireframe); break;
	case ' ':
		cout << "Resetting" << endl; break;
	case 'q':
		glutDestroyWindow(windowID);
		exit(0);
	}
}

void mousePassiveFunc(int x, int y)
{
}


void init(float angle)
{
	quad = gluNewQuadric();
	collisionConfig = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfig);
	broadphase = new btDbvtBroadphase();
	solver = new btSequentialImpulseConstraintSolver();
	world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);
	world->setGravity(btVector3(0, -10, 0));

	btContactSolverInfo& info = world->getSolverInfo();
	info.m_splitImpulse = 1; //enable split impulse feature
	//optionally set the m_splitImpulsePenetrationThreshold (only used when m_splitImpulse  is enabled)
	//only enable split impulse position correction when the penetration is deeper than this m_splitImpulsePenetrationThreshold, otherwise use the regular velocity/position constraint coupling (Baumgarte).
	info.m_splitImpulsePenetrationThreshold = -0.02;

	// Setup Debug Drawer
	debugDrawer = new GLBulletDebugDrawer();
	debugDrawer->setDebugMode(btIDebugDraw::DBG_NoDebug);
	world->setDebugDrawer(debugDrawer);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(fovy, (double)width / (double)height, _zNear, _zFar);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glMatrixMode(GL_MODELVIEW_MATRIX);

	//Light values and coordinates
	GLfloat ambientLight[] = { 0.3f, 0.3f, 0.3f, 1.0f };
	GLfloat diffuseLight[] = { 0.7f, 0.7f, 0.7f, 1.0f };
	//GLfloat lightPos[] = { 20.0f, 20.0f, 50.0f, 1.0f };
	GLfloat lightPos[] = { 0.0f, 50.0f, -25.0f, 1.0f };
	//GLfloat lightPos[] = { 0.0f, 20.0f, 0.0f, 1.0f };
	glEnable(GL_LIGHT0);

	glEnable(GL_DEPTH_TEST);
	glFrontFace(GL_CCW);
	//glEnable(GL_CULL_FACE);
	glPolygonMode(GL_FRONT, GL_FILL);
	// Hidden surface removal // Counterclockwise polygons face out // Do not calculate inside of jet // Enable lighting
	glEnable(GL_LIGHTING);
	// Set up and enable light 0
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
	// Enable color tracking
	glEnable(GL_COLOR_MATERIAL);
	// Set material properties to follow glColor values
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

	glClearColor(0.1f, 0.1f, 0.1f, 3.0f);

	//Rescale normals to unit length
	glEnable(GL_NORMALIZE);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

	glShadeModel(GL_FLAT);



	getMatrix(); //Init matrix

	//Translate camera
	glPushMatrix();
	glLoadIdentity();
	glTranslatef(0, -4, -20.0);
	glMultMatrixd(_matrix);
	getMatrix();
	glPopMatrix();

	//glClearColor(0, 0, 0, 1);
	//glMatrixMode(GL_PROJECTION);
	//glLoadIdentity();
	//gluPerspective(angle, 640.0 / 480.0, 1, 1000);
	//glMatrixMode(GL_MODELVIEW);
	////initskybox();
	//glEnable(GL_DEPTH_TEST);

	//glEnable(GL_BLEND);
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glewInit();
}

void specialFunc(int key, int x, int y)
{
	std::cout << 1 << std::endl;
	if (key == 116)
	{
		alt = true;
	}
	else if (key == 114)
	{
		ctrl = true;
	}

	switch (key)
	{
	case GLUT_KEY_UP:
		cout << "Key up" << endl;
		globe.rotate(0, 5, 0);
		break;
	case GLUT_KEY_DOWN:
		cout << "Key down" << endl;
		globe.rotate(0, -5, 0);
		break;
	case GLUT_KEY_LEFT:
		cout << "Key left" << endl;
		globe.rotate(5, 0, 0);
		break;
	case GLUT_KEY_RIGHT:
		cout << "Key right" << endl;
		globe.rotate(-5, 0, 0);
		break;
	}
}
void specialFuncUp(int key, int x, int y)
{
	if (key == 116)
	{
		alt = false;
	}
	else if (key == 114)
	{
		ctrl = false;
	}
}

int main(int argc, char **argv)
{
	glutInit(&argc, argv);
	//Print contex info
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);	//double buffer
	glutInitWindowSize(width, height);
	glutInitWindowPosition(0, 0);
	windowID = glutCreateWindow("477 - Snow globe");
	glutDisplayFunc(display);
	glutReshapeFunc(changeSize);
	glutTimerFunc(10, timerFunction, 1);

	glutMouseFunc(mouseEvent);
	glutMotionFunc(mouseMoveEvent);
	glutKeyboardFunc(handleKeyPress);
	glutPassiveMotionFunc(mousePassiveFunc);
	glutSpecialFunc(specialFunc);
	glutSpecialUpFunc(specialFuncUp);

	float angle = 50;
	init(angle);

	for (int i = 0; i < 3; ++i)
	{
		Present pre(2, 0, 2, 20, 0.5);
		pre.init(world);
		pre.setBoxColor(i % 3 == 0 ? 1 : 0, i % 3 == 1 ? 1 : 0, i % 3 == 2 ? 1 : 0);
		presents.push_back(pre);
	}

	initTime();
	globe.init(world);
	snowmanager.init();

	for (int i = 0; i < 10; i++)
	{
		for (int j = 0; j < 20; j++)
		{
			int zmod = j;
			if (zmod < 10)
			{
				zmod = -(zmod % 10);
			}
			addSnowflake(spawnLoc.x + i*0.3, spawnLoc.y, spawnLoc.z + zmod*0.3);
		}
	}

	glutMainLoop();

	/* Cleanup */
	for (int i = 0; i < presents.size(); ++i)
	{
		presents[i].Delete();
	}

	for (int i = 0; i<bodies.size(); i++)
	{
		world->removeCollisionObject(bodies[i]);
		btMotionState* motionState = bodies[i]->getMotionState();
		btCollisionShape* shape = bodies[i]->getCollisionShape();
		delete bodies[i];
		delete shape;
		delete motionState;
	}
	delete dispatcher;
	delete collisionConfig;
	delete solver;
	delete broadphase;
	delete world;
	gluDeleteQuadric(quad);
	return 0;
}