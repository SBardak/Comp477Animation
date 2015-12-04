//
// Test Class taken from https://code.google.com/p/opengl-tutorial-org/source/browse/misc05_picking
//
// Most basic implementation of the btIDebugDraw class for drawing physical objects (AABB and more specific collision objects) to the screen
#ifndef GL_DEBUG_DRAW_H
#define GL_DEBUG_DRAW_H
// Include Bullet
#include "GL/glut.h"
#include <btBulletDynamicsCommon.h>


class GLBulletDebugDrawer : public btIDebugDraw
{
public:
	
	void SetMatrices(const glm::mat4 &pViewMatrix, const glm::mat4 &pProjectionMatrix){
		glUseProgram(0);
		glMatrixMode(GL_MODELVIEW);
		glLoadMatrixf(&pViewMatrix[0][0]);
		glMatrixMode(GL_PROJECTION);
		glLoadMatrixf(&pProjectionMatrix[0][0]);
	}
	
	virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& color){
		glColor3f(color.x(), color.y(), color.z());
		glBegin(GL_LINES);
		glVertex3f(from.x(), from.y(), from.z());
		glVertex3f(to.x(), to.y(), to.z());
		glEnd();
	}
	virtual void drawContactPoint(const btVector3 &, const btVector3 &, btScalar, int, const btVector3 &){}
	virtual void reportErrorWarning(const char *){}
	virtual void draw3dText(const btVector3 &, const char *){}
	virtual void setDebugMode(int p){
		mDebugMode = p;
	}
	int getDebugMode() const { return mDebugMode; }
	int mDebugMode;
};
#endif