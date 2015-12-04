#ifndef RIGID_SNOWFLAKE_H
#define RIGID_SNOWFLAKE_H

#include <GL\glew.h>
#include "Snowflake.h"
#include "btBulletDynamicsCommon.h"

class RigidSnowflake : public Snowflake
{
public:
	btRigidBody* body;

	RigidSnowflake(btRigidBody* rigidBody, float width, float height, btVector3 &worldGravity = btVector3(0,-10,0)) : Snowflake(0, 0, 0, width, height)
	{
		body = rigidBody;
		initialGravity = gravityForce = worldGravity;
		progress = 0.0f;
	}

	virtual void Update(float dt);

	//Toggle wind affect on/off
	void SetApplyWind(bool on)
	{
		if (!applyWind && on)
		{
			//wake up body
			body->activate();
			body->applyCentralForce(windForce);
		}
		applyWind = on;
		if (!on)
		{
			resetGravity();
		}
	}

	void setCustomGravityForce(const btVector3 &gravityForce)
	{
		this->gravityForce = gravityForce;
	}

	/*
	Wind force is applied to custom gravity force whenever update is called
	*/
	void setWindForce(const btVector3 &windForce)
	{
		this->windForce = windForce;
	}

	void resetGravity()
	{
		gravityForce = initialGravity;
		body->setGravity(gravityForce);
	}

	virtual float* getPosition();
private:
	bool applyWind = false;
	static const float rateOfChange;
	btVector3 initialGravity;
	btVector3 gravityForce;
	btVector3 windForce;
	btVector3 currWindForce;
	float progress;
};

#endif