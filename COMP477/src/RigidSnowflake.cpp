#include "RigidSnowflake.h"
#define SMALL_PI 3.14159265359

const float RigidSnowflake::rateOfChange = 0.5f;

float* RigidSnowflake::getPosition()
{
	btTransform trans;
	body->getMotionState()->getWorldTransform(trans);
	btVector3 pos = trans.getOrigin();
	position[0] = pos.x();
	position[1] = pos.y();
	position[2] = pos.z();
	return position;
}

void RigidSnowflake::Update(float dt)
{
	if (applyWind)
	{
		progress += rateOfChange * dt;
		if (progress > 1.0f)
		{
			progress -= (int)progress;
		}
		float angle = progress * SMALL_PI * 2;
		float cosAngle = cosf(angle);
		float sinAngle = sinf(angle);
		currWindForce = btVector3(cosAngle, 0.0, -sinAngle) * windForce;
		//btVector3 crossProd = btCross(currWindForce, -gravityForce.normalized());
		btVector3 force = btVector3(0.0, abs(sinAngle) * windForce.y(), 0.0f) + currWindForce;

		force += gravityForce;
		body->setGravity(force);
		//body->applyCentralForce(currWindForce);
	}	
}