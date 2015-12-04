#ifndef SNOWMANAGER_H
#define SNOWMANAGER_H

#include "opengl-util\ShaderUtility.h"
#include "RigidSnowflake.h"
#include <vector>
#include <glm\glm.hpp>
#include "bullet-util\BulletUtility.h"

class SnowManager
{
public:
	std::vector<RigidSnowflake> snowflakes;

	GLuint programID;

	GLuint CameraRight_worldspace_ID;
	GLuint CameraUp_worldspace_ID;
	GLuint ViewProjMatrixID;
	GLuint BillboardPosID;
	GLuint BillboardSizeID;

	GLuint TextureID;

	btSphereShape* snowShape;

	~SnowManager()
	{
		snowflakes.clear();
	}

	void init();

	btRigidBody* addSnowflake(btDynamicsWorld* world, float x = 0.0f, float y = 0.0f, float z = 0.0f);

	btRigidBody* createSnowflake(float x = 0.0f, float y = 0.0f, float z = 0.0f);

	void SetApplyWind(bool on)
	{
		windOn = on;
		for (std::vector<RigidSnowflake>::iterator it = snowflakes.begin(); it < snowflakes.end(); ++it)
		{
			(*it).SetApplyWind(on);
		}
	}

	void SetWindForce(const btVector3 &windForce)
	{
		for (std::vector<RigidSnowflake>::iterator it = snowflakes.begin(); it < snowflakes.end(); ++it)
		{
			(*it).setWindForce(windForce);
		}
	}

	void Update(float dt)
	{
		if (!snowflakes.empty())
		{
			for (std::vector<RigidSnowflake>::iterator it = snowflakes.begin(); it < snowflakes.end(); ++it)
			{
				(*it).Update(dt);
			}
		}		
	}

	void Draw(const glm::mat4 &ViewMatrix, const glm::mat4 &projectionMatrix);

	bool IsWindOn()
	{
		return windOn;
	}

private:
	bool windOn = false;

	static const float snowflakeRadius;

};

#endif