#include "SnowManager.h"

const float SnowManager::snowflakeRadius = 0.2f;

void SnowManager::init()
{
	// Create and compile our GLSL program from the shaders
	programID = LoadShaders("Billboard.vertexshader", "Billboard.fragmentshader");

	// Vertex shader
	CameraRight_worldspace_ID = glGetUniformLocation(programID, "CameraRight_worldspace");
	CameraUp_worldspace_ID = glGetUniformLocation(programID, "CameraUp_worldspace");
	ViewProjMatrixID = glGetUniformLocation(programID, "VP");
	BillboardPosID = glGetUniformLocation(programID, "BillboardPos");
	BillboardSizeID = glGetUniformLocation(programID, "BillboardSize");

	TextureID = glGetUniformLocation(programID, "myTextureSampler");

	Snowflake::InitVertexBuffers();

	snowShape = new btSphereShape(btScalar(snowflakeRadius));
}

btRigidBody* SnowManager::addSnowflake(btDynamicsWorld* world, float x, float y, float z)
{
	/// Create Dynamic Objects
	btTransform startTransform;
	startTransform.setIdentity();

	btScalar	mass(0.2f);

	startTransform.setOrigin(btVector3(
		btScalar(x),
		btScalar(y),
		btScalar(z)));


	btRigidBody* body = createRigidBody(world, mass, startTransform, snowShape);

	body->setAngularFactor(btScalar(0.2f));
	body->setDamping(body->getLinearDamping(), btScalar(0.95));
	RigidSnowflake newSnowflake(body, snowflakeRadius * 2, snowflakeRadius * 2);
	newSnowflake.setWindForce(btVector3(100.0f, 25.0f, 100.0f));
	snowflakes.push_back(newSnowflake);
	return body;
}

btRigidBody* SnowManager::createSnowflake(float x, float y, float z)
{
	/// Create Dynamic Objects
	btTransform startTransform;
	startTransform.setIdentity();

	btScalar	mass(0.1f);

	startTransform.setOrigin(btVector3(
		btScalar(x),
		btScalar(y),
		btScalar(z)));


	btRigidBody* body = createRigidBody(mass, startTransform, snowShape);

	body->setAngularFactor(btScalar(0.2f));
	body->setDamping(btScalar(0.5f), btScalar(0.95));
	RigidSnowflake newSnowflake(body, snowflakeRadius * 2, snowflakeRadius * 2);
	newSnowflake.setWindForce(btVector3(20.0f, 25.0f, 20.0f));
	snowflakes.push_back(newSnowflake);
	return body;
}

void SnowManager::Draw(const glm::mat4 &ViewMatrix, const glm::mat4 &projectionMatrix)
{
	glUseProgram(programID);

	Snowflake::InitDraw();

	// Bind our texture in Texture Unit 0
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, Snowflake::Texture);
	// Set our "myTextureSampler" sampler to user Texture Unit 0
	glUniform1i(TextureID, 0);

	// Rotation component of view matrix is orthogonal so inverse is simply transpose
	glUniform3f(CameraRight_worldspace_ID, ViewMatrix[0][0], ViewMatrix[1][0], ViewMatrix[2][0]);
	glUniform3f(CameraUp_worldspace_ID, ViewMatrix[0][1], ViewMatrix[1][1], ViewMatrix[2][1]);

	glm::mat4 viewProjectionMatrix = projectionMatrix * ViewMatrix;

	glUniformMatrix4fv(ViewProjMatrixID, 1, GL_FALSE, &viewProjectionMatrix[0][0]);

	for (int i = 0; i < snowflakes.size(); i++)
	{
		float* pos = snowflakes[i].getPosition();
		glUniform3f(BillboardPosID, pos[0], pos[1], pos[2]);
		glUniform2f(BillboardSizeID, snowflakes[i].getWidth(), snowflakes[i].getHeight());

		snowflakes[i].Draw();
	}

	Snowflake::FinishDraw();
	glUseProgram(0);
}