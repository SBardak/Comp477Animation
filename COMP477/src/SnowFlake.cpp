#include "SnowFlake.h"

const float Snowflake::vertexBuffer[] = {
	-0.5f, -0.5f, 0.0f,
	0.5f, -0.5f, 0.0f,
	-0.5f, 0.5f, 0.0f,
	0.5f, 0.5f, 0.0f,
};

GLuint Snowflake::vertexArrayID;

GLuint Snowflake::Texture;

GLuint Snowflake::vertexBufferID;

Snowflake::Snowflake(float x, float y, float z, float width, float height)
{
	position[0] = x;
	position[1] = y;
	position[2] = z;

	size[0] = width;
	size[1] = height;
}

void Snowflake::InitVertexBuffers()
{
	glGenVertexArrays(1, &vertexArrayID);
	glBindVertexArray(vertexArrayID);

	Texture = loadDDS("Snowflake.DDS");

	glGenBuffers(1, &vertexBufferID);
	glBindBuffer(GL_ARRAY_BUFFER, vertexBufferID);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertexBuffer), vertexBuffer, GL_DYNAMIC_DRAW);
}

void Snowflake::InitDraw()
{
	glBindVertexArray(vertexArrayID);
	// 1rst attribute buffer : vertices
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, vertexBufferID);
	glVertexAttribPointer(
		0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
		3,                  // size
		GL_FLOAT,           // type
		GL_FALSE,           // normalized?
		0,                  // stride
		(void*)0            // array buffer offset
		);
}

void Snowflake::Draw()
{
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
}

void Snowflake::FinishDraw()
{
	glDisableVertexAttribArray(0);
	glBindVertexArray(0);
}

void Snowflake::CleanUp()
{
	glDeleteBuffers(1, &vertexBufferID);
	glDeleteTextures(1, &Texture);
	glDeleteVertexArrays(1, &vertexArrayID);
}

/*
//Build physics object
btCollisionShape * shape = new btSphereShape(0.2f);
createRigidbody(shape);
mBody->setAngularFactor(btScalar(0.2f));
mBody->setDamping(mBody->getLinearDamping(), btScalar(0.9));
*/