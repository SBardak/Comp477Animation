#ifndef SNOWFLAKE_H
#define SNOWFLAKE_H

#include "opengl-util\TextureUtility.h"

class Snowflake
{
public:
	static GLuint vertexArrayID;

	// The VBO containing the 4 vertices of the particles.
	static const float vertexBuffer[12];

	static GLuint Texture;

	static GLuint vertexBufferID;

	Snowflake(float x, float y, float z, float width, float height);

	static void InitVertexBuffers();

	static void InitDraw();

	void Draw();

	static void FinishDraw();

	virtual void Update() {}

	static void CleanUp();

	virtual float* getPosition() { return position; }
	float getWidth() { return size[0]; }
	float getHeight() { return size[1]; }
	

protected:
	float position[3];
	float size[2];

};

#endif