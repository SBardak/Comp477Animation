#ifndef TEXTURE_UTILITY_H
#define TEXTURE_UTILITY_H
#include <GL\glew.h>

/*
* Utility class to help load textures
http://www.opengl-tutorial.org/beginners-tutorials/tutorial-5-a-textured-cube/
*/

// Load a .DDS file 
GLuint loadDDS(const char * imagepath);

#endif
