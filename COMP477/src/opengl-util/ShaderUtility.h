/* 
Utility class to help load shaders
http://www.opengl-tutorial.org/beginners-tutorials/tutorial-2-the-first-triangle/
*/ 
#ifndef SHADER_UTILITY_H
#define SHADER_UTILITY_H

#include <GL\glew.h>
/*
  Loads and compiles shader code and returns the program id
*/
GLuint LoadShaders(const char * vertex_file_path, const char * fragment_file_path);

#endif
