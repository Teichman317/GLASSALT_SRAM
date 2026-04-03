#ifndef SHADER_H
#define SHADER_H

#ifdef USE_GLES
#include <GLES2/gl2.h>
#else
#include <GL/glew.h>
#endif

GLuint shader_load(const char *vert_path, const char *frag_path);

#endif
