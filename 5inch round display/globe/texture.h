#ifndef TEXTURE_H
#define TEXTURE_H

#ifdef USE_GLES
#include <GLES2/gl2.h>
#else
#include <GL/glew.h>
#endif

GLuint texture_load(const char *path);

#endif
