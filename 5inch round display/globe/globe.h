#ifndef GLOBE_H
#define GLOBE_H

#ifdef USE_GLES
#include <GLES2/gl2.h>
#else
#include <GL/glew.h>
#endif

typedef struct {
    GLuint vbo;
    GLuint ebo;
#ifndef USE_GLES
    GLuint vao;
#endif
    int    index_count;
} Globe;

void globe_create(Globe *g, int stacks, int slices);
void globe_draw(const Globe *g);
void globe_destroy(Globe *g);

#endif
