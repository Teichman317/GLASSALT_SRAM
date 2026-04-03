#ifndef GLOBE_H
#define GLOBE_H

#include <GL/glew.h>

typedef struct {
    GLuint vao;
    GLuint vbo;
    GLuint ebo;
    int    index_count;
} Globe;

void globe_create(Globe *g, int stacks, int slices);
void globe_draw(const Globe *g);
void globe_destroy(Globe *g);

#endif
