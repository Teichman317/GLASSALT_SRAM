#include "globe.h"
#include <math.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void globe_create(Globe *g, int stacks, int slices)
{
    int vert_count  = (stacks + 1) * (slices + 1);
    int floats_per  = 8; /* x,y,z, nx,ny,nz, u,v */
    float *verts    = malloc(vert_count * floats_per * sizeof(float));
    float *vp       = verts;

    for (int i = 0; i <= stacks; i++) {
        float phi = (float)M_PI * i / stacks;
        float sp  = sinf(phi);
        float cp  = cosf(phi);
        for (int j = 0; j <= slices; j++) {
            float theta = 2.0f * (float)M_PI * j / slices;
            float st = sinf(theta);
            float ct = cosf(theta);

            float x = sp * ct;
            float y = cp;
            float z = sp * st;

            *vp++ = x;  *vp++ = y;  *vp++ = z;   /* position */
            *vp++ = x;  *vp++ = y;  *vp++ = z;   /* normal = position (unit sphere) */
            *vp++ = (float)j / slices;             /* u */
            *vp++ = (float)i / stacks;             /* v */
        }
    }

    /* index buffer — two triangles per quad */
    int quad_count = stacks * slices;
    int idx_count  = quad_count * 6;
    unsigned int *indices = malloc(idx_count * sizeof(unsigned int));
    unsigned int *ip = indices;

    for (int i = 0; i < stacks; i++) {
        for (int j = 0; j < slices; j++) {
            int row0 = i * (slices + 1) + j;
            int row1 = row0 + (slices + 1);

            *ip++ = row0;
            *ip++ = row1;
            *ip++ = row0 + 1;

            *ip++ = row0 + 1;
            *ip++ = row1;
            *ip++ = row1 + 1;
        }
    }

    g->index_count = idx_count;

    glGenVertexArrays(1, &g->vao);
    glBindVertexArray(g->vao);

    glGenBuffers(1, &g->vbo);
    glBindBuffer(GL_ARRAY_BUFFER, g->vbo);
    glBufferData(GL_ARRAY_BUFFER, vert_count * floats_per * sizeof(float),
                 verts, GL_STATIC_DRAW);

    glGenBuffers(1, &g->ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, g->ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, idx_count * sizeof(unsigned int),
                 indices, GL_STATIC_DRAW);

    int stride = floats_per * sizeof(float);
    /* location 0: position */
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void *)0);
    /* location 1: normal */
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void *)(3 * sizeof(float)));
    /* location 2: uv */
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, stride, (void *)(6 * sizeof(float)));

    glBindVertexArray(0);

    free(verts);
    free(indices);
}

void globe_draw(const Globe *g)
{
    glBindVertexArray(g->vao);
    glDrawElements(GL_TRIANGLES, g->index_count, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

void globe_destroy(Globe *g)
{
    glDeleteBuffers(1, &g->vbo);
    glDeleteBuffers(1, &g->ebo);
    glDeleteVertexArrays(1, &g->vao);
}
