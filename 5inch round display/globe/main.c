#include <stdio.h>
#include <math.h>

#ifdef USE_GLES
#include <GLES2/gl2.h>
#else
#include <GL/glew.h>
#endif

#include <SDL2/SDL.h>
#include "globe.h"
#include "shader.h"
#include "texture.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define WIN_W 1080
#define WIN_H 1080

/* ── minimal 4x4 matrix helpers (column-major) ── */

static void mat4_identity(float *m)
{
    for (int i = 0; i < 16; i++) m[i] = 0.0f;
    m[0] = m[5] = m[10] = m[15] = 1.0f;
}

static void mat4_mul(float *out, const float *a, const float *b)
{
    float tmp[16];
    for (int c = 0; c < 4; c++)
        for (int r = 0; r < 4; r++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++)
                s += a[r + k * 4] * b[k + c * 4];
            tmp[r + c * 4] = s;
        }
    for (int i = 0; i < 16; i++) out[i] = tmp[i];
}

static void mat4_rotate_y(float *m, float rad)
{
    mat4_identity(m);
    float c = cosf(rad), s = sinf(rad);
    m[0] = c;  m[8]  = s;
    m[2] = -s; m[10] = c;
}

static void mat4_rotate_x(float *m, float rad)
{
    mat4_identity(m);
    float c = cosf(rad), s = sinf(rad);
    m[5] = c;  m[9]  = -s;
    m[6] = s;  m[10] = c;
}

static void mat4_perspective(float *m, float fov_deg, float aspect,
                             float znear, float zfar)
{
    for (int i = 0; i < 16; i++) m[i] = 0.0f;
    float f = 1.0f / tanf(fov_deg * (float)M_PI / 360.0f);
    m[0]  = f / aspect;
    m[5]  = f;
    m[10] = (zfar + znear) / (znear - zfar);
    m[11] = -1.0f;
    m[14] = 2.0f * zfar * znear / (znear - zfar);
}

static void mat4_translate(float *m, float x, float y, float z)
{
    mat4_identity(m);
    m[12] = x;  m[13] = y;  m[14] = z;
}

/* ── circular mask overlay ── */

static GLuint mask_vbo, mask_prog;
#ifndef USE_GLES
static GLuint mask_vao;
#endif

#ifdef USE_GLES
static const char *mask_vs_src =
    "attribute vec2 aPos;\n"
    "varying vec2 vUV;\n"
    "void main() {\n"
    "    vUV = aPos * 0.5 + 0.5;\n"
    "    gl_Position = vec4(aPos, 0.0, 1.0);\n"
    "}\n";

static const char *mask_fs_src =
    "precision mediump float;\n"
    "varying vec2 vUV;\n"
    "void main() {\n"
    "    vec2 c = vUV - vec2(0.5);\n"
    "    if (dot(c,c) < 0.25) discard;\n"
    "    gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);\n"
    "}\n";
#else
static const char *mask_vs_src =
    "#version 330 core\n"
    "layout(location=0) in vec2 aPos;\n"
    "out vec2 vUV;\n"
    "void main() {\n"
    "    vUV = aPos * 0.5 + 0.5;\n"
    "    gl_Position = vec4(aPos, 0.0, 1.0);\n"
    "}\n";

static const char *mask_fs_src =
    "#version 330 core\n"
    "in vec2 vUV;\n"
    "out vec4 FragColor;\n"
    "void main() {\n"
    "    vec2 c = vUV - vec2(0.5);\n"
    "    if (dot(c,c) < 0.25) discard;\n"
    "    FragColor = vec4(0.0, 0.0, 0.0, 1.0);\n"
    "}\n";
#endif

static void mask_init(void)
{
    float quad[] = { -1,-1, 1,-1, -1,1, 1,1 };

#ifndef USE_GLES
    glGenVertexArrays(1, &mask_vao);
    glBindVertexArray(mask_vao);
#endif

    glGenBuffers(1, &mask_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, mask_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad), quad, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);

#ifndef USE_GLES
    glBindVertexArray(0);
#endif

    /* compile inline shaders */
    GLuint vs = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vs, 1, &mask_vs_src, NULL);
    glCompileShader(vs);
    GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fs, 1, &mask_fs_src, NULL);
    glCompileShader(fs);
    mask_prog = glCreateProgram();
    glAttachShader(mask_prog, vs);
    glAttachShader(mask_prog, fs);
#ifdef USE_GLES
    glBindAttribLocation(mask_prog, 0, "aPos");
#endif
    glLinkProgram(mask_prog);
    glDeleteShader(vs);
    glDeleteShader(fs);
}

static void mask_draw(void)
{
    glUseProgram(mask_prog);
#ifdef USE_GLES
    glBindBuffer(GL_ARRAY_BUFFER, mask_vbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
#else
    glBindVertexArray(mask_vao);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glBindVertexArray(0);
#endif
}

/* ── main ── */

int main(int argc, char *argv[])
{
    (void)argc; (void)argv;

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }

#ifdef USE_GLES
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#else
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);
#endif

    SDL_Window *win = SDL_CreateWindow("Globe",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WIN_W, WIN_H, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
    if (!win) {
        fprintf(stderr, "SDL_CreateWindow: %s\n", SDL_GetError());
        return 1;
    }

    SDL_GLContext ctx = SDL_GL_CreateContext(win);
    SDL_GL_SetSwapInterval(1); /* vsync */

#ifndef USE_GLES
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "glewInit failed\n");
        return 1;
    }
#endif

    printf("GL: %s\n", glGetString(GL_VERSION));

    glEnable(GL_DEPTH_TEST);
#ifndef USE_GLES
    glEnable(GL_MULTISAMPLE);
#endif
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    /* load resources */
#ifdef USE_GLES
    GLuint prog = shader_load("vert_es.glsl", "frag_es.glsl");
#else
    GLuint prog = shader_load("vert.glsl", "frag.glsl");
#endif
    if (!prog) return 1;

    GLuint tex = texture_load("earth_blue.jpg");
    if (!tex) {
        fprintf(stderr, "Failed to load earth_blue.jpg — place an equirectangular Earth "
                "texture named earth_blue.jpg in this directory.\n");
        return 1;
    }

    Globe globe;
    globe_create(&globe, 64, 64);
    mask_init();

    GLint loc_mvp      = glGetUniformLocation(prog, "uMVP");
    GLint loc_model    = glGetUniformLocation(prog, "uModel");
    GLint loc_tex      = glGetUniformLocation(prog, "uTex");
    GLint loc_lightdir = glGetUniformLocation(prog, "uLightDir");

    float angle = 0.0f;
    Uint32 last = SDL_GetTicks();
    int running = 1;

    while (running) {
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT) running = 0;
            if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_ESCAPE) running = 0;
        }

        Uint32 now = SDL_GetTicks();
        float dt = (now - last) / 1000.0f;
        last = now;
        angle += 20.0f * dt;
        if (angle > 360.0f) angle -= 360.0f;

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        /* build matrices */
        float ry[16], rx[16], model[16], view[16], proj[16], vp[16], mvp[16];

        mat4_rotate_y(ry, angle * (float)M_PI / 180.0f);
        mat4_rotate_x(rx, -23.5f * (float)M_PI / 180.0f); /* Earth axial tilt */
        mat4_mul(model, rx, ry);

        mat4_translate(view, 0.0f, 0.0f, -2.8f);
        mat4_perspective(proj, 45.0f, 1.0f, 0.1f, 100.0f);
        mat4_mul(vp, proj, view);
        mat4_mul(mvp, vp, model);

        /* draw globe */
        glUseProgram(prog);
        glUniformMatrix4fv(loc_mvp, 1, GL_FALSE, mvp);
        glUniformMatrix4fv(loc_model, 1, GL_FALSE, model);
        glUniform1i(loc_tex, 0);
        glUniform3f(loc_lightdir, 0.0f, 0.3f, 1.0f); /* sun direction — front-on */

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, tex);
        globe_draw(&globe);

        /* circular mask — black out corners to preview round display */
        glDisable(GL_DEPTH_TEST);
        mask_draw();
        glEnable(GL_DEPTH_TEST);

        SDL_GL_SwapWindow(win);
    }

    globe_destroy(&globe);
    glDeleteProgram(prog);
    glDeleteTextures(1, &tex);
    SDL_GL_DeleteContext(ctx);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
