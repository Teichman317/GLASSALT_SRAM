#include "shader.h"
#include <stdio.h>
#include <stdlib.h>

static char *read_file(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "Cannot open %s\n", path); return NULL; }
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    rewind(f);
    char *buf = malloc(len + 1);
    fread(buf, 1, len, f);
    buf[len] = '\0';
    fclose(f);
    return buf;
}

static GLuint compile(const char *src, GLenum type)
{
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, NULL);
    glCompileShader(s);
    GLint ok;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(s, sizeof(log), NULL, log);
        fprintf(stderr, "Shader compile error:\n%s\n", log);
    }
    return s;
}

GLuint shader_load(const char *vert_path, const char *frag_path)
{
    char *vsrc = read_file(vert_path);
    char *fsrc = read_file(frag_path);
    if (!vsrc || !fsrc) { free(vsrc); free(fsrc); return 0; }

    GLuint vs = compile(vsrc, GL_VERTEX_SHADER);
    GLuint fs = compile(fsrc, GL_FRAGMENT_SHADER);
    free(vsrc);
    free(fsrc);

    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
#ifdef USE_GLES
    glBindAttribLocation(prog, 0, "aPos");
    glBindAttribLocation(prog, 1, "aNorm");
    glBindAttribLocation(prog, 2, "aUV");
#endif
    glLinkProgram(prog);

    GLint ok;
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetProgramInfoLog(prog, sizeof(log), NULL, log);
        fprintf(stderr, "Shader link error:\n%s\n", log);
    }

    glDeleteShader(vs);
    glDeleteShader(fs);
    return prog;
}
