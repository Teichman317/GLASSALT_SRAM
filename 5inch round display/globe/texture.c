#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include "texture.h"
#include <stdio.h>

GLuint texture_load(const char *path)
{
    int w, h, channels;
    stbi_set_flip_vertically_on_load(0);
    unsigned char *data = stbi_load(path, &w, &h, &channels, 4); /* force RGBA */
    if (!data) {
        fprintf(stderr, "Failed to load %s: %s\n", path, stbi_failure_reason());
        return 0;
    }

    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    stbi_image_free(data);
    printf("Loaded %s: %dx%d\n", path, w, h);
    return tex;
}
