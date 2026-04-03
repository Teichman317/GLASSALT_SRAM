#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNorm;
layout(location = 2) in vec2 aUV;

uniform mat4 uMVP;
uniform mat4 uModel;

out vec3 vNorm;
out vec2 vUV;

void main() {
    vNorm = mat3(uModel) * aNorm;
    vUV   = aUV;
    gl_Position = uMVP * vec4(aPos, 1.0);
}
