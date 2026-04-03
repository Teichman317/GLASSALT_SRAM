attribute vec3 aPos;
attribute vec3 aNorm;
attribute vec2 aUV;

uniform mat4 uMVP;
uniform mat4 uModel;

varying vec3 vNorm;
varying vec2 vUV;

void main() {
    vNorm = mat3(uModel) * aNorm;
    vUV   = aUV;
    gl_Position = uMVP * vec4(aPos, 1.0);
}
