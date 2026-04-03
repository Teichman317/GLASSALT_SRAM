#version 330 core
in vec3 vNorm;
in vec2 vUV;

uniform sampler2D uTex;
uniform vec3 uLightDir;

out vec4 FragColor;

void main() {
    vec3 n     = normalize(vNorm);
    float diff = max(dot(n, normalize(uLightDir)), 0.0);
    vec4 col   = texture(uTex, vUV);
    FragColor  = vec4(col.rgb * (0.15 + 0.85 * diff), 1.0);
}
