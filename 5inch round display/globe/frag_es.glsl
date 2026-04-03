precision mediump float;

varying vec3 vNorm;
varying vec2 vUV;

uniform sampler2D uTex;
uniform vec3 uLightDir;

void main() {
    vec3 n     = normalize(vNorm);
    float diff = max(dot(n, normalize(uLightDir)), 0.0);
    vec4 col   = texture2D(uTex, vUV);
    gl_FragColor = vec4(col.rgb * (0.35 + 0.65 * diff), 1.0);
}
