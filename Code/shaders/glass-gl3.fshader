#version 150

uniform vec3 uLight, uLight2, uColor;

in vec3 vNormal;
in vec3 vPosition;

out vec4 fragColor;

void main() {
    vec3 N = normalize(vNormal);
    vec3 P = vPosition;
    vec3 I = normalize(P);
    float cosTheta = abs(dot(I, N));
    float fresnel = pow(1.0 - cosTheta, 4.0);

    float depth = 1.0 * vPosition.z;

    float sigma = 30.0;

    float intensity = fresnel * exp(-sigma * depth);


    fragColor = vec4(0.5, 0.0, 1.0, fresnel + 0.3);
}
