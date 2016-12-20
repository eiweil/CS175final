#version 330

uniform samplerCube texUnit0;

in vec3 vNormal;
in vec4 vPosition;

out fragColor;

vec3 reflect(vec3 w, vec3 n) {
	return - w + n * (dot(w, n)*2.0);
}

void main(void) {
	vec3 normal = normalize(vNormal);
	vec3 reflected = reflect(normalize(vec3(-vPosition)), normal); 
	vec4 texColor0 = textureCube(texUnit0, reflected);
	fragColor = vec4(texColor0.r, texColor0.g, texColor0.b, 1.0);
}