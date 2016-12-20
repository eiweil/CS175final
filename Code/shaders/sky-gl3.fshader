#version 120

uniform samplerCube CubeMap;

//out vec4 fragColor;

void main()
{
    gl_FragColor = textureCube(CubeMap, gl_TexCoord[0].stp);
}