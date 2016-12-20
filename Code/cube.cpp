#include "math.h"
#include "cube.h"
#ifdef __MAC__
#   include <OpenGL/gl3.h>
#   include <GLUT/glut.h>
#else
#include <GL/glew.h>
#   include <GL/glut.h>
#endif
#include <cstddef>
#include <vector>

// allocate memory for cube grid
bool CubeGrid::MemoryInit() {
	vertices = new CubeGridVtx[(maxGridSize + 1) * (maxGridSize + 1) * (maxGridSize + 1)];
	cubes = new CubeGridCube[maxGridSize * maxGridSize * maxGridSize];

	if(!vertices || !cubes)
	{
		return false;
	}

	return true;
}


bool CubeGrid::Init(int gridSize) {

	// total vertices
	numVertices = (gridSize + 1) * (gridSize + 1) * (gridSize + 1);
	
	int v = 0;

	for(int i=0; i < gridSize + 1; i++)
	{
		for(int j= 0; j < gridSize + 1; j++)
		{
			for(int k = 0; k < gridSize + 1; k++)
			{
				vertices[v].pos = Cvec3((i*20.0)/(gridSize)-10.0, (j*20.0)/(gridSize)-10.0, (k*20.0)/(gridSize)-10.0);
				v++;
			}
		}
	}

	// Generate cubes
	numCubes = (gridSize) * (gridSize) * (gridSize);

	int c = 0;

	for(int i = 0; i < gridSize; i++)
	{
		for(int j = 0; j < gridSize; j++)
		{
			for(int k = 0; k < gridSize; k++)
			{
				cubes[c].vertices[0] = &vertices[(i*(gridSize+1)+j)*(gridSize+1)+k];
				cubes[c].vertices[1] = &vertices[(i*(gridSize+1)+j)*(gridSize+1)+k+1];
				cubes[c].vertices[2] = &vertices[(i*(gridSize+1)+(j+1))*(gridSize+1)+k+1];
				cubes[c].vertices[3] = &vertices[(i*(gridSize+1)+(j+1))*(gridSize+1)+k];
				cubes[c].vertices[4] = &vertices[((i+1)*(gridSize+1)+j)*(gridSize+1)+k];
				cubes[c].vertices[5] = &vertices[((i+1)*(gridSize+1)+j)*(gridSize+1)+k+1];
				cubes[c].vertices[6] = &vertices[((i+1)*(gridSize+1)+(j+1))*(gridSize+1)+k+1];
				cubes[c].vertices[7] = &vertices[((i+1)*(gridSize+1)+(j+1))*(gridSize+1)+k];

				c++;
			}
		}
	}
	return true;
}

std::vector<SurfaceVtx> CubeGrid::CreateSurface(double threshold){
	numFacesDrawn = 0;

	static SurfaceVtx edgeVertices[12];
	std::vector<SurfaceVtx> verts;

	//check all cubes
	for(int i = 0; i < numCubes; i++)
	{
		//check which vertices are inside/outside the surface
		unsigned char cubeIndex=0;
		int power = 1;

		for(int v = 0; v < 8; v++) {
			if(cubes[i].vertices[v]->value < threshold)
				cubeIndex |= power;
				power *= 2;	
		}

		int usedEdges=edgeTable[cubeIndex];
	
		if(usedEdges==0 || usedEdges==255)
			continue;

		
		for(int currentEdge=0; currentEdge<12; currentEdge++)
		{
			if(usedEdges & 1<<currentEdge)
			{
				CubeGridVtx * v1 = cubes[i].vertices[verticesAtEndsOfEdges[currentEdge*2  ]];
				CubeGridVtx * v2 = cubes[i].vertices[verticesAtEndsOfEdges[currentEdge*2+1]];
			
				double delta = (threshold - v1->value)/(v2->value - v1->value);
				edgeVertices[currentEdge].pos[0]=v1->pos[0] + delta*(v2->pos[0] - v1->pos[0]);
				edgeVertices[currentEdge].pos[1]=v1->pos[1] + delta*(v2->pos[1] - v1->pos[1]);
				edgeVertices[currentEdge].pos[2]=v1->pos[2] + delta*(v2->pos[2] - v1->pos[2]);
				
				edgeVertices[currentEdge].normal[0]=v1->normal[0] + delta*(v2->normal[0] - v1->normal[0]);
				edgeVertices[currentEdge].normal[1]=v1->normal[1] + delta*(v2->normal[1] - v1->normal[1]);
				edgeVertices[currentEdge].normal[2]=v1->normal[2] + delta*(v2->normal[2] - v1->normal[2]);
			}
		}

		
		//send the vertices
		for(int k=0; triTable[cubeIndex][k]!=-1; k+=3)
		{
			verts.push_back(edgeVertices[triTable[cubeIndex][k]]);
			verts.push_back(edgeVertices[triTable[cubeIndex][k+2]]);
			verts.push_back(edgeVertices[triTable[cubeIndex][k+1]]);
							
			numFacesDrawn++;
		}
	}
	return verts;
}

// free memory for cube grid
void CubeGrid::FreeCubeGrid() {
	if(vertices) {
		delete [] vertices;
	}
		
	vertices = NULL;
	numVertices = 0;

	if(cubes) {
		delete [] cubes;
	}
	cubes = NULL;
	numCubes = 0;
}