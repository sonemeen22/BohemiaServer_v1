#include <vector>
#include <fstream>
#include <iostream>
#include <cmath>
#include <cstring>

#include "Recast.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"

struct InputMesh
{
	std::vector<float> verts; // x y z
	std::vector<int>   tris;  // indices
};

class NavMeshManager
{
public:
	int GenNavMesh();
private:
	bool LoadUnityGeometry(const char* path, InputMesh& out);
};

