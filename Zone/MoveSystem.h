// NavMeshLoader.h
#pragma once

#define GLM_ENABLE_EXPERIMENTAL

#include "glm/glm.hpp"
#include <glm/common.hpp>
#include <glm/gtc/epsilon.hpp>
#include <glm/gtx/norm.hpp> 
#include <unordered_map>
// ===== PhysX 核心 =====
#include <PxPhysicsAPI.h>
#include "extensions/PxDefaultSimulationFilterShader.h"

// Detour
#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>

#include <DetourCommon.h>
#include <fstream>
#include <cassert>

#include <vector>
#include <string>
#include <iostream>

using namespace physx;

//typedef unsigned long long dtPolyRef;

struct Player
{
    dtPolyRef polyRef;
    PxRigidDynamic* actor;
    PxVec3 position;
    PxVec3 velocity;
    float radius;
    float halfHeight;
    float speed;
};

class MoveSystem {
public:
    void Init(Player& player);
    void ServerTick(Player& p, float dt);

    dtNavMesh* navMesh = nullptr;
    dtNavMeshQuery* navQuery = nullptr;
    dtQueryFilter filter;

private:
    PxVec3 MoveOnNavMesh(Player& p, const PxVec3& delta);
    PxVec3 ResolvePhysics(const PxVec3& pos);

    bool LoadNavMeshFile(const char* filename);
    bool InitNavQuery();
    void InitQueryFilter();

    bool FindPlayerPolyRef(const float* startPos, dtPolyRef& outRef, float* nearestPt);

    void Destroy();

    static PxDefaultAllocator      gAllocator;
    static PxDefaultErrorCallback  gErrorCallback;

    PxFoundation* foundation = nullptr;
    PxPhysics* physics = nullptr;
    PxScene* scene = nullptr;
};
