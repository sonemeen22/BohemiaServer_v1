#include "MoveSystem.h"


PxDefaultAllocator MoveSystem::gAllocator;
PxDefaultErrorCallback MoveSystem::gErrorCallback;

void MoveSystem::Init(Player& player)
{
    LoadNavMeshFile("./Data/navmesh_data.bin");
    InitNavQuery();
    InitQueryFilter();

    float start_pos[3] = { player.position.x, player.position.y, player.position.z };
    FindPlayerPolyRef(start_pos, player.polyRef, start_pos);

    foundation = PxCreateFoundation(
        PX_PHYSICS_VERSION,
        gAllocator,
        gErrorCallback
    );

    PxSceneDesc sceneDesc(physics->getTolerancesScale());

    sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);

    // CPU dispatcher
    sceneDesc.cpuDispatcher = PxDefaultCpuDispatcherCreate(2);

    // 碰撞过滤（PhysX 5 仍然有效）
    sceneDesc.filterShader = PxDefaultSimulationFilterShader;

    // 可选但推荐
    sceneDesc.flags |= PxSceneFlag::eENABLE_ACTIVE_ACTORS;

    PxScene* scene = physics->createScene(sceneDesc);

    PxRigidDynamic* actor =
        physics->createRigidDynamic(PxTransform(player.position));

    actor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);

    scene->addActor(*actor);

    player.actor = actor;

    PxShape* shape = physics->createShape(
        PxCapsuleGeometry(player.radius, player.halfHeight),
        *physics->createMaterial(0.5f, 0.5f, 0.0f)
    );

    actor->attachShape(*shape);
    shape->release(); // actor 持有

}

void MoveSystem::Destroy()
{
    scene->release();
    physics->release();
    foundation->release();
}

void MoveSystem::ServerTick(Player& p, float dt)
{
    // 1. 期望位移
    PxVec3 desiredMove = p.velocity * dt;

    // 2. NavMesh 限制
    PxVec3 navPos = MoveOnNavMesh(p, desiredMove);

    // 3. PhysX 碰撞 & 高度
    PxVec3 finalPos = ResolvePhysics(navPos);

    // 4. 写回 PhysX Kinematic
    p.actor->setKinematicTarget(PxTransform(finalPos));

    p.position = finalPos;
}


PxVec3 MoveSystem::MoveOnNavMesh(Player& p, const PxVec3& delta)
{
    float start[3] = { p.position.x, p.position.y, p.position.z };
    float end[3] = { start[0] + delta.x, start[1], start[2] + delta.z };

    float result[3];
    dtPolyRef visited[16];
    int visitedCount = 0;

    navQuery->moveAlongSurface(
        p.polyRef,
        start,
        end,
        &filter,
        result,
        visited,
        &visitedCount,
        16
    );

    if (visitedCount > 0)
        p.polyRef = visited[visitedCount - 1];

    return PxVec3(result[0], result[1], result[2]);
}

PxVec3 MoveSystem::ResolvePhysics(const PxVec3& pos)
{
    PxRaycastBuffer hit;

    PxVec3 origin = pos + PxVec3(0, 2.0f, 0);
    PxVec3 dir(0, -1, 0);

    if (scene->raycast(origin, dir, 5.0f, hit))
    {
        PxVec3 result = pos;
        result.y = hit.block.position.y;
        return result;
    }

    return pos;
}

bool MoveSystem::LoadNavMeshFile(const char* filename)
{
    std::ifstream in(filename, std::ios::binary);
    if (!in.is_open())
    {
        return false;
    }

    // 获取文件大小
    in.seekg(0, std::ios::end);
    size_t fileSize = in.tellg();
    in.seekg(0, std::ios::beg);

    // 读到 buffer
    std::vector<unsigned char> data(fileSize);
    in.read((char*)data.data(), fileSize);
    in.close();

    // 创建 dtNavMesh 对象
    navMesh = dtAllocNavMesh();
    if (!navMesh)
    {
        return false;
    }

    dtStatus status = navMesh->init(data.data(), (int)fileSize, DT_TILE_FREE_DATA);
    if (dtStatusFailed(status))
    {
        dtFreeNavMesh(navMesh);
        navMesh = nullptr;
        std::cout << "LoadNavMeshFile fail" << std::endl;
        return false;
    }

    return true;
}

bool MoveSystem::InitNavQuery()
{
    navQuery = dtAllocNavMeshQuery();
    if (!navQuery)
    {
        return false;
    }

    // 最大节点数 = 1024~4096，决定 moveAlongSurface 搜索深度
    dtStatus status = navQuery->init(navMesh, 2048);
    if (dtStatusFailed(status))
    {
        return false;
    }

    return true;
}

void MoveSystem::InitQueryFilter()
{
    // 包含所有 poly
    filter.setIncludeFlags(0xFFFF);
    filter.setExcludeFlags(0);
}

bool MoveSystem::FindPlayerPolyRef(const float* startPos, dtPolyRef& outRef, float* nearestPt)
{
    float extents[3] = { 0.5f, 1.0f, 0.5f };
    dtStatus status = navQuery->findNearestPoly(
        startPos,
        // 搜索扩展范围：角色半径
        extents,
        &filter,
        &outRef,
        nearestPt
    );
    return dtStatusSucceed(status);
}


