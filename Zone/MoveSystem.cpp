#include "MoveSystem.h"

PxDefaultAllocator MoveSystem::gAllocator;
PxDefaultErrorCallback MoveSystem::gErrorCallback;

//-------------------------- 初始化 --------------------------
void MoveSystem::Init(Player& player)
{
    if (!LoadNavMeshFile("./Data/navmesh.bin"))
    {
        std::cerr << "Error: Failed to load NavMesh!" << std::endl;
        return;
    }

    if (!InitNavQuery())
    {
        std::cerr << "Error: Failed to initialize NavMeshQuery!" << std::endl;
        return;
    }

    InitQueryFilter();

    // 查找玩家起始 polyRef
    float start_pos[3] = { player.position.x, player.position.y, player.position.z };
    if (!FindPlayerPolyRef(start_pos, player.polyRef, start_pos))
    {
        std::cout << "Warning: Player start position not on NavMesh, using NavMesh center." << std::endl;

        // 以 NavMesh 中心或任意有效多边形作为起点
        float navCenter[3] = { 0.0f, 0.0f, 0.0f };
        if (!FindPlayerPolyRef(navCenter, player.polyRef, navCenter))
        {
            std::cerr << "Error: cannot find any valid poly on NavMesh! Using default origin." << std::endl;
            player.polyRef = 0;
            player.position = PxVec3(0, 0, 0);
        }
        else
        {
            player.position = PxVec3(navCenter[0], navCenter[1], navCenter[2]);
        }
    }
    else
    {
        player.position = PxVec3(start_pos[0], start_pos[1], start_pos[2]);
    }

    // PhysX 初始化
    foundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
    physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, PxTolerancesScale(), true, nullptr);
    assert(physics);

    PxSceneDesc sceneDesc(physics->getTolerancesScale());
    sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
    sceneDesc.cpuDispatcher = PxDefaultCpuDispatcherCreate(2);
    sceneDesc.filterShader = PxDefaultSimulationFilterShader;
    sceneDesc.flags |= PxSceneFlag::eENABLE_ACTIVE_ACTORS;

    scene = physics->createScene(sceneDesc);

    // 创建角色 Kinematic Actor
    PxRigidDynamic* actor = physics->createRigidDynamic(PxTransform(player.position));
    actor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
    scene->addActor(*actor);
    player.actor = actor;

    // 绑定胶囊形状
    PxShape* shape = physics->createShape(
        PxCapsuleGeometry(player.radius, player.halfHeight),
        *physics->createMaterial(0.5f, 0.5f, 0.0f)
    );
    actor->attachShape(*shape);
    shape->release();
}

//-------------------------- 销毁 --------------------------
void MoveSystem::Destroy()
{
    scene->release();
    physics->release();
    foundation->release();
}

//-------------------------- 服务端 Tick --------------------------
void MoveSystem::ServerTick(Player& p, float dt)
{
    PxVec3 desiredMove = p.velocity * dt;

    // 移动到 NavMesh 上
    PxVec3 navPos = MoveOnNavMesh(p, desiredMove);

    // 可选：贴地高度（结合 PhysX 射线）
    navPos = ResolvePhysics(navPos);

    // 写回 Kinematic Actor
    p.actor->setKinematicTarget(PxTransform(navPos));
    p.position = navPos;
}

//-------------------------- 安全 NavMesh 移动 --------------------------
PxVec3 MoveSystem::MoveOnNavMesh(Player& p, const PxVec3& delta)
{
    float start[3] = { p.position.x, p.position.y, p.position.z };
    float end[3] = { start[0] + delta.x, start[1], start[2] + delta.z };
    float result[3] = { start[0], start[1], start[2] };

    dtPolyRef visited[16];
    int visitedCount = 0;

    dtStatus status = navQuery->moveAlongSurface(
        p.polyRef,
        start,
        end,
        &filter,
        result,
        visited,
        &visitedCount,
        16
    );

    if (dtStatusFailed(status) || visitedCount == 0)
    {
        // 保持当前位置，防止非法值
        std::cout << "Warning: moveAlongSurface failed! status=" << status
            << " visitedCount=" << visitedCount << std::endl;
        return PxVec3(start[0], start[1], start[2]);
    }

    p.polyRef = visited[visitedCount - 1];
    return PxVec3(result[0], result[1], result[2]);
}

//-------------------------- PhysX 贴地 --------------------------
PxVec3 MoveSystem::ResolvePhysics(const PxVec3& pos)
{
    float halfHeight = 0.2f;
    PxRaycastBuffer hit;
    PxVec3 origin = pos + PxVec3(0, halfHeight + 0.1f, 0);
    PxVec3 dir(0, -1, 0);

    if (scene->raycast(origin, dir, halfHeight + 1.0f, hit))
    {
        PxVec3 result = pos;
        result.y = hit.block.position.y + halfHeight; // 保持胶囊底部接触地面
        return result;
    }

    return pos;
}

bool MoveSystem::LoadNavMeshFile(const char* filename)
{
    std::ifstream in(filename, std::ios::binary);
    if (!in.is_open())
    {
        std::cout << "LoadNavMeshFile open fail." << std::endl;
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
        std::cout << "dtAllocNavMesh fail." << std::endl;
        return false;
    }

    std::cout << "LoadNavMeshFile ok.1" << std::endl;

    dtStatus status = navMesh->init(data.data(), (int)fileSize, DT_TILE_FREE_DATA);
    if (dtStatusFailed(status))
    {
        dtFreeNavMesh(navMesh);
        navMesh = nullptr;
        std::cout << "LoadNavMeshFile fail." << std::endl;
        return false;
    }

    std::cout << "LoadNavMeshFile ok.2" << std::endl;

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
