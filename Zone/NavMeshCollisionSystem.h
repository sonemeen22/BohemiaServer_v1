// NavMeshCollisionSystem.h
#pragma once
#include "NavMeshLoader.h"
#include "NavMeshPathFinder.h"
#include <vector>

struct CollisionInfo {
    bool hasCollision;
    glm::vec3 collisionPoint;
    glm::vec3 collisionNormal;
    float penetrationDepth;
    uint32_t triangleId;
};

class NavMeshCollisionSystem {
public:
    NavMeshCollisionSystem(const NavMeshData& navMeshData);

    // 角色与NavMesh的碰撞检测
    CollisionInfo CheckCharacterCollision(const glm::vec3& position,
        float radius,
        float height);

    // 移动物体碰撞检测
    /*CollisionInfo CheckMovementCollision(const glm::vec3& startPos,
        const glm::vec3& endPos,
        float radius);*/

    // 射线碰撞检测
    /*bool RaycastNavMesh(const glm::vec3& origin,
        const glm::vec3& direction,
        float maxDistance,
        glm::vec3& hitPoint,
        glm::vec3& hitNormal);*/

    // 获取可行走位置
    glm::vec3 GetWalkablePosition(const glm::vec3& desiredPos,
        float agentRadius);

private:
    const NavMeshData& navMesh_;
    NavMeshPathFinder pathFinder_;

    // 胶囊体与三角形碰撞检测
    bool CapsuleTriangleCollision(const glm::vec3& capsuleBottom,
        float capsuleRadius,
        float capsuleHeight,
        const NavMeshTriangle& triangle,
        CollisionInfo& collisionInfo);

    // 球体与三角形碰撞检测
    bool SphereTriangleCollision(const glm::vec3& sphereCenter,
        float sphereRadius,
        const NavMeshTriangle& triangle,
        CollisionInfo& collisionInfo);

    // 计算点到三角形的最短距离
    float PointToTriangleDistance(const glm::vec3& point,
        const glm::vec3& v0,
        const glm::vec3& v1,
        const glm::vec3& v2,
        glm::vec3& closestPoint);
};

