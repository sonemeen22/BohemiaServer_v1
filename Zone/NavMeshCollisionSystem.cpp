// NavMeshCollisionSystem.cpp
#include "NavMeshCollisionSystem.h"
#include <algorithm>
#include <cmath>

NavMeshCollisionSystem::NavMeshCollisionSystem(const NavMeshData& navMeshData)
    : navMesh_(navMeshData), pathFinder_(navMeshData) {
}

CollisionInfo NavMeshCollisionSystem::CheckCharacterCollision(
    const glm::vec3& position, float radius, float height) {

    CollisionInfo collisionInfo{};
    collisionInfo.hasCollision = false;

    // 胶囊体表示角色
    glm::vec3 capsuleBottom = position;
    capsuleBottom.y += radius; // 底部球体中心
    glm::vec3 capsuleTop = capsuleBottom;
    capsuleTop.y += height - 2 * radius; // 顶部球体中心

    // 查询周围的三角形
    int gridX = static_cast<int>((position.x - navMesh_.boundsMin.x) / navMesh_.cellSize);
    int gridY = static_cast<int>((position.y - navMesh_.boundsMin.y) / navMesh_.cellSize);
    int gridZ = static_cast<int>((position.z - navMesh_.boundsMin.z) / navMesh_.cellSize);

    int minGridX = std::max(0, gridX - 1);
    int maxGridX = std::min(navMesh_.gridWidth - 1, gridX + 1);
    int minGridY = std::max(0, gridY - 1);
    int maxGridY = std::min(navMesh_.gridHeight - 1, gridY + 1);
    int minGridZ = std::max(0, gridZ - 1);
    int maxGridZ = std::min(navMesh_.gridDepth - 1, gridZ + 1);

    for (int x = minGridX; x <= maxGridX; ++x) {
        for (int y = minGridY; y <= maxGridY; ++y) {
            for (int z = minGridZ; z <= maxGridZ; ++z) {
                int gridIdx = x + y * navMesh_.gridWidth +
                    z * navMesh_.gridWidth * navMesh_.gridHeight;

                for (uint32_t triId : navMesh_.spatialGrid[gridIdx]) {
                    const auto& triangle = navMesh_.triangles[triId];

                    // 检查胶囊体与三角形碰撞
                    if (CapsuleTriangleCollision(capsuleBottom, radius,
                        height, triangle, collisionInfo)) {
                        collisionInfo.triangleId = triId;
                        collisionInfo.hasCollision = true;
                        return collisionInfo;
                    }
                }
            }
        }
    }

    return collisionInfo;
}

bool NavMeshCollisionSystem::CapsuleTriangleCollision(
    const glm::vec3& capsuleBottom, float capsuleRadius, float capsuleHeight,
    const NavMeshTriangle& triangle, CollisionInfo& collisionInfo) {

    // 简化实现：检查胶囊体的两个球体与三角形的碰撞
    glm::vec3 capsuleTop = capsuleBottom;
    capsuleTop.y += capsuleHeight - 2 * capsuleRadius;

    // 检查底部球体
    CollisionInfo sphereCollision;
    if (SphereTriangleCollision(capsuleBottom, capsuleRadius,
        triangle, sphereCollision)) {
        collisionInfo = sphereCollision;
        return true;
    }

    // 检查顶部球体
    if (SphereTriangleCollision(capsuleTop, capsuleRadius,
        triangle, sphereCollision)) {
        collisionInfo = sphereCollision;
        return true;
    }

    // 检查圆柱体部分（简化为线段与三角形）
    // ... 实现线段与三角形碰撞检测

    return false;
}

bool NavMeshCollisionSystem::SphereTriangleCollision(
    const glm::vec3& sphereCenter, float sphereRadius,
    const NavMeshTriangle& triangle, CollisionInfo& collisionInfo) {

    // 找到球心到三角形的最近点
    glm::vec3 closestPoint;
    float distance = PointToTriangleDistance(sphereCenter,
        triangle.v0,
        triangle.v1,
        triangle.v2,
        closestPoint);

    if (distance <= sphereRadius) {
        collisionInfo.hasCollision = true;
        collisionInfo.collisionPoint = closestPoint;
        collisionInfo.collisionNormal = glm::normalize(sphereCenter - closestPoint);
        collisionInfo.penetrationDepth = sphereRadius - distance;
        return true;
    }

    return false;
}

float NavMeshCollisionSystem::PointToTriangleDistance(
    const glm::vec3& point, const glm::vec3& v0,
    const glm::vec3& v1, const glm::vec3& v2,
    glm::vec3& closestPoint) {

    // 实现点与三角形距离计算（使用重心坐标）
    glm::vec3 edge0 = v1 - v0;
    glm::vec3 edge1 = v2 - v0;
    glm::vec3 v0ToPoint = point - v0;

    float a = glm::dot(edge0, edge0);
    float b = glm::dot(edge0, edge1);
    float c = glm::dot(edge1, edge1);
    float d = glm::dot(edge0, v0ToPoint);
    float e = glm::dot(edge1, v0ToPoint);

    float det = a * c - b * b;
    float s = b * e - c * d;
    float t = b * d - a * e;

    if (s + t < det) {
        if (s < 0.0f) {
            if (t < 0.0f) {
                // 区域4
                if (d < 0.0f) {
                    s = std::clamp(-d / a, 0.0f, 1.0f);
                    t = 0.0f;
                }
                else {
                    s = 0.0f;
                    t = std::clamp(-e / c, 0.0f, 1.0f);
                }
            }
            else {
                // 区域3
                s = 0.0f;
                t = std::clamp(-e / c, 0.0f, 1.0f);
            }
        }
        else if (t < 0.0f) {
            // 区域5
            s = std::clamp(-d / a, 0.0f, 1.0f);
            t = 0.0f;
        }
        else {
            // 区域0
            float invDet = 1.0f / det;
            s *= invDet;
            t *= invDet;
        }
    }
    else {
        if (s < 0.0f) {
            // 区域2
            float tmp0 = b + d;
            float tmp1 = c + e;
            if (tmp1 > tmp0) {
                float numer = tmp1 - tmp0;
                float denom = a - 2 * b + c;
                s = std::clamp(numer / denom, 0.0f, 1.0f);
                t = 1.0f - s;
            }
            else {
                s = 0.0f;
                t = std::clamp(tmp1 / c, 0.0f, 1.0f);
            }
        }
        else if (t < 0.0f) {
            // 区域6
            float tmp0 = b + e;
            float tmp1 = a + d;
            if (tmp1 > tmp0) {
                float numer = tmp1 - tmp0;
                float denom = a - 2 * b + c;
                t = std::clamp(numer / denom, 0.0f, 1.0f);
                s = 1.0f - t;
            }
            else {
                t = 0.0f;
                s = std::clamp(tmp1 / a, 0.0f, 1.0f);
            }
        }
        else {
            // 区域1
            float numer = c + e - b - d;
            if (numer <= 0.0f) {
                s = 0.0f;
            }
            else {
                float denom = a - 2 * b + c;
                s = std::clamp(numer / denom, 0.0f, 1.0f);
            }
            t = 1.0f - s;
        }
    }

    closestPoint = v0 + s * edge0 + t * edge1;
    return glm::distance(point, closestPoint);
}

glm::vec3 NavMeshCollisionSystem::GetWalkablePosition(
    const glm::vec3& desiredPos, float agentRadius) {

    // 1. 先找到最近的可行走点
    glm::vec3 adjustedPos = desiredPos;

    // 2. 查询当前位置的三角形
    uint32_t triId = pathFinder_.FindTriangleContainingPoint(desiredPos);
    if (triId != UINT32_MAX) {
        const auto& triangle = navMesh_.triangles[triId];

        // 检查该点是否在可行走区域内
        if (triangle.area == 0) { // 可行走区域
            return desiredPos;
        }
    }

    // 3. 如果不在可行走区域，寻找最近可行走点
    glm::vec3 closestPoint;
    float minDistance = FLT_MAX;

    // 搜索周围的三角形
    int gridX = static_cast<int>((desiredPos.x - navMesh_.boundsMin.x) / navMesh_.cellSize);
    int gridY = static_cast<int>((desiredPos.y - navMesh_.boundsMin.y) / navMesh_.cellSize);
    int gridZ = static_cast<int>((desiredPos.z - navMesh_.boundsMin.z) / navMesh_.cellSize);

    int searchRadius = 2;

    for (int dx = -searchRadius; dx <= searchRadius; ++dx) {
        for (int dy = -searchRadius; dy <= searchRadius; ++dy) {
            for (int dz = -searchRadius; dz <= searchRadius; ++dz) {
                int x = std::clamp(gridX + dx, 0, navMesh_.gridWidth - 1);
                int y = std::clamp(gridY + dy, 0, navMesh_.gridHeight - 1);
                int z = std::clamp(gridZ + dz, 0, navMesh_.gridDepth - 1);

                int gridIdx = x + y * navMesh_.gridWidth +
                    z * navMesh_.gridWidth * navMesh_.gridHeight;

                for (uint32_t triId : navMesh_.spatialGrid[gridIdx]) {
                    const auto& triangle = navMesh_.triangles[triId];

                    if (triangle.area == 0) { // 可行走三角形
                        // 计算点到三角形平面的投影
                        glm::vec3 pointOnTriangle;
                        float distance = PointToTriangleDistance(desiredPos,
                            triangle.v0,
                            triangle.v1,
                            triangle.v2,
                            pointOnTriangle);

                        if (distance < minDistance) {
                            minDistance = distance;
                            closestPoint = pointOnTriangle;
                        }
                    }
                }
            }
        }
    }

    if (minDistance < FLT_MAX) {
        adjustedPos = closestPoint;
    }

    return adjustedPos;
}
