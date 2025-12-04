// NavMeshPathFinder.cpp
#include "NavMeshPathFinder.h"
#include <algorithm>
#include <cmath>
#include <iostream>

NavMeshPathFinder::NavMeshPathFinder(const NavMeshData& navMeshData)
    : navMesh_(navMeshData) {
}

// 方法1：使用叉积计算三角形面积（3D）
inline float TriangleArea(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
    glm::vec3 ab = b - a;
    glm::vec3 ac = c - a;
    glm::vec3 ret = glm::cross(ab, ac);
    return ret.length() * 0.5f;
}

// 或者如果需要在3D空间中判断（使用三角形面积的正负号判断）
bool NavMeshPathFinder::Cross(const glm::vec3& apex, const glm::vec3& left, const glm::vec3& right) {
    // 计算左右向量与顶点形成的三角形的有向面积
    // 如果左右向量交叉，则面积会改变符号

    // 将3D投影到2D（忽略高度）或者使用原始3D坐标根据需求调整
    glm::vec2 apex2d(apex.x, apex.z);
    glm::vec2 left2d(left.x, left.z);
    glm::vec2 right2d(right.x, right.z);

    glm::vec2 leftVec = left2d - apex2d;
    glm::vec2 rightVec = right2d - apex2d;

    // 计算二维叉积
    float cross = leftVec.x * rightVec.y - leftVec.y * rightVec.x;

    // 如果叉积 <= 0，表示左右向量交叉或共线
    return cross <= 0.0f;
}

// 检查两点之间是否有直接可通行的路径
bool NavMeshPathFinder::IsDirectPathWalkable(const glm::vec3& start, const glm::vec3& end) {
    // 方法1: 简单的高度检查（如果高度差过大，不可行走）
    float heightDiff = std::abs(start.y - end.y);
    if (heightDiff > maxStepHeight_) { // maxStepHeight_ 是成员变量，表示最大可跨越高度
        return false;
    }

    // 方法2: 射线检测（检查路径上是否有障碍物）
    glm::vec3 direction = end - start;
    float distance = glm::length(direction);

    if (distance < 0.001f) {
        return true;
    }

    direction = glm::normalize(direction);

    // 使用NavMesh进行线段检测
    // 这里假设你有检查线段是否在NavMesh上的功能
    return IsSegmentOnNavMesh(start, end);
}

// 辅助函数：检查线段是否在NavMesh上
bool NavMeshPathFinder::IsSegmentOnNavMesh(const glm::vec3& start, const glm::vec3& end) {
    // 简单实现：采样线段上的多个点，检查每个点是否在NavMesh三角形内
    const int numSamples = 10;
    float step = 1.0f / numSamples;

    for (int i = 0; i <= numSamples; ++i) {
        float t = i * step;
        glm::vec3 samplePoint = start + t * (end - start);

        // 提升一点高度避免浮点误差
        samplePoint.y += 0.1f;

        // 向下射线检测，找到所在的三角形
        RaycastHit hit;
        if (RaycastNavMesh(samplePoint, glm::vec3(0, -1, 0), 1.0f, hit)) {
            // 点在地面上
            continue;
        }
        else {
            // 点不在NavMesh上
            return false;
        }
    }

    return true;
}

// 辅助函数：射线检测NavMesh
bool NavMeshPathFinder::RaycastNavMesh(const glm::vec3& origin, const glm::vec3& direction,
    float maxDistance, RaycastHit& hit) {
    // 这里需要实现射线与NavMesh三角形的相交检测
    // 简化实现：遍历所有三角形，找到最近的交点

    bool hasHit = false;
    float closestDistance = maxDistance;

    for (auto& triangle : navMesh_.triangles) {
        // 这里调用三角形射线相交检测
        // 需要实现 TriangleRayIntersection 函数
        float distance;
        if (TriangleRayIntersection(origin, direction, triangle, distance)) {
            if (distance < closestDistance && distance > 0.001f) {
                closestDistance = distance;
                hit.position = origin + direction * distance;
                hit.triangleIndex = &triangle - &navMesh_.triangles[0];
                hasHit = true;
            }
        }
    }

    return hasHit;
}

// 三角形射线相交检测（Möller–Trumbore 算法）
bool NavMeshPathFinder::TriangleRayIntersection(
    const glm::vec3& rayOrigin,
    const glm::vec3& rayDirection,
    const NavMeshTriangle& triangle,
    float& outDistance) {

    const float EPSILON = 0.000001f;

    glm::vec3 vertex0 = triangle.v0;
    glm::vec3 vertex1 = triangle.v1;
    glm::vec3 vertex2 = triangle.v2;

    // 计算三角形的两条边
    glm::vec3 edge1 = vertex1 - vertex0;
    glm::vec3 edge2 = vertex2 - vertex0;

    // 计算行列式（开始求解线性方程组）
    glm::vec3 h = glm::cross(rayDirection, edge2);
    float a = glm::dot(edge1, h);

    // 射线与三角形平行
    if (a > -EPSILON && a < EPSILON) {
        return false;
    }

    float f = 1.0f / a;
    glm::vec3 s = rayOrigin - vertex0;
    float u = f * glm::dot(s, h);

    // u 坐标超出三角形范围
    if (u < 0.0f || u > 1.0f) {
        return false;
    }

    glm::vec3 q = glm::cross(s, edge1);
    float v = f * glm::dot(rayDirection, q);

    // v 坐标超出三角形范围，或者 u+v > 1
    if (v < 0.0f || u + v > 1.0f) {
        return false;
    }

    // 计算交点距离
    float t = f * glm::dot(edge2, q);

    // 交点必须在射线正方向上
    if (t > EPSILON) {
        outDistance = t;
        return true;
    }

    return false;
}

std::vector<glm::vec3> NavMeshPathFinder::FindPath(const glm::vec3& start,
    const glm::vec3& end) {
    std::vector<glm::vec3> path;

    // 1. 找到包含起点和终点的三角形
    uint32_t startTri = FindTriangleContainingPoint(start);
    uint32_t endTri = FindTriangleContainingPoint(end);

    if (startTri == UINT32_MAX || endTri == UINT32_MAX) {
        std::cout << "起点或终点不在NavMesh上" << std::endl;
        return path;
    }

    // 2. A*算法寻找三角形路径
    std::priority_queue<PathNode, std::vector<PathNode>, std::greater<PathNode>> openSet;
    std::unordered_map<uint32_t, PathNode> allNodes;
    std::unordered_set<uint32_t> closedSet;

    PathNode startNode{ startTri, 0.0f, CalculateHeuristic(startTri, endTri), startTri };
    openSet.push(startNode);
    allNodes[startTri] = startNode;

    while (!openSet.empty()) {
        PathNode current = openSet.top();
        openSet.pop();

        if (current.triangleId == endTri) {
            // 重建路径
            std::vector<uint32_t> trianglePath;
            uint32_t currentId = current.triangleId;

            while (currentId != startTri) {
                trianglePath.push_back(currentId);
                currentId = allNodes[currentId].parentId;
            }
            trianglePath.push_back(startTri);
            std::reverse(trianglePath.begin(), trianglePath.end());

            // 3. 使用漏斗算法优化路径点
            return FunnelAlgorithm(trianglePath, start, end);
        }

        if (closedSet.find(current.triangleId) != closedSet.end()) {
            continue;
        }
        closedSet.insert(current.triangleId);

        // 处理邻居三角形
        auto neighbors = GetNeighborTriangles(current.triangleId);
        for (uint32_t neighborId : neighbors) {
            if (closedSet.find(neighborId) != closedSet.end()) {
                continue;
            }

            const auto& currentTri = navMesh_.triangles[current.triangleId];
            const auto& neighborTri = navMesh_.triangles[neighborId];

            // 计算移动成本
            float moveCost = current.gCost +
                glm::distance(GetTriangleCenter(current.triangleId),
                    GetTriangleCenter(neighborId)) *
                neighborTri.areaCost;

            auto it = allNodes.find(neighborId);
            if (it == allNodes.end() || moveCost < it->second.gCost) {
                PathNode neighborNode{
                    neighborId,
                    moveCost,
                    CalculateHeuristic(neighborId, endTri),
                    current.triangleId
                };

                allNodes[neighborId] = neighborNode;
                openSet.push(neighborNode);
            }
        }
    }

    return path; // 找不到路径
}

std::vector<uint32_t> NavMeshPathFinder::GetNeighborTriangles(uint32_t triangleId) {
    std::vector<uint32_t> neighbors;

    // 检查三角形ID是否有效
    if (triangleId >= navMesh_.triangles.size()) {
        return neighbors;
    }

    const NavMeshTriangle& currentTriangle = navMesh_.triangles[triangleId];

    // 遍历所有三角形，查找共享边的相邻三角形
    for (uint32_t otherTriId = 0; otherTriId < navMesh_.triangles.size(); ++otherTriId) {
        // 跳过自己
        if (otherTriId == triangleId) {
            continue;
        }

        const NavMeshTriangle& otherTriangle = navMesh_.triangles[otherTriId];

        // 检查是否共享一条边（两个顶点）
        int sharedVertices = 0;

        // 比较所有顶点对
        std::vector<glm::vec3> currentVerts = {
            currentTriangle.v0, currentTriangle.v1, currentTriangle.v2
        };
        std::vector<glm::vec3> otherVerts = {
            otherTriangle.v0, otherTriangle.v1, otherTriangle.v2
        };

        // 判断两个顶点是否相同（考虑浮点精度误差）
        auto verticesEqual = [](const glm::vec3& a, const glm::vec3& b) -> bool {
            const float EPSILON = 0.001f; // 设置合适的容差值
            return glm::length(a - b) < EPSILON;
            };

        // 计算共享的顶点数量
        for (const auto& cv : currentVerts) {
            for (const auto& ov : otherVerts) {
                if (verticesEqual(cv, ov)) {
                    sharedVertices++;
                    break;
                }
            }
        }

        // 如果共享2个顶点，说明是相邻三角形
        if (sharedVertices == 2) {
            neighbors.push_back(otherTriId);
        }
    }

    return neighbors;
}

glm::vec3 NavMeshPathFinder::GetTriangleCenter(uint32_t triangleId) {
    // 检查三角形ID是否有效
    if (triangleId >= navMesh_.triangles.size()) {
        // 返回一个无效点或者抛出异常
        return glm::vec3(0.0f, 0.0f, 0.0f);
    }

    const NavMeshTriangle& triangle = navMesh_.triangles[triangleId];

    // 计算三角形的质心（三个顶点的平均值）
    glm::vec3 center = (triangle.v0 + triangle.v1 + triangle.v2) / 3.0f;

    return center;
}

float NavMeshPathFinder::CalculateHeuristic(uint32_t fromTriId, uint32_t toTriId) {
    // 检查三角形ID是否有效
    if (fromTriId >= navMesh_.triangles.size() ||
        toTriId >= navMesh_.triangles.size()) {
        return std::numeric_limits<float>::max();
    }

    // 方法1：使用三角形中心之间的欧几里得距离作为启发值
    glm::vec3 fromCenter = GetTriangleCenter(fromTriId);
    glm::vec3 toCenter = GetTriangleCenter(toTriId);

    // 计算三维空间中的直线距离
    return glm::length(toCenter - fromCenter);
}

// 漏斗算法实现
std::vector<glm::vec3> NavMeshPathFinder::FunnelAlgorithm(
    const std::vector<uint32_t>& trianglePath,
    const glm::vec3& start,
    const glm::vec3& end) {

    std::vector<glm::vec3> pathPoints;
    pathPoints.push_back(start);

    if (trianglePath.empty()) {
        pathPoints.push_back(end);
        return pathPoints;
    }

    // 收集所有门户边
    std::vector<std::pair<glm::vec3, glm::vec3>> portals;

    for (size_t i = 0; i < trianglePath.size() - 1; ++i) {
        const auto& triA = navMesh_.triangles[trianglePath[i]];
        const auto& triB = navMesh_.triangles[trianglePath[i + 1]];

        // 找到共享边
        std::vector<glm::vec3> sharedVertices;
        std::vector<glm::vec3> triAVertices = { triA.v0, triA.v1, triA.v2 };
        std::vector<glm::vec3> triBVertices = { triB.v0, triB.v1, triB.v2 };

        for (const auto& vA : triAVertices) {
            for (const auto& vB : triBVertices) {
                if (glm::distance(vA, vB) < 0.001f) {
                    sharedVertices.push_back(vA);
                }
            }
        }

        if (sharedVertices.size() == 2) {
            portals.push_back({ sharedVertices[0], sharedVertices[1] });
        }
    }

    // 漏斗算法核心
    glm::vec3 apex = start;
    glm::vec3 left = portals[0].first;
    glm::vec3 right = portals[0].second;

    for (size_t i = 1; i < portals.size(); ++i) {
        const auto& portal = portals[i];

        // 更新左右边界
        if (TriangleArea(apex, left, portal.first) >= 0) {
            if (TriangleArea(apex, right, portal.first) > 0) {
                left = portal.first;
            }
        }

        if (TriangleArea(apex, right, portal.second) <= 0) {
            if (TriangleArea(apex, left, portal.second) < 0) {
                right = portal.second;
            }
        }

        // 如果左右边界交叉，添加路径点并重置漏斗
        if (Cross(apex, left, right)) {
            pathPoints.push_back(apex);
            apex = left;
            left = portal.first;
            right = portal.second;
        }
    }

    pathPoints.push_back(end);

    // 简化路径（去除不必要的点）
    std::vector<glm::vec3> simplifiedPath;
    simplifiedPath.push_back(pathPoints[0]);

    for (size_t i = 1; i < pathPoints.size() - 1; ++i) {
        glm::vec3 prev = simplifiedPath.back();
        glm::vec3 next = pathPoints[i + 1];

        // 检查直线是否可行走
        if (!IsDirectPathWalkable(prev, next)) {
            simplifiedPath.push_back(pathPoints[i]);
        }
    }

    simplifiedPath.push_back(end);

    return simplifiedPath;
}



bool NavMeshPathFinder::IsPointWalkable(const glm::vec3& point, float agentRadius) {
    uint32_t triId = FindTriangleContainingPoint(point);
    if (triId == UINT32_MAX) return false;

    const auto& triangle = navMesh_.triangles[triId];

    // 检查区域是否可行走
    if (triangle.area != 0) { // 0通常是可行走区域
        return false;
    }

    // 检查是否有障碍物（简化的碰撞检测）
    glm::vec3 center = GetTriangleCenter(triId);
    float distanceToCenter = glm::distance(point, center);

    return true;
}

uint32_t NavMeshPathFinder::FindTriangleContainingPoint(const glm::vec3& point) {
    // 使用空间网格加速查询
    int gridX = static_cast<int>((point.x - navMesh_.boundsMin.x) / navMesh_.cellSize);
    int gridY = static_cast<int>((point.y - navMesh_.boundsMin.y) / navMesh_.cellSize);
    int gridZ = static_cast<int>((point.z - navMesh_.boundsMin.z) / navMesh_.cellSize);

    gridX = std::max(0, std::min(navMesh_.gridWidth - 1, gridX));
    gridY = std::max(0, std::min(navMesh_.gridHeight - 1, gridY));
    gridZ = std::max(0, std::min(navMesh_.gridDepth - 1, gridZ));

    int gridIdx = gridX + gridY * navMesh_.gridWidth +
        gridZ * navMesh_.gridWidth * navMesh_.gridHeight;

    // 检查网格中的三角形
    for (uint32_t triId : navMesh_.spatialGrid[gridIdx]) {
        const auto& tri = navMesh_.triangles[triId];

        if (PointInTriangle(point, tri.v0, tri.v1, tri.v2)) {
            return triId;
        }
    }

    return UINT32_MAX;
}

bool NavMeshPathFinder::PointInTriangle(const glm::vec3& point,
    const glm::vec3& v0,
    const glm::vec3& v1,
    const glm::vec3& v2) {
    // 使用重心坐标法
    glm::vec3 v0v1 = v1 - v0;
    glm::vec3 v0v2 = v2 - v0;
    glm::vec3 v0p = point - v0;

    float dot00 = glm::dot(v0v2, v0v2);
    float dot01 = glm::dot(v0v2, v0v1);
    float dot02 = glm::dot(v0v2, v0p);
    float dot11 = glm::dot(v0v1, v0v1);
    float dot12 = glm::dot(v0v1, v0p);

    float invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
    float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

    return (u >= 0) && (v >= 0) && (u + v <= 1);
}
