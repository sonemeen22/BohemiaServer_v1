// NavMeshPathFinder.h
#pragma once
#include "NavMeshLoader.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>

#define INVALID_TRIANGLE_INDEX -1

struct PathNode {
    uint32_t triangleId;
    float gCost;      // 从起点到当前点的成本
    float hCost;      // 到终点的预估成本
    float fCost() const { return gCost + hCost; }
    uint32_t parentId;

    bool operator>(const PathNode& other) const {
        return fCost() > other.fCost();
    }
};

// RaycastHit 结构体定义（放在类定义内部或外部）
struct RaycastHit {
    glm::vec3 position;      // 碰撞点位置
    glm::vec3 normal;        // 碰撞点法线
    float distance;          // 碰撞距离
    uint32_t triangleIndex;  // 碰撞三角形索引
    bool hit;               // 是否发生碰撞

    RaycastHit() : distance(std::numeric_limits<float>::max()),
        triangleIndex(0), hit(false) {
    }
};

// Triangle 结构体定义
struct Triangle {
    glm::vec3 v0;  // 顶点0
    glm::vec3 v1;  // 顶点1
    glm::vec3 v2;  // 顶点2

    // 邻接三角形索引（可选，用于A*和漏斗算法优化）
    uint32_t neighbor0;  // 与边(v0,v1)相对的邻接三角形
    uint32_t neighbor1;  // 与边(v1,v2)相对的邻接三角形  
    uint32_t neighbor2;  // 与边(v2,v0)相对的邻接三角形

    // 三角形ID（可选）
    uint32_t id;

    // 中心点（预计算提高性能）
    glm::vec3 center;

    // 法线（预计算）
    glm::vec3 normal;

    // 构造器
    Triangle() : v0(0), v1(0), v2(0),
        neighbor0(INVALID_TRIANGLE_INDEX),
        neighbor1(INVALID_TRIANGLE_INDEX),
        neighbor2(INVALID_TRIANGLE_INDEX),
        id(0) {
    }

    Triangle(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, uint32_t triId = 0)
        : v0(a), v1(b), v2(c), id(triId) {
        // 预计算中心点和法线
        center = (v0 + v1 + v2) / 3.0f;
        normal = glm::normalize(glm::cross(v1 - v0, v2 - v0));
    }

    // 获取指定索引的顶点
    glm::vec3 GetVertex(int index) const {
        switch (index) {
        case 0: return v0;
        case 1: return v1;
        case 2: return v2;
        default: return v0;
        }
    }

    // 获取指定索引的邻接三角形
    uint32_t GetNeighbor(int index) const {
        switch (index) {
        case 0: return neighbor0;
        case 1: return neighbor1;
        case 2: return neighbor2;
        default: return INVALID_TRIANGLE_INDEX;
        }
    }

    // 设置指定索引的邻接三角形
    void SetNeighbor(int index, uint32_t neighborIdx) {
        switch (index) {
        case 0: neighbor0 = neighborIdx; break;
        case 1: neighbor1 = neighborIdx; break;
        case 2: neighbor2 = neighborIdx; break;
        }
    }

    // 获取三角形的边
    std::pair<glm::vec3, glm::vec3> GetEdge(int edgeIndex) const {
        switch (edgeIndex) {
        case 0: return { v0, v1 };  // 边 v0-v1
        case 1: return { v1, v2 };  // 边 v1-v2
        case 2: return { v2, v0 };  // 边 v2-v0
        default: return { v0, v1 };
        }
    }

    // 计算三角形面积
    float GetArea() const {
        glm::vec3 edge1 = v1 - v0;
        glm::vec3 edge2 = v2 - v0;
        return 0.5f * glm::length(glm::cross(edge1, edge2));
    }

    // 获取三角形边界框
    void GetBoundingBox(glm::vec3& minPoint, glm::vec3& maxPoint) const {
        minPoint = glm::min(v0, glm::min(v1, v2));
        maxPoint = glm::max(v0, glm::max(v1, v2));
    }

    // 判断点是否在三角形内部（使用重心坐标法）
    bool ContainsPoint(const glm::vec3& point, float epsilon = 0.001f) const {
        //return IsPointInTriangle(point, *this, epsilon);
        return true;
    }

    // 计算点到三角形平面的投影距离
    float DistanceToPlane(const glm::vec3& point) const {
        return glm::dot(normal, point - v0);
    }

    // 获取三角形平面上的投影点
    glm::vec3 ProjectPointToPlane(const glm::vec3& point) const {
        float dist = DistanceToPlane(point);
        return point - normal * dist;
    }

    // 计算三角形周长
    float GetPerimeter() const {
        float edge0 = glm::distance(v0, v1);
        float edge1 = glm::distance(v1, v2);
        float edge2 = glm::distance(v2, v0);
        return edge0 + edge1 + edge2;
    }

    // 序列化/反序列化支持（可选）
    std::vector<float> ToFloatArray() const {
        std::vector<float> result(9); // 3个顶点 * 3个分量
        result[0] = v0.x; result[1] = v0.y; result[2] = v0.z;
        result[3] = v1.x; result[4] = v1.y; result[5] = v1.z;
        result[6] = v2.x; result[7] = v2.y; result[8] = v2.z;
        return result;
    }
};

class NavMeshPathFinder {
public:
    NavMeshPathFinder(const NavMeshData& navMeshData);

    std::vector<glm::vec3> FindPath(const glm::vec3& start,
        const glm::vec3& end);

    bool IsPointWalkable(const glm::vec3& point, float agentRadius = 0.5f);
    //glm::vec3 FindClosestPoint(const glm::vec3& point);

    // 路径寻找辅助方法
    uint32_t FindTriangleContainingPoint(const glm::vec3& point);

private:
    const NavMeshData& navMesh_;

    const float maxStepHeight_ = 0.6;

    std::vector<uint32_t> GetNeighborTriangles(uint32_t triangleId);
    float CalculateHeuristic(uint32_t fromTriId, uint32_t toTriId);
    glm::vec3 GetTriangleCenter(uint32_t triangleId);
    bool PointInTriangle(const glm::vec3& point,
        const glm::vec3& v0,
        const glm::vec3& v1,
        const glm::vec3& v2);

    // 射线检测
    /*bool Raycast(const glm::vec3& origin, const glm::vec3& direction,
        float maxDistance, glm::vec3& hitPoint);*/

    // 漏斗算法优化路径
    std::vector<glm::vec3> FunnelAlgorithm(const std::vector<uint32_t>& trianglePath,
        const glm::vec3& start,
        const glm::vec3& end);

    bool Cross(const glm::vec3& apex, const glm::vec3& left, const glm::vec3& right);

    bool IsDirectPathWalkable(const glm::vec3& start, const glm::vec3& end);

    bool IsSegmentOnNavMesh(const glm::vec3& start, const glm::vec3& end);

    bool RaycastNavMesh(const glm::vec3& origin, const glm::vec3& direction,
        float maxDistance, RaycastHit& hit);

    bool TriangleRayIntersection(
        const glm::vec3& rayOrigin,
        const glm::vec3& rayDirection,
        const NavMeshTriangle& triangle,
        float& outDistance);
};
