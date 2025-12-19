// NavMeshLoader.h
#pragma once
#include <vector>
#include <string>
#include "glm/glm.hpp"
#include <glm/common.hpp>
#include <glm/gtc/epsilon.hpp>
#include <unordered_map>

struct NavMeshTriangle {
    glm::vec3 v0, v1, v2;
    glm::vec3 normal;
    int area;
    float areaCost;

    uint32_t neighbors[3]; // 每条边一个
};

struct NavMeshData {
    std::vector<glm::vec3> vertices;
    std::vector<uint32_t> indices;
    std::vector<NavMeshTriangle> triangles;
    std::vector<int> areas;
    glm::vec3 boundsMin;
    glm::vec3 boundsMax;

    // 空间加速结构
    std::vector<std::vector<uint32_t>> spatialGrid;
    float cellSize;
    int gridWidth, gridHeight, gridDepth;
};

struct EdgeKey {
    glm::vec3 a;
    glm::vec3 b;

    bool operator==(const EdgeKey& other) const {
        return (glm::all(glm::epsilonEqual(a, other.a, 1e-4f)) &&
            glm::all(glm::epsilonEqual(b, other.b, 1e-4f))) ||
            (glm::all(glm::epsilonEqual(a, other.b, 1e-4f)) &&
                glm::all(glm::epsilonEqual(b, other.a, 1e-4f)));
    }
};

struct EdgeKeyHash {
    size_t operator()(const EdgeKey& e) const {
        auto h = [](const glm::vec3& v) {
            return std::hash<int>()(int(v.x * 1000)) ^
                std::hash<int>()(int(v.y * 1000)) ^
                std::hash<int>()(int(v.z * 1000));
            };
        return h(e.a) ^ h(e.b);
    }
};

class NavMeshLoader {
public:
    bool LoadFromFile(const std::string& filepath);
    const NavMeshData& GetData() const { return data_; }

private:
    NavMeshData data_;

    void BuildSpatialGrid();
    //void CalculateTriangleNormals();
    void BuildTriangleAdjacency();

    // 辅助函数
    glm::vec3 CalculateTriangleNormal(const glm::vec3& v0,
        const glm::vec3& v1,
        const glm::vec3& v2);
};
