// NavMeshLoader.h
#pragma once
#include <vector>
#include <string>
#include "glm/glm.hpp"
#include <unordered_map>

struct NavMeshTriangle {
    glm::vec3 v0, v1, v2;
    glm::vec3 normal;
    int area;
    float areaCost;
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

class NavMeshLoader {
public:
    bool LoadFromFile(const std::string& filepath);
    const NavMeshData& GetData() const { return data_; }

private:
    NavMeshData data_;

    void BuildSpatialGrid();
    //void CalculateTriangleNormals();

    // 辅助函数
    glm::vec3 CalculateTriangleNormal(const glm::vec3& v0,
        const glm::vec3& v1,
        const glm::vec3& v2);
};
