// NavMeshLoader.cpp
#include "NavMeshLoader.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cmath>

bool NavMeshLoader::LoadFromFile(const std::string& filepath) {
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "无法打开NavMesh文件: " << filepath << std::endl;
        return false;
    }

    // 读取顶点
    uint32_t vertexCount;
    file.read(reinterpret_cast<char*>(&vertexCount), sizeof(vertexCount));
    data_.vertices.resize(vertexCount);

    for (auto& vertex : data_.vertices) {
        file.read(reinterpret_cast<char*>(&vertex.x), sizeof(float));
        file.read(reinterpret_cast<char*>(&vertex.y), sizeof(float));
        file.read(reinterpret_cast<char*>(&vertex.z), sizeof(float));
    }

    // 读取索引
    uint32_t indexCount;
    file.read(reinterpret_cast<char*>(&indexCount), sizeof(indexCount));
    data_.indices.resize(indexCount);
    file.read(reinterpret_cast<char*>(data_.indices.data()),
        indexCount * sizeof(uint32_t));

    // 读取区域数据
    uint32_t areaCount;
    file.read(reinterpret_cast<char*>(&areaCount), sizeof(areaCount));
    data_.areas.resize(areaCount);
    file.read(reinterpret_cast<char*>(data_.areas.data()),
        areaCount * sizeof(int));

    // 读取边界
    file.read(reinterpret_cast<char*>(&data_.boundsMin.x), sizeof(float));
    file.read(reinterpret_cast<char*>(&data_.boundsMin.y), sizeof(float));
    file.read(reinterpret_cast<char*>(&data_.boundsMin.z), sizeof(float));
    file.read(reinterpret_cast<char*>(&data_.boundsMax.x), sizeof(float));
    file.read(reinterpret_cast<char*>(&data_.boundsMax.y), sizeof(float));
    file.read(reinterpret_cast<char*>(&data_.boundsMax.z), sizeof(float));

    file.close();

    // 构建三角形数据
    size_t triangleCount = data_.indices.size() / 3;
    data_.triangles.reserve(triangleCount);

    for (size_t i = 0; i < triangleCount; ++i) {
        uint32_t idx0 = data_.indices[i * 3];
        uint32_t idx1 = data_.indices[i * 3 + 1];
        uint32_t idx2 = data_.indices[i * 3 + 2];

        NavMeshTriangle triangle;
        triangle.v0 = data_.vertices[idx0];
        triangle.v1 = data_.vertices[idx1];
        triangle.v2 = data_.vertices[idx2];
        triangle.area = (i < data_.areas.size()) ? data_.areas[i] : 0;
        triangle.normal = CalculateTriangleNormal(triangle.v0, triangle.v1, triangle.v2);

        // 根据区域类型设置成本
        switch (triangle.area) {
        case 0: triangle.areaCost = 1.0f; break;  // Walkable
        case 1: triangle.areaCost = 2.0f; break;  // Not Walkable
        case 2: triangle.areaCost = 5.0f; break;  // Jump
        case 3: triangle.areaCost = 3.0f; break;  // Water
        default: triangle.areaCost = 10.0f; break;
        }

        data_.triangles.push_back(triangle);
    }

    // 构建空间加速结构
    BuildSpatialGrid();

    std::cout << "NavMesh加载完成: " << data_.vertices.size() << " 顶点, "
        << triangleCount << " 三角形" << std::endl;

    return true;
}

void NavMeshLoader::BuildSpatialGrid() {
    // 计算网格参数
    glm::vec3 boundsSize = data_.boundsMax - data_.boundsMin;
    data_.cellSize = std::max({ boundsSize.x, boundsSize.y, boundsSize.z }) / 50.0f;

    data_.gridWidth = static_cast<int>(ceil(boundsSize.x / data_.cellSize));
    data_.gridHeight = static_cast<int>(ceil(boundsSize.y / data_.cellSize));
    data_.gridDepth = static_cast<int>(ceil(boundsSize.z / data_.cellSize));

    size_t gridSize = data_.gridWidth * data_.gridHeight * data_.gridDepth;
    data_.spatialGrid.resize(gridSize);

    // 将三角形分配到网格单元
    for (uint32_t triIdx = 0; triIdx < data_.triangles.size(); ++triIdx) {
        const auto& tri = data_.triangles[triIdx];

        // 计算三角形AABB
        glm::vec3 triMin = {
            std::min({tri.v0.x, tri.v1.x, tri.v2.x}),
            std::min({tri.v0.y, tri.v1.y, tri.v2.y}),
            std::min({tri.v0.z, tri.v1.z, tri.v2.z})
        };

        glm::vec3 triMax = {
            std::max({tri.v0.x, tri.v1.x, tri.v2.x}),
            std::max({tri.v0.y, tri.v1.y, tri.v2.z}),
            std::max({tri.v0.z, tri.v1.z, tri.v2.z})
        };

        // 转换到网格坐标
        int minX = static_cast<int>((triMin.x - data_.boundsMin.x) / data_.cellSize);
        int minY = static_cast<int>((triMin.y - data_.boundsMin.y) / data_.cellSize);
        int minZ = static_cast<int>((triMin.z - data_.boundsMin.z) / data_.cellSize);

        int maxX = static_cast<int>((triMax.x - data_.boundsMin.x) / data_.cellSize);
        int maxY = static_cast<int>((triMax.y - data_.boundsMin.y) / data_.cellSize);
        int maxZ = static_cast<int>((triMax.z - data_.boundsMin.z) / data_.cellSize);

        // 限制边界
        minX = std::max(0, minX); maxX = std::min(data_.gridWidth - 1, maxX);
        minY = std::max(0, minY); maxY = std::min(data_.gridHeight - 1, maxY);
        minZ = std::max(0, minZ); maxZ = std::min(data_.gridDepth - 1, maxZ);

        // 添加到对应网格单元
        for (int x = minX; x <= maxX; ++x) {
            for (int y = minY; y <= maxY; ++y) {
                for (int z = minZ; z <= maxZ; ++z) {
                    int gridIdx = x + y * data_.gridWidth + z * data_.gridWidth * data_.gridHeight;
                    data_.spatialGrid[gridIdx].push_back(triIdx);
                }
            }
        }
    }
}

glm::vec3 NavMeshLoader::CalculateTriangleNormal(const glm::vec3& v0,
    const glm::vec3& v1,
    const glm::vec3& v2) {
    glm::vec3 edge1 = v1 - v0;
    glm::vec3 edge2 = v2 - v0;

    glm::vec3 normal = {
        edge1.y * edge2.z - edge1.z * edge2.y,
        edge1.z * edge2.x - edge1.x * edge2.z,
        edge1.x * edge2.y - edge1.y * edge2.x
    };

    float length = sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
    if (length > 0) {
        normal.x /= length;
        normal.y /= length;
        normal.z /= length;
    }

    return normal;
}
