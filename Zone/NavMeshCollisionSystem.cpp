#include "NavMeshCollisionSystem.h"
#include <iostream>
#include <algorithm>

void NavMeshMovementSystem::TickMove(
    ServerAgent& agent,
    const glm::vec3& desiredDelta
) {
    const NavMeshTriangle& tri =
        navMesh_.triangles[agent.currentTri];

    glm::vec3 tryPos = agent.position + desiredDelta;
    glm::vec3 projected = ProjectToTrianglePlane(tryPos, tri);

    // 1. 仍在当前三角形
    if (PointInTriangle(projected, tri)) {
        agent.position = projected;
        return;
    }

    // 2. 找穿越边
    int edge = FindCrossedEdge(agent.position, projected, tri);
    if (edge < 0) {
        agent.position = projected;
        return;
    }

    uint32_t nextTri = tri.neighbors[edge];
    if (nextTri == UINT32_MAX) {
        // 撞墙
        agent.position = ClampToEdge(agent.position, tri, edge);
        return;
    }

    const NavMeshTriangle& next =
        navMesh_.triangles[nextTri];

    // 3. 坡度 / 台阶检查
    if (!CheckSlopeAndStep(agent, tri, next)) {
        agent.position = ClampToEdge(agent.position, tri, edge);
        return;
    }

    // 4. 合法切换
    agent.currentTri = nextTri;
    agent.position = ProjectToTrianglePlane(tryPos, next);
}

glm::vec3 NavMeshMovementSystem::ProjectToTrianglePlane(
    const glm::vec3& p,
    const NavMeshTriangle& tri
) const {
    float d = glm::dot(tri.normal, tri.v0);
    float t = (d - glm::dot(tri.normal, p)) /
        glm::dot(tri.normal, tri.normal);
    return p + tri.normal * t;
}

bool NavMeshMovementSystem::PointInTriangle(
    const glm::vec3& p,
    const NavMeshTriangle& tri
) const {
    glm::vec3 c0 = glm::cross(tri.v1 - tri.v0, p - tri.v0);
    glm::vec3 c1 = glm::cross(tri.v2 - tri.v1, p - tri.v1);
    glm::vec3 c2 = glm::cross(tri.v0 - tri.v2, p - tri.v2);

    return glm::dot(c0, tri.normal) >= 0 &&
        glm::dot(c1, tri.normal) >= 0 &&
        glm::dot(c2, tri.normal) >= 0;
}

int NavMeshMovementSystem::FindCrossedEdge(
    const glm::vec3& from,
    const glm::vec3& to,
    const NavMeshTriangle& tri
) const {
    const glm::vec3 verts[3] = {
        tri.v0, tri.v1, tri.v2
    };

    for (int i = 0; i < 3; ++i) {
        const glm::vec3& a = verts[i];
        const glm::vec3& b = verts[(i + 1) % 3];

        glm::vec3 edgeNormal =
            glm::cross(b - a, tri.normal);

        float sideFrom = glm::dot(from - a, edgeNormal);
        float sideTo = glm::dot(to - a, edgeNormal);

        if (sideFrom >= 0 && sideTo < 0)
            return i;
    }

    return -1;
}

glm::vec3 NavMeshMovementSystem::ClampToEdge(
    const glm::vec3& p,
    const NavMeshTriangle& tri,
    int edgeIndex
) const {
    const glm::vec3 verts[3] = {
        tri.v0, tri.v1, tri.v2
    };

    glm::vec3 a = verts[edgeIndex];
    glm::vec3 b = verts[(edgeIndex + 1) % 3];

    glm::vec3 ab = b - a;
    float t = glm::dot(p - a, ab) / glm::dot(ab, ab);
    t = std::clamp(t, 0.0f, 1.0f);

    return a + t * ab;
}

bool NavMeshMovementSystem::CheckSlopeAndStep(
    const ServerAgent& agent,
    const NavMeshTriangle& from,
    const NavMeshTriangle& to
) const {
    float slope =
        acos(glm::dot(from.normal, to.normal));

    if (slope > agent.maxSlopeRadians)
        return false;

    float heightDelta =
        to.v0.y - from.v0.y;

    return fabs(heightDelta) <= agent.maxStepHeight;
}


uint32_t NavMeshMovementSystem::FindInitialTriangle(const glm::vec3& pos, const NavMeshData& navMesh) {
    float minDist = FLT_MAX;
    uint32_t bestTri = UINT32_MAX;

    for (uint32_t i = 0; i < navMesh.triangles.size(); ++i) {
        const auto& tri = navMesh.triangles[i];

        glm::vec3 projected = ProjectToTrianglePlane(pos, tri);

        if (!PointInTriangle(projected, tri))
            continue;

        float dist = std::abs(pos.y - projected.y);
        if (dist < minDist) {
            minDist = dist;
            bestTri = i;
        }
    }

    if (bestTri == UINT32_MAX) {
        std::cout << "Warning: position not on NavMesh, default to 0" << std::endl;
        return 0;
    }

    return bestTri;
}


