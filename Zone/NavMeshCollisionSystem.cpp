#include "NavMeshCollisionSystem.h"
#include <iostream>
#include <algorithm>

NavMeshMovementSystem::NavMeshMovementSystem(const NavMeshData& navMeshData)
    : navMesh_(navMeshData)
{

}

void NavMeshMovementSystem::TickMove(
    ServerAgent& agent,
    const glm::vec3& desiredDelta)
{
    // 0. 零位移保护
    if (glm::length2(desiredDelta) < 1e-8f)
    {
        return;
    }

    // 1. 当前三角形索引合法性检查
    if (agent.currentTri >= navMesh_.triangles.size()) {
        std::cerr << "[TickMove] currentTri out of bounds!" << std::endl;
        return;
    }

    const NavMeshTriangle& tri = navMesh_.triangles[agent.currentTri];

    // 2. 投影到当前三角形平面
    glm::vec3 tryPos = agent.position + desiredDelta;
    glm::vec3 projected = ProjectToTrianglePlane(tryPos, tri);

    // 3. 如果仍在当前三角形内，直接移动
    if (PointInTriangle(projected, tri)) {
        agent.position = projected;
        return;
    }

    // 4. 找穿越边
    int edge = FindCrossedEdge2D(agent.position, projected, tri);

    if (edge < 0 || edge >= 3) {
        // 没找到边，夹在三角形内或原地保持
        agent.position = ClosestPointOnTriangle(projected, tri);
        return;
    }

    uint32_t nextTri = tri.neighbors[edge];

    if (nextTri == UINT32_MAX || nextTri >= navMesh_.triangles.size()) {
        // 撞墙，夹在边上
        agent.position = ClampToEdge(projected, tri, edge);
        return;
    }

    const NavMeshTriangle& next = navMesh_.triangles[nextTri];

    // 5. 坡度 / 台阶限制
    if (!CheckSlopeAndStep(agent, tri, next)) {
        agent.position = ClampToEdge(projected, tri, edge);
        return;
    }

    // 6. 合法切换到下一个三角形
    agent.currentTri = nextTri;
    agent.position = ProjectToTrianglePlane(tryPos, next);

    // 防止y方向微小漂移导致穿模
    if (glm::abs(agent.position.y - agent.position.y) < 1e-4f)
    {
        agent.position.y = agent.position.y;
    }
}


int NavMeshMovementSystem::FindCrossedEdge2D(
    const glm::vec3& from,
    const glm::vec3& to,
    const NavMeshTriangle& tri)
{
    // 法线归一化保护
    glm::vec3 n = tri.normal;
    if (glm::length2(n) < 1e-8f) return -1;
    n = glm::normalize(n);

    // 构建局部坐标系
    glm::vec3 u = tri.v1 - tri.v0;
    if (glm::length2(u) < 1e-8f) return -1;
    u = glm::normalize(u);

    glm::vec3 v = glm::cross(n, u);
    if (glm::length2(v) < 1e-8f) return -1;
    v = glm::normalize(v);

    auto toLocal2D = [&](const glm::vec3& p) -> glm::vec2 {
        glm::vec3 d = p - tri.v0;
        return glm::vec2(glm::dot(d, u), glm::dot(d, v));
        };

    glm::vec2 from2D = toLocal2D(from);
    glm::vec2 to2D = toLocal2D(to);

    glm::vec2 verts2D[3] = { toLocal2D(tri.v0), toLocal2D(tri.v1), toLocal2D(tri.v2) };

    for (int i = 0; i < 3; ++i) {
        glm::vec2 a = verts2D[i];
        glm::vec2 b = verts2D[(i + 1) % 3];
        glm::vec2 edge = b - a;
        glm::vec2 normal(-edge.y, edge.x);

        float sideFrom = glm::dot(from2D - a, normal);
        float sideTo = glm::dot(to2D - a, normal);

        if (sideFrom >= 0 && sideTo < 0)
            return i;
    }

    return -1;
}

glm::vec3 NavMeshMovementSystem::ClosestPointOnTriangle(
    const glm::vec3& p,
    const NavMeshTriangle& tri)
{
    const glm::vec3& a = tri.v0;
    const glm::vec3& b = tri.v1;
    const glm::vec3& c = tri.v2;

    // Check if P in vertex region outside A
    glm::vec3 ab = b - a;
    glm::vec3 ac = c - a;
    glm::vec3 ap = p - a;
    float d1 = glm::dot(ab, ap);
    float d2 = glm::dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f)
        return a;

    // Check if P in vertex region outside B
    glm::vec3 bp = p - b;
    float d3 = glm::dot(ab, bp);
    float d4 = glm::dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3)
        return b;

    // Check if P in edge region of AB
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        float v = d1 / (d1 - d3);
        return a + v * ab;
    }

    // Check if P in vertex region outside C
    glm::vec3 cp = p - c;
    float d5 = glm::dot(ab, cp);
    float d6 = glm::dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6)
        return c;

    // Check if P in edge region of AC
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        float w = d2 / (d2 - d6);
        return a + w * ac;
    }

    // Check if P in edge region of BC
    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + w * (c - b);
    }

    // P inside face region. Compute barycentric coordinates
    float denom = 1.0f / (va + vb + vc);
    float v = vb * denom;
    float w = vc * denom;

    return a + ab * v + ac * w;
}


/*void NavMeshMovementSystem::TickMove(
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
}*/

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


