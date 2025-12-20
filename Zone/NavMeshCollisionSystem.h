#include "NavMeshLoader.h"

struct MoveInput {
    glm::vec3 desiredDelta;   // 客户端预测的位移
    float deltaTime;
};

struct ServerAgent {
    glm::vec3 position;
    uint32_t currentTri;

    float maxSlopeRadians = glm::radians(45.0f);
    float maxStepHeight = 0.6f;
};

class NavMeshMovementSystem {
public:
    NavMeshMovementSystem(const NavMeshData& navMesh);

    void TickMove(ServerAgent& agent, const glm::vec3& desiredDelta);

    uint32_t FindInitialTriangle(const glm::vec3& pos, const NavMeshData& navMesh);

    const NavMeshData& navMesh_;

    glm::vec3 ProjectToTrianglePlane(
        const glm::vec3& p,
        const NavMeshTriangle& tri
    ) const;

    bool PointInTriangle(
        const glm::vec3& p,
        const NavMeshTriangle& tri
    ) const;

    int FindCrossedEdge(
        const glm::vec3& from,
        const glm::vec3& to,
        const NavMeshTriangle& tri
    ) const;

    glm::vec3 ClampToEdge(
        const glm::vec3& p,
        const NavMeshTriangle& tri,
        int edgeIndex
    ) const;

    bool CheckSlopeAndStep(
        const ServerAgent& agent,
        const NavMeshTriangle& from,
        const NavMeshTriangle& to
    ) const;

    glm::vec3 ClosestPointOnTriangle(
        const glm::vec3& p,
        const NavMeshTriangle& tri);

    int FindCrossedEdge2D(
        const glm::vec3& from,
        const glm::vec3& to,
        const NavMeshTriangle& tri);
};
