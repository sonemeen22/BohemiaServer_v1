#include "NavMeshLoader.h"
#include "NavMeshCollisionSystem.h"
#include <iostream>

struct Player
{
    glm::vec3 position;
    float radius;
    float speed;
    float height;
};

struct MovementInput
{
    glm::vec3 position;
    float moveX;
    float moveZ;
};

// GameServer.cpp - 集成NavMesh到游戏服务器
class GameServer {
private:
    NavMeshLoader navMeshLoader_;
    std::shared_ptr<NavMeshCollisionSystem> collisionSystem_;
    std::shared_ptr<NavMeshPathFinder> pathFinder_;

public:
    bool Initialize() {
        // 1. 加载NavMesh数据
        if (!navMeshLoader_.LoadFromFile("Data/navmesh_data.bin")) {
            std::cerr << "Failed to load NavMesh data" << std::endl;
            return false;
        }

        // 2. 初始化系统
        const auto& navMeshData = navMeshLoader_.GetData();
        collisionSystem_ = std::make_shared<NavMeshCollisionSystem>(navMeshData);
        pathFinder_ = std::make_shared<NavMeshPathFinder>(navMeshData);

        return true;
    }

    void ProcessPlayerMovement(Player& player, const MovementInput& input) {
        glm::vec3 desiredPosition = player.position;

        // 应用移动输入
        desiredPosition.x += input.moveX * player.speed;
        desiredPosition.z += input.moveZ * player.speed;

        // 碰撞检测
        CollisionInfo collision = collisionSystem_->CheckCharacterCollision(
            desiredPosition, player.radius, player.height);

        if (collision.hasCollision) {
            // 处理碰撞响应
            glm::vec3 response = collision.collisionNormal * collision.penetrationDepth;
            desiredPosition += response;

            // 再次验证位置
            desiredPosition = collisionSystem_->GetWalkablePosition(
                desiredPosition, player.radius);
        }

        // 验证移动是否合法
        if (IsMovementValid(player.position, desiredPosition, player.radius)) {
            player.position = desiredPosition;
        }
        else {
            // 发送位置纠正
            //SendPositionCorrection(player);
        }
    }

    std::vector<glm::vec3> CalculatePath(const glm::vec3& start,
        const glm::vec3& end,
        float agentRadius) {
        // 获取可行走起点和终点
        glm::vec3 walkableStart = collisionSystem_->GetWalkablePosition(start, agentRadius);
        glm::vec3 walkableEnd = collisionSystem_->GetWalkablePosition(end, agentRadius);

        // 计算路径
        return pathFinder_->FindPath(walkableStart, walkableEnd);
    }

private:
    bool IsMovementValid(const glm::vec3& from, const glm::vec3& to, float radius) {
        // 检查移动路径是否可行走
        glm::vec3 direction = to - from;
        float distance = glm::length(direction);

        if (distance > 0) {
            direction = direction / distance;

            // 沿路径采样检查
            int samples = static_cast<int>(distance / radius) + 1;
            float step = distance / samples;

            for (int i = 1; i <= samples; ++i) {
                glm::vec3 samplePoint = from + direction * (step * i);

                if (!pathFinder_->IsPointWalkable(samplePoint, radius)) {
                    return false;
                }
            }
        }

        return true;
    }
};
