#include "NavMeshLoader.h"
#include "NavMeshCollisionSystem.h"
#include <iostream>
#include <ctime>
#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include "proto/mmo.pb.h"

using boost::asio::ip::tcp;

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
    const int port_ = 1008;

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

    /*void Run()
    {
        try
        {
            boost::asio::io_context io_context;

            tcp::acceptor acceptor(io_context, tcp::endpoint(tcp::v4(), port_));

            for (;;)
            {
                tcp::socket socket(io_context);
                acceptor.accept(socket);

                std::array<char, 1024> buf;
                boost::system::error_code error;

                MoveRequest move_request;

                std::vector<char> buffer;
                char temp_buffer[1024];

                size_t len = socket.receive(boost::asio::buffer(buf), 0, error);

                if (error == boost::asio::error::eof)
                {
                    break; // Connection closed cleanly by peer.
                }
                else if (error)
                {
                    throw boost::system::system_error(error); // Some other error.
                }

                //std::cout.write(buf.data(), len);
                std::string data(buf.data());
                
                move_request.ParseFromString(data);
                std::cout << "MoveRequest player_id:" << move_request.player_id();
                std::cout << ",position_x:" << move_request.position_x();
                std::cout << ",position_y:" << move_request.position_y();
                std::cout << ",position_z:" << move_request.position_z() << std::endl;

                MoveBroadcast move_broadcast;
                move_broadcast.set_player_id(move_request.player_id());
                move_broadcast.set_position_x(move_request.position_x());
                move_broadcast.set_position_y(move_request.position_y());
                move_broadcast.set_position_z(move_request.position_z());

                // 序列化 MoveBroadcast 为字符串
                std::string serialized_data;
                if (!move_broadcast.SerializeToString(&serialized_data))
                {
                    std::cerr << "Failed to serialize MoveBroadcast" << std::endl;
                    continue;
                }

                // 发送序列化后的数据
                boost::system::error_code ignored_error;
                boost::asio::write(socket,
                    boost::asio::buffer(serialized_data.data(), serialized_data.size()),
                    ignored_error);
            }
        }
        catch (std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
    }*/

    void Run1()
    {
        try {
            boost::asio::io_context io_context;
            tcp::acceptor acceptor(io_context, tcp::endpoint(tcp::v4(), 8080));

            std::cout << "Server started on port 8080...\n";

            tcp::socket socket(io_context);
            acceptor.accept(socket);
            std::cout << "Client connected.\n";

            handle_client1(std::move(socket));
        }
        catch (std::exception& e) {
            std::cerr << "Exception: " << e.what() << "\n";
        }
    }

    void handle_client1(tcp::socket socket)
    {
        try {
            while (true)
            {
                // 1. 读取长度
                uint32_t net_size;
                boost::asio::read(socket, boost::asio::buffer(&net_size, sizeof(net_size)));

                uint32_t size = ntohl(net_size);
                std::cout << "size:" << size << std::endl;

                // 2. 按长度读取完整消息
                std::vector<char> buffer(size);
                boost::asio::read(socket, boost::asio::buffer(buffer));

                // 3. protobuf 解析
                MoveRequest req;
                if (!req.ParseFromArray(buffer.data(), buffer.size())) {
                    std::cerr << "Failed to parse MoveRequest\n";
                    break;
                }

                std::cout << "Player:" << req.player_id()
                    << " Pos x:" << req.position().x()
                    << " Pos y:" << req.position().y()
                    << " Pos z:" << req.position().z()
                    << " vx:" << req.velocity().x()
                    << " vy:" << req.velocity().y()
                    << " vz:" << req.velocity().z()
                    << " horizontal:" << req.mouse_move().x()
                    << " vertical:" << req.mouse_move().y()

                    << " forward x:" << req.camera().forward().x()
                    << " forward y:" << req.camera().forward().y()
                    << " forward z:" << req.camera().forward().z()

                    << " right x:" << req.camera().right().x()
                    << " right y:" << req.camera().right().y()
                    << " right z:" << req.camera().right().z()

                    << std::endl;

                // 4. 构建响应
                MoveBroadcast broadcast;
                broadcast.set_player_id(req.player_id());
                broadcast.mutable_position()->set_x(req.position().x());
                broadcast.mutable_position()->set_y(req.position().y());
                broadcast.mutable_position()->set_z(req.position().z());

                std::string out;
                broadcast.SerializeToString(&out);

                // 5. 发送（带长度）
                uint32_t out_size = htonl(out.size());
                boost::asio::write(socket, boost::asio::buffer(&out_size, 4));
                boost::asio::write(socket, boost::asio::buffer(out));
            }
        }
        catch (const std::exception& e) {
            std::cout << "Client disconnected: " << e.what() << "\n";
        }
    }

    void Run()
    {
        try
        {
            boost::asio::io_context io_context;
            tcp::acceptor acceptor(io_context, tcp::endpoint(tcp::v4(), port_));

            for (;;) {
                tcp::socket socket(io_context);
                acceptor.accept(socket);
                handle_client(std::move(socket));
                io_context.run();
                // 为每个客户端创建独立线程
                //std::thread(handle_client, std::move(socket)).detach();
            }
        }
        catch (std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
    }

    void handle_client(tcp::socket socket)
    {
        try {
            while (true)
            {
                // 1. 读取长度
                uint32_t net_size;
                boost::asio::read(socket, boost::asio::buffer(&net_size, sizeof(net_size)));

                uint32_t size = ntohl(net_size);

                // 2. 按长度读取完整消息
                std::vector<char> buffer(size);
                boost::asio::read(socket, boost::asio::buffer(buffer));

                // 3. protobuf 解析
                MoveRequest req;
                if (!req.ParseFromArray(buffer.data(), buffer.size())) {
                    std::cerr << "Failed to parse MoveRequest\n";
                    break;
                }


                // 4. 构建响应
                MoveBroadcast broadcast;
                broadcast.set_player_id(req.player_id());

                std::string out;
                broadcast.SerializeToString(&out);

                // 5. 发送（带长度）
                uint32_t out_size = htonl(out.size());
                boost::asio::write(socket, boost::asio::buffer(&out_size, 4));
                boost::asio::write(socket, boost::asio::buffer(out));
            }
        }
        catch (const std::exception& e) {
            std::cout << "Client disconnected: " << e.what() << "\n";
        }
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
