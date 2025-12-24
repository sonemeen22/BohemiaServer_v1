#include "NavMeshLoader.h"
#include "NavMeshCollisionSystem.h"
#include <iostream>
#include <ctime>
#include <iostream>
#include <string>
#include <chrono>
#include <boost/asio.hpp>
#include "proto/mmo.pb.h"
#include "Util.hpp"
#include "MoveSystem.h"

using boost::asio::ip::tcp;

struct MovementInput
{
    glm::vec3 position;
    float moveX;
    float moveZ;
};

struct CameraInfo
{
    glm::vec3 forward;
    glm::vec3 right;
};

struct PlayerInputSnapshot
{
    glm::vec2 mouse;
    CameraInfo camera;
};

constexpr int FPS = 30;
constexpr std::chrono::milliseconds FRAME_TIME(1000 / FPS);
constexpr float kStepTime = 1.0f / FPS;

// GameServer.cpp - 集成NavMesh到游戏服务器
class GameServer {
private:
    NavMeshLoader navMeshLoader_;
    std::shared_ptr<MoveSystem> move_system_;
    const int port_ = 1008;

    glm::vec2 mouse_input_;
    CameraInfo camera_info_;
    glm::vec3 position_;
    glm::vec3 projected_velocity_;
    glm::vec3 normal_vector_;

    float movement_speed_ = 5;

    std::mutex input_mtx_;
    PlayerInputSnapshot latest_input_;
    std::atomic<bool> has_input_{ false };

    std::mutex position_mtx_;

    boost::asio::io_context io_context_;

    Player player_;

    ServerAgent server_agent_;

    glm::vec3 desired_delta_;

public:
    bool Initialize() {
        // 1. 加载NavMesh数据
        if (!navMeshLoader_.LoadFromFile("Data/navmesh_data.bin")) {
            std::cerr << "Failed to load NavMesh data" << std::endl;
            return false;
        }

        // 2. 初始化系统
        const auto& navMeshData = navMeshLoader_.GetData();
        move_system_ = std::make_shared<MoveSystem>();

        normal_vector_.x = 0;
        normal_vector_.y = 1;
        normal_vector_.z = 0;

        position_.x = 0;
        position_.y = 0;
        position_.z = 0;

        player_.position = PxVec3(0, 0, 0);
        player_.halfHeight = 0.95;
        player_.radius = 0.4;
        player_.speed = 5;
        player_.velocity = PxVec3(0, 0, 0);

        return true;
    }

    void Run()
    {
        try {
            std::thread gameThread(&GameServer::GameRun, this);
            gameThread.detach();

            tcp::acceptor acceptor(io_context_, tcp::endpoint(tcp::v4(), 8080));

            std::cout << "Server started on port 8080...\n";

            while (true)
            {
                tcp::socket socket(io_context_);
                acceptor.accept(socket);
                std::cout << "Client connected.\n";

                std::thread t(
                    &GameServer::handle_client1,
                    this,
                    std::move(socket)
                );
                t.detach();
            }
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

                //Util::SetVec3(position_, req.position());
                /*mouse_input_.x = req.mouse_move().x();
                mouse_input_.y = req.mouse_move().y();
                Util::SetVec3(camera_info_.forward, req.camera().forward());
                Util::SetVec3(camera_info_.right, req.camera().right());*/

                PlayerInputSnapshot snapshot;
                snapshot.mouse = {
                    req.mouse_move().x(),
                    req.mouse_move().y()
                };
                Util::SetVec3(snapshot.camera.forward, req.camera().forward());
                Util::SetVec3(snapshot.camera.right, req.camera().right());
                

                {
                    std::lock_guard<std::mutex> lock(input_mtx_);
                    latest_input_ = snapshot;
                    has_input_ = true;
                }

                {
                    // 轻量锁，或 atomic / double buffer
                    std::lock_guard<std::mutex> lock(position_mtx_);

                    // 4. 构建响应
                    MoveBroadcast broadcast;
                    broadcast.set_player_id(req.player_id());
                    broadcast.mutable_position()->set_x(player_.position.x);
                    broadcast.mutable_position()->set_y(player_.position.y);
                    broadcast.mutable_position()->set_z(player_.position.z);

                    std::string out;
                    broadcast.SerializeToString(&out);

                    // 5. 发送（带长度）
                    uint32_t out_size = htonl(out.size());
                    boost::asio::write(socket, boost::asio::buffer(&out_size, 4));
                    boost::asio::write(socket, boost::asio::buffer(out));
                }
            }
        }
        catch (const std::exception& e) {
            std::cout << "Client disconnected: " << e.what() << "\n";
        }
    }

    void GameRun()
    {
        using clock = std::chrono::steady_clock;
        auto nextFrameTime = clock::now();

        while (true)
        {
            nextFrameTime += FRAME_TIME;

            Update();

            std::this_thread::sleep_until(nextFrameTime);
        }
    }

    void Update()
    {
        PlayerInputSnapshot input{};
        bool has_input = false;

        {
            std::lock_guard<std::mutex> lock(input_mtx_);
            if (has_input_) {
                input = latest_input_;
                has_input = true;
            }
            else
            {
                input = latest_input_;
            }
        }

        HandleMovement(input);

        // 服务器权威移动
        {
            std::lock_guard<std::mutex> lock(position_mtx_);
            move_system_->ServerTick(player_, kStepTime);
        }
    }

    void HandleMovement(const PlayerInputSnapshot& input)
    {
        glm::vec3 moveDirection =
            input.camera.forward * input.mouse.y +
            input.camera.right * input.mouse.x;

        if (glm::length(moveDirection) < 0.0001f)
        {
            projected_velocity_ = glm::vec3(0);
            return;
        }

        moveDirection.y = 0;
        moveDirection = glm::normalize(moveDirection) * movement_speed_;

        projected_velocity_ = Util::ProjectOnPlane(moveDirection, normal_vector_);
        
        Util::SetPxVec3(player_.velocity, projected_velocity_);
    }
};
