// NetworkManager.cpp
#include "NetworkManager.h"
#include <spdlog/spdlog.h>
#include <chrono>

NetworkManager::NetworkManager(uint16_t port, size_t thread_pool_size)
    : io_context_()
    , acceptor_(io_context_, tcp::endpoint(tcp::v4(), port))
    , work_(asio::make_work_guard(io_context_))
    , heartbeat_timer_(io_context_) {

    thread_pool_ = std::make_unique<ThreadPool>(thread_pool_size);

    // 设置acceptor选项
    acceptor_.set_option(asio::ip::tcp::acceptor::reuse_address(true));

    SPDLOG_INFO("NetworkManager initialized on port {}", port);
}

NetworkManager::~NetworkManager() {
    Stop();
    SPDLOG_INFO("NetworkManager destroyed");
}

bool NetworkManager::Start() {
    if (is_running_) {
        SPDLOG_WARN("NetworkManager already running");
        return false;
    }

    is_running_ = true;
    is_stopping_ = false;

    // 启动IO线程
    size_t hardware_concurrency = std::thread::hardware_concurrency();
    size_t io_thread_count = std::max(static_cast<size_t>(2), hardware_concurrency / 2);

    SPDLOG_INFO("Starting {} IO threads", io_thread_count);

    for (size_t i = 0; i < io_thread_count; ++i) {
        io_threads_.emplace_back([this]() {
            SPDLOG_DEBUG("IO thread started");
            try {
                io_context_.run();
                SPDLOG_DEBUG("IO thread stopped");
            }
            catch (const std::exception& e) {
                SPDLOG_ERROR("IO thread exception: {}", e.what());
            }
            });
    }

    // 开始接受连接
    DoAccept();

    // 启动心跳检测
    StartHeartbeatCheck();

    SPDLOG_INFO("NetworkManager started successfully");
    return true;
}

void NetworkManager::Stop() {
    if (!is_running_ || is_stopping_) return;

    is_stopping_ = true;
    SPDLOG_INFO("Stopping NetworkManager...");

    // 停止工作对象
    work_.reset();

    // 关闭所有会话
    {
        std::unique_lock lock(sessions_mutex_);
        for (auto& [session_id, session] : sessions_) {
            if (session && session->IsConnected()) {
                session->Close();
            }
        }
        sessions_.clear();
    }

    // 停止定时器
    heartbeat_timer_.cancel();

    // 停止IO上下文
    io_context_.stop();

    // 等待线程结束
    for (auto& thread : io_threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    io_threads_.clear();

    // 等待线程池任务完成
    if (thread_pool_) {
        thread_pool_->Stop();
    }

    is_running_ = false;
    SPDLOG_INFO("NetworkManager stopped");
}

void NetworkManager::Run() {
    if (!is_running_) {
        Start();
    }

    // 主线程可以执行其他任务
    while (is_running_ && !is_stopping_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void NetworkManager::DoAccept() {
    auto new_session = std::make_shared<Session>(io_context_);

    acceptor_.async_accept(new_session->Socket(),
        [this, new_session](const system::error_code& error) {
            HandleAccept(new_session, error);
        });
}

void NetworkManager::HandleAccept(Session::Pointer new_session, const system::error_code& error) {
    if (error) {
        if (error != asio::error::operation_aborted) {
            SPDLOG_ERROR("Accept error: {}", error.message());
        }
        return;
    }

    // 生成session id
    uint32_t session_id = next_session_id_++;
    new_session->SetSessionId(session_id);

    // 设置处理器
    new_session->SetMessageHandler(
        std::bind(&NetworkManager::OnSessionMessage, this,
            std::placeholders::_1, std::placeholders::_2));

    new_session->SetCloseHandler(
        std::bind(&NetworkManager::OnSessionClosed, this, new_session));

    // 保存会话
    {
        std::unique_lock lock(sessions_mutex_);
        sessions_[session_id] = new_session;
    }

    total_connections_++;

    // 启动会话
    new_session->Start();

    // 通知连接处理器
    if (connection_handler_) {
        connection_handler_(new_session);
    }

    SPDLOG_INFO("New session accepted: {}, total: {}",
        session_id, GetActiveConnections());

    // 继续接受新连接
    if (!is_stopping_) {
        DoAccept();
    }
}

void NetworkManager::OnSessionMessage(Session::Pointer session, const std::vector<char>& data) {
    if (!session || data.size() < sizeof(PacketHeader)) {
        SPDLOG_WARN("Invalid message received");
        return;
    }

    // 解析packet header
    const PacketHeader* header = reinterpret_cast<const PacketHeader*>(data.data());

    // 提取消息体
    std::vector<char> body(data.begin() + sizeof(PacketHeader), data.end());

    // 使用线程池处理消息
    thread_pool_->Submit([this, session, msg_id = header->packet_id, body]() {
        if (packet_handler_) {
            packet_handler_(session, msg_id, body);
        }
        });
}

void NetworkManager::OnSessionClosed(Session::Pointer session) {
    if (!session) return;

    uint32_t session_id = session->GetSessionId();

    // 从会话表中移除
    {
        std::unique_lock lock(sessions_mutex_);
        sessions_.erase(session_id);
    }

    // 通知断开连接处理器
    if (disconnection_handler_) {
        disconnection_handler_(session);
    }

    SPDLOG_INFO("Session {} closed, active: {}",
        session_id, GetActiveConnections());
}

void NetworkManager::SendToSession(uint32_t session_id, const std::vector<char>& data) {
    Session::Pointer session = GetSession(session_id);
    if (session && session->IsConnected()) {
        session->Send(data);
    }
}

void NetworkManager::SendToSession(uint32_t session_id, uint16_t msg_id, const void* data, size_t size) {
    // 构造完整的数据包
    PacketHeader header;
    header.magic_number = PacketHeader::MAGIC_NUMBER;
    header.packet_id = msg_id;
    //header.size = sizeof(PacketHeader) + size;
    header.timestamp = TimeUtils::GetServerTime();
    header.session_id = session_id;
    header.checksum = 0; // 实际应用中应该计算校验和

    std::vector<char> packet(sizeof(PacketHeader) + size);
    memcpy(packet.data(), &header, sizeof(PacketHeader));
    if (size > 0 && data != nullptr) {
        memcpy(packet.data() + sizeof(PacketHeader), data, size);
    }

    SendToSession(session_id, packet);
}

void NetworkManager::Broadcast(const std::vector<char>& data, uint32_t exclude_session_id) {
    std::shared_lock lock(sessions_mutex_);

    for (const auto& [session_id, session] : sessions_) {
        if (session_id != exclude_session_id && session && session->IsConnected()) {
            session->Send(data);
        }
    }
}

void NetworkManager::BroadcastToGroup(const std::vector<uint32_t>& session_ids, const std::vector<char>& data) {
    std::shared_lock lock(sessions_mutex_);

    for (uint32_t session_id : session_ids) {
        auto it = sessions_.find(session_id);
        if (it != sessions_.end() && it->second && it->second->IsConnected()) {
            it->second->Send(data);
        }
    }
}

Session::Pointer NetworkManager::GetSession(uint32_t session_id) {
    std::shared_lock lock(sessions_mutex_);
    auto it = sessions_.find(session_id);
    return (it != sessions_.end()) ? it->second : nullptr;
}

void NetworkManager::CloseSession(uint32_t session_id) {
    Session::Pointer session = GetSession(session_id);
    if (session) {
        session->Close();
    }
}

std::vector<uint32_t> NetworkManager::GetAllSessionIds() const {
    std::shared_lock lock(sessions_mutex_);
    std::vector<uint32_t> ids;
    ids.reserve(sessions_.size());

    for (const auto& [session_id, _] : sessions_) {
        ids.push_back(session_id);
    }

    return ids;
}

uint64_t NetworkManager::GetBytesReceived() const {
    std::shared_lock lock(sessions_mutex_);
    uint64_t total = 0;

    for (const auto& [_, session] : sessions_) {
        if (session) {
            // 这里需要Session类提供GetBytesReceived方法
            // total += session->GetBytesReceived();
        }
    }

    return total;
}

uint64_t NetworkManager::GetBytesSent() const {
    std::shared_lock lock(sessions_mutex_);
    uint64_t total = 0;

    for (const auto& [_, session] : sessions_) {
        if (session) {
            // 这里需要Session类提供GetBytesSent方法
            // total += session->GetBytesSent();
        }
    }

    return total;
}

void NetworkManager::StartHeartbeatCheck() {
    heartbeat_timer_.expires_after(std::chrono::milliseconds(HEARTBEAT_INTERVAL));
    heartbeat_timer_.async_wait([this](const system::error_code& error) {
        if (!error && !is_stopping_) {
            CheckHeartbeat();
            StartHeartbeatCheck(); // 重新设置定时器
        }
        });
}

void NetworkManager::CheckHeartbeat() {
    std::shared_lock lock(sessions_mutex_);
    uint64_t current_time = TimeUtils::GetServerTime();

    for (const auto& [session_id, session] : sessions_) {
        if (session) {
            // 检查最后活动时间
            // uint64_t last_active = session->GetLastActiveTime();
            // if (current_time - last_active > HEARTBEAT_TIMEOUT) {
            //     SPDLOG_WARN("Session {} heartbeat timeout, closing", session_id);
            //     CloseSession(session_id);
            // }
        }
    }
}

