#pragma once
#include "Session.h"
#include <unordered_map>
#include <atomic>
#include <thread>
#include <functional>
#include <shared_mutex>
#include "ThreadPool.h"

class NetworkManager {
public:
    using SessionHandler = std::function<void(Session::Pointer)>;
    using PacketHandler = std::function<void(Session::Pointer, uint16_t, const std::vector<char>&)>;

    NetworkManager(uint16_t port, size_t thread_pool_size = 4);
    ~NetworkManager();

    bool Start();
    void Stop();
    void Run();

    // 发送数据到指定session
    void SendToSession(uint32_t session_id, const std::vector<char>& data);
    void SendToSession(uint32_t session_id, uint16_t msg_id, const void* data, size_t size);

    // 广播数据
    void Broadcast(const std::vector<char>& data, uint32_t exclude_session_id = 0);
    void BroadcastToGroup(const std::vector<uint32_t>& session_ids, const std::vector<char>& data);

    // 会话管理
    Session::Pointer GetSession(uint32_t session_id);
    void CloseSession(uint32_t session_id);
    std::vector<uint32_t> GetAllSessionIds() const;

    // 设置处理器
    void SetConnectionHandler(SessionHandler handler) { connection_handler_ = handler; }
    void SetDisconnectionHandler(SessionHandler handler) { disconnection_handler_ = handler; }
    void SetPacketHandler(PacketHandler handler) { packet_handler_ = handler; }

    // 统计信息
    uint64_t GetTotalConnections() const { return total_connections_; }
    uint64_t GetActiveConnections() const { return sessions_.size(); }
    uint64_t GetBytesReceived() const;
    uint64_t GetBytesSent() const;

private:
    void DoAccept();
    void HandleAccept(Session::Pointer new_session, const system::error_code& error);
    void OnSessionMessage(Session::Pointer session, const std::vector<char>& data);
    void OnSessionClosed(Session::Pointer session);

    // 心跳检测
    void StartHeartbeatCheck();
    void CheckHeartbeat();

private:
    asio::io_context io_context_;
    asio::ip::tcp::acceptor acceptor_;
    asio::executor_work_guard<asio::io_context::executor_type> work_;

    // 线程池
    std::unique_ptr<ThreadPool> thread_pool_;
    std::vector<std::thread> io_threads_;

    // 会话管理
    std::unordered_map<uint32_t, Session::Pointer> sessions_;
    mutable std::shared_mutex sessions_mutex_;

    // 处理器
    SessionHandler connection_handler_;
    SessionHandler disconnection_handler_;
    PacketHandler packet_handler_;

    // 统计
    std::atomic<uint32_t> next_session_id_{ 1 };
    std::atomic<uint64_t> total_connections_{ 0 };

    // 状态控制
    std::atomic<bool> is_running_{ false };
    std::atomic<bool> is_stopping_{ false };

    // 定时器
    asio::steady_timer heartbeat_timer_;
    const uint32_t HEARTBEAT_INTERVAL = 30000; // 30秒
    const uint32_t HEARTBEAT_TIMEOUT = 90000;  // 90秒超时
};

