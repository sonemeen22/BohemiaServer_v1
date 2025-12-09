// Session.hpp
#pragma once
#include <boost/asio.hpp>
#include <functional>
#include <memory>
#include <queue>
#include <mutex>
#include <atomic>
#include <boost/system/error_code.hpp> 
#include "Protocol.hpp"
#include "TimeUtils.hpp"

using namespace boost::asio::ip;
using namespace boost;
using namespace Network;

class Session : public std::enable_shared_from_this<Session> {
public:
    using Pointer = std::shared_ptr<Session>;
    using MessageHandler = std::function<void(Pointer, const std::vector<char>&)>;

    Session(asio::io_context& io_context);
    ~Session();

    tcp::socket& Socket() { return socket_; }

    void Start();
    void Close();
    void Send(const std::vector<char>& data);
    void SetMessageHandler(MessageHandler handler) { message_handler_ = handler; }
    void SetCloseHandler(std::function<void()> handler) { close_handler_ = handler; }

    uint32_t GetSessionId() const { return session_id_; }
    void SetSessionId(uint32_t id) { session_id_ = id; }
    bool IsConnected() const { return is_connected_; }

private:
    void AsyncReadHeader();
    void AsyncReadBody();
    void AsyncWrite();
    void HandleReadHeader(const system::error_code& error, size_t bytes_transferred);
    void HandleReadBody(const system::error_code& error, size_t bytes_transferred);
    void HandleWrite(const system::error_code& error, size_t bytes_transferred);

    // 数据包处理
    bool ValidateHeader(const PacketHeader& header);
    void ProcessPacket();

private:
    tcp::socket socket_;
    asio::io_context::strand strand_;

    // 接收缓冲区
    PacketHeader read_header_;
    std::vector<char> read_buffer_;

    // 发送队列
    std::queue<std::vector<char>> write_queue_;
    std::mutex write_mutex_;
    std::atomic<bool> is_writing_{ false };

    // 状态管理
    std::atomic<bool> is_connected_{ false };
    std::atomic<bool> is_closing_{ false };
    uint32_t session_id_{ 0 };

    // 处理器
    MessageHandler message_handler_;
    std::function<void()> close_handler_;

    // 统计信息
    std::atomic<uint64_t> bytes_received_{ 0 };
    std::atomic<uint64_t> bytes_sent_{ 0 };
    std::chrono::steady_clock::time_point connect_time_;
};
