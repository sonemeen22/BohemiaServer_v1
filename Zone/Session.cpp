#include "Session.h"
#include <iostream>
#include <spdlog/spdlog.h>

constexpr size_t MAX_PACKET_SIZE = 1024 * 1024; // 1MB
constexpr size_t HEADER_SIZE = sizeof(PacketHeader);

Session::Session(asio::io_context& io_context)
    : socket_(io_context)
    , strand_(io_context)
    , connect_time_(std::chrono::steady_clock::now()) {

    // 设置Socket选项
    asio::ip::tcp::no_delay no_delay_option(true);
    socket_.set_option(no_delay_option);

    asio::socket_base::receive_buffer_size receive_buffer_option(65536);
    socket_.set_option(receive_buffer_option);

    asio::socket_base::send_buffer_size send_buffer_option(65536);
    socket_.set_option(send_buffer_option);

    // 设置keep alive
    asio::socket_base::keep_alive keep_alive_option(true);
    socket_.set_option(keep_alive_option);
}

Session::~Session() {
    Close();
    SPDLOG_INFO("Session {} destroyed", session_id_);
}

void Session::Start() {
    if (is_connected_) return;

    is_connected_ = true;
    is_closing_ = false;

    // 获取远程端点信息
    try {
        auto remote_endpoint = socket_.remote_endpoint();
        SPDLOG_INFO("New connection from {}:{}",
            remote_endpoint.address().to_string(),
            remote_endpoint.port());
    }
    catch (const std::exception& e) {
        SPDLOG_WARN("Cannot get remote endpoint: {}", e.what());
    }

    // 开始异步读取
    AsyncReadHeader();
}

void Session::Close() {
    if (is_closing_ || !is_connected_) return;

    is_closing_ = true;
    is_connected_ = false;

    asio::post(strand_, [this, self = shared_from_this()]() {
        try {
            if (socket_.is_open()) {
                socket_.shutdown(tcp::socket::shutdown_both);
                socket_.close();
                SPDLOG_INFO("Session {} closed", session_id_);
            }
        }
        catch (const std::exception& e) {
            SPDLOG_ERROR("Error closing socket: {}", e.what());
        }

        if (close_handler_) {
            close_handler_();
        }
        });
}

void Session::AsyncReadHeader() {
    if (!is_connected_ || is_closing_) return;

    // 重置header
    memset(&read_header_, 0, sizeof(read_header_));

    asio::async_read(socket_,
        asio::buffer(&read_header_, HEADER_SIZE),
        asio::bind_executor(strand_,
            [this, self = shared_from_this()](const system::error_code& error, size_t bytes_transferred) {
                HandleReadHeader(error, bytes_transferred);
            }));
}

void Session::HandleReadHeader(const system::error_code& error, size_t bytes_transferred) {
    if (error) {
        if (error != asio::error::eof && error != asio::error::operation_aborted) {
            SPDLOG_WARN("Read header error: {}, session: {}", error.message(), session_id_);
        }
        Close();
        return;
    }

    if (bytes_transferred != HEADER_SIZE) {
        SPDLOG_WARN("Incomplete header received: {}/{} bytes", bytes_transferred, HEADER_SIZE);
        Close();
        return;
    }

    // 验证header
    if (!ValidateHeader(read_header_)) {
        SPDLOG_WARN("Invalid packet header from session {}", session_id_);
        Close();
        return;
    }

    // 准备读取body
    //read_buffer_.resize(read_header_.size - HEADER_SIZE);
    AsyncReadBody();
}

bool Session::ValidateHeader(const PacketHeader& header) {
    // 基本验证
    /*if (header.size < HEADER_SIZE || header.size > MAX_PACKET_SIZE) {
        SPDLOG_WARN("Invalid packet size: {}", header.size);
        return false;
    }*/

    if (header.magic_number != PacketHeader::MAGIC_NUMBER) {
        SPDLOG_WARN("Invalid magic number: 0x{:x}", header.magic_number);
        return false;
    }

    // 时间戳验证（防止重放攻击）
    uint64_t current_time = TimeUtils::GetServerTime();
    if (header.timestamp > current_time + 5000 || // 未来5秒内
        header.timestamp < current_time - 30000) { // 过去30秒内
        SPDLOG_WARN("Invalid timestamp: {}, current: {}", header.timestamp, current_time);
        return false;
    }

    return true;
}

void Session::AsyncReadBody() {
    if (!is_connected_ || is_closing_) return;

    asio::async_read(socket_,
        asio::buffer(read_buffer_),
        asio::bind_executor(strand_,
            [this, self = shared_from_this()](const system::error_code& error, size_t bytes_transferred) {
                HandleReadBody(error, bytes_transferred);
            }));
}

void Session::HandleReadBody(const system::error_code& error, size_t bytes_transferred) {
    if (error) {
        SPDLOG_WARN("Read body error: {}, session: {}", error.message(), session_id_);
        Close();
        return;
    }

    if (bytes_transferred != read_buffer_.size()) {
        SPDLOG_WARN("Incomplete body received: {}/{} bytes",
            bytes_transferred, read_buffer_.size());
        Close();
        return;
    }

    bytes_received_ += bytes_transferred + HEADER_SIZE;

    // 处理数据包
    ProcessPacket();

    // 继续读取下一个包
    AsyncReadHeader();
}

void Session::ProcessPacket() {
    if (message_handler_) {
        // 复制数据并传递给处理器
        std::vector<char> packet_data;
        packet_data.resize(HEADER_SIZE + read_buffer_.size());

        // 复制header
        memcpy(packet_data.data(), &read_header_, HEADER_SIZE);
        // 复制body
        memcpy(packet_data.data() + HEADER_SIZE, read_buffer_.data(), read_buffer_.size());

        message_handler_(shared_from_this(), packet_data);
    }
}

void Session::Send(const std::vector<char>& data) {
    if (!is_connected_ || is_closing_ || data.empty()) return;

    asio::post(strand_, [this, self = shared_from_this(), data]() {
        bool write_in_progress = !write_queue_.empty();
        write_queue_.push(data);

        if (!write_in_progress && !is_writing_) {
            AsyncWrite();
        }
        });
}

void Session::AsyncWrite() {
    if (write_queue_.empty() || is_writing_ || !is_connected_ || is_closing_) return;

    is_writing_ = true;

    const auto& data = write_queue_.front();

    asio::async_write(socket_,
        asio::buffer(data.data(), data.size()),
        asio::bind_executor(strand_,
            [this, self = shared_from_this()](const system::error_code& error, size_t bytes_transferred) {
                HandleWrite(error, bytes_transferred);
            }));
}

void Session::HandleWrite(const system::error_code& error, size_t bytes_transferred) {
    is_writing_ = false;

    if (error) {
        SPDLOG_WARN("Write error: {}, session: {}", error.message(), session_id_);
        Close();
        return;
    }

    if (!write_queue_.empty()) {
        bytes_sent_ += bytes_transferred;
        write_queue_.pop();

        // 继续发送队列中的下一个数据包
        if (!write_queue_.empty()) {
            AsyncWrite();
        }
    }
}
