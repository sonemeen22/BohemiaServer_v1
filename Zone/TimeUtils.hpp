// TimeUtils.hpp - 时间工具类
#pragma once
#include <chrono>
#include <cstdint>
#include <atomic>
#include <mutex>
#include <string>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <vector>

namespace Network {

    class TimeUtils {
    public:
        // 获取当前服务器时间（毫秒级）
        static uint64_t GetServerTime() {
            auto now = std::chrono::system_clock::now();
            return std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()).count();
        }

        // 获取高精度时间戳（微秒级）
        static uint64_t GetHighPrecisionTime() {
            auto now = std::chrono::high_resolution_clock::now();
            return std::chrono::duration_cast<std::chrono::microseconds>(
                now.time_since_epoch()).count();
        }

        // 获取单调递增时间（不受系统时间调整影响）
        static uint64_t GetMonotonicTime() {
            static std::chrono::steady_clock::time_point start_time =
                std::chrono::steady_clock::now();

            auto now = std::chrono::steady_clock::now();
            return std::chrono::duration_cast<std::chrono::milliseconds>(
                now - start_time).count();
        }

        // 格式化时间戳为可读字符串
        static std::string FormatTime(uint64_t timestamp, bool include_ms = true) {
            std::time_t time_t_value = timestamp / 1000;
            std::tm* tm_info = std::localtime(&time_t_value);

            std::stringstream ss;
            ss << std::put_time(tm_info, "%Y-%m-%d %H:%M:%S");

            if (include_ms) {
                uint64_t milliseconds = timestamp % 1000;
                ss << "." << std::setw(3) << std::setfill('0') << milliseconds;
            }

            return ss.str();
        }

        // 获取当前格式化时间
        static std::string GetCurrentTimeString(bool include_ms = true) {
            return FormatTime(GetServerTime(), include_ms);
        }

        // 计算时间差（毫秒）
        static int64_t GetTimeDiff(uint64_t timestamp1, uint64_t timestamp2) {
            return static_cast<int64_t>(timestamp1) - static_cast<int64_t>(timestamp2);
        }

        // 检查时间是否在有效范围内
        static bool IsTimeValid(uint64_t timestamp, uint64_t tolerance_ms = 30000) {
            uint64_t current_time = GetServerTime();
            return std::abs(static_cast<int64_t>(current_time - timestamp)) <= tolerance_ms;
        }

        // 线程安全的时间戳生成器（用于包ID）
        class TimestampGenerator {
        private:
            std::atomic<uint64_t> last_timestamp_{ 0 };
            std::mutex mutex_;

        public:
            uint64_t Generate() {
                uint64_t current = GetServerTime();
                uint64_t last = last_timestamp_.load(std::memory_order_acquire);

                // 如果当前时间小于等于上次时间，增加1毫秒
                if (current <= last) {
                    std::lock_guard lock(mutex_);
                    last = last_timestamp_.load(std::memory_order_relaxed);
                    if (current <= last) {
                        current = last + 1;
                    }
                    last_timestamp_.store(current, std::memory_order_release);
                }
                else {
                    last_timestamp_.store(current, std::memory_order_release);
                }

                return current;
            }

            // 生成递增的ID（结合时间戳和计数器）
            uint64_t GenerateUniqueId() {
                static thread_local uint16_t counter = 0;
                uint64_t timestamp = Generate() << 16; // 左移16位，给计数器留空间
                return timestamp | (counter++ & 0xFFFF);
            }
        };
    };

    // 服务器时间管理器（支持时间同步和调整）
    class ServerTimeManager {
    private:
        std::atomic<uint64_t> server_start_time_{ 0 };
        std::atomic<int64_t> time_offset_{ 0 }; // 客户端时间偏移
        std::atomic<uint64_t> last_sync_time_{ 0 };
        mutable std::mutex sync_mutex_;

        // 时间同步历史记录
        struct SyncRecord {
            uint64_t client_time;
            uint64_t server_time;
            uint64_t rtt; // 往返时间
        };
        std::vector<SyncRecord> sync_history_;

    public:
        ServerTimeManager() {
            server_start_time_ = TimeUtils::GetServerTime();
        }

        // 获取校准后的服务器时间
        uint64_t GetAdjustedTime() const {
            return TimeUtils::GetServerTime() + time_offset_.load(std::memory_order_acquire);
        }

        // 处理客户端时间同步请求
        struct SyncRequest {
            uint64_t client_send_time;
            uint64_t client_receive_time; // 可选
        };

        struct SyncResponse {
            uint64_t client_send_time;
            uint64_t server_receive_time;
            uint64_t server_send_time;
            int64_t time_offset;
            uint64_t rtt;
        };

        SyncResponse ProcessSyncRequest(const SyncRequest& request) {
            uint64_t server_receive_time = TimeUtils::GetServerTime();
            uint64_t server_send_time = TimeUtils::GetServerTime();

            SyncResponse response;
            response.client_send_time = request.client_send_time;
            response.server_receive_time = server_receive_time;
            response.server_send_time = server_send_time;

            // 计算RTT和偏移（简单版本）
            if (request.client_receive_time > 0) {
                response.rtt = (server_receive_time - request.client_send_time) +
                    (request.client_receive_time - server_send_time);
                response.time_offset = ((server_receive_time - request.client_send_time) -
                    (request.client_receive_time - server_send_time)) / 2;
            }
            else {
                response.rtt = server_receive_time - request.client_send_time;
                response.time_offset = response.rtt / 2; // 假设对称延迟
            }

            // 记录同步历史
            {
                std::lock_guard lock(sync_mutex_);
                sync_history_.push_back({
                    request.client_send_time,
                    server_receive_time,
                    response.rtt
                    });

                // 保持历史记录数量
                if (sync_history_.size() > 100) {
                    sync_history_.erase(sync_history_.begin());
                }
            }

            last_sync_time_ = TimeUtils::GetServerTime();

            return response;
        }

        // 基于历史记录计算平均偏移
        int64_t CalculateAverageOffset() const {
            std::lock_guard lock(sync_mutex_);
            if (sync_history_.empty()) {
                return 0;
            }

            int64_t total_offset = 0;
            for (const auto& record : sync_history_) {
                total_offset += static_cast<int64_t>(record.server_time - record.client_time);
            }

            return total_offset / static_cast<int64_t>(sync_history_.size());
        }

        // 获取服务器运行时间
        uint64_t GetUptime() const {
            return TimeUtils::GetServerTime() - server_start_time_.load(std::memory_order_acquire);
        }

        // 格式化运行时间
        std::string GetFormattedUptime() const {
            uint64_t uptime_ms = GetUptime();
            uint64_t seconds = uptime_ms / 1000;
            uint64_t minutes = seconds / 60;
            uint64_t hours = minutes / 60;
            uint64_t days = hours / 24;

            std::stringstream ss;
            if (days > 0) ss << days << "d ";
            if (hours % 24 > 0) ss << hours % 24 << "h ";
            if (minutes % 60 > 0) ss << minutes % 60 << "m ";
            ss << seconds % 60 << "s";

            return ss.str();
        }
    };

}

