// Protocol.hpp
#pragma once
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <string>
#include <vector>
#include <chrono>
#include <stdexcept>
#include <algorithm>
#include <functional>
#include <iostream>
#include <iomanip>
#include <optional>
#include <sstream>

// 网络字节序转换（跨平台）
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <arpa/inet.h>
#endif

#include "proto/mmo.pb.h"

namespace Network {

    // 协议版本
    enum class ProtocolVersion : uint16_t {
        V1_0 = 0x0100,
        V1_1 = 0x0101,
        V2_0 = 0x0200
    };

    // 消息类型枚举
    enum class MessageType : uint16_t {
        // 系统消息 (0x0000-0x0FFF)
        HEARTBEAT = 0x0001,
        HANDSHAKE = 0x0002,
        DISCONNECT = 0x0003,
        RECONNECT = 0x0004,

        // 登录/认证 (0x1000-0x10FF)
        LOGIN_REQUEST = 0x1001,
        LOGIN_RESPONSE = 0x1002,
        AUTH_TOKEN = 0x1003,

        // 玩家操作 (0x2000-0x2FFF)
        PLAYER_MOVE = 0x2001,
        PLAYER_ACTION = 0x2002,
        PLAYER_CHAT = 0x2003,
        PLAYER_STATE = 0x2004,

        // 游戏逻辑 (0x3000-0x3FFF)
        ENTITY_SPAWN = 0x3001,
        ENTITY_DESPAWN = 0x3002,
        ENTITY_UPDATE = 0x3003,
        COMBAT = 0x3004,

        // 世界交互 (0x4000-0x4FFF)
        WORLD_QUERY = 0x4001,
        WORLD_UPDATE = 0x4002,

        // 错误码 (0xF000-0xFFFF)
        ERROR_RESPONSE = 0xF001
    };

    // 压缩类型
    enum class CompressionType : uint8_t {
        NONE = 0x00,
        ZLIB = 0x01,
        LZ4 = 0x02,
        SNAPPY = 0x03
    };

    // 加密类型
    enum class EncryptionType : uint8_t {
        NONE = 0x00,
        AES_128 = 0x01,
        AES_256 = 0x02,
        XOR = 0x03  // 仅用于测试
    };

    // 序列化类型
    enum class SerializationType : uint8_t {
        BINARY = 0x00,
        JSON = 0x01,
        PROTOBUF = 0x02,
        FLATBUFFERS = 0x03
    };

    // 移动同步协议
    /*struct MoveRequest {
        uint32_t playerId;
        float positionX;
        float positionY;
        float positionZ;
        float rotationY;
        float velocityX;
        float velocityY;
        float velocityZ;
        uint64_t timestamp;
    };

    struct MoveBroadcast {
        uint32_t playerId;
        float positionX, positionY, positionZ;
        float rotationY;
        uint64_t serverTime;
        uint32_t state; // 移动状态
    };*/

    // 字节序转换辅助函数
    class ByteOrder {
    public:
        // 主机序转网络序
        template<typename T>
        static typename std::enable_if<std::is_integral<T>::value || std::is_enum<T>::value, T>::type
            HostToNetwork(T value) {
            if constexpr (sizeof(T) == 1) {
                return value;
            }
            else if constexpr (sizeof(T) == 2) {
                return htons(static_cast<uint16_t>(value));
            }
            else if constexpr (sizeof(T) == 4) {
                return htonl(static_cast<uint32_t>(value));
            }
            else if constexpr (sizeof(T) == 8) {
                uint64_t host_value = static_cast<uint64_t>(value);
                uint32_t high_part = htonl(static_cast<uint32_t>(host_value >> 32));
                uint32_t low_part = htonl(static_cast<uint32_t>(host_value & 0xFFFFFFFF));
                return static_cast<T>((static_cast<uint64_t>(low_part) << 32) | high_part);
            }
            else {
                static_assert(sizeof(T) <= 8, "Unsupported type size");
                return value;
            }
        }

        // 网络序转主机序
        template<typename T>
        static typename std::enable_if<std::is_integral<T>::value || std::is_enum<T>::value, T>::type
            NetworkToHost(T value) {
            if constexpr (sizeof(T) == 1) {
                return value;
            }
            else if constexpr (sizeof(T) == 2) {
                return ntohs(static_cast<uint16_t>(value));
            }
            else if constexpr (sizeof(T) == 4) {
                return ntohl(static_cast<uint32_t>(value));
            }
            else if constexpr (sizeof(T) == 8) {
                uint64_t net_value = static_cast<uint64_t>(value);
                uint32_t low_part = ntohl(static_cast<uint32_t>(net_value & 0xFFFFFFFF));
                uint32_t high_part = ntohl(static_cast<uint32_t>(net_value >> 32));
                return static_cast<T>((static_cast<uint64_t>(low_part) << 32) | high_part);
            }
            else {
                static_assert(sizeof(T) <= 8, "Unsupported type size");
                return value;
            }
        }

        // 检查系统字节序
        static bool IsLittleEndian() {
            static const uint16_t test = 0x0001;
            return *reinterpret_cast<const uint8_t*>(&test) == 0x01;
        }

        // 交换字节序
        template<typename T>
        static T SwapBytes(T value) {
            T result = 0;
            uint8_t* src = reinterpret_cast<uint8_t*>(&value);
            uint8_t* dst = reinterpret_cast<uint8_t*>(&result);

            for (size_t i = 0; i < sizeof(T); ++i) {
                dst[i] = src[sizeof(T) - 1 - i];
            }

            return result;
        }
    };

    // CRC32校验和计算
    class CRC32 {
    private:
        static const uint32_t CRC32_TABLE[256];
        static bool table_initialized;

        static void InitializeTable() {
            if (table_initialized) return;

            for (uint32_t i = 0; i < 256; ++i) {
                uint32_t c = i;
                for (int j = 0; j < 8; ++j) {
                    c = (c & 1) ? (0xEDB88320 ^ (c >> 1)) : (c >> 1);
                }
                const_cast<uint32_t&>(CRC32_TABLE[i]) = c;
            }

            table_initialized = true;
        }

    public:
        static uint32_t Calculate(const void* data, size_t length, uint32_t crc = 0xFFFFFFFF) {
            InitializeTable();

            const uint8_t* bytes = static_cast<const uint8_t*>(data);
            crc = crc ^ 0xFFFFFFFF;

            for (size_t i = 0; i < length; ++i) {
                crc = CRC32_TABLE[(crc ^ bytes[i]) & 0xFF] ^ (crc >> 8);
            }

            return crc ^ 0xFFFFFFFF;
        }

        static uint32_t Calculate(const std::vector<uint8_t>& data, uint32_t crc = 0xFFFFFFFF) {
            return Calculate(data.data(), data.size(), crc);
        }
    };

    // PacketHeader类定义
#pragma pack(push, 1)  // 紧密打包，无填充
    class PacketHeader {
    public:
        // 构造函数
        PacketHeader() = default;

        explicit PacketHeader(MessageType type)
            : message_type(static_cast<uint16_t>(type)) {
            timestamp = GetCurrentTimestamp();
        }

        // 成员变量
        uint32_t magic_number = MAGIC_NUMBER;    // 魔数 0x4D4D4F52 ("MMOR")
        uint16_t version = PROTOCOL_VERSION;     // 协议版本
        uint16_t message_type = 0;               // 消息类型
        uint32_t payload_length = 0;             // 载荷长度
        uint32_t packet_id = 0;                  // 包ID（用于乱序重组）
        uint64_t timestamp = 0;                  // 时间戳
        uint32_t session_id = 0;                 // 会话ID
        uint16_t flags = 0;                      // 标志位
        uint8_t  compression = 0;                // 压缩类型
        uint8_t  encryption = 0;                 // 加密类型
        uint8_t  serialization = 0;              // 序列化类型
        uint8_t  reserved = 0;                   // 保留字段
        uint32_t checksum = 0;                   // CRC32校验和

        // 常量定义
        static constexpr uint32_t MAGIC_NUMBER = 0x4D4D4F52;  // "MMOR"
        static constexpr uint16_t PROTOCOL_VERSION = 0x0200;  // 2.0版本
        static constexpr size_t HEADER_SIZE = 44;             // 头部大小
        static constexpr uint32_t MAX_PACKET_SIZE = 1024 * 1024; // 最大包大小 1MB

        // 标志位定义
        static constexpr uint16_t FLAG_COMPRESSED = 0x0001;  // 压缩标志
        static constexpr uint16_t FLAG_ENCRYPTED = 0x0002;  // 加密标志
        static constexpr uint16_t FLAG_FRAGMENTED = 0x0004;  // 分片标志
        static constexpr uint16_t FLAG_LAST_FRAGMENT = 0x0008; // 最后一个分片
        static constexpr uint16_t FLAG_RELIABLE = 0x0010;  // 可靠传输
        static constexpr uint16_t FLAG_URGENT = 0x0020;  // 紧急消息
        static constexpr uint16_t FLAG_ACK = 0x0040;  // 确认消息
        static constexpr uint16_t FLAG_NACK = 0x0080;  // 否定确认

        // 工具方法
        static uint64_t GetCurrentTimestamp() {
            auto now = std::chrono::system_clock::now();
            auto duration = now.time_since_epoch();
            return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
        }

        // 标志位操作
        bool IsCompressed() const { return (flags & FLAG_COMPRESSED) != 0; }
        bool IsEncrypted() const { return (flags & FLAG_ENCRYPTED) != 0; }
        bool IsFragmented() const { return (flags & FLAG_FRAGMENTED) != 0; }
        bool IsLastFragment() const { return (flags & FLAG_LAST_FRAGMENT) != 0; }
        bool IsReliable() const { return (flags & FLAG_RELIABLE) != 0; }
        bool IsUrgent() const { return (flags & FLAG_URGENT) != 0; }
        bool IsAck() const { return (flags & FLAG_ACK) != 0; }
        bool IsNack() const { return (flags & FLAG_NACK) != 0; }

        void SetCompressed(bool value) {
            if (value) flags |= FLAG_COMPRESSED;
            else flags &= ~FLAG_COMPRESSED;
        }

        void SetEncrypted(bool value) {
            if (value) flags |= FLAG_ENCRYPTED;
            else flags &= ~FLAG_ENCRYPTED;
        }

        void SetFragmented(bool value) {
            if (value) flags |= FLAG_FRAGMENTED;
            else flags &= ~FLAG_FRAGMENTED;
        }

        void SetLastFragment(bool value) {
            if (value) flags |= FLAG_LAST_FRAGMENT;
            else flags &= ~FLAG_LAST_FRAGMENT;
        }

        void SetReliable(bool value) {
            if (value) flags |= FLAG_RELIABLE;
            else flags &= ~FLAG_RELIABLE;
        }

        void SetUrgent(bool value) {
            if (value) flags |= FLAG_URGENT;
            else flags &= ~FLAG_URGENT;
        }

        void SetAck(bool value) {
            if (value) flags |= FLAG_ACK;
            else flags &= ~FLAG_ACK;
        }

        void SetNack(bool value) {
            if (value) flags |= FLAG_NACK;
            else flags &= ~FLAG_NACK;
        }

        // 验证头部有效性
        bool Validate() const {
            // 检查魔数
            if (magic_number != MAGIC_NUMBER) {
                return false;
            }

            // 检查协议版本（支持1.x和2.x版本）
            uint16_t major_version = (version & 0xFF00) >> 8;
            if (major_version < 1 || major_version > 2) {
                return false;
            }

            // 检查包大小限制
            if (payload_length > MAX_PACKET_SIZE - HEADER_SIZE) {
                return false;
            }

            // 检查时间戳（不能是未来的时间，也不能太旧）
            uint64_t current_time = GetCurrentTimestamp();
            uint64_t time_diff = current_time > timestamp ?
                current_time - timestamp : timestamp - current_time;

            // 允许5分钟的时间误差
            if (time_diff > 5 * 60 * 1000) {
                return false;
            }

            // 检查分片标志一致性
            if (IsFragmented() && payload_length == 0) {
                return false;
            }

            // 检查压缩和加密标志
            if ((IsCompressed() && compression == 0) ||
                (IsEncrypted() && encryption == 0)) {
                return false;
            }

            return true;
        }

        // 计算校验和（不包含checksum字段本身）
        uint32_t CalculateChecksum() const {
            // 创建一个不包含checksum的临时副本
            PacketHeader temp = *this;
            temp.checksum = 0;

            // 计算CRC32
            return CRC32::Calculate(&temp, HEADER_SIZE - sizeof(checksum));
        }

        // 更新校验和
        void UpdateChecksum() {
            checksum = CalculateChecksum();
        }

        // 验证校验和
        bool VerifyChecksum() const {
            return checksum == CalculateChecksum();
        }

        // 序列化为网络字节序
        std::vector<uint8_t> Serialize() const {
            std::vector<uint8_t> buffer(HEADER_SIZE);
            PacketHeader network_header = *this;

            // 转换为网络字节序
            network_header.magic_number = ByteOrder::HostToNetwork(magic_number);
            network_header.version = ByteOrder::HostToNetwork(version);
            network_header.message_type = ByteOrder::HostToNetwork(message_type);
            network_header.payload_length = ByteOrder::HostToNetwork(payload_length);
            network_header.packet_id = ByteOrder::HostToNetwork(packet_id);
            network_header.timestamp = ByteOrder::HostToNetwork(timestamp);
            network_header.session_id = ByteOrder::HostToNetwork(session_id);
            network_header.flags = ByteOrder::HostToNetwork(flags);
            network_header.checksum = ByteOrder::HostToNetwork(checksum);

            // 拷贝到缓冲区
            std::memcpy(buffer.data(), &network_header, HEADER_SIZE);

            return buffer;
        }

        // 从网络字节序反序列化
        static PacketHeader Deserialize(const uint8_t* data, size_t length) {
            if (length < HEADER_SIZE) {
                throw std::runtime_error("Insufficient data for packet header");
            }

            PacketHeader header;
            std::memcpy(&header, data, HEADER_SIZE);

            // 转换为主机字节序
            header.magic_number = ByteOrder::NetworkToHost(header.magic_number);
            header.version = ByteOrder::NetworkToHost(header.version);
            header.message_type = ByteOrder::NetworkToHost(header.message_type);
            header.payload_length = ByteOrder::NetworkToHost(header.payload_length);
            header.packet_id = ByteOrder::NetworkToHost(header.packet_id);
            header.timestamp = ByteOrder::NetworkToHost(header.timestamp);
            header.session_id = ByteOrder::NetworkToHost(header.session_id);
            header.flags = ByteOrder::NetworkToHost(header.flags);
            header.checksum = ByteOrder::NetworkToHost(header.checksum);

            return header;
        }

        // 静态方法：创建特定类型的头部
        static PacketHeader CreateHeartbeat(uint32_t session_id) {
            PacketHeader header(MessageType::HEARTBEAT);
            header.session_id = session_id;
            header.SetReliable(true);
            return header;
        }

        static PacketHeader CreateMove(uint32_t session_id, uint32_t packet_id) {
            PacketHeader header(MessageType::PLAYER_MOVE);
            header.session_id = session_id;
            header.packet_id = packet_id;
            header.SetReliable(false);  // 移动消息通常不需要可靠传输
            header.SetUrgent(true);     // 移动消息需要低延迟
            return header;
        }

        static PacketHeader CreateLoginResponse(uint32_t session_id, bool success) {
            PacketHeader header(success ? MessageType::LOGIN_RESPONSE : MessageType::ERROR_RESPONSE);
            header.session_id = session_id;
            header.SetReliable(true);
            return header;
        }

        // 调试输出
        std::string ToString() const {
            std::stringstream ss;
            ss << "PacketHeader {\n"
                << "  magic_number: 0x" << std::hex << magic_number << std::dec << "\n"
                << "  version: " << ((version >> 8) & 0xFF) << "." << (version & 0xFF) << "\n"
                << "  message_type: 0x" << std::hex << message_type << std::dec
                << " (" << MessageTypeToString(static_cast<MessageType>(message_type)) << ")\n"
                << "  payload_length: " << payload_length << " bytes\n"
                << "  packet_id: " << packet_id << "\n"
                << "  timestamp: " << timestamp << "\n"
                << "  session_id: " << session_id << "\n"
                << "  flags: 0x" << std::hex << flags << std::dec << " ["
                << (IsCompressed() ? "COMPRESSED " : "")
                << (IsEncrypted() ? "ENCRYPTED " : "")
                << (IsFragmented() ? "FRAGMENTED " : "")
                << (IsReliable() ? "RELIABLE " : "")
                << (IsUrgent() ? "URGENT" : "")
                << "]\n"
                << "  compression: " << static_cast<int>(compression) << "\n"
                << "  encryption: " << static_cast<int>(encryption) << "\n"
                << "  serialization: " << static_cast<int>(serialization) << "\n"
                << "  checksum: 0x" << std::hex << checksum << std::dec
                << " (" << (VerifyChecksum() ? "VALID" : "INVALID") << ")\n"
                << "}";
            return ss.str();
        }

        // 消息类型转字符串
        static std::string MessageTypeToString(MessageType type) {
            switch (type) {
            case MessageType::HEARTBEAT:      return "HEARTBEAT";
            case MessageType::HANDSHAKE:      return "HANDSHAKE";
            case MessageType::DISCONNECT:     return "DISCONNECT";
            case MessageType::RECONNECT:      return "RECONNECT";
            case MessageType::LOGIN_REQUEST:  return "LOGIN_REQUEST";
            case MessageType::LOGIN_RESPONSE: return "LOGIN_RESPONSE";
            case MessageType::AUTH_TOKEN:     return "AUTH_TOKEN";
            case MessageType::PLAYER_MOVE:    return "PLAYER_MOVE";
            case MessageType::PLAYER_ACTION:  return "PLAYER_ACTION";
            case MessageType::PLAYER_CHAT:    return "PLAYER_CHAT";
            case MessageType::PLAYER_STATE:   return "PLAYER_STATE";
            case MessageType::ENTITY_SPAWN:   return "ENTITY_SPAWN";
            case MessageType::ENTITY_DESPAWN: return "ENTITY_DESPAWN";
            case MessageType::ENTITY_UPDATE:  return "ENTITY_UPDATE";
            case MessageType::COMBAT:         return "COMBAT";
            case MessageType::WORLD_QUERY:    return "WORLD_QUERY";
            case MessageType::WORLD_UPDATE:   return "WORLD_UPDATE";
            case MessageType::ERROR_RESPONSE: return "ERROR_RESPONSE";
            default:                          return "UNKNOWN";
            }
        }

        // 生成下一个包ID（线程安全）
        static uint32_t GeneratePacketId() {
            static std::atomic<uint32_t> counter{ 0 };
            return counter.fetch_add(1, std::memory_order_relaxed);
        }
    };
#pragma pack(pop)  // 恢复默认打包

    // Packet类：完整的网络数据包
    class Packet {
    private:
        PacketHeader header_;
        std::vector<uint8_t> payload_;

    public:
        Packet() = default;

        Packet(MessageType type, const std::vector<uint8_t>& payload = {})
            : header_(type) {
            header_.payload_length = static_cast<uint32_t>(payload.size());
            payload_ = payload;
            header_.UpdateChecksum();
        }

        Packet(const PacketHeader& header, const std::vector<uint8_t>& payload)
            : header_(header), payload_(payload) {
        }

        // 获取头部引用
        const PacketHeader& GetHeader() const { return header_; }
        PacketHeader& GetHeader() { return header_; }

        // 获取载荷
        const std::vector<uint8_t>& GetPayload() const { return payload_; }
        std::vector<uint8_t>& GetPayload() { return payload_; }

        // 设置载荷并更新头部
        void SetPayload(const std::vector<uint8_t>& payload) {
            payload_ = payload;
            header_.payload_length = static_cast<uint32_t>(payload.size());
            header_.UpdateChecksum();
        }

        // 序列化整个包
        std::vector<uint8_t> Serialize() const {
            std::vector<uint8_t> header_data = header_.Serialize();
            std::vector<uint8_t> result;
            result.reserve(header_data.size() + payload_.size());

            result.insert(result.end(), header_data.begin(), header_data.end());
            result.insert(result.end(), payload_.begin(), payload_.end());

            return result;
        }

        // 反序列化整个包
        static Packet Deserialize(const uint8_t* data, size_t length) {
            if (length < PacketHeader::HEADER_SIZE) {
                throw std::runtime_error("Insufficient data for packet");
            }

            // 解析头部
            PacketHeader header = PacketHeader::Deserialize(data, length);

            // 验证头部
            if (!header.Validate()) {
                throw std::runtime_error("Invalid packet header");
            }

            // 验证校验和
            if (!header.VerifyChecksum()) {
                throw std::runtime_error("Packet checksum verification failed");
            }

            // 提取载荷
            std::vector<uint8_t> payload;
            if (header.payload_length > 0) {
                if (length < PacketHeader::HEADER_SIZE + header.payload_length) {
                    throw std::runtime_error("Incomplete payload data");
                }

                payload.assign(
                    data + PacketHeader::HEADER_SIZE,
                    data + PacketHeader::HEADER_SIZE + header.payload_length
                );
            }

            return Packet(header, payload);
        }

        // 调试输出
        std::string ToString() const {
            std::stringstream ss;
            ss << header_.ToString() << "\n";
            ss << "Payload: " << payload_.size() << " bytes";

            if (!payload_.empty()) {
                ss << " [";
                size_t preview_size = std::min(payload_.size(), static_cast<size_t>(16));
                for (size_t i = 0; i < preview_size; ++i) {
                    ss << std::hex << std::setw(2) << std::setfill('0')
                        << static_cast<int>(payload_[i]) << " ";
                }
                if (payload_.size() > preview_size) {
                    ss << "...";
                }
                ss << std::dec << "]";
            }

            return ss.str();
        }

        // 压缩载荷
        bool Compress(CompressionType type = CompressionType::LZ4) {
            if (payload_.empty() || header_.IsCompressed()) {
                return false;
            }

            // 这里实现具体的压缩逻辑
            // 简化版本：标记为压缩，实际压缩由上层实现
            header_.compression = static_cast<uint8_t>(type);
            header_.SetCompressed(true);
            header_.UpdateChecksum();

            return true;
        }

        // 解压缩载荷
        bool Decompress() {
            if (!header_.IsCompressed() || payload_.empty()) {
                return false;
            }

            // 这里实现具体的解压缩逻辑
            header_.SetCompressed(false);
            header_.compression = 0;
            header_.UpdateChecksum();

            return true;
        }
    };

    // 分片包管理器
    class PacketFragmenter {
    public:
        struct Fragment {
            PacketHeader header;
            std::vector<uint8_t> data;
            uint16_t fragment_index;
            uint16_t total_fragments;
        };

        // 将大数据包分片
        static std::vector<Fragment> FragmentPacket(
            const Packet& packet,
            size_t max_fragment_size = 1400) {  // MTU大小

            std::vector<Fragment> fragments;

            const auto& payload = packet.GetPayload();
            size_t total_size = payload.size();
            size_t fragment_count = (total_size + max_fragment_size - 1) / max_fragment_size;

            if (fragment_count <= 1) {
                // 不需要分片
                Fragment frag;
                frag.header = packet.GetHeader();
                frag.header.SetFragmented(false);
                frag.data = payload;
                frag.fragment_index = 0;
                frag.total_fragments = 1;
                fragments.push_back(frag);
                return fragments;
            }

            uint32_t base_packet_id = packet.GetHeader().packet_id;

            for (uint16_t i = 0; i < fragment_count; ++i) {
                size_t offset = i * max_fragment_size;
                size_t size = std::min(max_fragment_size, total_size - offset);

                Fragment frag;
                frag.header = packet.GetHeader();
                frag.header.packet_id = base_packet_id + i;  // 为每个分片分配唯一ID
                frag.header.payload_length = static_cast<uint32_t>(size);
                frag.header.SetFragmented(true);

                if (i == fragment_count - 1) {
                    frag.header.SetLastFragment(true);
                }

                frag.fragment_index = i;
                frag.total_fragments = static_cast<uint16_t>(fragment_count);

                // 复制分片数据
                frag.data.assign(
                    payload.begin() + offset,
                    payload.begin() + offset + size
                );

                fragments.push_back(frag);
            }

            return fragments;
        }

        // 重组分片包
        static std::optional<Packet> ReassembleFragments(
            const std::vector<Fragment>& fragments) {

            if (fragments.empty()) {
                return std::nullopt;
            }

            // 检查是否所有分片都已到达
            uint16_t expected_count = fragments[0].total_fragments;
            std::vector<bool> received(expected_count, false);

            for (const auto& frag : fragments) {
                if (frag.fragment_index < expected_count) {
                    received[frag.fragment_index] = true;
                }
            }

            // 检查是否有缺失的分片
            if (std::any_of(received.begin(), received.end(), [](bool b) { return !b; })) {
                return std::nullopt;
            }

            // 按索引排序
            std::vector<Fragment> sorted_fragments = fragments;
            std::sort(sorted_fragments.begin(), sorted_fragments.end(),
                [](const Fragment& a, const Fragment& b) {
                    return a.fragment_index < b.fragment_index;
                });

            // 重组数据
            std::vector<uint8_t> reassembled_data;
            for (const auto& frag : sorted_fragments) {
                reassembled_data.insert(reassembled_data.end(),
                    frag.data.begin(), frag.data.end());
            }

            // 创建重组后的包
            PacketHeader header = fragments[0].header;
            header.payload_length = static_cast<uint32_t>(reassembled_data.size());
            header.SetFragmented(false);
            header.SetLastFragment(false);
            header.UpdateChecksum();

            return Packet(header, reassembled_data);
        }
    };

} // namespace Network



