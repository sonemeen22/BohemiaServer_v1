#pragma once
#include <vector>
#include <thread>
#include <queue>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <future>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <chrono>
#include <optional>
#include <iostream>
#include <unordered_map>

class ThreadPool {
public:
    // 任务优先级
    enum class Priority : uint8_t {
        LOW = 0,
        NORMAL = 1,
        HIGH = 2,
        CRITICAL = 3
    };

    // 线程状态
    enum class ThreadState : uint8_t {
        IDLE,
        BUSY,
        STOPPING
    };

    // 统计信息
    struct Statistics {
        size_t total_tasks_executed{ 0 };
        size_t total_tasks_failed{ 0 };
        size_t tasks_pending{ 0 };
        size_t threads_busy{ 0 };
        size_t threads_idle{ 0 };
        double avg_task_duration_ms{ 0.0 };
        uint64_t total_running_time_ms{ 0 };
    };

    // 配置
    struct Config {
        size_t min_threads{ 1 };
        size_t max_threads{ std::thread::hardware_concurrency() };
        size_t queue_capacity{ 1000 };
        size_t max_batch_size{ 10 };
        bool enable_auto_scaling{ true };
        bool enable_task_stealing{ true };
        std::chrono::milliseconds thread_keep_alive_time{ 5000 };
        std::chrono::milliseconds stats_collection_interval{ 1000 };
    };

    explicit ThreadPool(const Config& config = Config());
    ~ThreadPool();

    // 禁用拷贝和移动
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;
    ThreadPool(ThreadPool&&) = delete;
    ThreadPool& operator=(ThreadPool&&) = delete;

    // 提交任务（返回future）
    template<class F, class... Args>
    auto Submit(F&& f, Args&&... args)
        -> std::future<typename std::invoke_result_t<F, Args...>>;

    // 提交带优先级的任务
    template<class F, class... Args>
    auto SubmitWithPriority(Priority priority, F&& f, Args&&... args)
        -> std::future<typename std::invoke_result_t<F, Args...>>;

    // 提交批量任务
    template<class F, class... Args>
    std::vector<std::future<typename std::invoke_result_t<F, Args...>>>
        SubmitBatch(size_t count, F&& f, Args&&... args);

    // 异步提交（不等待结果）
    template<class F, class... Args>
    void Post(F&& f, Args&&... args);

    // 等待所有任务完成
    void WaitAll();

    // 停止线程池
    void Stop();

    // 重启线程池
    void Restart();

    // 调整线程数量
    void Resize(size_t num_threads);

    // 获取统计信息
    Statistics GetStatistics() const;

    // 获取配置
    Config GetConfig() const { return config_; }

    // 获取当前线程池大小
    size_t GetThreadCount() const;

    // 获取等待任务数量
    size_t GetPendingTaskCount() const;

    // 检查是否正在运行
    bool IsRunning() const { return !stop_; }

    // 设置线程异常处理器
    void SetExceptionHandler(std::function<void(const std::exception&)> handler);

private:
    struct Task {
        using FuncType = std::function<void()>;

        FuncType func;
        Priority priority{ Priority::NORMAL };
        std::chrono::steady_clock::time_point enqueue_time;
        std::optional<std::string> task_name;

        bool operator<(const Task& other) const {
            return static_cast<uint8_t>(priority) <
                static_cast<uint8_t>(other.priority);
        }

        bool operator>(const Task& other) const {
            return static_cast<uint8_t>(priority) >
                static_cast<uint8_t>(other.priority);
        }
    };

    struct WorkerInfo {
        std::thread thread;
        ThreadState state{ ThreadState::IDLE };
        std::thread::id thread_id;
        std::chrono::steady_clock::time_point last_task_time;
        size_t tasks_executed{ 0 };
        size_t tasks_failed{ 0 };
    };

    // 线程工作函数
    void WorkerThread(size_t worker_id);

    // 任务窃取
    bool TryStealTask(Task& task);

    // 自动缩放线程
    void AutoScaleThreads();

    // 收集统计信息
    void CollectStatistics();

    // 清理空闲线程
    void CleanupIdleThreads();

    // 获取任务（支持优先级）
    bool GetTask(Task& task);

    // 初始化工作线程
    void InitializeWorkers(size_t num_threads);

    // 设置线程亲和性（可选）
    void SetThreadAffinity(std::thread& thread, size_t cpu_id);

private:
    mutable std::mutex pool_mutex_;
    std::condition_variable condition_;
    std::condition_variable condition_empty_;

    // 任务队列（使用优先队列）
    std::priority_queue<Task, std::vector<Task>, std::greater<Task>> task_queue_;
    size_t queue_capacity_;

    // 工作线程管理
    std::vector<std::unique_ptr<WorkerInfo>> workers_;
    std::unordered_map<std::thread::id, size_t> thread_id_to_index_;

    // 配置
    Config config_;

    // 状态控制
    std::atomic<bool> stop_{ false };
    std::atomic<bool> pause_{ false };
    std::atomic<size_t> active_threads_{ 0 };
    std::atomic<size_t> pending_tasks_{ 0 };
    std::atomic<size_t> completed_tasks_{ 0 };
    std::atomic<size_t> failed_tasks_{ 0 };

    // 统计
    mutable std::mutex stats_mutex_;
    Statistics stats_;
    std::chrono::steady_clock::time_point pool_start_time_;
    std::chrono::steady_clock::time_point last_stats_time_;

    // 异常处理器
    std::function<void(const std::exception&)> exception_handler_;

    // 自动缩放定时器
    std::thread auto_scale_thread_;
    std::thread stats_collector_thread_;
    std::thread cleanup_thread_;

    // 任务窃取相关
    std::vector<std::queue<Task>> worker_local_queues_;
    std::mutex local_queue_mutex_;

    // 性能计数器
    std::atomic<uint64_t> total_task_duration_ns_{ 0 };
};
