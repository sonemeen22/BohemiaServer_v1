#include "ThreadPool.h"
#include <algorithm>
#include <numeric>
#include <sstream>
#include <random>

ThreadPool::ThreadPool(const Config& config)
    : config_(config)
    , queue_capacity_(config.queue_capacity)
    , pool_start_time_(std::chrono::steady_clock::now()) {

    // 验证配置
    if (config_.min_threads == 0) {
        config_.min_threads = 1;
    }

    if (config_.max_threads < config_.min_threads) {
        config_.max_threads = config_.min_threads;
    }

    if (config_.queue_capacity == 0) {
        config_.queue_capacity = 1000;
    }

    // 初始化工作线程
    InitializeWorkers(config_.min_threads);

    // 启动自动缩放线程（如果启用）
    if (config_.enable_auto_scaling) {
        auto_scale_thread_ = std::thread([this]() {
            while (!stop_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                if (!pause_ && !stop_) {
                    AutoScaleThreads();
                }
            }
            });
    }

    // 启动统计收集线程
    stats_collector_thread_ = std::thread([this]() {
        while (!stop_) {
            std::this_thread::sleep_for(config_.stats_collection_interval);
            if (!stop_) {
                CollectStatistics();
            }
        }
        });

    // 启动空闲线程清理线程
    cleanup_thread_ = std::thread([this]() {
        while (!stop_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            if (!pause_ && !stop_) {
                CleanupIdleThreads();
            }
        }
        });

    // 初始化本地队列（用于任务窃取）
    if (config_.enable_task_stealing) {
        worker_local_queues_.resize(config_.max_threads);
    }
}

ThreadPool::~ThreadPool() {
    Stop();
}

void ThreadPool::InitializeWorkers(size_t num_threads) {
    std::lock_guard lock(pool_mutex_);

    size_t current_size = workers_.size();
    if (num_threads < current_size) {
        // 减少线程数
        for (size_t i = num_threads; i < current_size; ++i) {
            if (workers_[i]) {
                workers_[i]->state = ThreadState::STOPPING;
            }
        }
        condition_.notify_all();

        // 等待线程退出
        for (size_t i = num_threads; i < current_size; ++i) {
            if (workers_[i] && workers_[i]->thread.joinable()) {
                workers_[i]->thread.join();
            }
        }

        workers_.resize(num_threads);
    }
    else if (num_threads > current_size) {
        // 增加线程数
        workers_.reserve(num_threads);
        for (size_t i = current_size; i < num_threads; ++i) {
            auto worker = std::make_unique<WorkerInfo>();
            worker->state = ThreadState::IDLE;
            worker->thread = std::thread(&ThreadPool::WorkerThread, this, i);
            worker->thread_id = worker->thread.get_id();
            worker->last_task_time = std::chrono::steady_clock::now();

            // 设置线程亲和性（可选）
            // SetThreadAffinity(worker->thread, i % std::thread::hardware_concurrency());

            workers_.push_back(std::move(worker));
            thread_id_to_index_[workers_.back()->thread_id] = i;
        }
    }

    active_threads_ = num_threads;
}

void ThreadPool::WorkerThread(size_t worker_id) {
    WorkerInfo* worker_info = nullptr;

    // 获取worker信息
    {
        std::lock_guard lock(pool_mutex_);
        if (worker_id < workers_.size()) {
            worker_info = workers_[worker_id].get();
        }
    }

    if (!worker_info) {
        return;
    }

    std::string thread_name = "Worker-" + std::to_string(worker_id);
#ifdef __linux__
    pthread_setname_np(pthread_self(), thread_name.c_str());
#endif

    while (true) {
        Task task;
        bool got_task = false;

        {
            std::unique_lock lock(pool_mutex_);

            // 等待任务或停止信号
            condition_.wait(lock, [this, worker_info]() {
                return stop_ ||
                    worker_info->state == ThreadState::STOPPING ||
                    !task_queue_.empty() ||
                    (config_.enable_task_stealing);
                });

            // 检查是否需要退出
            if (stop_ || worker_info->state == ThreadState::STOPPING) {
                worker_info->state = ThreadState::STOPPING;
                break;
            }

            // 获取任务
            got_task = GetTask(task);

            if (got_task) {
                worker_info->state = ThreadState::BUSY;
                worker_info->last_task_time = std::chrono::steady_clock::now();
                --pending_tasks_;
            }
            else {
                worker_info->state = ThreadState::IDLE;
                continue;
            }
        }

        // 执行任务
        if (got_task) {
            auto start_time = std::chrono::steady_clock::now();

            try {
                task.func();
                ++completed_tasks_;
                if (worker_info) {
                    ++worker_info->tasks_executed;
                }
            }
            catch (const std::exception& e) {
                ++failed_tasks_;
                if (worker_info) {
                    ++worker_info->tasks_failed;
                }

                // 调用异常处理器
                if (exception_handler_) {
                    try {
                        exception_handler_(e);
                    }
                    catch (...) {
                        // 忽略异常处理器本身的异常
                    }
                }
                else {
                    // 默认异常处理：记录日志
                    std::cerr << "ThreadPool task exception in worker "
                        << worker_id << ": " << e.what() << std::endl;
                }
            }
            catch (...) {
                ++failed_tasks_;
                if (worker_info) {
                    ++worker_info->tasks_failed;
                }

                if (exception_handler_) {
                    try {
                        exception_handler_(std::runtime_error("Unknown exception"));
                    }
                    catch (...) {
                        // 忽略异常处理器本身的异常
                    }
                }
            }

            auto end_time = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
                end_time - start_time).count();

            total_task_duration_ns_ += duration;

            // 更新worker状态
            if (worker_info) {
                worker_info->state = ThreadState::IDLE;
            }

            // 通知可能等待的WaitAll
            if (pending_tasks_ == 0) {
                condition_empty_.notify_all();
            }
        }
    }
}

bool ThreadPool::GetTask(Task& task) {
    std::lock_guard lock(pool_mutex_);

    if (task_queue_.empty()) {
        // 尝试任务窃取
        if (config_.enable_task_stealing) {
            return TryStealTask(task);
        }
        return false;
    }

    task = std::move(task_queue_.top());
    task_queue_.pop();
    return true;
}

bool ThreadPool::TryStealTask(Task& task) {
    if (!config_.enable_task_stealing || worker_local_queues_.empty()) {
        return false;
    }

    std::lock_guard lock(local_queue_mutex_);

    // 随机选择开始位置，避免总是从同一个worker窃取
    static thread_local std::mt19937 generator(std::random_device{}());
    std::uniform_int_distribution<size_t> distribution(0, worker_local_queues_.size() - 1);
    size_t start_idx = distribution(generator);

    for (size_t i = 0; i < worker_local_queues_.size(); ++i) {
        size_t idx = (start_idx + i) % worker_local_queues_.size();
        if (!worker_local_queues_[idx].empty()) {
            task = std::move(worker_local_queues_[idx].front());
            worker_local_queues_[idx].pop();
            return true;
        }
    }

    return false;
}

template<class F, class... Args>
auto ThreadPool::Submit(F&& f, Args&&... args)
-> std::future<typename std::invoke_result_t<F, Args...>> {

    using ReturnType = typename std::invoke_result_t<F, Args...>;

    auto task = std::make_shared<std::packaged_task<ReturnType()>>(
        std::bind(std::forward<F>(f), std::forward<Args>(args)...)
    );

    std::future<ReturnType> result = task->get_future();

    {
        std::unique_lock lock(pool_mutex_);

        if (stop_) {
            throw std::runtime_error("ThreadPool is stopped");
        }

        // 检查队列容量
        if (task_queue_.size() >= queue_capacity_) {
            throw std::runtime_error("ThreadPool queue is full");
        }

        // 添加任务到队列
        task_queue_.push(Task{
            [task]() { (*task)(); },
            Priority::NORMAL,
            std::chrono::steady_clock::now(),
            std::nullopt
            });

        ++pending_tasks_;
    }

    condition_.notify_one();
    return result;
}

template<class F, class... Args>
auto ThreadPool::SubmitWithPriority(Priority priority, F&& f, Args&&... args)
-> std::future<typename std::invoke_result_t<F, Args...>> {

    using ReturnType = typename std::invoke_result_t<F, Args...>;

    auto task = std::make_shared<std::packaged_task<ReturnType()>>(
        std::bind(std::forward<F>(f), std::forward<Args>(args)...)
    );

    std::future<ReturnType> result = task->get_future();

    {
        std::unique_lock lock(pool_mutex_);

        if (stop_) {
            throw std::runtime_error("ThreadPool is stopped");
        }

        if (task_queue_.size() >= queue_capacity_) {
            throw std::runtime_error("ThreadPool queue is full");
        }

        task_queue_.push(Task{
            [task]() { (*task)(); },
            priority,
            std::chrono::steady_clock::now(),
            std::nullopt
            });

        ++pending_tasks_;
    }

    condition_.notify_one();
    return result;
}

template<class F, class... Args>
std::vector<std::future<typename std::invoke_result_t<F, Args...>>>
ThreadPool::SubmitBatch(size_t count, F&& f, Args&&... args) {

    using ReturnType = typename std::invoke_result_t<F, Args...>;
    std::vector<std::future<ReturnType>> results;
    results.reserve(count);

    {
        std::unique_lock lock(pool_mutex_);

        if (stop_) {
            throw std::runtime_error("ThreadPool is stopped");
        }

        if (task_queue_.size() + count > queue_capacity_) {
            throw std::runtime_error("ThreadPool queue capacity exceeded");
        }

        for (size_t i = 0; i < count; ++i) {
            auto task = std::make_shared<std::packaged_task<ReturnType()>>(
                std::bind(std::forward<F>(f), std::forward<Args>(args)...)
            );

            results.push_back(task->get_future());

            task_queue_.push(Task{
                [task]() { (*task)(); },
                Priority::NORMAL,
                std::chrono::steady_clock::now(),
                std::nullopt
                });

            ++pending_tasks_;
        }
    }

    if (count > 1) {
        condition_.notify_all();
    }
    else if (count == 1) {
        condition_.notify_one();
    }

    return results;
}

template<class F, class... Args>
void ThreadPool::Post(F&& f, Args&&... args) {
    {
        std::unique_lock lock(pool_mutex_);

        if (stop_) {
            return;
        }

        if (task_queue_.size() >= queue_capacity_) {
            // 队列满时，根据策略处理
            // 这里简单丢弃，实际应用中可能需要其他策略
            return;
        }

        auto task = std::make_shared<std::function<void()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );

        task_queue_.push(Task{
            [task]() { (*task)(); },
            Priority::NORMAL,
            std::chrono::steady_clock::now(),
            std::nullopt
            });

        ++pending_tasks_;
    }

    condition_.notify_one();
}

void ThreadPool::WaitAll() {
    std::unique_lock lock(pool_mutex_);
    condition_empty_.wait(lock, [this]() {
        return pending_tasks_ == 0 || stop_;
        });
}

void ThreadPool::Stop() {
    if (stop_) {
        return;
    }

    {
        std::lock_guard lock(pool_mutex_);
        stop_ = true;

        // 标记所有线程为停止状态
        for (auto& worker : workers_) {
            if (worker) {
                worker->state = ThreadState::STOPPING;
            }
        }
    }

    // 通知所有线程
    condition_.notify_all();
    condition_empty_.notify_all();

    // 等待辅助线程
    if (auto_scale_thread_.joinable()) {
        auto_scale_thread_.join();
    }

    if (stats_collector_thread_.joinable()) {
        stats_collector_thread_.join();
    }

    if (cleanup_thread_.joinable()) {
        cleanup_thread_.join();
    }

    // 等待工作线程
    {
        std::lock_guard lock(pool_mutex_);
        for (auto& worker : workers_) {
            if (worker && worker->thread.joinable()) {
                worker->thread.join();
            }
        }
        workers_.clear();
    }

    // 清空任务队列
    std::priority_queue<Task, std::vector<Task>, std::greater<Task>> empty_queue;
    std::swap(task_queue_, empty_queue);
    pending_tasks_ = 0;
}

void ThreadPool::Restart() {
    Stop();

    {
        std::lock_guard lock(pool_mutex_);
        stop_ = false;

        // 重新初始化
        InitializeWorkers(config_.min_threads);

        // 重置统计信息
        stats_ = Statistics{};
        pool_start_time_ = std::chrono::steady_clock::now();
        completed_tasks_ = 0;
        failed_tasks_ = 0;
        pending_tasks_ = 0;
        total_task_duration_ns_ = 0;
    }

    // 重新启动辅助线程
    if (config_.enable_auto_scaling) {
        auto_scale_thread_ = std::thread([this]() {
            while (!stop_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                AutoScaleThreads();
            }
            });
    }

    stats_collector_thread_ = std::thread([this]() {
        while (!stop_) {
            std::this_thread::sleep_for(config_.stats_collection_interval);
            CollectStatistics();
        }
        });

    cleanup_thread_ = std::thread([this]() {
        while (!stop_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            CleanupIdleThreads();
        }
        });
}

void ThreadPool::Resize(size_t num_threads) {
    if (num_threads < config_.min_threads || num_threads > config_.max_threads) {
        throw std::invalid_argument("Thread count out of range");
    }

    if (stop_) {
        throw std::runtime_error("ThreadPool is stopped");
    }

    InitializeWorkers(num_threads);
}

void ThreadPool::AutoScaleThreads() {
    std::unique_lock lock(pool_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
        return;
    }

    if (stop_ || pause_) {
        return;
    }

    size_t current_threads = workers_.size();
    size_t pending = pending_tasks_;
    size_t busy_threads = 0;

    // 统计忙线程数量
    for (const auto& worker : workers_) {
        if (worker && worker->state == ThreadState::BUSY) {
            ++busy_threads;
        }
    }

    // 自动缩放策略
    if (pending > current_threads * 2 && current_threads < config_.max_threads) {
        // 任务太多，增加线程
        size_t new_size = std::min(current_threads * 2, config_.max_threads);
        size_t add_count = new_size - current_threads;

        for (size_t i = 0; i < add_count; ++i) {
            auto worker = std::make_unique<WorkerInfo>();
            worker->state = ThreadState::IDLE;
            worker->thread = std::thread(&ThreadPool::WorkerThread, this, workers_.size());
            worker->thread_id = worker->thread.get_id();
            worker->last_task_time = std::chrono::steady_clock::now();

            workers_.push_back(std::move(worker));
            thread_id_to_index_[workers_.back()->thread_id] = workers_.size() - 1;
        }

        active_threads_ = workers_.size();

    }
    else if (busy_threads < current_threads / 2 && current_threads > config_.min_threads) {
        // 太多空闲线程，减少线程
        size_t remove_count = std::min(current_threads - busy_threads,
            current_threads - config_.min_threads);

        if (remove_count > 0) {
            // 标记要停止的线程（从后往前）
            for (size_t i = 0; i < remove_count && i < workers_.size(); ++i) {
                size_t idx = workers_.size() - 1 - i;
                if (workers_[idx] && workers_[idx]->state == ThreadState::IDLE) {
                    workers_[idx]->state = ThreadState::STOPPING;
                }
            }

            condition_.notify_all();
        }
    }
}

void ThreadPool::CleanupIdleThreads() {
    std::unique_lock lock(pool_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
        return;
    }

    if (stop_ || pause_) {
        return;
    }

    auto now = std::chrono::steady_clock::now();
    size_t current_size = workers_.size();

    // 清理长时间空闲的线程
    for (size_t i = 0; i < workers_.size(); ++i) {
        if (workers_[i] &&
            workers_[i]->state == ThreadState::IDLE &&
            now - workers_[i]->last_task_time > config_.thread_keep_alive_time &&
            current_size > config_.min_threads) {

            workers_[i]->state = ThreadState::STOPPING;
            condition_.notify_all();

            // 等待线程退出
            if (workers_[i]->thread.joinable()) {
                lock.unlock();
                workers_[i]->thread.join();
                lock.lock();
            }

            // 从映射中移除
            thread_id_to_index_.erase(workers_[i]->thread_id);

            // 移除worker
            workers_.erase(workers_.begin() + i);
            --current_size;
            --i; // 调整索引
        }
    }

    active_threads_ = current_size;
}

void ThreadPool::CollectStatistics() {
    std::lock_guard lock(stats_mutex_);

    auto now = std::chrono::steady_clock::now();

    // 更新统计信息
    stats_.total_tasks_executed = completed_tasks_;
    stats_.total_tasks_failed = failed_tasks_;
    stats_.tasks_pending = pending_tasks_;
    stats_.total_running_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - pool_start_time_).count();

    // 计算平均任务时长
    if (completed_tasks_ > 0) {
        stats_.avg_task_duration_ms =
            (total_task_duration_ns_ / 1000000.0) / completed_tasks_;
    }

    // 统计线程状态
    {
        std::lock_guard pool_lock(pool_mutex_);
        stats_.threads_busy = 0;
        stats_.threads_idle = 0;

        for (const auto& worker : workers_) {
            if (worker) {
                if (worker->state == ThreadState::BUSY) {
                    ++stats_.threads_busy;
                }
                else if (worker->state == ThreadState::IDLE) {
                    ++stats_.threads_idle;
                }
            }
        }
    }

    last_stats_time_ = now;
}

ThreadPool::Statistics ThreadPool::GetStatistics() const {
    std::lock_guard lock(stats_mutex_);
    return stats_;
}

size_t ThreadPool::GetThreadCount() const {
    std::lock_guard lock(pool_mutex_);
    return workers_.size();
}

size_t ThreadPool::GetPendingTaskCount() const {
    return pending_tasks_;
}

void ThreadPool::SetExceptionHandler(std::function<void(const std::exception&)> handler) {
    exception_handler_ = std::move(handler);
}

void ThreadPool::SetThreadAffinity(std::thread& thread, size_t cpu_id) {
    // 平台特定的线程亲和性设置
    // 这里只是一个示例，实际实现需要根据平台调整
#ifdef __linux__
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_id, &cpuset);

    pthread_t thread_handle = thread.native_handle();
    pthread_setaffinity_np(thread_handle, sizeof(cpu_set_t), &cpuset);
#elif defined(_WIN32)
    // Windows实现
    // SetThreadAffinityMask(thread.native_handle(), 1ULL << cpu_id);
#endif
}
